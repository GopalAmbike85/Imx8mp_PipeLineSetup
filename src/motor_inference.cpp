// motor_inference.cpp  (v1.1.0)
//
// See motor_inference.h for the module's contract and rationale (NPU by
// default as of 2026-09-07, with an unresolved accuracy caveat; CPU
// fallback via delegate_path=NULL). Ported from cnr-model-test-harness's
// controller_test_harness/example.cpp - same constants, same pipeline
// (controller_pipeline.hpp, copied verbatim into this repo).

#include "motor_inference.h"

#include <algorithm>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <memory>
#include <thread>
#include <vector>

#include "controller_pipeline.hpp"

#include "tensorflow/lite/delegates/external/external_delegate.h"
#include "tensorflow/lite/interpreter.h"
#include "tensorflow/lite/kernels/register.h"
#include "tensorflow/lite/model.h"

namespace {

// ---- Configuration / placeholders (identical to the offline harness) ----
constexpr int kSeqLen = 20;  // must match model input shape (1, SEQ_LEN, 2)
constexpr double kControlTs = 0.02;      // assumed nominal loop period (s), 50 Hz
constexpr double kThetaLimitDeg = 90.0;  // max bend angle at full deflection
constexpr double kPhiOffset = 0.0;       // radians, added to phi
constexpr double kAnchorDistance = 0.05; // min cumulative (u, v) distance

// Model input/output quantization parameters (from the model's own
// tensor metadata - see cnr-model-test-harness/controller_test_harness/
// quant_info.txt).
constexpr float kInputScale = 0.4401035010814667f;
constexpr int kInputZeroPoint = -7;
constexpr float kOutputScale = 0.04787081480026245f;
constexpr int kOutputZeroPoint = -128;

// Low-pass filter (CascadedEMA) applied to the 3 tendon motor outputs.
constexpr double kLpfFc = 10.0;  // cutoff frequency (Hz)
constexpr int kLpfStages = 2;

constexpr int kNumTendonMotors = 3;
constexpr double kInsertionMotorPosition = 0.0;  // constant placeholder

std::unique_ptr<tflite::FlatBufferModel> g_model;
std::unique_ptr<tflite::Interpreter> g_interpreter;
TfLiteTensor *g_input_tensor = nullptr;
TfLiteTensor *g_output_tensor = nullptr;
TfLiteDelegate *g_npu_delegate = nullptr;

std::unique_ptr<cnr_pipeline::HistoryWindowBuilder> g_history;
std::unique_ptr<cnr_pipeline::CascadedEMA> g_lpf;
bool g_lpf_initialized = false;

const double kThetaLimitRad = cnr_pipeline::DegToRad(kThetaLimitDeg);

}  // namespace

int motor_inference_init(const char *model_path, const char *delegate_path) {
    g_model = tflite::FlatBufferModel::BuildFromFile(model_path);
    if (!g_model) {
        std::cerr << "motor_inference: failed to load model: " << model_path
                  << "\n";
        return -1;
    }

    tflite::ops::builtin::BuiltinOpResolver resolver;
    if (tflite::InterpreterBuilder(*g_model, resolver)(&g_interpreter) !=
            kTfLiteOk ||
        !g_interpreter) {
        std::cerr << "motor_inference: failed to build interpreter\n";
        return -1;
    }

    unsigned hw = std::thread::hardware_concurrency();
    g_interpreter->SetNumThreads(hw > 0 ? static_cast<int>(hw) : 4);

    const bool use_npu = delegate_path && std::strlen(delegate_path) > 0;
    if (use_npu) {
        TfLiteExternalDelegateOptions ext_options =
            TfLiteExternalDelegateOptionsDefault(delegate_path);
        g_npu_delegate = TfLiteExternalDelegateCreate(&ext_options);
        if (!g_npu_delegate) {
            std::cerr << "motor_inference: failed to load NPU delegate from "
                      << delegate_path << "\n";
            return -1;
        }
        if (g_interpreter->ModifyGraphWithDelegate(g_npu_delegate) !=
            kTfLiteOk) {
            std::cerr << "motor_inference: NPU delegate rejected the model "
                         "graph\n";
            return -1;
        }
        std::cerr << "motor_inference: NPU delegate active: " << delegate_path
                   << "\n";
    } else {
        std::cerr << "motor_inference: running on CPU, threads="
                   << (hw > 0 ? static_cast<int>(hw) : 4) << "\n";
    }

    if (g_interpreter->AllocateTensors() != kTfLiteOk) {
        std::cerr << "motor_inference: AllocateTensors failed\n";
        return -1;
    }

    g_input_tensor = g_interpreter->tensor(g_interpreter->inputs()[0]);
    g_output_tensor = g_interpreter->tensor(g_interpreter->outputs()[0]);

    if (g_input_tensor->type != kTfLiteInt8 ||
        g_output_tensor->type != kTfLiteInt8) {
        std::cerr << "motor_inference: expected int8 input/output tensors\n";
        return -1;
    }
    if (g_input_tensor->bytes != static_cast<size_t>(kSeqLen * 2) ||
        g_output_tensor->bytes != static_cast<size_t>(kNumTendonMotors)) {
        std::cerr << "motor_inference: model tensor shapes don't match "
                     "expected (input "
                  << g_input_tensor->bytes << " bytes, expected "
                  << (kSeqLen * 2) << "; output " << g_output_tensor->bytes
                  << " bytes, expected " << kNumTendonMotors << ")\n";
        return -1;
    }

    g_history = std::make_unique<cnr_pipeline::HistoryWindowBuilder>(
        kSeqLen, kAnchorDistance);
    g_lpf = std::make_unique<cnr_pipeline::CascadedEMA>(
        kNumTendonMotors, kControlTs, kLpfFc, kLpfStages);
    g_lpf_initialized = false;

    if (use_npu) {
        // Absorb the NPU's one-time graph-compile cost (~10.4s measured on
        // this board) HERE, during startup and before either vhost can
        // accept a connection - this server's lws_service() loop is
        // single-threaded and synchronous, so an uncompiled first real
        // Invoke() would block ALL servicing for that entire window,
        // likely dropping both the controller and ICU connections. Input
        // content doesn't matter for this - compilation depends on graph
        // structure, not data - so zeroed input is fine. This throwaway
        // call must not touch g_history/g_lpf (a real session hasn't
        // started yet), so it writes the input tensor directly rather
        // than going through motor_inference_compute().
        std::memset(g_input_tensor->data.int8, kInputZeroPoint,
                    g_input_tensor->bytes);
        if (g_interpreter->Invoke() != kTfLiteOk) {
            std::cerr << "motor_inference: NPU warm-up invoke failed\n";
            return -1;
        }
    }

    return 0;
}

void motor_inference_reset(void) {
    // TFLite's LSTM op keeps its own hidden/cell state in the interpreter's
    // internal "variable" tensors, which otherwise persist across Invoke()
    // calls regardless of what HistoryWindowBuilder/CascadedEMA are doing -
    // the offline batch tool never hit this because it creates a brand-new
    // interpreter per run, but this server reuses one interpreter across
    // many independent controller sessions. Found via a real two-session
    // test on hardware: the exact same quantized input window produced a
    // different model output on a second session versus the first, with
    // this call missing. Without it, a new session's first prediction is
    // contaminated by whatever the LSTM's state happened to be at the end
    // of the PREVIOUS session.
    if (g_interpreter) g_interpreter->ResetVariableTensors();
    if (g_history) g_history->Reset();
    g_lpf_initialized = false;
}

int motor_inference_compute(double joystick_x, double joystick_y,
                             double motor_out[4]) {
    const double x = std::clamp(joystick_x, -1.0, 1.0);
    const double y = std::clamp(joystick_y, -1.0, 1.0);

    double theta, phi;
    cnr_pipeline::XyToAngles(x, y, kThetaLimitRad, theta, phi);
    phi += kPhiOffset;

    double u, v;
    cnr_pipeline::FeatureTransform(theta, phi, u, v);

    const auto window = g_history->Build(u, v);  // (kSeqLen, 2) float

    int8_t *in_data = g_input_tensor->data.int8;
    for (int t = 0; t < kSeqLen; ++t) {
        in_data[t * 2 + 0] =
            cnr_pipeline::Quantize(window[t][0], kInputScale, kInputZeroPoint);
        in_data[t * 2 + 1] =
            cnr_pipeline::Quantize(window[t][1], kInputScale, kInputZeroPoint);
    }

    if (g_interpreter->Invoke() != kTfLiteOk) {
        std::cerr << "motor_inference: inference failed\n";
        return -1;
    }

    const int8_t *out_data = g_output_tensor->data.int8;
    std::vector<double> tendon(kNumTendonMotors);
    for (int k = 0; k < kNumTendonMotors; ++k) {
        tendon[k] =
            cnr_pipeline::Dequantize(out_data[k], kOutputScale, kOutputZeroPoint);
    }

    // Seed the filter with the first prediction (of this session) so it
    // starts at the actual initial pose instead of ramping up from zero.
    if (!g_lpf_initialized) {
        g_lpf->Reset(tendon);
        g_lpf_initialized = true;
    }
    const auto filtered = g_lpf->Step(tendon);

    motor_out[0] = filtered[0];
    motor_out[1] = filtered[1];
    motor_out[2] = filtered[2];
    motor_out[3] = kInsertionMotorPosition;
    return 0;
}
