#ifndef MOTOR_INFERENCE_H
#define MOTOR_INFERENCE_H

#include <stddef.h>

/*
 * C-linkage wrapper around the C++ TfLSTM controller model (joystick ->
 * motor positions), so the rest of this project can stay plain C.
 *
 * Ported from cnr-model-test-harness/controller_test_harness/example.cpp
 * (see that repo's README.md for the full pipeline description and every
 * tunable parameter's meaning).
 *
 * NPU (default) vs CPU: switched to the NPU/VX-delegate path 2026-09-07 at
 * explicit user request, made with full knowledge of an UNRESOLVED
 * accuracy caveat - see cnr-tflite-npu-findings. The NPU delegate produces
 * motor positions up to ~0.49 units different from the validated CPU/
 * Python reference, for reasons not root-caused. That divergence has not
 * been evaluated against the robot's real mechanical tolerance. Pass an
 * empty/NULL delegate_path to motor_inference_init() to fall back to the
 * CPU path that was this project's original, accuracy-matched default.
 *
 * Two risks specific to serving the NPU delegate from THIS long-lived
 * server (not present in the offline batch tool, which is one-shot):
 *   1. The NPU pays a large ONE-TIME graph-compile cost on its first
 *      Invoke() (~10.4s measured on this board). Because this server's
 *      event loop is single-threaded and synchronous, an uncontrolled
 *      first-frame compile would block ALL vhost servicing for that
 *      whole window, likely dropping the controller/ICU connections.
 *      motor_inference_init() therefore performs a throwaway warm-up
 *      Invoke() itself, still during startup and before any vhost can
 *      accept a connection, so this cost is paid once, isolated, and
 *      before it can affect a real session.
 *   2. motor_inference_reset()'s ResetVariableTensors() call was proven
 *      necessary and sufficient for the CPU path (see its own comment),
 *      but whether it reaches into the NPU delegate's own internal LSTM
 *      state the same way has NOT been independently verified as of this
 *      writing - re-run the multi-session hardware test after switching
 *      to NPU before trusting session isolation on that path.
 *
 * PLACEHOLDER PARAMETERS: THETA_LIMIT, PHI_OFFSET, CONTROL_TS,
 * ANCHOR_DISTANCE, LPF_FC/LPF_STAGES, and the constant insertion-motor
 * position are all still placeholder values inherited unchanged from the
 * offline test harness (see motor_inference.cpp) - none of them have been
 * tuned against the real robot. This module computes real motor positions
 * from real controller input, but with unvalidated tuning constants.
 */

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Loads 'model_path' and builds a TFLite interpreter. If 'delegate_path'
 * is non-NULL and non-empty, applies it as a TFLite external delegate
 * (e.g. "/usr/lib/libvx_delegate.so" for this board's NPU) and performs a
 * throwaway warm-up Invoke() to absorb its one-time compile cost right
 * here - see the NPU risk note above for why that matters on this server
 * specifically. Pass NULL or "" for the CPU-only path (all detected
 * cores, no warm-up needed - CPU has no comparable compile cost).
 *
 * Call exactly once, before the first motor_inference_compute() call and
 * before any controller frame can arrive (i.e. before the vhosts start
 * accepting connections) - there is no lazy/first-call initialization.
 *
 * Returns 0 on success, -1 on failure (model, interpreter, delegate, or
 * warm-up error; a message is printed to stderr).
 */
int motor_inference_init(const char *model_path, const char *delegate_path);

/*
 * Clears the motion-history window and re-seeds the low-pass filter on its
 * next sample, as if no controller input had ever been seen. Call when a
 * new controller session begins (LWS_CALLBACK_ESTABLISHED), so a
 * reconnecting controller doesn't inherit a previous session's trajectory
 * or filter state.
 */
void motor_inference_reset(void);

/*
 * Runs one pipeline step: joystick (joystick_x, joystick_y, each assumed
 * in [-1.0, 1.0]) -> ... -> motor_out[0..2] (tendon motor positions) and
 * motor_out[3] (insertion motor position - currently a fixed placeholder,
 * independent of the model, matching the offline harness).
 *
 * Returns 0 on success, -1 if TFLite inference failed (motor_out is left
 * unmodified in that case).
 */
int motor_inference_compute(double joystick_x, double joystick_y, double motor_out[4]);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_INFERENCE_H */
