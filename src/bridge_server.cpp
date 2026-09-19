#include "bridge_server.h"
#include "bridge_common.h"
#include "clock_offset.h"
#include "frame_bridge.h"
#include "frame_stats.h"
#include "motor_inference.h"
#include "timing_log.h"

#include <netinet/in.h>
#include <netinet/tcp.h>
#include <string.h>
#include <time.h>

/* Shared payload buffer for the most recently forwarded frame. libwebsockets
 * requires LWS_PRE bytes of headroom before the payload so it can prepend
 * the WebSocket framing in place. */
static unsigned char s_payload[LWS_PRE + MAX_BRIDGED_FRAME_LEN];
static size_t s_payload_len;

static struct lws_context *s_context;

/*
 * This module doesn't own the shared protocols table (main.c does), so it
 * can't take &protocol directly. Instead it captures a pointer to its own
 * protocol entry via lws_get_protocol() the first time an ICU client
 * connects - the standard way to recover that pointer from inside a
 * callback that doesn't have direct access to the static table.
 */
static const struct lws_protocols *s_protocol;

static double current_epoch_seconds(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    return (double)ts.tv_sec + (double)ts.tv_nsec / 1e9;
}

static int callback_icu(struct lws *wsi, enum lws_callback_reasons reason,
                         void *user, void *in, size_t len)
{
    (void)user;
    (void)in;
    (void)len;

    switch (reason) {
    case LWS_CALLBACK_ESTABLISHED: {
        lwsl_notice("ICU connected\n");
        if (!s_protocol) {
            s_protocol = lws_get_protocol(wsi);
        }
        /* Disable Nagle's algorithm - this bridge's traffic is one short
         * JSON frame at a time, not a continuous stream, which is exactly
         * what Nagle/delayed-ACK interaction tends to stall for tens of
         * milliseconds. libwebsockets exposes no context/vhost option for
         * this, so it's set directly on the accepted socket. */
        int nodelay = 1;
        setsockopt(lws_get_socket_fd(wsi), IPPROTO_TCP, TCP_NODELAY,
                   &nodelay, sizeof(nodelay));
        break;
    }

    case LWS_CALLBACK_SERVER_WRITEABLE:
        if (s_payload_len > 0) {
            int written = lws_write(wsi, s_payload + LWS_PRE, s_payload_len, LWS_WRITE_TEXT);
            if (written < (int)s_payload_len) {
                lwsl_err("lws_write failed/truncated (%d of %zu)\n", written, s_payload_len);
                return -1;
            }
        }
        break;

    case LWS_CALLBACK_CLOSED:
        lwsl_notice("ICU disconnected\n");
        break;

    default:
        break;
    }

    return 0;
}

lws_callback_function *bridge_server_callback(void)
{
    return callback_icu;
}

void bridge_server_start(struct lws_context *context)
{
    s_context = context;
    s_payload_len = 0;
    s_protocol = NULL;
}

void bridge_server_forward(const char *raw_frame, size_t raw_len, double received_ntp_time)
{
    /* Extraction + NPU inference run FIRST, directly on 'raw_frame', before
     * any timing capture or frame-text work - this ordering is what makes
     * the forward_ntp_time capture below actually mean what its comment
     * says. (Fixed 2026-09-07: this used to run AFTER forward_ntp_time was
     * captured, so base_station_processing_time silently excluded NPU
     * inference entirely and always read ~0 - see
     * controller-bridge-motor-inference memory for how that was found.) */
    bool have_joystick = false;
    bool have_motor = false;
    double joystick_x = 0.0, joystick_y = 0.0;
    double motor[4] = {0.0, 0.0, 0.0, 0.0};

    if (bridge_extract_joystick(raw_frame, raw_len, &joystick_x, &joystick_y) == 0) {
        have_joystick = true;
        if (motor_inference_compute(joystick_x, joystick_y, motor) == 0) {
            have_motor = true;
        } else {
            lwsl_err("motor inference failed, forwarding without motor fields\n");
        }
    }

    /* Captured as close to the actual send as this synchronous call
     * allows - inference is already done, so the only work remaining
     * between this and the real lws_write() is frame-text splicing
     * (stamp/reformat/append, all cheap substring operations) and the
     * writable-callback dispatch. base_station_processing_time (derived
     * from this) now reflects the true receive-to-send cost, NPU
     * inference included. */
    double forward_ntp_time = current_epoch_seconds();

    /* bridge_stamp_frame() leaves these untouched if "ntp_time"/"timestamp"
     * isn't found in the frame (fail-open path) - initialized here so that
     * rare case still records a defined (if meaningless) value instead of
     * reading uninitialized memory. */
    double frame_transfer_time = 0.0, processing_time = 0.0;
    int n = bridge_stamp_frame(raw_frame, raw_len, received_ntp_time, forward_ntp_time,
                                (char *)(s_payload + LWS_PRE), MAX_BRIDGED_FRAME_LEN,
                                &frame_transfer_time, &processing_time);
    if (n < 0) {
        lwsl_err("stamped frame did not fit in %d-byte buffer, dropping\n", MAX_BRIDGED_FRAME_LEN);
        return;
    }

    frame_stats_record(frame_transfer_time, processing_time);

    /* Appends "frame_transfer_time_corrected" - an APPROXIMATE estimate,
     * not ground truth, see clock_offset.h for the full rationale and
     * limitations. Added alongside the existing "frame_transfer_time"
     * (left untouched, same as before) rather than replacing it, so any
     * existing consumer relying on that field's original defined meaning
     * is unaffected. Same fail-open posture as the rest of this function:
     * if it doesn't fit, the frame still forwards without this one field. */
    double corrected_transfer_time =
        clock_offset_correct(frame_transfer_time, received_ntp_time);
    int with_correction = bridge_append_field(
        (char *)(s_payload + LWS_PRE), (size_t)n, MAX_BRIDGED_FRAME_LEN,
        "frame_transfer_time_corrected", corrected_transfer_time);
    if (with_correction >= 0) {
        n = with_correction;
    } else {
        lwsl_err("corrected-transfer-time field did not fit in %d-byte buffer, "
                 "forwarding without it\n", MAX_BRIDGED_FRAME_LEN);
    }

    /* CSV row for offline latency analysis (see timing_log.h) - a no-op
     * unless timing_log_init() was given a path on the command line.
     * received_ntp_time (this board's own clock, not the controller's) is
     * used as the row's wall-clock reference so rows stay chronologically
     * meaningful even though frame_transfer_time's own clock-drift problem
     * (see clock_offset.h) is untouched by this. */
    timing_log_record(received_ntp_time, frame_transfer_time,
                       corrected_transfer_time, processing_time);

    /* Splice in the (already-computed) joystick reformat and motor fields.
     * Fails open, same as bridge_stamp_frame() above: a frame with no
     * joystick fields, or a failed inference, still forwards the stamped
     * (timing-only) frame rather than being dropped - motor_1_pos etc. are
     * simply absent from that one forwarded frame. */
    if (have_joystick) {
        /* Rewrite joystick_x/joystick_y to a fixed "%.8f" precision in the
         * outgoing frame - the original text (whatever precision/format
         * the controller sent) is otherwise passed through unchanged. */
        int reformatted = bridge_reformat_joystick(
            (char *)(s_payload + LWS_PRE), (size_t)n, MAX_BRIDGED_FRAME_LEN,
            joystick_x, joystick_y);
        if (reformatted >= 0) {
            n = reformatted;
        } else {
            lwsl_err("joystick reformat did not fit in %d-byte buffer, "
                     "forwarding without reformatting\n", MAX_BRIDGED_FRAME_LEN);
        }

        if (have_motor) {
            int with_motor = bridge_append_motor_fields(
                (char *)(s_payload + LWS_PRE), (size_t)n, MAX_BRIDGED_FRAME_LEN,
                motor[0], motor[1], motor[2], motor[3]);
            if (with_motor >= 0) {
                n = with_motor;
            } else {
                lwsl_err("stamped+motor frame did not fit in %d-byte buffer, "
                         "forwarding without motor fields\n", MAX_BRIDGED_FRAME_LEN);
            }
        }
    }

    s_payload_len = (size_t)n;
    if (s_protocol) {
        lws_callback_on_writable_all_protocol(s_context, s_protocol);
    }
}
