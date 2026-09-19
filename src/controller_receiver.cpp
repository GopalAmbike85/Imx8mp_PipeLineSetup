#include "controller_receiver.h"
#include "bridge_common.h"
#include "bridge_server.h"
#include "clock_offset.h"
#include "frame_bridge.h"
#include "gpio_toggle.h"
#include "motor_inference.h"

#include <string.h>
#include <time.h>

/* Per-connection state: accumulates a frame's bytes across possibly-
 * multiple LWS_CALLBACK_RECEIVE calls (a WebSocket text message is not
 * guaranteed to arrive in a single callback, even when small - see
 * lws_is_first_fragment()/lws_is_final_fragment() below). */
typedef struct {
    char buf[MAX_CONTROLLER_FRAME_LEN];
    size_t len;
} controller_session_t;

static double current_epoch_seconds(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    return (double)ts.tv_sec + (double)ts.tv_nsec / 1e9;
}

static int callback_controller(struct lws *wsi, enum lws_callback_reasons reason,
                                void *user, void *in, size_t len)
{
    (void)wsi;
    controller_session_t *session = (controller_session_t *)user;

    switch (reason) {
    case LWS_CALLBACK_ESTABLISHED:
        lwsl_notice("controller connected\n");
        session->len = 0;
        /* A new session starts with a clean motion history and filter -
         * without this, a reconnecting controller would inherit the
         * previous session's trajectory/EMA state (this project assumes a
         * single controller session at a time, matching s_payload etc.
         * being process-global rather than per-session). */
        motor_inference_reset();
        /* A new session may have a different clock offset than the last
         * one - see clock_offset.h - so its rolling-minimum window starts
         * fresh too, rather than inheriting a possibly-stale estimate. */
        clock_offset_reset();
        break;

    case LWS_CALLBACK_RECEIVE:
        if (lws_is_first_fragment(wsi)) {
            session->len = 0;
        }

        if (session->len + len > MAX_CONTROLLER_FRAME_LEN) {
            lwsl_err("controller frame exceeds %d-byte buffer, dropping\n",
                     MAX_CONTROLLER_FRAME_LEN);
            session->len = 0;
            break;
        }

        memcpy(session->buf + session->len, in, len);
        session->len += len;

        if (lws_is_final_fragment(wsi)) {
            /* Hardware-visible marker of the receive-to-forward window -
             * ON now the frame is fully received, OFF right after
             * bridge_server_forward() hands it off for sending on the
             * ICU (8080) side. See gpio_toggle.h for the pin/rationale;
             * clock-independent cross-check against the software timing
             * fields in timing_log.h. */
            gpio_toggle_set(1);

            /* Stamp receipt time now, at the earliest possible instant
             * after the frame is complete, then hand off directly to the
             * ICU-facing side - no file, no polling interval, no second
             * process to hop through. */
            double received_ntp_time = current_epoch_seconds();

            double sent_ntp_time;
            if (bridge_extract_sent_time(session->buf, session->len, &sent_ntp_time) == 0) {
                lwsl_notice("controller frame received, sent_ntp_time=%.6f\n", sent_ntp_time);
            } else {
                lwsl_notice("controller frame received, no ntp_time/timestamp field\n");
            }
            /* session->buf is NOT null-terminated (only the first
             * session->len bytes are valid data) - "%.*s" bounds the print
             * to exactly that many bytes instead of relying on a terminator
             * that may not be there. */
            lwsl_notice("controller frame content (%zu bytes): %.*s\n",
                        session->len, (int)session->len, session->buf);

            bridge_server_forward(session->buf, session->len, received_ntp_time);
            gpio_toggle_set(0);
            session->len = 0;
        }
        break;

    case LWS_CALLBACK_CLOSED:
        lwsl_notice("controller disconnected\n");
        break;

    default:
        break;
    }

    return 0;
}

lws_callback_function *controller_receiver_callback(void)
{
    return callback_controller;
}

size_t controller_receiver_session_size(void)
{
    return sizeof(controller_session_t);
}
