#ifndef BRIDGE_SERVER_H
#define BRIDGE_SERVER_H

#include <libwebsockets.h>
#include <stddef.h>

/* The callback function for the "telemetry" server protocol (ICU-facing) -
 * register this in the shared protocols table passed to
 * lws_create_context(). Kept as "telemetry" so an ICU already built
 * against WebSocketApp/ControllerRelay doesn't need to change subprotocol
 * names. */
lws_callback_function *bridge_server_callback(void);

/* Prepares the ICU-facing broadcast side. Call once before any ICU client
 * connects and before the first bridge_server_forward() call. */
void bridge_server_start(struct lws_context *context);

/*
 * Stamps 'raw_frame' ('raw_len' bytes - a single frame received directly
 * from the controller, see controller_receiver.c) with this bridge's own
 * receipt and forward times and immediately forwards it to every connected
 * ICU client.
 *
 * Called once per received controller frame, directly from the receive
 * callback - no file, no polling, no second process. Nothing is ever sent
 * to the ICU unless a real controller frame was actually received. NPU
 * inference (motor_inference_compute()) runs BEFORE the "forward" time is
 * captured, so the resulting "base_station_processing_time" field
 * reflects the true receive-to-send cost, inference included - not just
 * the frame-text splicing.
 */
void bridge_server_forward(const char *raw_frame, size_t raw_len, double received_ntp_time);

#endif /* BRIDGE_SERVER_H */
