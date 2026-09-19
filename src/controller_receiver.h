#ifndef CONTROLLER_RECEIVER_H
#define CONTROLLER_RECEIVER_H

#include <libwebsockets.h>
#include <stddef.h>

/* The callback function for the "controller" protocol (controller-facing,
 * receive-only) - register this in the shared protocols table. On each
 * fully-reassembled frame, stamps this process's own receipt time and
 * hands it directly to bridge_server_forward() - no file, no second
 * process. */
lws_callback_function *controller_receiver_callback(void);

/* Per-session state size for the controller protocol's
 * per_session_data_size - pass this to the protocol table entry. */
size_t controller_receiver_session_size(void);

#endif /* CONTROLLER_RECEIVER_H */
