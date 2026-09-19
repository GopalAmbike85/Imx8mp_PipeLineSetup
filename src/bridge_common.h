#ifndef BRIDGE_COMMON_H
#define BRIDGE_COMMON_H

/* Maximum size of one raw controller frame, matching WebSocketApp's
 * MAX_CONTROLLER_FRAME_LEN convention. */
#define MAX_CONTROLLER_FRAME_LEN 512

/* Maximum size of one bridged (stamped) frame - the raw frame plus four
 * inserted fields (controller_sent_ntp_time, frame_transfer_time,
 * base_station_processing_time, controller_to_base_station_received_ntp_time
 * - "ntp_time" is renamed/inserted in place of the original, not added on
 * top - see frame_bridge.c), roughly 150 bytes of growth. Generously sized
 * for headroom, matching ControllerRelay's precedent. */
#define MAX_BRIDGED_FRAME_LEN 1024

#endif /* BRIDGE_COMMON_H */
