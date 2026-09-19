#ifndef FRAME_BRIDGE_H
#define FRAME_BRIDGE_H

#include <stddef.h>

/*
 * Rebuilds 'in' (a raw controller frame, 'in_len' bytes, as received
 * directly from the controller - no intermediate log file, unlike the
 * separate WebSocketApp + ControllerRelay pipeline this project replaces
 * for lower latency) into 'out', with the full send/receive/forward timing
 * chain:
 *
 *   - the controller's own send-time field (T0) is renamed to
 *     "controller_sent_ntp_time" and kept unchanged - looks for "ntp_time"
 *     first, then falls back to "timestamp" (a second real controller
 *     frame variant observed in the field uses that key name instead;
 *     see controller-bridge-motor-inference memory for both captures)
 *   - "controller_to_base_station_received_ntp_time" is added, holding
 *     'received_ntp_time' (T1 - this process's own current time at the
 *     moment it finished receiving the frame)
 *   - a new "ntp_time" field is inserted holding 'forward_ntp_time' (T2 -
 *     this process's own current time at the moment it forwards the frame,
 *     captured just before the actual send)
 *   - "frame_transfer_time" is added: T1 - T0 (Wi-Fi/network delay,
 *     controller -> this process)
 *   - "base_station_processing_time" is added: T2 - T1 (this process's own
 *     handling time - with no file/second-process hop, this should be
 *     microseconds, not milliseconds)
 *
 * Field names match ControllerRelay's frame_stamp.c / WebSocketApp's
 * controller_stamp.c output exactly, so a downstream ICU consumer sees an
 * identical frame shape regardless of which pipeline produced it. This is
 * a plain text substitution, not a JSON parser, matching this project
 * family's minimal-dependency style.
 *
 * If neither "ntp_time" nor "timestamp" is found (unexpected frame shape),
 * 'in' is copied to 'out' unmodified, '*out_frame_transfer_time'/
 * '*out_processing_time' are left untouched, and this fails open rather
 * than dropping the frame - the caller should treat a non-negative return
 * with untouched timing outputs as "no timing data available for this
 * frame" (check the return value, not the timing outputs, to detect this
 * case).
 *
 * 'out_frame_transfer_time' and 'out_processing_time' (both optional - may
 * be NULL) receive the same T1-T0 and T2-T1 values embedded in the
 * stamped frame, as plain doubles in seconds, so a caller doing statistics
 * (e.g. rolling-window min/avg/max) doesn't need to re-parse them back out
 * of the JSON text.
 *
 * Returns the number of bytes written to 'out' (not null-terminated,
 * matching 'in_len' semantics), or -1 if 'out_cap' was too small to hold
 * the result.
 */
int bridge_stamp_frame(const char *in, size_t in_len,
                        double received_ntp_time, double forward_ntp_time,
                        char *out, size_t out_cap,
                        double *out_frame_transfer_time, double *out_processing_time);

/*
 * Extracts "joystick_x" and "joystick_y" numeric values from 'in' ('in_len'
 * bytes) - a plain substring search for the literal key text, same
 * technique as bridge_stamp_frame()'s "ntp_time" lookup, so it works
 * regardless of the keys' nesting depth (e.g. under "joystick_position" in
 * the real controller_to_base_station schema) without needing a JSON
 * parser.
 *
 * Returns 0 and fills '*out_x'/'*out_y' if both keys were found, or -1 if
 * either was missing (outputs left untouched) - the caller should treat -1
 * as "no joystick data in this frame" and skip motor inference for it
 * rather than treating it as a fatal error, matching this project's
 * fail-open style for unexpected frame shapes.
 */
int bridge_extract_joystick(const char *in, size_t in_len,
                             double *out_x, double *out_y);

/*
 * Extracts the controller's own send-time value from 'in' ('in_len' bytes) -
 * looks for "ntp_time" first, then falls back to "timestamp", same key
 * lookup as bridge_stamp_frame()'s T0 field, so a caller that just wants to
 * know/log what T0 was for a frame doesn't need to re-run bridge_stamp_frame()
 * or duplicate its key-fallback logic.
 *
 * Returns 0 and fills '*out_value' if either key was found, or -1 if neither
 * was present (output left untouched).
 */
int bridge_extract_sent_time(const char *in, size_t in_len, double *out_value);

/*
 * Rewrites the VALUES of "joystick_x" and "joystick_y" in 'buf' (currently
 * 'len' bytes, capacity 'cap') to 'joystick_x'/'joystick_y' formatted as
 * "%.8f", leaving the key names and everything else untouched. Same
 * plain-text splice technique as bridge_append_motor_fields() - not a JSON
 * parser.
 *
 * Fails open per-key: a key not present in 'buf' is simply left as-is
 * (not an error) - both fields are optional in that sense. Returns the
 * new total length, or -1 if the result would exceed 'cap' (buf is left
 * unmodified in that failure case).
 */
int bridge_reformat_joystick(char *buf, size_t len, size_t cap,
                              double joystick_x, double joystick_y);

/*
 * Appends four motor-position fields - "motor_1_pos", "motor_2_pos",
 * "motor_3_pos", "motor_4_insertion_pos" (field names match the offline
 * cnr-model-test-harness tool's motor_commands_output.csv columns) - to
 * 'buf' (currently 'len' bytes, a JSON-object-shaped buffer with capacity
 * 'cap'), inserted just before the buffer's last '}' character. Plain text
 * insertion, not a JSON parser, matching this project's style - 'buf' must
 * already be a complete, single top-level JSON object (as produced by
 * bridge_stamp_frame()).
 *
 * Returns the new length, or -1 if 'buf' has no '}' or the result would
 * exceed 'cap' (buf is left unmodified in either failure case).
 */
int bridge_append_motor_fields(char *buf, size_t len, size_t cap,
                                double m1, double m2, double m3, double m4);

/*
 * Appends one field, `,"key":value` (value formatted "%.6f"), to 'buf'
 * (currently 'len' bytes, capacity 'cap'), inserted just before the
 * buffer's last '}' - same technique and same caller contract as
 * bridge_append_motor_fields(), generalized to one arbitrary named field
 * instead of the four fixed motor ones. Returns the new length, or -1 if
 * 'buf' has no '}' or the result would exceed 'cap' (buf is left
 * unmodified in either failure case).
 */
int bridge_append_field(char *buf, size_t len, size_t cap,
                         const char *key, double value);

#endif /* FRAME_BRIDGE_H */
