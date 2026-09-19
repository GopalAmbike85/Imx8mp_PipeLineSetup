#ifndef CLOCK_OFFSET_H
#define CLOCK_OFFSET_H

/*
 * Approximate correction for the controller's clock offset relative to
 * this base station's clock.
 *
 * Background: "frame_transfer_time" (frame_bridge.h's bridge_stamp_frame())
 * is computed as received_ntp_time - controller_sent_ntp_time - a straight
 * difference of two independent clocks. Real measurements found this
 * offset is NOT a small, steady drift: across different sessions it has
 * ranged from ~15ms to over 2 SECONDS, with no meaningful linear trend
 * within any single session (see controller-bridge-motor-inference
 * memory, 2026-09-07 investigation). There is no way to fix this properly
 * from this codebase alone - that would need either NTP/PTP sync on the
 * controller (outside this project) or a round-trip-time scheme (which
 * would require making the controller vhost bidirectional - it is
 * receive-only by explicit design - and assumes the controller firmware
 * would cooperate, which is unverified).
 *
 * What this module provides instead: a rolling-minimum offset estimator,
 * the same principle NTP itself uses for offset filtering. Real network
 * delay can only ADD to whatever the clock offset is, never subtract from
 * it, so the smallest frame_transfer_time value seen recently is the best
 * available estimate of "offset + minimum possible network delay" -
 * subtracting it approximates the actual variable network delay.
 *
 * LIMITATIONS - this is an approximation, not ground truth:
 *   - If the underlying offset jumps (observed to happen), there is a
 *     transient window (up to the rolling window length) where the
 *     corrected value is wrong - negative if the offset just decreased,
 *     or still inflated if it just increased - until the window's minimum
 *     catches up with the new baseline.
 *   - It cannot distinguish "the offset changed" from "the network
 *     genuinely got slower right now".
 *   - It is only as good as the assumption that SOME sample in the
 *     rolling window experienced close to zero real network delay.
 */

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Clears the rolling window, as if no samples had ever been seen - call
 * when a new controller session begins (LWS_CALLBACK_ESTABLISHED),
 * matching this project's other per-session resets
 * (motor_inference_reset(), HistoryWindowBuilder::Reset()). A new session
 * may have a different clock offset than the previous one, so stale
 * samples should not carry over.
 */
void clock_offset_reset(void);

/*
 * Feeds one new 'frame_transfer_time' sample (seconds, as returned by
 * bridge_stamp_frame()) into the rolling window and returns
 * 'frame_transfer_time' minus the current rolling minimum, as an
 * approximate correction for the controller's clock offset.
 *
 * 'now' is THIS PROCESS's own current time (e.g. the same
 * received_ntp_time passed to bridge_stamp_frame()) - used only to age
 * samples out of the rolling window, never the controller's clock.
 */
double clock_offset_correct(double frame_transfer_time, double now);

#ifdef __cplusplus
}
#endif

#endif /* CLOCK_OFFSET_H */
