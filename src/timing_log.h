#ifndef TIMING_LOG_H
#define TIMING_LOG_H

/*
 * Optional per-frame CSV logging of the timing fields bridge_server_forward()
 * already computes (frame_transfer_time, frame_transfer_time_corrected,
 * base_station_processing_time) - for offline latency-distribution analysis
 * over a real capture window, instead of the short live snapshots
 * frame_stats.h summarizes to stdout every 100 frames. This runs alongside
 * frame_stats.h, not in place of it - same input values, different purpose
 * (a full CSV trail vs. a rolling console summary).
 *
 * Disabled by default. A 50Hz control loop can produce tens of thousands of
 * rows per minute, so this is meant to be turned on for a bounded capture
 * window (e.g. while gathering data for a customer-facing timing proposal),
 * not left on as an always-on production feature.
 */

/*
 * path: CSV file to append to. NULL or "" disables logging entirely - every
 * other function in this module then becomes a no-op. If the file doesn't
 * already exist (or is empty), a header row is written first; if it already
 * has content, new rows are appended after it, so restarting the bridge
 * mid-capture doesn't lose or duplicate the header.
 *
 * Returns 0 on success (including the disabled case), -1 if a path was given
 * but the file could not be opened. Intentionally NOT treated as fatal by
 * the caller - losing this diagnostic log must never take down the real
 * controller-to-ICU pipeline.
 */
int timing_log_init(const char *path);

/*
 * Appends one CSV row: wall_epoch_seconds (this board's own CLOCK_REALTIME,
 * %.6f - not the controller's clock, so it carries none of
 * frame_transfer_time's clock-drift problem), then frame_transfer_time,
 * frame_transfer_time_corrected, and base_station_processing_time, all
 * converted to milliseconds (%.4f) for direct readability during analysis.
 * No-op if logging was never enabled or failed to open.
 *
 * Flushed after every row: a syscall per frame is negligible next to the
 * ~2ms NPU inference cost this log exists to characterize, and a bounded
 * diagnostic capture should not lose its last batch of rows if the process
 * is killed mid-run.
 */
void timing_log_record(double wall_epoch_seconds,
                        double frame_transfer_time,
                        double frame_transfer_time_corrected,
                        double base_station_processing_time);

/* Closes the file cleanly if logging was enabled. Call once at shutdown. */
void timing_log_close(void);

#endif /* TIMING_LOG_H */
