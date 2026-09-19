#ifndef FRAME_STATS_H
#define FRAME_STATS_H

/*
 * Accumulates 'frame_transfer_time' and 'processing_time' (both seconds,
 * as computed by frame_bridge.c's bridge_stamp_frame()) into a rolling
 * window. Every 100 frames, prints min/avg/max (in milliseconds) for all
 * three timings - frame_transfer_time, base_station_processing_time, and
 * their sum (total base-station-side latency, controller send to ICU
 * forward) - to stdout, then starts a fresh window.
 *
 * Call once per successfully forwarded frame, from bridge_server_forward().
 */
void frame_stats_record(double frame_transfer_time, double processing_time);

#endif /* FRAME_STATS_H */
