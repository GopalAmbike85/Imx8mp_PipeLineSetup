#include "timing_log.h"

#include <cstdio>

static FILE *s_file = nullptr;

int timing_log_init(const char *path)
{
    if (path == nullptr || path[0] == '\0') {
        return 0; /* disabled */
    }

    /* "a" (append): if the file already has rows from an earlier run in the
     * same capture window, new rows continue after them instead of
     * clobbering the file. fseek+ftell (rather than trusting fopen("a")'s
     * initial position, which the C standard leaves unspecified until the
     * first write) is what actually decides whether a header is needed. */
    s_file = fopen(path, "a");
    if (!s_file) {
        fprintf(stderr, "timing_log: failed to open '%s' for append - "
                        "continuing without timing log\n", path);
        return -1;
    }

    fseek(s_file, 0, SEEK_END);
    if (ftell(s_file) == 0) {
        fprintf(s_file,
                "wall_epoch_seconds,frame_transfer_time_ms,"
                "frame_transfer_time_corrected_ms,base_station_processing_time_ms\n");
        fflush(s_file);
    }

    return 0;
}

void timing_log_record(double wall_epoch_seconds,
                        double frame_transfer_time,
                        double frame_transfer_time_corrected,
                        double base_station_processing_time)
{
    if (!s_file) {
        return;
    }

    fprintf(s_file, "%.6f,%.4f,%.4f,%.4f\n",
            wall_epoch_seconds,
            frame_transfer_time * 1000.0,
            frame_transfer_time_corrected * 1000.0,
            base_station_processing_time * 1000.0);
    fflush(s_file);
}

void timing_log_close(void)
{
    if (s_file) {
        fclose(s_file);
        s_file = nullptr;
    }
}
