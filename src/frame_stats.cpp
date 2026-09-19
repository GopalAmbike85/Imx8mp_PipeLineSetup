#include "frame_stats.h"

#include <float.h>
#include <stdio.h>

#define WINDOW_SIZE 100

typedef struct {
    double sum, min, max;
} running_stat_t;

static int s_count;
static running_stat_t s_transfer;
static running_stat_t s_processing;
static running_stat_t s_total;

static void stat_reset(running_stat_t *s)
{
    s->sum = 0.0;
    s->min = DBL_MAX;
    s->max = -DBL_MAX;
}

static void stat_add(running_stat_t *s, double value)
{
    s->sum += value;
    if (value < s->min) {
        s->min = value;
    }
    if (value > s->max) {
        s->max = value;
    }
}

static void stat_print(const char *label, const running_stat_t *s, int count)
{
    printf("  %-30s min=%8.3fms  avg=%8.3fms  max=%8.3fms\n",
           label, s->min * 1000.0, (s->sum / count) * 1000.0, s->max * 1000.0);
}

void frame_stats_record(double frame_transfer_time, double processing_time)
{
    if (s_count == 0) {
        stat_reset(&s_transfer);
        stat_reset(&s_processing);
        stat_reset(&s_total);
    }

    double total = frame_transfer_time + processing_time;

    stat_add(&s_transfer, frame_transfer_time);
    stat_add(&s_processing, processing_time);
    stat_add(&s_total, total);
    s_count++;

    if (s_count >= WINDOW_SIZE) {
        printf("=== timing stats over last %d frames ===\n", s_count);
        stat_print("frame_transfer_time", &s_transfer, s_count);
        stat_print("base_station_processing_time", &s_processing, s_count);
        stat_print("total (transfer + processing)", &s_total, s_count);
        fflush(stdout);
        s_count = 0; /* next call starts a fresh window */
    }
}
