#ifndef STATS_H
#define STATS_H

#include "hrtimer.h"

typedef struct time_stats {
        hrtimer_t min;
        hrtimer_t max;
        hrtimer_t sum;

        hrtimer_t start_time;
        int count;
        int state;
} time_stats_t;

void time_stats_clear(time_stats_t *stats);
void time_stats_start(time_stats_t *stats);
void time_stats_stop(time_stats_t *stats);

void time_stats_accum(time_stats_t *accum, const time_stats_t *stat);
void time_stats_print(const char *name, const time_stats_t *stats);

#define TIME(stat, code) do {                   \
                time_stats_start(stat);         \
                code;                           \
                time_stats_stop(stat);          \
        } while(0)

typedef struct count_stats {
        int min;
        int max;
        long long sum;

        int count;
} count_stats_t;

void count_stats_clear(count_stats_t *stats);
void count_stats_add(count_stats_t *stats, int count);
void count_stats_accum(count_stats_t *accum, const count_stats_t *stat);
void count_stats_print(const char *name, const count_stats_t *stats);

#endif /* STATS_H */
