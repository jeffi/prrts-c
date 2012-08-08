#include <assert.h>
#include <stdio.h>
#include <limits.h>
#include "stats.h"

/*
 * could use just true/false or 0/1 but wantted something to help
 * detect uninitialized values
 */
#define STATE_STOPPED 5
#define STATE_RUNNING 42

void
time_stats_clear(time_stats_t *stats)
{
        stats->min = HRTIMER_MAX;
        stats->max = HRTIMER_MIN;
        stats->sum = 0;
        stats->count = 0;
        stats->state = STATE_STOPPED;
}

void
time_stats_start(time_stats_t *stats)
{
        assert(stats->state == STATE_STOPPED);
        stats->start_time = hrtimer_get();
        stats->state = STATE_RUNNING;
}

void
time_stats_stop(time_stats_t *stats)
{
        hrtimer_t duration = hrtimer_get() - stats->start_time;

        assert(stats->state == STATE_RUNNING);

        stats->state = STATE_STOPPED;

        if (duration < stats->min)
                stats->min = duration;

        if (duration > stats->max)
                stats->max = duration;

        stats->sum += duration;
        stats->count++;
}

void
time_stats_accum(time_stats_t *accum, const time_stats_t *stat)
{
        assert(accum->state == STATE_STOPPED);
        assert(stat->state == STATE_STOPPED);

        if (stat->min < accum->min)
                accum->min = stat->min;

        if (stat->max > accum->max)
                accum->max = stat->max;

        accum->sum += stat->sum;
        accum->count += stat->count;
}

void
time_stats_print(const char *name, const time_stats_t *stats)
{
        printf("%s: %d calls, %fs total, %f us/call avg (min=%f, max=%f)",
               name, stats->count,
               stats->sum / (double)HRTICK,
               stats->sum / (double)stats->count * (1e+6 / (double)HRTICK),
               stats->min * (1e+6 / (double)HRTICK),
               stats->max * (1e+6 / (double)HRTICK));
}

void
count_stats_clear(count_stats_t *stats)
{
        stats->min = INT_MAX;
        stats->max = INT_MIN;
        stats->sum = 0;
        stats->count = 0;
}

void
count_stats_add(count_stats_t *stats, int count)
{
        if (count < stats->min)
                stats->min = count;
        if (count > stats->max)
                stats->max = count;
        stats->sum += count;
        stats->count++;
}

void
count_stats_accum(count_stats_t *accum, const count_stats_t *stat)
{
        if (stat->min < accum->min)
                accum->min = stat->min;
        if (stat->max > accum->max)
                accum->max = stat->max;
        accum->sum += stat->sum;
        accum->count += stat->count;
}

void
count_stats_print(const char *name, const count_stats_t *stats)
{
        printf("%s: %d calls, avg=%f, min=%d, max=%d",
               name,
               stats->count,
               stats->sum / (double)stats->count,
               stats->min,
               stats->max);
}
