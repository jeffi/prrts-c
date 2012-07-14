#ifndef PRRTS_H
#define PRRTS_H

#include <stdbool.h>
#include "kdtree.h"

#define REGION_SPLIT_AXIS 0
#define INITIAL_NEAR_LIST_CAPACITY 1024

typedef bool (*prrts_in_goal_func)(void *system, const double *config);
typedef bool (*prrts_clear_func)(void *system, const double *config);
typedef bool (*prrts_link_func)(void *system, const double *a, const double *b);
typedef void *(*prrts_system_data_alloc_func)(int thread_no, const double *sample_min, const double *sample_max);
typedef void (*prrts_system_data_free_func)(void *system);

typedef struct prrts_system {
        size_t dimensions;

        const double *init;
        const double *min;
        const double *max;
        const double *target;

        prrts_system_data_alloc_func system_data_alloc_func;
        prrts_system_data_free_func system_data_free_func;

        kd_dist_func dist_func;
        prrts_in_goal_func in_goal_func;
        prrts_clear_func clear_func;
        prrts_link_func link_func;
} prrts_system_t;

typedef struct prrts_options {
        double gamma;
        bool regional_sampling;
        int samples_per_step;
} prrts_options_t;

typedef struct prrts_solution {
        double path_cost;
        size_t path_length;
        const double *configs[0];
} prrts_solution_t;

prrts_solution_t* prrts_run_for_duration(prrts_system_t *system, prrts_options_t *options, int thread_count, long duration);
prrts_solution_t* prrts_run_for_samples(prrts_system_t *system, prrts_options_t *options, int thread_count, size_t sample_count);
prrts_solution_t* prrts_run_indefinitely(prrts_system_t *system, prrts_options_t *options, int thread_count);

#endif /* PRRTS_H */
