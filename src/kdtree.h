
#ifndef KD_TREE_H
#define KD_TREE_H

/* for size_t */
#include <stddef.h>
#include <stdint.h>
#include "alloc.h"

#ifdef CHECK_CRCS
#include "crc.h"
#endif

typedef double (*kd_dist_func)(const double *a, const double *b);
typedef void (*kd_near_callback)(void *data, int no, void *value, double dist);

typedef struct kd_tree {
        size_t dimensions;
        const double *min;
        const double *max;
        struct kd_node *root;
        kd_dist_func dist_func;
        tl_mempool_t mempool;
} kd_tree_t;

kd_tree_t *kd_create_tree(size_t dimensions, const double *min, const double *max,
                          kd_dist_func dist_func,
                          const double *root_config, void *root_value);
void kd_insert(kd_tree_t *t, const double *config, void *value);
void *kd_nearest(kd_tree_t *t, const double *target, double *dist);
int kd_near(kd_tree_t *t, const double *target, double radius, kd_near_callback callback, void *cb_data);

#endif
