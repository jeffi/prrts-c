#include <stdlib.h>
#include <assert.h>
#include <string.h>
#include <stdio.h>
#include <err.h>
#include <float.h>
#include <math.h>
#include <errno.h>
#include "atomic.h"
#include "kdtree.h"
#include "alloc.h"
#include "crc.h"

#define USE_TL_MEMPOOL
#define MEMPOOL_CHUNKSIZE 65000

#if defined(__STDC__) && defined(__STDC_VERSION__) && __STDC_VERSION__ >= 199901L
#define STACK_ALLOC
#else
#include <alloca.h>
#endif

kd_tree_t *
kd_create_tree(size_t dimensions, const double *min, const double *max, kd_dist_func dist_func,
               const double *init, void *value)
{
        kd_tree_t *tree;
        kd_node_t *root;
        int error;

        assert(min != NULL);
        assert(max != NULL);

        if ((tree = malloc(sizeof(kd_tree_t))) == NULL)
                return NULL;

        if ((root = malloc(sizeof(kd_node_t))) == NULL) {
                free(tree);
                return NULL;
        }

        tree->dimensions = dimensions;
        tree->min = min;
        tree->max = max;
        tree->dist_func = dist_func;
        tree->root = root;

#ifdef USE_TL_MEMPOOL
        if ((error = tl_mempool_init(&tree->mempool, MEMPOOL_CHUNKSIZE)) != 0) {
                free(root);
                free(tree);
                errno = error;
                return NULL;
        }
#endif

        root->config = init;
        root->value = value;
        /* __sync_set(&root->a, NULL); */
        /* __sync_set(&root->b, NULL); */
        root->a = NULL;
        root->b = NULL;

        CRC32_SET(root, config, crc32);

        __sync_synchronize();
        
        return tree;
}

void
kd_insert(kd_tree_t *tree, const double *config, void *value)
{
        size_t dimensions = tree->dimensions;
        double min[dimensions];
        double max[dimensions];
        kd_node_t *n = tree->root;
        size_t depth = 0;
        kd_node_t *new_node;
        kd_node_t *actual;
        kd_node_t *a, *b;

        memcpy(min, tree->min, sizeof(double) * dimensions);
        memcpy(max, tree->max, sizeof(double) * dimensions);

#ifdef USE_TL_MEMPOOL
        if ((new_node = tl_alloc(&tree->mempool, sizeof(kd_node_t))) == NULL)
                err(1, "failed to allocate new node");
#else
        if ((new_node = malloc(sizeof(kd_node_t))) == NULL)
                err(1, "failed to allocate new node");
#endif

        new_node->config = config;
        new_node->value = value;
        /* __sync_set(&new_node->a, NULL); */
        /* __sync_set(&new_node->b, NULL); */
        new_node->a = NULL;
        new_node->b = NULL;

        CRC32_SET(new_node, config, crc32);

        /* make sure that new_node is available */
        __sync_synchronize();
        /* TODO: a full memory barrier is probably overkill here */

        for (;; ++depth) {
                size_t axis = depth % dimensions;
                double mp = (min[axis] + max[axis]) / 2.0;

                double v = config[axis];

                if (v < mp) {
                        /* a-side */
                        a = __sync_get(&n->a);
                        if (a == NULL) {
                                actual = __sync_val_compare_and_swap(&n->a, NULL, new_node);
                                if (actual == NULL) {
                                        break;
                                }
                                /* TODO: t->stat_concurrent_inserts++; */
                                n = actual;
                        } else {
                                n = a;
                        }
                        max[axis] = mp;
                } else {
                        /* b-side */
                        b = __sync_get(&n->b);
                        if (b == NULL) {
                                actual = __sync_val_compare_and_swap(&n->b, NULL, new_node);
                                if (actual == NULL) {
                                        break;
                                }
                                /* TODO: t->stat_concurrent_inserts++; */
                                n = actual;
                        } else {
                                n = b;
                        }
                        min[axis] = mp;
                }
        }

        /* TODO: add to depth stat */
}

typedef struct kd_nearest {
        const kd_tree_t *tree;
        const double *target;

        double *min;
        double *max;

        double dist;
        void *nearest;
} kd_nearest_t;

static void
kd_nearest_impl(kd_nearest_t *t, kd_node_t *n, size_t axis)
{
        double d;
        double mp;
        double tmp;
        kd_node_t *a;
        kd_node_t *b;

        d = (t->tree->dist_func)(n->config, t->target);

        if (d < t->dist) {
                CRC32_CHECK(n, config, crc32);
                t->dist = d;
                t->nearest = n->value;
        }

        mp = (t->min[axis] + t->max[axis]) / 2.0;

        if (t->target[axis] < mp) {
                /* a-side */
                a = __sync_get(&n->a);
                if (a != NULL) {
                        tmp = t->max[axis];
                        t->max[axis] = mp;
                        kd_nearest_impl(t, a, (axis + 1) % t->tree->dimensions);
                        t->max[axis] = tmp;
                }

                b = __sync_get(&n->b);
                if (b != NULL) {
                        tmp = fabs(mp - t->target[axis]);
                        if (tmp < t->dist) {
                                tmp = t->min[axis];
                                t->min[axis] = mp;
                                kd_nearest_impl(t, b, (axis + 1) % t->tree->dimensions);
                                t->min[axis] = tmp;
                        }
                }
        } else {
                /* b-side */
                b = __sync_get(&n->b);
                if (b != NULL) {
                        tmp = t->min[axis];
                        t->min[axis] = mp;
                        kd_nearest_impl(t, b, (axis + 1) % t->tree->dimensions);
                        t->min[axis] = tmp;
                }

                a = __sync_get(&n->a);
                if (a != NULL) {
                        tmp = fabs(mp - t->target[axis]);
                        if (tmp < t->dist) {
                                tmp = t->max[axis];
                                t->max[axis] = mp;
                                kd_nearest_impl(t, a, (axis + 1) % t->tree->dimensions);
                                t->max[axis] = tmp;
                        }
                }
        }
}

void *
kd_nearest(kd_tree_t *tree, const double *target, double *dist)
{
        const size_t dimensions = tree->dimensions;
        kd_nearest_t t;

        /* C99 stack allocation */
        double min[dimensions];
        double max[dimensions];

        memcpy(min, tree->min, sizeof(double) * dimensions);
        memcpy(max, tree->max, sizeof(double) * dimensions);

        t.tree = tree;
        t.target = target;
        t.min = min;
        t.max = max;
        t.dist = DBL_MAX;

        kd_nearest_impl(&t, tree->root, 0);

        if (dist != NULL && t.nearest != NULL)
                *dist = t.dist;

        return t.nearest;
}

typedef struct kd_near {
        const kd_tree_t *tree;
        const double *target;
        double radius;
        kd_near_callback callback;
        void *cb_data;

        double *min;
        double *max;
        int count;
} kd_near_t;

static void
kd_near_impl(kd_near_t *t, kd_node_t *n, size_t axis)
{
        double d, mp, dm, tmp;
        kd_node_t *a;
        kd_node_t *b;

        d = (t->tree->dist_func)(n->config, t->target);

        if (d < t->radius) {
                CRC32_CHECK(n, config, crc32);
                (t->callback)(t->cb_data, t->count++, n->value, d);
        }
        
        mp = (t->min[axis] + t->max[axis]) / 2.0;
        dm = fabs(mp - t->target[axis]);

        a = __sync_get(&n->a);
        if (a != NULL && (t->target[axis] < mp || dm < t->radius)) {
                /* in or near a-side */
                tmp = t->max[axis];
                t->max[axis] = mp;
                kd_near_impl(t, a, (axis + 1) % t->tree->dimensions);
                t->max[axis] = tmp;
        }

        b = __sync_get(&n->b);
        if (b != NULL && (mp <= t->target[axis] || dm < t->radius)) {
                /* in or near b-side */
                tmp = t->min[axis];
                t->min[axis] = mp;
                kd_near_impl(t, b, (axis + 1) % t->tree->dimensions);
                t->min[axis] = tmp;
        }
}

int
kd_near(kd_tree_t *tree, const double *target, double radius, kd_near_callback callback, void *cb_data)
{
        const size_t dimensions = tree->dimensions;
        kd_near_t t;
        double min[dimensions];
        double max[dimensions];

        memcpy(min, tree->min, sizeof(double) * dimensions);
        memcpy(max, tree->max, sizeof(double) * dimensions);

        t.tree = tree;
        t.target = target;
        t.radius = radius;
        t.min = min;
        t.max = max;
        t.callback = callback;
        t.cb_data = cb_data;
        t.count = 0;

        kd_near_impl(&t, tree->root, 0);

        return t.count;
}

