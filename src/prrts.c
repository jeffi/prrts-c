#include <config.h>

#define SET_CPU_AFFINITY

#ifdef _OPENMP
# include <omp.h>
#else
# ifdef __linux__ 
#  define _GNU_SOURCE
#  include <sched.h>
#  include <unistd.h>
#  include <sys/syscall.h>
# elif defined(BSD) || defined(__APPLE__)
#  include <sys/param.h>
#  include <sys/sysctl.h>
# endif
# include <pthread.h>
#endif /* _OPENMP */

#include <assert.h>
#include <err.h>
#include <limits.h>
#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stddef.h>
#include "kdtree.h"
#include "alloc.h"
#include "mt19937a.h"
#include "hrtimer.h"
#include "stats.h"
#include "atomic.h"
#include "prrts.h"
#include "crc.h"

#define LOCAL_HEAP_SIZE 256000

/*
 * A single node in the RRT* tree.  The in_goal and config fields are
 * constant.  The link field changes as the node is rewired to better
 * paths.
 */
typedef struct prrts_node {
        union {
                struct prrts_link * volatile link;
                char _pad[16]; /* intel recommends 16-byte field alignment */
        };

#ifdef CHECK_CRCS
        crc32_t crc32;
#endif

        bool in_goal;
        const double *config;
} prrts_node_t;

/*
 * A link between nodes in the RRT* tree.  Following the parent
 * pointer repeatedly will walk a path back to the initial
 * configuration--reversing the order will walk a path from the
 * initial configuration to the target configuration.
 */
typedef struct prrts_link {
        struct prrts_node *node;
        struct prrts_link *parent;
        double link_cost;
        double path_cost;

#ifdef CHECK_CRCS
        crc32_t crc32;
#endif

#ifdef REF_COUNT
        volatile ref_count;
#endif

        struct prrts_link * volatile first_child; /* linked list of children */
        struct prrts_link * volatile next_sibling; /* next element in the list */
} prrts_link_t;

/*
 * Data maintained per prrt run.  It contains the shared configuration
 * and communication fields for the workers.
 */
typedef struct prrts_runtime {
        /*
         * unchanging fields are first and padded to be in a separate
         * cache-line than the changing fields
         */
        union {
                struct {
                        kd_tree_t *kd_tree;
                        struct prrts_node *root;

                        size_t thread_count;
                        size_t sample_limit;
                        hrtimer_t time_limit;
                        hrtimer_t start_time;
                };
                char _pad1[CACHE_LINE_SIZE];
        };

        /*
         * Runtime status fields, these are constantly modified and
         * checked as more samples are added to the graph.
         */
        union {
                struct {
                        volatile size_t step_no;
                        volatile bool done;
                };
                char _pad2[CACHE_LINE_SIZE];
        };

        /*
         * The best path field, also modified often, but in a separate
         * process.  It gets its own cache line too.
         */
        struct prrts_link * volatile best_path;
} prrts_runtime_t;

/*
 * The near-list data structure built up during a call to kd_near
 */
typedef struct near_list {
        struct prrts_link * link; /* link returned by kd_near */
        double link_cost; /* distance from link's node to near node */
        double path_cost; /* path distance through link to near node */
} near_list_t;


/*
 * The "local_heap" structure is a worker/thread-local heap used for
 * allocating RRT* nodes, links and configurations.  We do large
 * allocations of the local_heap and pointer arithmetic to allocate
 * elements.  
 *
 * The "local_heap" serves the following purposes:
 *
 * - Allocations are fast (just some pointer arithmetic usually)
 *
 * - Calls to malloc are infrequent.  If malloc's implementation
 *   serializes access, we remove the blocking overhead of it.
 *
 * - Clean-up after computation is simple--just free the local_heap
 *   nodes.
 *
 * - Allows us to minimizes overhead of getting cache-aligned memory
 *   blocks.
 */
typedef struct local_heap {
        struct local_heap *prev;
        int offset;
        char data[LOCAL_HEAP_SIZE];
} local_heap_t;

/*
 * A worker struct is a per-worker-thread allocated structure.  It is
 * essentially a thread-local storage structure that contains
 * everything a worker-thread needs to do its share of the
 * computation.  An alternate approach would be to use
 * pthread_setspecific, etc..., but this approach allows use to adapt
 * to other threading implementations without reliance on the pthread
 * API.
 */
typedef struct worker {
#ifdef _OPENMP
        int thread_id;
#else
        pthread_t thread_id;
#endif

        struct prrts_runtime *runtime;
        struct prrts_system *system;
        struct prrts_options *options;

        int thread_no;
        void *system_data;
        mt19937a_t rnd;

        double *new_config;

        near_list_t *near_list;
        size_t near_list_size;
        size_t near_list_capacity;

        const double *sample_min;
        const double *sample_max;

        local_heap_t *config_heap;
        local_heap_t *node_heap;
        local_heap_t *link_heap;

        long clear_count;
        long sample_count;
        double link_dist_sum;
        time_stats_t near_time;
        time_stats_t link_time;
        time_stats_t sort_time;
        time_stats_t insert_time;
        time_stats_t update_time;
        count_stats_t near_list_size_stats;
} worker_t;

#ifdef CHECK_CRCS

static inline crc32_t
compute_crc32_node(prrts_node_t *node, size_t dimensions) {
        crc32_t accum = CRC32_INIT;
        crc32_accum(&accum, &node->in_goal, sizeof(bool));
        crc32_accum(&accum, &node->config, sizeof(double *));
        crc32_accum(&accum, &node->config[0], sizeof(double) * dimensions);
        return crc32_finish(accum);
}

static inline crc32_t
compute_crc32_link(prrts_link_t *link) {
        crc32_t accum = CRC32_INIT;
        crc32_accum(&accum, &link->node, sizeof(struct prrts_node *));
        crc32_accum(&accum, &link->parent, sizeof(struct prrts_link *));
        crc32_accum(&accum, &link->link_cost, sizeof(double));
        crc32_accum(&accum, &link->path_cost, sizeof(double));
        return crc32_finish(accum);
}

static inline void
set_crc32_node(prrts_node_t *node, size_t dimensions) {
        node->crc32 = compute_crc32_node(node, dimensions);
}

#define check_crc32_node(node, dimensions) do {                         \
                crc32_t __crc = compute_crc32_node(node, dimensions);   \
                assert(__crc == node->crc32);                           \
        } while (0)

static inline void
set_crc32_link(prrts_link_t *link) {
        link->crc32 = compute_crc32_link(link);
}

#define check_crc32_link(link) do {                             \
                crc32_t __crc = compute_crc32_link(link);       \
                assert(__crc == link->crc32);                   \
        } while (0)

#else

#define set_crc32_node(node, dimensions) do {} while(0)
#define check_crc32_node(node, dimensions) do {} while(0)
#define set_crc32_link(link) do {} while(0)
#define check_crc32_link(link) do {} while(0)

#endif

static local_heap_t *
local_heap_create(size_t align)
{
        local_heap_t *heap;

        if ((heap = malloc(sizeof(local_heap_t))) == NULL)
                err(1, "failed to allocate local_heap");

        heap->prev = NULL;
        heap->offset = aligned_offset(heap->data, align);
        assert(heap->offset < align);

        return heap;
}

static void
local_heap_free(local_heap_t *heap)
{
        local_heap_t *prev;

        while (heap != NULL) {
                prev = heap->prev;
                free(heap);
                heap = prev;
        }
}

static void *
local_alloc(local_heap_t **local_heap, size_t size, size_t align)
{
        size_t aligned_size = ALIGN_UP(size, align);
        local_heap_t *fh = *local_heap;
        int old_offset = fh->offset;
        int new_offset = old_offset + aligned_size;
        void *ptr;

        assert(size < LOCAL_HEAP_SIZE);

        if (new_offset >= LOCAL_HEAP_SIZE) {
                if ((fh = malloc(sizeof(local_heap_t))) == NULL)
                        err(1, "failed to allocate local_heap block");

                /* printf("%d: new heap\n", worker->thread_no); */
                fh->prev = *local_heap;
                *local_heap = fh;

                old_offset = aligned_offset(fh->data, align);
                assert(old_offset < align);
                new_offset = old_offset + aligned_size;
        }

        fh->offset = new_offset;
        ptr = fh->data + old_offset;

        /* make sure we're returing aligned pointers */
        assert((((size_t)ptr) & (align - 1)) == 0);

        /* make sure the ptr we're returning is within the heap */
        assert(((size_t)ptr) > ((size_t)fh));
        assert(((size_t)ptr) + size < ((size_t)fh) + sizeof(local_heap_t));

        return ptr;
}

static void
random_sample(worker_t *worker, double *config)
{
        size_t dimensions = worker->system->dimensions;
        int i;

        for (i=dimensions ; --i>=0 ; ) {
                config[i] = mt19937a_genrand_res53(&worker->rnd)
                        * (worker->sample_max[i] - worker->sample_min[i])
                        + worker->sample_min[i];
        }
}

static bool
is_link_expired(prrts_link_t *link)
{
        prrts_node_t *node = link->node;

        smp_read_barrier_depends();
        
        return node->link != link;
}

static prrts_link_t *
remove_first_child(prrts_link_t *parent)
{
        prrts_link_t *child;
        prrts_link_t *sibling;

        assert(is_link_expired(parent));

        do {
                child = parent->first_child;
                if (child == NULL) {
                        return NULL;
                }
                sibling = child->next_sibling;
        } while (!__sync_bool_compare_and_swap(&parent->first_child, child, sibling));

        if (!__sync_bool_compare_and_swap(&child->next_sibling, sibling, NULL))
                assert(false);

        return child;
}

static bool
remove_child(prrts_link_t *parent, prrts_link_t *child)
{
        prrts_link_t *sibling;
        prrts_link_t *n;
        prrts_link_t *p;

        assert(is_link_expired(child));
        assert(child->parent == parent);

        for (;;) {
                n = parent->first_child;

                if (n == child) {
                        /* the child is at the head of the list of children */

                        sibling = child->next_sibling;
                        
                        if (__sync_bool_compare_and_swap(&parent->first_child, child, sibling)) {
                                break;
                        } else {
                                continue;
                        }
                }

                if (n == NULL) {
                        return false;
                }

                for (;;) {
                        p = n;

                        n = n->next_sibling;

                        if (n == NULL) {
                                /*
                                 * iterated through the entire list of
                                 * children and did not find the
                                 * child.  possibly another thread has
                                 * already removed it.
                                 */
                                return false;
                        }

                        if (n == child) {
                                /*
                                 * found it.  try to remove the child
                                 * by linking the previous node's
                                 * sibling to the nodes next sibling.
                                 */
                                sibling = child->next_sibling;
                                if (__sync_bool_compare_and_swap(&p->next_sibling, child, sibling)) {
                                        goto done;
                                } else {
                                        break;
                                }
                        }
                }
        }

done:
        __sync_bool_compare_and_swap(&child->next_sibling, sibling, NULL);

        return true;
}

static void
worker_near_callback(void *worker_arg, int no, void *near_node, double dist)
{
        worker_t *worker = (worker_t*)worker_arg;
        prrts_link_t *near_link;

        assert(no == worker->near_list_size);
        assert(no <= worker->near_list_capacity);

        /* check if we need to grow the array */
        if (no == worker->near_list_capacity) {
                /*
                 * we're not using "local_alloc" here since this
                 * reallocation is relatively infrequent, and it
                 * allows us to make use of realloc.
                 */
                if ((worker->near_list = realloc(worker->near_list, sizeof(near_list_t) * 2 * no)) == NULL)
                        err(1, "failed to reallocate cost nodes (requested %lu bytes)", (unsigned long)(sizeof(near_list_t) * 2 * no));
                
		worker->near_list_capacity = 2 * no;
        }

        near_link = ((prrts_node_t*)near_node)->link;

        /* TODO: reference acquire on near_link */
        
        check_crc32_link(near_link);

        worker->near_list[no].link = near_link;
        worker->near_list[no].link_cost = dist;
        worker->near_list[no].path_cost = near_link->path_cost + dist;

        worker->near_list_size++;
}

static prrts_link_t *
create_link(worker_t *worker, prrts_node_t *node, double link_cost, prrts_link_t *parent)
{
        prrts_link_t *new_link;

        assert(link_cost > 0);

        if ((new_link = (prrts_link_t*)local_alloc(&worker->link_heap, sizeof(prrts_link_t), CACHE_LINE_SIZE)) == NULL)
                err(1, "failed to create link");

        new_link->node = node;
        new_link->link_cost = link_cost;
        new_link->path_cost = link_cost + parent->path_cost;
        new_link->parent = parent;
	new_link->next_sibling = NULL;
	new_link->first_child = NULL;

        set_crc32_link(new_link);

        return new_link;
}

static void
add_child_link(prrts_link_t *parent, prrts_link_t *child)
{
        prrts_link_t *expected = NULL;
        prrts_link_t *next_sibling;

        do {
                next_sibling = parent->first_child;

                /*
                 * We could use:
                 *
                 * __sync_set(&child->next_sibling, next_sibling); 
                 *
                 * but using __sync_bool_compare_and_swap allows us to
                 * test our assumption that this is the only thread
                 * adding the child.  It also allows us to use a
                 * builtin atomic.
                 */
                if (!__sync_bool_compare_and_swap(&child->next_sibling, expected, next_sibling))
                        assert(false);

                expected = next_sibling;

        } while (!__sync_bool_compare_and_swap(&parent->first_child, next_sibling, child));
}

static prrts_node_t *
create_node(worker_t *worker, double *config, size_t dimensions, bool in_goal, double link_cost, prrts_link_t *parent_link)
{
        prrts_node_t *new_node;
        prrts_link_t *link;

        if ((new_node = local_alloc(&worker->node_heap, sizeof(prrts_node_t), CACHE_LINE_SIZE/2)) == NULL)
                err(1, "failed to create node");
        
        /* memcpy(new_node->config, config, sizeof(double) * dimensions); */
        new_node->in_goal = in_goal;
        new_node->config = config;

        set_crc32_node(new_node, worker->system->dimensions);

        link = create_link(worker, new_node, link_cost, parent_link);

         /*
          * The new node and link will not be visible to other threads
          * until we call add_child_link.
          */
        new_node->link = link;

        smp_write_barrier();

        add_child_link(parent_link, link);

        return new_node;
}

static prrts_link_t *
set_node_link(worker_t *worker, prrts_node_t *node, prrts_link_t *old_link, double link_cost, prrts_link_t *parent_link)
{
        prrts_link_t *new_link;

        new_link = create_link(worker, node, link_cost, parent_link);

        smp_write_barrier();

        if (__sync_bool_compare_and_swap(&node->link, old_link, new_link)) {
                assert(new_link->path_cost <= old_link->path_cost);
                add_child_link(parent_link, new_link);
                return new_link;
        } else {
                return NULL;
        }
}

static bool
can_link(worker_t *worker, const double *a, const double *b, double link_cost)
{
        bool r;

        worker->link_dist_sum += link_cost;

        time_stats_start(&worker->link_time);
        r = (worker->system->link_func)(worker->system_data, a, b);
        time_stats_stop(&worker->link_time);

        return r;
}

static void
update_best_path(worker_t *worker, prrts_link_t *link, double radius)
{
        prrts_node_t *node;
        double link_dist_to_goal;
        prrts_system_t *system = worker->system;
        prrts_link_t *best_path;
        double best_dist;
        double dist_to_target;

        check_crc32_link(link);

        node = link->node;

        check_crc32_node(node, system->dimensions);

        if (node->in_goal) {
                link_dist_to_goal = link->path_cost;
        } else if (system->target != NULL) {
                /* goal-biased search */
                dist_to_target = (system->dist_func)(node->config, system->target);
                if (dist_to_target > radius ||
                    !can_link(worker, node->config, system->target, dist_to_target))
                        return;
                link_dist_to_goal = link->path_cost + dist_to_target;
        } else {
                return;
        }

        do {
                best_path = worker->runtime->best_path;

                if (best_path != NULL) {
                        best_dist = best_path->path_cost;

                        if (!best_path->node->in_goal) {
                                /*
                                 * if the current best is not in goal
                                 * itself, it must have a link to a
                                 * target goal configuration
                                 */
                                best_dist += (system->dist_func)(
                                        best_path->node->config, system->target);

                                /* TODO: the dist_func is called for
                                 * every update, might be worth
                                 * caching */
                        }

                        if (link_dist_to_goal >= best_dist) {
                                return;
                        }
                }
        } while (!__sync_bool_compare_and_swap(&worker->runtime->best_path, best_path, link));

#if 0
        printf("New best path found, dist=%f\n", link_dist_to_goal);
#endif
        
}

static void
update_children(worker_t *worker, prrts_link_t *new_parent, prrts_link_t *old_parent, double radius)
{
        prrts_link_t *old_child;
        prrts_link_t *new_child;
        prrts_node_t *node;
        double path_cost;

        check_crc32_link(old_parent);

        assert(new_parent->node == old_parent->node);
        assert(is_link_expired(old_parent));
        assert(new_parent->path_cost <= old_parent->path_cost);

        for (;;) {
                old_child = remove_first_child(old_parent);

                if (old_child == NULL) {
                        if (is_link_expired(new_parent)) {
                                old_parent = new_parent;
                                new_parent = old_parent->node->link;
                                continue;
                        }

                        return;
                }

                check_crc32_link(old_child);

                if (is_link_expired(old_child)) {
                        /* TODO: stat_concurrent_rewirings++ */
                        assert(worker->runtime->thread_count > 1);
                        continue;
                }

                path_cost = new_parent->path_cost + old_child->link_cost;

                node = old_child->node;

                if (node->link->parent->node != old_parent->node) {
                        continue;
                }

                new_child = set_node_link(worker, node, old_child, old_child->link_cost, new_parent);

                if (new_child != NULL) {
                        /* TODO: stat_updated_children_count++ */
                        update_children(worker, new_child, old_child, radius);
                        update_best_path(worker, new_child, radius);
                } else {
                        /* TODO: stat_concurrent_rewirings++ */
                        assert(worker->runtime->thread_count > 1);
                        assert(node->link != old_child);
                }
        }
}

static void
rewire(worker_t *worker, prrts_link_t *old_link, double link_cost, prrts_node_t *new_parent, double radius)
{
        prrts_node_t *node;
        prrts_link_t *parent_link;
        prrts_link_t *new_link;
        prrts_link_t *updated_old_link;
        double path_cost;

        check_crc32_link(old_link);

        assert(old_link->parent != NULL);
        assert(old_link->parent->node != new_parent);

        node = old_link->node;

        check_crc32_node(node, worker->system->dimensions);

        parent_link = new_parent->link;

        check_crc32_link(parent_link);

        path_cost = parent_link->path_cost + link_cost;

        /* check if rewiring would create a shorter path */
        if (path_cost >= old_link->path_cost)
                return;

        /* make sure rewiring is possible */
        if (!can_link(worker, old_link->node->config, new_parent->config, link_cost))
                return;

        /*
         * Rewire the node.  This loop continues to attempt atomic
         * updates until either the update succeeds or the path_cost
         * of the old_link is found to be better than what we are
         * trying to put in.
         */
        do {
                new_link = set_node_link(worker, node, old_link, link_cost, parent_link);

                if (new_link != NULL) {
                        /* TODO: stat_rewired_links++ */
                        
                        update_children(worker, new_link, old_link, radius);
                        update_best_path(worker, new_link, radius);

                        if (is_link_expired(parent_link)) {
                                update_children(
                                        worker,
                                        parent_link->node->link,
                                        parent_link,
                                        radius);
                        }

                        /*
                         * Setting the new_link expires the old_link
                         * but does not remove it from the parent.
                         * Here we just do a little clean up to remove
                         * the old_link.  This is placed after the
                         * parent expiration check because an expired
                         * parent will already take care of this
                         * issue, and the call below will be O(1)
                         * instead of O(n).
                         */
                        if (!remove_child(old_link->parent, old_link)) {
                                assert(worker->runtime->thread_count > 1);
                        }

                        return;
                }

                /*
                 * We shouldn't see a concurrent update on 1 thread,
                 * unless there is something wrong with the order in
                 * which rewire (see note in calling method)
                 */
                assert(worker->runtime->thread_count > 1);

                /* TODO: stat_concurrent_rewirings++ */

                updated_old_link = node->link;
                assert(old_link != updated_old_link);

                check_crc32_link(updated_old_link);

                old_link = updated_old_link;

                /* try again, no need to check/recalculated path cost
                 * or link */
        } while (path_cost < old_link->path_cost);

}

static int
near_list_compare(const void *a, const void *b)
{
        double a_cost = ((near_list_t*)a)->path_cost;
        double b_cost = ((near_list_t*)b)->path_cost;

        return (a_cost < b_cost ? -1 : (a_cost > b_cost ? 1 : 0));
}

/* TODO: steer makes some assumptions, it's implementation should be
 * left to the system */
static void
steer(size_t dimensions, double *new_config, const double *target_config, double scale)
{
        unsigned i;

        for (i=0 ; i<dimensions ; ++i) {
                new_config[i] = target_config[i] + (new_config[i] - target_config[i]) * scale;
        }
}

static bool
worker_step(worker_t *worker, int step_no)
{
        prrts_system_t *system = worker->system;
        double radius;
        double nearest_dist, dist;
        int near_list_size;
        prrts_node_t *nearest;
        prrts_node_t *new_node;
        prrts_link_t *link;
        unsigned i, j;
        double *new_config = worker->new_config;

        random_sample(worker, new_config);

        if (!(system->clear_func)(worker->system_data, new_config))
                return false;

        radius = worker->options->gamma *
                pow(log((double)step_no + 1.0) / (double)(step_no + 1.0),
                    1.0 / (double)system->dimensions);

        assert(radius > 0.0);

        worker->near_list_size = 0;

        TIME(&worker->near_time,
             near_list_size = kd_near(worker->runtime->kd_tree, new_config, radius, worker_near_callback, worker));

        assert(near_list_size == worker->near_list_size);
        
        if (near_list_size == 0) {
                /*
                 * nothing found in the vicinity, try steering towards
                 * the nearest
                 */

                nearest = (prrts_node_t*)kd_nearest(worker->runtime->kd_tree, new_config, &nearest_dist);

                assert(radius <= nearest_dist);

                steer(system->dimensions, new_config, nearest->config, radius / nearest_dist);

                if (!(system->clear_func)(worker->system_data, new_config))
                        return false;

                dist = (system->dist_func)(new_config, nearest->config);

                assert(dist <= nearest_dist);

                if (!can_link(worker, nearest->config, new_config, dist))
                        return false;

                /*
                 * The new configuration has a link.  Create a new
                 * node and insert it into the kd-tree.  There are no
                 * near nodes to rewire, no children node to update,
                 * so we can return immediately
                 */
                new_node = create_node(
                        worker,
                        new_config,
                        system->dimensions,
                        system->target == NULL && (system->in_goal_func)(worker->system_data, new_config),
                        dist,
                        nearest->link);

                update_best_path(worker, new_node->link, radius);

                kd_insert(worker->runtime->kd_tree, new_node->config, new_node);
                return true;
        }

        count_stats_add(&worker->near_list_size_stats, near_list_size);

        /*
         * At this point the near_list array is populated, and the
         * next step is to sort by their path distances in ascending
         * order, to reduce the number of calls to link that follow.
         */
        TIME(&worker->sort_time,
             qsort(worker->near_list, near_list_size, sizeof(near_list_t), near_list_compare));

        time_stats_start(&worker->update_time);
        for (i=0 ; i<near_list_size ; ++i) {
                link = worker->near_list[i].link;

                check_crc32_link(link);
                check_crc32_node(link->node, system->dimensions);

                if (can_link(worker, link->node->config, new_config, worker->near_list[i].link_cost)) {
                        /*
                         * We've found a near node that the new
                         * configuration can link to.  We're
                         * guaranteed at this point that we're linking
                         * to the shortest reachable path because of
                         * the previous sort.  (Caveat that another
                         * what another thread might do to the nodes
                         * after this one)
                         */
                        new_node = create_node(
                                worker,
                                new_config,
                                system->dimensions,
                                system->target == NULL && (system->in_goal_func)(worker->system_data, new_config),
                                worker->near_list[i].link_cost,
                                link);

                        update_best_path(worker, new_node->link, radius);

                        /*
                         * insert into the kd-tree.  After the
                         * kd-insert, other threads might modify the
                         * new_node's path.
                         */
                        kd_insert(worker->runtime->kd_tree, new_node->config, new_node);

                        /* TODO: reference release on near_list[i]->link */
                        
                        /*
                         * Now we rewire the remaining nodes in the
                         * near list through this the new_node if the
                         * resulting path would be shorter.
                         *
                         * Iteration is done in reverse order (from
                         * most distant to closest), to reduce the
                         * likelihood of a node getting updated twice
                         * (or more) during the rewiring.  E.g. if
                         * node A is rewired through new_node, and
                         * node B goes through A, it will be
                         * recursively rewired.  If B also appears in
                         * the near list, then B would be rewired
                         * again.
                         */
                        for (j=near_list_size ; --j > i ; ) {
                                rewire(worker,
                                       worker->near_list[j].link,
                                       worker->near_list[j].link_cost,
                                       new_node,
                                       radius);

                                /* TODO: reference release on near_list[j].link */
                        }

                        time_stats_stop(&worker->update_time);
                        return true;
                }

                /* TODO: reference release on near_list[j].link */
        }
        time_stats_stop(&worker->update_time);

        /*
         * At this point, we've iterated through every near node and
         * found nothing that links.  We return false indicating no
         * new samples were added.
         */

        return false;
}

static void
create_worker_system_data(worker_t *worker)
{
        prrts_runtime_t *runtime = worker->runtime;
        prrts_system_t *system = worker->system;
        double *sample_min;
        double *sample_max;
        double min, t;
        int thread_no = worker->thread_no;
        int thread_count = runtime->thread_count;

        if (!worker->options->regional_sampling) {
                worker->sample_min = system->min;
                worker->sample_max = system->max;
        } else {
                sample_min = array_copy(system->min, system->dimensions);
                sample_max = array_copy(system->max, system->dimensions);

                min = system->min[REGION_SPLIT_AXIS];
                t = (system->max[REGION_SPLIT_AXIS] - min) / thread_count;
                sample_min[REGION_SPLIT_AXIS] = min + thread_no * t;
                if (thread_no+1 < thread_count) {
                        sample_max[REGION_SPLIT_AXIS] = min + (thread_no+1) * t;
                }

                worker->sample_min = sample_min;
                worker->sample_max = sample_max;
        }

        worker->system_data = (system->system_data_alloc_func)(
                thread_no, worker->sample_min, worker->sample_max);
}

static void *
worker_main(void *arg)
{
        worker_t *worker = (worker_t*)arg;
        bool is_main_thread = (worker->thread_no == 0);
        prrts_runtime_t *runtime = worker->runtime;
        prrts_system_t *system = worker->system;
        size_t dimensions = system->dimensions;
        int step_no;

        worker->near_list = malloc(sizeof(near_list_t) * INITIAL_NEAR_LIST_CAPACITY);
        worker->near_list_size = 0;
        worker->near_list_capacity = INITIAL_NEAR_LIST_CAPACITY;

        worker->config_heap = local_heap_create(CACHE_LINE_SIZE/2);
        worker->node_heap = local_heap_create(CACHE_LINE_SIZE/2);
        worker->link_heap = local_heap_create(CACHE_LINE_SIZE);

        create_worker_system_data(worker);

        worker->sample_count = 0;
        worker->clear_count = 0;
        worker->link_dist_sum = 0.0;
        time_stats_clear(&worker->near_time);
        time_stats_clear(&worker->link_time);
        time_stats_clear(&worker->sort_time);
        time_stats_clear(&worker->insert_time);
        time_stats_clear(&worker->update_time);
        count_stats_clear(&worker->near_list_size_stats);

        worker->new_config = local_alloc(&worker->config_heap, sizeof(double) * dimensions, CACHE_LINE_SIZE/2);

        step_no = runtime->step_no;

        while (!worker->runtime->done) {

                worker->sample_count++;

                if (worker_step(worker, step_no)) {
                        worker->clear_count++;

                        worker->new_config = local_alloc(&worker->config_heap, sizeof(double) * dimensions, CACHE_LINE_SIZE/2);

                        /*
                         * Successfullly added a configuration, update
                         * the shared total step count
                         */
                        step_no = __sync_fetch_and_add(&runtime->step_no, 1);

#if 0
                        printf("[%d] step_no = %d, clear=%f%%, samples/sec=%f, near_time=%f, sort_time=%f, update_time=%f, link_time=%f\n",
                               worker->thread_no,
                               step_no, worker->clear_count * 100.0 / (double)worker->sample_count,
                               worker->sample_count * (double)(HRTICK) / (double)(hrtimer_get() - worker->runtime->start_time),
                               worker->near_time.sum / (double)worker->near_time.count,
                               /* worker->near_time.count, */

                               worker->sort_time.sum / (double)worker->sort_time.count,
                               /* worker->sort_time.count, */

                               worker->update_time.sum / (double)worker->update_time.count,
                               /* worker->update_time.count, */

                               worker->link_time.sum / (double)worker->link_time.count);
#endif
                               

                        if (step_no >= runtime->sample_limit) {
                                /* __sync_set(&runtime->done, true); */
                                __sync_bool_compare_and_swap(&runtime->done, false, true);
                        }
                } else {
                        /*
                         * unsuccessful in adding.  refresh the
                         * step_no, but do not increment it.
                         */
                        step_no = runtime->step_no;
                }

                if (is_main_thread &&
                    runtime->time_limit != 0 && 
                    hrtimer_get() > runtime->time_limit)
                {
                        /* __sync_set(&runtime->done, true); */
                        __sync_bool_compare_and_swap(&runtime->done, false, true);
                }
        }

        /* issue a write barrier mostly for the thread-local stats */
        smp_write_barrier();

        return NULL;
}

static prrts_solution_t *
path_to_solution(prrts_system_t *system, prrts_link_t *path)
{
        size_t dimensions = system->dimensions;
        size_t n;
        int i;
        prrts_solution_t *s;
        prrts_link_t *ptr;
        double *config_ptr;

        if (path == NULL) {
                return NULL;
        }

        n = 0;
        for ( ptr = path ; ptr != NULL ; ptr = ptr->parent ) {
                check_crc32_link(ptr);
                check_crc32_node(ptr->node, dimensions);
                n++;
        }

        /*
         * if there is a target configuration, we include it in the
         * solution
         */
        if (system->target != NULL)
                n++;

        if ((s = malloc(sizeof(prrts_solution_t) + sizeof(double *) * n + sizeof(double) * dimensions * n)) == NULL)
                err(1, "failed to create solution");

        s->path_cost = path->path_cost;
        s->path_length = n;
        
        config_ptr = (void*)(&s->configs[n]);

        i = n;

        if (system->target != NULL) {
                s->configs[--i] = memcpy(config_ptr, system->target, sizeof(double) * dimensions);
                config_ptr += dimensions;
        }

        for ( ptr = path ; ptr != NULL ; ptr = ptr->parent) {
                s->configs[--i] = memcpy(config_ptr, ptr->node->config, sizeof(double) * dimensions);
                config_ptr += dimensions;
        }

        assert(i == 0);

        return s;
}

#ifdef _OPENMP
#define get_num_procs() omp_get_max_threads()
#else
static int
get_num_procs()
{
#ifdef __linux__
        return sysconf(_SC_NPROCESSORS_ONLN);
#elif defined(BSD) || defined(__APPLE__)

        int procs;
        int mib[4];
        size_t len = sizeof(procs);

        mib[0] = CTL_HW;
        mib[1] = HW_AVAILCPU;

        sysctl(mib, 2, &procs, &len, NULL, 0);

        if (procs < 1) {
                mib[1] = HW_NCPU;
                sysctl(mib, 2, &procs, &len, NULL, 0);

                if (procs < 1)
                        procs = 1;
        }

        return procs;

#else
        return -1;
#endif
}
#endif /* _OPENMP */


/*
 * This function starts up thread_count-1 worker threads and runs one
 * of the worker processes on the calling thread.  If compiled with
 * OpenMP, an omp pragma is used to start the threads, otherwise
 * pthreads is used.  The openmp startup code is significantly shorter
 * than the pthread version.
 */
static void
start_workers(worker_t *workers, int thread_count)
{
#ifdef _OPENMP
        int i;

#pragma omp parallel for num_threads(thread_count) schedule(static,1) private(i)
        for (i=0 ; i<thread_count ; ++i) {
                worker_main(&workers[i]);
        }

        /*
         * that's all for the open mp version.  the parallel pragma
         * includes memory barriers
         */

#else /* _OPENMP */

        int i;
        pthread_attr_t pthread_attr;
        int error;
#if defined(SET_CPU_AFFINITY) && defined(__linux__)
        int num_cpus = get_num_procs();
        int cpu;
        cpu_set_t cpu_set;
#endif

        pthread_attr_init(&pthread_attr);

#if defined(SET_CPU_AFFINITY) && defined(__linux__)
        cpu = 0; /* sched_getcpu() */
        /* printf("cpu = %d, size=%d\n", cpu, (int)sizeof(cpu_set_t)); */

        CPU_ZERO(&cpu_set);

        /* tell the scheduler to keep this thread on its current cpu */
        CPU_SET(cpu, &cpu_set);
        if ((error = pthread_setaffinity_np(pthread_self(), sizeof(cpu_set_t), &cpu_set)) != 0)
                warn("failed to set thread 0 affinity to %d", (int)cpu);
        CPU_CLR(cpu, &cpu_set);
#endif

        /*
         * a write barrier here is probably not needed since
         * pthread_create likely takes care of it.  But this is done
         * once, so the overhead does not matter.
         */
        smp_write_barrier();

        /*
         * Start the threads, worker[0] is run on the calling thread,
         * the rest get their own new threads.
         */
        for (i=1 ; i<thread_count ; ++i) {
#if defined(SET_CPU_AFFINITY) && defined(__linux__)
                cpu = (cpu + (num_cpus/2) + 1) % num_cpus;
                CPU_SET(cpu, &cpu_set);
                /* printf("worker %u scheduled on %d\n", i, (int)((cpu + i) % num_cpus)); */
                if ((error = pthread_attr_setaffinity_np(&pthread_attr, sizeof(cpu_set_t), &cpu_set)) != 0)
                        warn("failed to set thread %u affinity to %d", i, (int)cpu);
#endif

                if ((error = pthread_create(&workers[i].thread_id, &pthread_attr, worker_main, &workers[i])) != 0)
                        warn("thread create failed on worker %u", i);

#if defined(SET_CPU_AFFINITY) && defined(__linux__)
                CPU_CLR(cpu, &cpu_set);
#endif
        }

        pthread_attr_destroy(&pthread_attr);

        /* run worker 0 on the calling thread */
        worker_main(&workers[0]);

        /*
         * After worker[0] completes, join the rest of the workers
         */
        for (i=1 ; i<thread_count ; ++i) {
                error = pthread_join(workers[i].thread_id, NULL);
                if (error != 0)
                        warn("join failed on worker %u", i);
        }

        /*
         * this is probably not necessary since pthread_join likely
         * performs the memory synchronization, but again, done
         * once.
         */
        smp_read_barrier_depends();        

#endif /* _OPENMP */

}

static prrts_solution_t *
prrts_run(prrts_system_t *system, prrts_options_t *options, int thread_count, long duration, size_t sample_count)
{
        char runtime_alloc[sizeof(prrts_runtime_t) + CACHE_LINE_SIZE];
        char root_node_alloc[sizeof(prrts_node_t) + CACHE_LINE_SIZE];
        char root_link_alloc[sizeof(prrts_link_t) + CACHE_LINE_SIZE];

        prrts_runtime_t *runtime;
        prrts_node_t *root_node;
        prrts_link_t *root_link;

        worker_t *workers;
        int i;


        double link_dist_sum;
        time_stats_t link_time;
        time_stats_t near_time;
        time_stats_t update_time;
        count_stats_t near_list_size_stats;

        prrts_solution_t *solution;

        int num_cpus;


        /* check that the calling parameters are sane */
        assert(system->dimensions > 0);
        assert(options->gamma > 1);
        assert(options->samples_per_step >= 1);
        assert(thread_count > 0);

        num_cpus = get_num_procs();
        if (num_cpus != -1 && thread_count > num_cpus)
                printf("WARNING!  thread count (%d) exceeds CPU count (%d)\n",
                       thread_count, num_cpus);

        /*
         * the "runtime" structure is shared by all workers to both
         * communicate system configuration and runtime state.  We use
         * some pointer arithmetic to make sure that the
         * stack-allocated data structures are aligned to cache lines.
         */
        runtime = (prrts_runtime_t*)ALIGN_UP(runtime_alloc, CACHE_LINE_SIZE);
        root_node = (prrts_node_t*)ALIGN_UP(root_node_alloc, CACHE_LINE_SIZE);
        root_link = (prrts_link_t*)ALIGN_UP(root_link_alloc, CACHE_LINE_SIZE);

        /* check that our alignments are working */
        assert((((size_t)&runtime->step_no) & (CACHE_LINE_SIZE-1)) == 0);
        assert((((size_t)&runtime->best_path) & (CACHE_LINE_SIZE-1)) == 0);

        /* initialize the shared runtime structure */
        runtime->thread_count = thread_count;
        runtime->time_limit = 0;
        runtime->sample_limit = sample_count;
        runtime->best_path = NULL;
        runtime->step_no = 1;
        runtime->done = false;

        root_node->link = root_link;
        root_node->in_goal = false;
        root_node->config = system->init;

        set_crc32_node(root_node, system->dimensions);

        root_link->node = root_node;
        root_link->parent = NULL;
        root_link->link_cost = 0.0;
        root_link->path_cost = 0.0;
        root_link->first_child = NULL;
        root_link->next_sibling = NULL;

        set_crc32_link(root_link);

        runtime->root = root_node;
        
        runtime->kd_tree = kd_create_tree(
                system->dimensions, system->min, system->max, system->dist_func,
                root_node->config, root_node);

        printf("Starting up (" 
#ifdef _OPENMP
               "using OpenMP"
#else
               "using pthreads"
#endif

#ifdef CHECK_CRCS
               ", checking crcs"
#endif
               ").  Configuration space is %d dimensions, Using %d threads.\n",
               (int)system->dimensions,
               (int)thread_count
                );

        /* start the clock */
        runtime->start_time = hrtimer_get();

        /*
         * We use the sequence generated by srandom/random as a quick
         * seed generator for the the thread-safe mersenne twister prng.
         */
        srandom(runtime->start_time);

        if (duration > 0)
                runtime->time_limit = runtime->start_time + duration;

        if ((workers = malloc(sizeof(worker_t) * thread_count)) == NULL)
                err(1, "failed to create workers array");

        /*
         * Set up the sampling regions for the
         * worker threads.
         */
        for (i=0 ; i<thread_count ; ++i) {
                workers[i].runtime = runtime;
                workers[i].system = system;
                workers[i].options = options;
                workers[i].thread_no = i;
                mt19937a_init_genrand(&workers[i].rnd, random());
        }

        start_workers(workers, thread_count);

        /* end_time (TODO) */
        hrtimer_get();

        link_dist_sum = 0.0;
        time_stats_clear(&link_time);
        time_stats_clear(&near_time);
        time_stats_clear(&update_time);
        count_stats_clear(&near_list_size_stats);

        for (i=0 ; i<thread_count ; ++i) {
                link_dist_sum += workers[i].link_dist_sum;
                time_stats_accum(&link_time, &workers[i].link_time);
                time_stats_accum(&near_time, &workers[i].near_time);
                time_stats_accum(&update_time, &workers[i].update_time);
                count_stats_accum(&near_list_size_stats, &workers[i].near_list_size_stats);
        }

#if 0
        printf("link stats: %d calls, %f total time, %f us/call, avg dist=%f\n",
               link_time.count,
               link_time.sum / (double)HRTICK,
               link_time.sum / (double)link_time.count * (1e+6 / (double)HRTICK),
               link_dist_sum / (double)link_time.count);
#endif

        count_stats_print("near size", &near_list_size_stats);
        printf("\n");
        time_stats_print("near time", &near_time);
        printf("\n");
        time_stats_print("link time", &link_time);
        printf(", avg dist=%f\n", link_dist_sum / (double)link_time.count);
        time_stats_print("update tm", &update_time);
        printf("\n");

        solution = path_to_solution(system, runtime->best_path);

        for (i=0 ; i<thread_count ; ++i) {
                free(workers[i].near_list);
                local_heap_free(workers[i].link_heap);
                local_heap_free(workers[i].node_heap);
                local_heap_free(workers[i].config_heap);

                if (options->regional_sampling) {
                        free((void*)workers[i].sample_min);
                        free((void*)workers[i].sample_max);
                }
        }

        free(workers);

        return solution;
}

prrts_solution_t *
prrts_run_for_duration(prrts_system_t *system, prrts_options_t *options, int thread_count, long duration)
{
        assert(duration > 0);

        return prrts_run(system, options, thread_count, duration, UINT_MAX);
}

prrts_solution_t *
prrts_run_for_samples(prrts_system_t *system, prrts_options_t *options, int thread_count, size_t sample_count)
{
        assert(sample_count > 0);
        assert(sample_count < UINT_MAX);

        return prrts_run(system, options, thread_count, 0, sample_count);
}

prrts_solution_t *
prrts_run_indefinitely(prrts_system_t *system, prrts_options_t *options, int thread_count)
{
        return prrts_run(system, options, thread_count, 0, UINT_MAX);
}

