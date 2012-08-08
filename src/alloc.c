#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <errno.h>
#include "alloc.h"

typedef struct pool_bucket {
        struct pool_bucket *prev;
        size_t offset;
        char data[0];
} pool_bucket_t;

static void
pool_destructor(void *data)
{
#if 0
        /*
         * It is not safe to assume that the memory allocated to this
         * pool is no longer in use.
         */
        pool_bucket_t *bucket = data;
        pool_bucket_t *tmp;

        while (bucket != NULL) {
                tmp = bucket->prev;
                free(bucket);
                bucket = tmp;
        }
#endif
}

int
tl_mempool_init(tl_mempool_t *pool, size_t chunk_size)
{
        int error;

        assert(chunk_size > sizeof(void *) * 100);

        if ((error = pthread_key_create(&pool->key, pool_destructor)) != 0)
                return error;

        pool->chunk_size = chunk_size;

        __sync_synchronize();

        return 0;
}

int
tl_mempool_destroy(tl_mempool_t *pool)
{
        int error;

        if ((error = pthread_key_delete(pool->key)) != 0)
                return error;

        return 0;
}

void *
tl_alloc(tl_mempool_t *pool, size_t size)
{
        size_t aligned_size = ALIGN_UP(size, CACHE_LINE_SIZE/2);
        pool_bucket_t *bucket;
        pool_bucket_t *tmp;
        void *ptr;

        assert(aligned_size >= size);
        assert(aligned_size <= pool->chunk_size);

        bucket = pthread_getspecific(pool->key);

        if (bucket == NULL || bucket->offset + size > pool->chunk_size) {
                tmp = bucket;
                if ((bucket = malloc(sizeof(pool_bucket_t) + pool->chunk_size)) == NULL)
                        return NULL;
                
                bucket->prev = tmp;
                bucket->offset = aligned_offset(bucket->data, CACHE_LINE_SIZE/2);
                
                pthread_setspecific(pool->key, bucket);
        }

        ptr = bucket->data + bucket->offset;
        bucket->offset += aligned_size;

        return ptr;
}

void
tl_free(tl_mempool_t *pool, void *ptr)
{
}

void *
memdup(const void *source, size_t bytes)
{
        void *copy;

        if (source == NULL)
                return NULL;

        if ((copy = malloc(bytes)) == NULL)
                return NULL;

        memcpy(copy, source, bytes);

        return copy;
}
