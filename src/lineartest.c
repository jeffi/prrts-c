#include <stdlib.h>
#include <stdio.h>
#include <assert.h>
#include <math.h>
#include "linear.h"

#define EPSILON 0.00001

static void
check_double(char *name, double expected, double actual, double eps)
{
        if (fabs(actual - expected) > eps) {
                printf("expected %f, got %f (%s)\n", expected, actual, name);
                abort();
        }
}

static void
test_identity_transform()
{
        vec4_t r;

        m4_transform_i(&r, &M4_IDENTITY, 2.0, 3.0, 4.0, 1.0);

        assert(r.x == 2.0);
        assert(r.y == 3.0);
        assert(r.z == 4.0);
        assert(r.w == 1.0);
}

static void
test_translate()
{
        mat4_t m;
        vec4_t r;

        m4_translate(&m, &M4_IDENTITY, 5.0, 4.0, 3.0);
        m4_transform_i(&r, &m, 6.0, 5.0, 4.0, 1.0);

        assert(r.x == 11.0);
        assert(r.y == 9.0);
        assert(r.z == 7.0);
        assert(r.w == 1.0);
}

static void
test_rotate_x()
{
        mat4_t m;
        vec4_t r;
        
        m4_rotate(&m, &M4_IDENTITY, M_PI / 4.0, 1.0, 0.0, 0.0);
        m4_transform_i(&r, &m, 10.0, 10.0, 10.0, 1.0);

        check_double("r.x", 10.0, r.x, EPSILON);
        check_double("r.y", 0.0, r.y, EPSILON);
        check_double("r.z", 10 * sqrt(2.0), r.z, EPSILON);
        assert(r.w == 1.0);
}

static void
test_rotate_y()
{
        mat4_t m;
        vec4_t r;
        
        m4_rotate(&m, &M4_IDENTITY, M_PI / 4.0, 0.0, 1.0, 0.0);
        m4_transform_i(&r, &m, 10.0, 10.0, 10.0, 1.0);

        check_double("r.x", 10 * sqrt(2.0), r.x, EPSILON);
        check_double("r.y", 10.0, r.y, EPSILON);
        check_double("r.z", 0.0, r.z, EPSILON);

        assert(r.w == 1.0);
}

static void
test_rotate_z()
{
        mat4_t m;
        vec4_t r;
        
        m4_rotate(&m, &M4_IDENTITY, M_PI / 4.0, 0.0, 0.0, 1.0);
        m4_transform_i(&r, &m, 10.0, 10.0, 10.0, 1.0);

        check_double("r.x", 0.0, r.x, EPSILON);
        check_double("r.y", 10 * sqrt(2.0), r.y, EPSILON);
        check_double("r.z", 10.0, r.z, EPSILON);

        assert(r.w == 1.0);
}

int
main(int argc, char *argv[])
{
        test_identity_transform();
        test_translate();
        test_rotate_x();
        test_rotate_y();
        test_rotate_z();

        return 0;
}
