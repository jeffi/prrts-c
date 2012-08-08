
INLINE
void
m4_mul(mat4_t *r, const mat4_t *a, const mat4_t *b)
{
        if (r != a && r != b) {
                r->m00 = a->m00 * b->m00 + a->m01 * b->m10 + a->m02 * b->m20 + a->m03 * b->m30;
                r->m01 = a->m00 * b->m01 + a->m01 * b->m11 + a->m02 * b->m21 + a->m03 * b->m31;
                r->m02 = a->m00 * b->m02 + a->m01 * b->m12 + a->m02 * b->m22 + a->m03 * b->m32;
                r->m03 = a->m00 * b->m03 + a->m01 * b->m13 + a->m02 * b->m23 + a->m03 * b->m33;
                r->m10 = a->m10 * b->m00 + a->m11 * b->m10 + a->m12 * b->m20 + a->m13 * b->m30;
                r->m11 = a->m10 * b->m01 + a->m11 * b->m11 + a->m12 * b->m21 + a->m13 * b->m31;
                r->m12 = a->m10 * b->m02 + a->m11 * b->m12 + a->m12 * b->m22 + a->m13 * b->m32;
                r->m13 = a->m10 * b->m03 + a->m11 * b->m13 + a->m12 * b->m23 + a->m13 * b->m33;
                r->m20 = a->m20 * b->m00 + a->m21 * b->m10 + a->m22 * b->m20 + a->m23 * b->m30;
                r->m21 = a->m20 * b->m01 + a->m21 * b->m11 + a->m22 * b->m21 + a->m23 * b->m31;
                r->m22 = a->m20 * b->m02 + a->m21 * b->m12 + a->m22 * b->m22 + a->m23 * b->m32;
                r->m23 = a->m20 * b->m03 + a->m21 * b->m13 + a->m22 * b->m23 + a->m23 * b->m33;
                r->m30 = a->m30 * b->m00 + a->m31 * b->m10 + a->m32 * b->m20 + a->m33 * b->m30;
                r->m31 = a->m30 * b->m01 + a->m31 * b->m11 + a->m32 * b->m21 + a->m33 * b->m31;
                r->m32 = a->m30 * b->m02 + a->m31 * b->m12 + a->m32 * b->m22 + a->m33 * b->m32;
                r->m33 = a->m30 * b->m03 + a->m31 * b->m13 + a->m32 * b->m23 + a->m33 * b->m33;
        } else {
                mat4_t s;
                s.m00 = a->m00 * b->m00 + a->m01 * b->m10 + a->m02 * b->m20 + a->m03 * b->m30;
                s.m01 = a->m00 * b->m01 + a->m01 * b->m11 + a->m02 * b->m21 + a->m03 * b->m31;
                s.m02 = a->m00 * b->m02 + a->m01 * b->m12 + a->m02 * b->m22 + a->m03 * b->m32;
                s.m03 = a->m00 * b->m03 + a->m01 * b->m13 + a->m02 * b->m23 + a->m03 * b->m33;
                s.m10 = a->m10 * b->m00 + a->m11 * b->m10 + a->m12 * b->m20 + a->m13 * b->m30;
                s.m11 = a->m10 * b->m01 + a->m11 * b->m11 + a->m12 * b->m21 + a->m13 * b->m31;
                s.m12 = a->m10 * b->m02 + a->m11 * b->m12 + a->m12 * b->m22 + a->m13 * b->m32;
                s.m13 = a->m10 * b->m03 + a->m11 * b->m13 + a->m12 * b->m23 + a->m13 * b->m33;
                s.m20 = a->m20 * b->m00 + a->m21 * b->m10 + a->m22 * b->m20 + a->m23 * b->m30;
                s.m21 = a->m20 * b->m01 + a->m21 * b->m11 + a->m22 * b->m21 + a->m23 * b->m31;
                s.m22 = a->m20 * b->m02 + a->m21 * b->m12 + a->m22 * b->m22 + a->m23 * b->m32;
                s.m23 = a->m20 * b->m03 + a->m21 * b->m13 + a->m22 * b->m23 + a->m23 * b->m33;
                s.m30 = a->m30 * b->m00 + a->m31 * b->m10 + a->m32 * b->m20 + a->m33 * b->m30;
                s.m31 = a->m30 * b->m01 + a->m31 * b->m11 + a->m32 * b->m21 + a->m33 * b->m31;
                s.m32 = a->m30 * b->m02 + a->m31 * b->m12 + a->m32 * b->m22 + a->m33 * b->m32;
                s.m33 = a->m30 * b->m03 + a->m31 * b->m13 + a->m32 * b->m23 + a->m33 * b->m33;
                *r = s;
        }
}

INLINE
void
m4_mul_mi(mat4_t *r, const mat4_t *a,
          double b00, double b01, double b02, double b03,
          double b10, double b11, double b12, double b13,
          double b20, double b21, double b22, double b23,
          double b30, double b31, double b32, double b33)
{
        if (r != a) {
                r->m00 = a->m00 * b00 + a->m01 * b10 + a->m02 * b20 + a->m03 * b30;
                r->m01 = a->m00 * b01 + a->m01 * b11 + a->m02 * b21 + a->m03 * b31;
                r->m02 = a->m00 * b02 + a->m01 * b12 + a->m02 * b22 + a->m03 * b32;
                r->m03 = a->m00 * b03 + a->m01 * b13 + a->m02 * b23 + a->m03 * b33;
                r->m10 = a->m10 * b00 + a->m11 * b10 + a->m12 * b20 + a->m13 * b30;
                r->m11 = a->m10 * b01 + a->m11 * b11 + a->m12 * b21 + a->m13 * b31;
                r->m12 = a->m10 * b02 + a->m11 * b12 + a->m12 * b22 + a->m13 * b32;
                r->m13 = a->m10 * b03 + a->m11 * b13 + a->m12 * b23 + a->m13 * b33;
                r->m20 = a->m20 * b00 + a->m21 * b10 + a->m22 * b20 + a->m23 * b30;
                r->m21 = a->m20 * b01 + a->m21 * b11 + a->m22 * b21 + a->m23 * b31;
                r->m22 = a->m20 * b02 + a->m21 * b12 + a->m22 * b22 + a->m23 * b32;
                r->m23 = a->m20 * b03 + a->m21 * b13 + a->m22 * b23 + a->m23 * b33;
                r->m30 = a->m30 * b00 + a->m31 * b10 + a->m32 * b20 + a->m33 * b30;
                r->m31 = a->m30 * b01 + a->m31 * b11 + a->m32 * b21 + a->m33 * b31;
                r->m32 = a->m30 * b02 + a->m31 * b12 + a->m32 * b22 + a->m33 * b32;
                r->m33 = a->m30 * b03 + a->m31 * b13 + a->m32 * b23 + a->m33 * b33;
        } else {
                mat4_t s;
                s.m00 = a->m00 * b00 + a->m01 * b10 + a->m02 * b20 + a->m03 * b30;
                s.m01 = a->m00 * b01 + a->m01 * b11 + a->m02 * b21 + a->m03 * b31;
                s.m02 = a->m00 * b02 + a->m01 * b12 + a->m02 * b22 + a->m03 * b32;
                s.m03 = a->m00 * b03 + a->m01 * b13 + a->m02 * b23 + a->m03 * b33;
                s.m10 = a->m10 * b00 + a->m11 * b10 + a->m12 * b20 + a->m13 * b30;
                s.m11 = a->m10 * b01 + a->m11 * b11 + a->m12 * b21 + a->m13 * b31;
                s.m12 = a->m10 * b02 + a->m11 * b12 + a->m12 * b22 + a->m13 * b32;
                s.m13 = a->m10 * b03 + a->m11 * b13 + a->m12 * b23 + a->m13 * b33;
                s.m20 = a->m20 * b00 + a->m21 * b10 + a->m22 * b20 + a->m23 * b30;
                s.m21 = a->m20 * b01 + a->m21 * b11 + a->m22 * b21 + a->m23 * b31;
                s.m22 = a->m20 * b02 + a->m21 * b12 + a->m22 * b22 + a->m23 * b32;
                s.m23 = a->m20 * b03 + a->m21 * b13 + a->m22 * b23 + a->m23 * b33;
                s.m30 = a->m30 * b00 + a->m31 * b10 + a->m32 * b20 + a->m33 * b30;
                s.m31 = a->m30 * b01 + a->m31 * b11 + a->m32 * b21 + a->m33 * b31;
                s.m32 = a->m30 * b02 + a->m31 * b12 + a->m32 * b22 + a->m33 * b32;
                s.m33 = a->m30 * b03 + a->m31 * b13 + a->m32 * b23 + a->m33 * b33;
                *r = s;
        }
}

INLINE
void
m4_rotate(mat4_t *m, const mat4_t *t, double a, double x, double y, double z)
{
        double d2 = x*x + y*y + z*z;
        double c = cos(a);
        double s = sin(a);
        double p;

        if (d2 != 1.0) {
                p = 1.0 / sqrt(d2);
                x *= p;
                y *= p;
                z *= p;
        }

        m4_mul_mi(m, t,

                  x*x * (1-c) + c,    x*y * (1-c) - z*s,  x*z * (1-c) + y*s,  0.0,
                  y*x * (1-c) + z*s,  y*y * (1-c) + c,    y*z * (1-c) - x*s,  0.0,
                  z*x * (1-c) - y*s,  z*y * (1-c) + x*s,  z*z * (1-c) + c,    0.0,
                  0.0,                0.0,                0.0,                1.0);

}

INLINE
void
m4_translate(mat4_t *r, const mat4_t *t, double x, double y, double z)
{
        m4_mul_mi(r, t,

                  1.0, 0.0, 0.0, x,
                  0.0, 1.0, 0.0, y,
                  0.0, 0.0, 1.0, z,
                  0.0, 0.0, 0.0, 1.0);
}

/*
 * 4x4 Matrix and homogeneous vector mulitplication.  The input
 * coordinates are specified in immediate parameters (not wrapped in a
 * vector).
 */
INLINE
void
m4_transform_i(vec4_t *r, const mat4_t *m, double x, double y, double z, double w)
{
        r->x = m->m00 * x + m->m01 * y + m->m02 * z + m->m03 * w;
        r->y = m->m10 * x + m->m11 * y + m->m12 * z + m->m13 * w;
        r->z = m->m20 * x + m->m21 * y + m->m22 * z + m->m23 * w;
        r->w = m->m30 * x + m->m31 * y + m->m32 * z + m->m33 * w;
}

/*
 * Like m4_transform_i, with only x,y, and z specified.  w is set to
 * 1.0, and used to perform a homogeneous divide to convert to a 3-D
 * vector.
 */
INLINE
void
m4_transform_i3(vec3_t *r, const mat4_t *m, double x, double y, double z)
{
        double w = 1.0 / (m->m30 * x + m->m31 * y + m->m32 * z + m->m33);

        r->x = (m->m00 * x + m->m01 * y + m->m02 * z + m->m03) * w;
        r->y = (m->m10 * x + m->m11 * y + m->m12 * z + m->m13) * w;
        r->z = (m->m20 * x + m->m21 * y + m->m22 * z + m->m23) * w;
}

INLINE
void
m4_transform_v(vec4_t *r, const mat4_t *m, const vec4_t *v)
{
        m4_transform_i(r, m, v->x, v->y, v->z, v->w);
}

INLINE
vec3_t *
v3_sub(vec3_t *r, const vec3_t *a, const vec3_t *b)
{
        r->x = a->x - b->x;
        r->y = a->y - b->y;
        r->z = a->z - b->z;
        return r;
}

INLINE
void
v3_add(vec3_t *r, const vec3_t *a, const vec3_t *b)
{
        r->x = a->x + b->x;
        r->y = a->y + b->y;
        r->z = a->z + b->z;
}

INLINE
void
v3_scale(vec3_t *r, const vec3_t *v, double s)
{
        r->x = v->x * s;
        r->y = v->y * s;
        r->z = v->z * s;
}

INLINE
double
v3_dot(const vec3_t *a, const vec3_t *b)
{
        return a->x * b->x + a->y * b->y + a->z * b->z;
}

INLINE
double
v3_len2(const vec3_t *v)
{
        return v3_dot(v, v);
}

INLINE
double
v3_len(const vec3_t *v)
{
        return sqrt(v3_len2(v));
}

INLINE
double
v3_dist2(const vec3_t *a, const vec3_t *b)
{
        vec3_t t;
        return v3_len2(v3_sub(&t, a, b));
}

INLINE
double
v3_dist(const vec3_t *a, const vec3_t *b)
{
        return sqrt(v3_dist2(a, b));
}

INLINE
void
v3_norm(vec3_t *r, const vec3_t *a)
{
        v3_scale(r, a, 1.0 / v3_len(a));
}

INLINE
vec2_t *
v2_sub(vec2_t *r, const vec2_t *a, const vec2_t *b)
{
        r->x = a->x - b->x;
        r->y = a->y - b->y;
        return r;
}

INLINE
double
v2_dot(const vec2_t *a, const vec2_t *b)
{
        return a->x * b->x + a->y * b->y;
}

INLINE
double
v2_len2(const vec2_t *a)
{
        return v2_dot(a, a);
}

INLINE
double
v2_dist2(const vec2_t *a, const vec2_t *b)
{
        vec2_t t;
        return v2_len2(v2_sub(&t, a, b));
}

INLINE
double
v2_dist(const vec2_t *a, const vec2_t *b)
{
        return sqrt(v2_dist2(a, b));
}

INLINE
void
m4_extract_translation(vec4_t *r, const mat4_t *m)
{
        r->x = m->m03;
        r->y = m->m13;
        r->z = m->m23;
        r->w = m->m33;
}
