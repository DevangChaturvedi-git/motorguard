/*
 * fixed_point.h
 *
 * Q16.16 fixed-point helpers.
 *
 * The Blue Pill's Cortex-M3 has no FPU, so any float math in the control
 * loop gets quietly software-emulated by libgcc - slow, and it bloats the
 * binary for no reason. Everything in the control path (Layer 1/3) is
 * done in Q16.16 instead: a signed 32-bit integer where the top 16 bits
 * are the integer part and the bottom 16 are the fraction. Multiplying
 * two Q16.16 numbers needs a 64-bit intermediate so it doesn't overflow,
 * which is what q_mul is for.
 *
 * Everything here is static inline - this is meant to compile down to a
 * couple of instructions, not an actual function call.
 */

#ifndef FIXED_POINT_H
#define FIXED_POINT_H

#include <stdint.h>

typedef int32_t q16_t;

#define Q16_SHIFT      16
#define Q16_ONE        (1 << Q16_SHIFT)
#define Q16_FROM_INT(x)   ((q16_t)((x) << Q16_SHIFT))
#define Q16_TO_INT(x)     ((int32_t)((x) >> Q16_SHIFT))

/* fixed-point literal from a percentage, e.g. Q16_FROM_PCT(75) -> 0.75.
 * multiplication instead of a left-shift specifically so this is safe
 * to call with a negative percentage too (Q16_FROM_PCT(-5)) - shifting
 * a negative value left is the kind of thing that works today and
 * bites someone in six months when a compiler gets stricter about it */
#define Q16_FROM_PCT(pct) ((q16_t)(((int64_t)(pct) * Q16_ONE) / 100))

static inline q16_t q_mul(q16_t a, q16_t b)
{
    int64_t tmp = (int64_t)a * (int64_t)b;
    return (q16_t)(tmp >> Q16_SHIFT);
}

static inline q16_t q_div(q16_t a, q16_t b)
{
    if (b == 0) {
        return (a >= 0) ? INT32_MAX : INT32_MIN;
    }
    int64_t tmp = ((int64_t)a << Q16_SHIFT);
    return (q16_t)(tmp / b);
}

static inline q16_t q_clamp(q16_t v, q16_t lo, q16_t hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

static inline q16_t q_abs(q16_t v)
{
    return (v < 0) ? -v : v;
}

/* squares a Q16.16 value without losing the low bits the way q_mul(v,v)
 * would if v is large - used by the I^2t accumulator where precision on
 * the way up matters more than raw speed */
static inline int64_t q_square_wide(q16_t v)
{
    return (int64_t)v * (int64_t)v;
}

#endif /* FIXED_POINT_H */
