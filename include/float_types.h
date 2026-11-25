/** @file float_types.h
 *
 * @brief Named floating-point widths (BARR-C:2018 rule 5.4.b.i).
 *
 * C99 and the toolchain's math.h do not define these names, so the project
 * does, and checks the widths they promise.
 */

#ifndef FLOAT_TYPES_H
#define FLOAT_TYPES_H

typedef float  float32_t;
typedef double float64_t;

_Static_assert(4U == sizeof(float32_t), "float32_t is not 32 bits");
_Static_assert(8U == sizeof(float64_t), "float64_t is not 64 bits");

#endif /* FLOAT_TYPES_H */

/*** end of file ***/
