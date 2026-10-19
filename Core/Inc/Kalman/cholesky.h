/* Numerical support adapted from sunsided/kalman-clib.
 * Copyright (c) 2014-2015 Markus Mayer and contributors.
 * MIT license: see LICENSES/kalman-clib-MIT.txt and THIRD_PARTY.md.
 * Original EJML credits below are retained.
 */
#ifndef _CHOLESKY_H_
#define _CHOLESKY_H_

#include "Kalman/compiler.h"
#include "Kalman/matrix.h"

/**
* \brief Decomposes a matrix into lower triangular form using Cholesky decomposition.
* \param[in] mat The matrix to decompose in place into a lower triangular matrix.
* \return Zero in case of success, nonzero if the matrix is not positive semi-definite.
*
* Kudos: https://code.google.com/p/efficient-java-matrix-library
*/
int cholesky_decompose_lower(register const matrix_t *const mat) HOT;

#endif
