#ifndef _MATRIX_H_
#define _MATRIX_H_

#include <stdint.h>
#include "Kalman/compiler.h"

/*!
 * \def INLINE_MATRIX Helper inline to switch from local inline to extern inline
 */
#ifndef INLINE_MATRIX
#define INLINE_MATRIX extern inline
#endif

/**
 * Matrix data type definition.
 */
typedef float matrix_data_t;

/**
 * \brief Matrix definition
 */
typedef struct
{
    /**
     * \brief Number of rows
     */
    uint_fast8_t rows;

    /**
     * \brief Number of columns
     */
    uint_fast8_t cols;

    /**
     * \brief Pointer to the data array of size {\see rows} x {\see cols}.
     */
    matrix_data_t *data;
} matrix_t;

/**
 * \brief Initializes a matrix structure.
 * \param[in] mat The matrix to initialize
 * \param[in] rows The number of rows
 * \param[in] cols The number of columns
 * \param[in] buffer The data buffer (of size {\see rows} x {\see cols}).
 */
void matrix_init(matrix_t *const mat, const uint_fast8_t rows, const uint_fast8_t cols, matrix_data_t *const buffer);

void matrix_setZero(matrix_t *const mat);

/**
 * \brief Inverts a lower triangular matrix.
 * \param[in] lower The lower triangular matrix to be inverted.
 * \param[in] inverse The calculated inverse of the lower triangular matrix.
 *
 * Kudos: https://code.google.com/p/efficient-java-matrix-library
 */
void matrix_invert_lower(const matrix_t *RESTRICT const lower, matrix_t *RESTRICT inverse) HOT;

/*!
 * \brief Performs a matrix multiplication such that {\ref c} = {\ref x} * {\ref b}
 * \param[in] a Matrix A
 * \param[in] x Vector x
 * \param[in] c Resulting vector C (will be overwritten)
 * \param[in] aux Auxiliary vector that can hold a column of {\ref b}
 *
 * Kudos: https://code.google.com/p/efficient-java-matrix-library
 */
void matrix_mult_rowvector(const matrix_t *RESTRICT const a, const matrix_t *RESTRICT const x, matrix_t *RESTRICT const c) HOT;

/*!
 * \brief Performs a matrix multiplication such that {\ref c} = {\ref c} + {\ref x} * {\ref b}
 * \param[in] a Matrix A
 * \param[in] x Vector x
 * \param[in] c Resulting vector C (will be added to)
 * \param[in] aux Auxiliary vector that can hold a column of {\ref b}
 *
 * Kudos: https://code.google.com/p/efficient-java-matrix-library
 */
void matrix_multadd_rowvector(const matrix_t *RESTRICT const a, const matrix_t *RESTRICT const x, matrix_t *RESTRICT const c) HOT;

/*!
 * \brief Performs a matrix multiplication such that {\ref c} = {\ref a} * {\ref b}
 * \param[in] a Matrix A
 * \param[in] b Matrix B
 * \param[in] c Resulting matrix C (will be overwritten)
 * \param[in] aux Auxiliary vector that can hold a column of {\ref b}
 *
 * Kudos: https://code.google.com/p/efficient-java-matrix-library
 */
void matrix_mult(const matrix_t *const a, const matrix_t *const b, const matrix_t *RESTRICT c, matrix_data_t *const baux) HOT;

/*!
 * \brief Performs a matrix multiplication with transposed B such that {\ref c} = {\ref a} * {\ref b'}
 * \param[in] a Matrix A
 * \param[in] b Matrix B
 * \param[in] c Resulting matrix C (will be overwritten)
 *
 * Kudos: https://code.google.com/p/efficient-java-matrix-library
 */
void matrix_mult_transb(const matrix_t *const a, const matrix_t *const b, const matrix_t *RESTRICT c) HOT;

/*!
 * \brief Performs a matrix multiplication with transposed B and adds the result to {\ref c} such that {\ref c} = {\ref c} + {\ref a} * {\ref b'}
 * \param[in] a Matrix A
 * \param[in] b Matrix B
 * \param[in] c Resulting matrix C (will be added to)
 *
 * Kudos: https://code.google.com/p/efficient-java-matrix-library
 */
void matrix_multadd_transb(const matrix_t *const a, const matrix_t *const b, const matrix_t *RESTRICT c) HOT;

/*!
 * \brief Performs a matrix multiplication with transposed B and scales the result such that {\ref c} = {\ref a} * {\ref b'} * {\ref scale}
 * \param[in] a Matrix A
 * \param[in] b Matrix B
 * \param[in] scale Scaling factor
 * \param[in] c Resulting matrix C(will be overwritten)
 *
 * Kudos: https://code.google.com/p/efficient-java-matrix-library
 */
void matrix_multscale_transb(const matrix_t *const a, const matrix_t *const b, register const matrix_data_t scale, const matrix_t *RESTRICT c) HOT;

void print_matrix(const matrix_t *a);

/*!
 * \brief Gets a matrix element
 * \param[in] mat The matrix to get from
 * \param[in] rows The row
 * \param[in] cols The column
 * \return The value at the given cell.
 */
matrix_data_t matrix_get(const matrix_t *const mat, const register uint_fast8_t row, const register uint_fast8_t column) PURE;

/*!
 * \brief Sets a matrix element
 * \param[in] mat The matrix to set
 * \param[in] rows The row
 * \param[in] cols The column
 * \param[in] value The value to set
 */
void matrix_set(matrix_t *mat, const register uint_fast8_t row, const register uint_fast8_t column, const register matrix_data_t value);

/*!
 * \brief Sets matrix elements in a symmetric matrix
 * \param[in] mat The matrix to set
 * \param[in] rows The row
 * \param[in] cols The column
 * \param[in] value The value to set
 */
void matrix_set_symmetric(matrix_t *mat, const register uint_fast8_t row, const register uint_fast8_t column, const register matrix_data_t value);

/*!
 * \brief Gets a pointer to a matrix row
 * \param[in] mat The matrix to get from
 * \param[in] rows The row
 * \param[out] row_data A pointer to the given matrix row
 */
void matrix_get_row_pointer(const matrix_t *const mat, const register uint_fast8_t row, matrix_data_t **row_data);

/*!
 * \brief Gets a copy of a matrix column
 * \param[in] mat The matrix to initialize
 * \param[in] rows The column
 * \param[in] row_data Pointer to an array of the correct length to hold a column of matrix {\ref mat}.
 */
void matrix_get_column_copy(const matrix_t *const mat, const register uint_fast8_t column, register matrix_data_t *const row_data) HOT;

/*!
 * \brief Gets a copy of a matrix row
 * \param[in] mat The matrix to initialize
 * \param[in] rows The row
 * \param[in] row_data Pointer to an array of the correct length to hold a row of matrix {\ref mat}.
 */
void matrix_get_row_copy(const matrix_t *const mat, const register uint_fast8_t row, register matrix_data_t *const row_data);

/*!
 * \brief Copies the matrix from {\ref mat} to {\ref target}
 * \param[in] mat The matrix to copy
 * \param[in] target The matrix to copy to
 */
void matrix_copy(const matrix_t *const mat, matrix_t *const target);

/*!
 * \brief Subtracts two matrices, using {\ref c} = {\ref a} - {\ref b}
 * \param[in] a The matrix to subtract from
 * \param[in] b The values to subtract
 * \param[in] c The output
 */
void matrix_sub(const matrix_t *const a, matrix_t *const b, const matrix_t *c) HOT;

/*!
 * \brief Subtracts two matrices in place, using {\ref b} = {\ref a} - {\ref b}
 * \param[in] a The matrix to subtract from
 * \param[in] b The values to subtract, also the output
 */
void matrix_sub_inplace_b(const matrix_t *RESTRICT const a, const matrix_t *RESTRICT b) HOT;

/*!
 * \brief Adds two matrices in place, using {\ref a} = {\ref a} + {\ref b}
 * \param[in] a The matrix to add to, also the output
 * \param[in] b The values to add
 */
void matrix_add_inplace(const matrix_t *a, const matrix_t *const b) HOT;

/*!
 * \brief Adds two matrices, using {\ref c} = {\ref a} + {\ref b}
 * \param[in] a The matrix to add to
 * \param[in] b The values to add
 * \param[in] c The output
 */
void matrix_add(const matrix_t *a, const matrix_t *const b, const matrix_t *c) HOT;

void matrix_print(const matrix_t *a);
#undef EXTERN_INLINE_MATRIX
#endif
