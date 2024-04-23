/*! ***************************************************************************
 *
 * \brief     Library of functions for calculating data features
 * \file      features.c
 * \author    Jeroen Veen - HAN Embedded Systems Engineering
 * \author    Hugo Arends - HAN Embedded Systems Engineering
 * \date      July 2023
 *
 * \see       STMicroelectronics (2022). AN5804 Machine learning core in the
 *            LSM6DSV16X.
 *
 * \copyright 2023 HAN University of Applied Sciences. All Rights Reserved.
 *            \n\n
 *            Permission is hereby granted, free of charge, to any person
 *            obtaining a copy of this software and associated documentation
 *            files (the "Software"), to deal in the Software without
 *            restriction, including without limitation the rights to use,
 *            copy, modify, merge, publish, distribute, sublicense, and/or sell
 *            copies of the Software, and to permit persons to whom the
 *            Software is furnished to do so, subject to the following
 *            conditions:
 *            \n\n
 *            The above copyright notice and this permission notice shall be
 *            included in all copies or substantial portions of the Software.
 *            \n\n
 *            THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 *            EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 *            OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 *            NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
 *            HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 *            WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 *            FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 *            OTHER DEALINGS IN THE SOFTWARE.
 *
 *****************************************************************************/

#include "features.h"

/*!
 * \brief Finds the minimum value in the input data
 *
 * The feature minimum finds the minimum value in the input data.
 *
 *
 *         .
 *       .   .
 *     .       .
 *    .          .                        .
 *                .                    .     .           . .
 *                 .                .           .      .     .
 *                  .            .                  .
 *                    .       .
 *                      .   .
 *                        X
 *
 *  ---------------------------------------------------------- ->t
 *    |                                                   |
 *    |<------------------- n samples ------------------->|
 *
 * Input parameters are not checked for validity in order to maximize
 * performance.
 *
 * \param[in]  data A pointer to the data array
 * \param[in]  n    The number of data items in the array
 *
 * \return The minimum value in the input data
 */
float min(float *data, const uint32_t n)
{
    float min = data[0];

    for(uint32_t i=1; i<n; ++i)
    {
        min = (data[i] < min) ? data[i] : min;
    }

    return min;
}

/*!
 * \brief Finds the maximum value in the input data
 *
 * The feature maximum finds the maximum value in the input data.
 *
 *
 *         X
 *       .   .
 *     .       .
 *    .          .                        .
 *                .                    .     .           . .
 *                 .                .           .      .     .
 *                  .            .                  .
 *                    .       .
 *                      .   .
 *                        .
 *
 *  ---------------------------------------------------------- ->t
 *    |                                                   |
 *    |<------------------- n samples ------------------->|
 *
 * Input parameters are not checked for validity in order to maximize
 * performance.
 *
 * \param[in]  data A pointer to the data array
 * \param[in]  n    The number of data items in the array
 *
 * \return The maximum value in the input data
 */
float max(float *data, const uint32_t n)
{
    float max = data[0];

    for(uint32_t i=1; i<n; ++i)
    {
        max = (data[i] > max) ? data[i] : max;
    }

    return max;
}

/*!
 * \brief Computes the avarage of the input data
 *
 * The feature mean computes the average of the input data with the following
 * formula: mean = (1/n) * (d_0 + d_1 + ... + d_(n-1)) where d_x is a data item
 * at position x in the input array.
 *
 * Input parameters are not checked for validity in order to maximize
 * performance.
 *
 * \param[in]  data A pointer to the data array
 * \param[in]  n    The number of data items in the array
 *
 * \return The avarage of the input data
 */
float mean(float *data, const uint32_t n)
{
    float sum = 0.0f;

    for(uint32_t i=0; i<n; ++i)
    {
        sum += data[i];
    }

    return sum / (float)n;
}

/*!
 * \brief Computes the variance of the input data
 *
 * Variance is the measure of how far the input data is spread from the average
 * value of that same input data.
 *
 * The feature variance computes the variance of the input data with the
 * following formula: variance = ((d_0 - mean)^2 + (d_1 - mean)^2 + ... +
 * (d_(n-1) - mean)^2) / n  where d_x is a data item at position x in the
 * input array.
 *
 * Input parameters are not checked for validity in order to maximize
 * performance.
 *
 * \param[in]  data A pointer to the data array
 * \param[in]  n    The number of data items in the array
 *
 * \return The variance of the input data
 */
float variance(float *data, const uint32_t n)
{
    float m = mean(data, n);

    float sq_diff = 0.0f;

    for(uint32_t i=0; i<n; ++i)
    {
        sq_diff += (data[i] - m) * (data[i] - m);
    }

    return sq_diff / (float)n;
}

/*!
 * \brief Computes the energy of the input data
 *
 * The energy of the input data is defined as the sum of squares. It is a
 * measure for the amount of variation in the input data.
 *
 * The feature energy computes the energy of the input data with the
 * following formula: energy =  (d_0^2 + d_1^2 + ... + d_(n-1)^2) where d_x is
 * a data item at position x in the input array.
 *
 * Input parameters are not checked for validity in order to maximize
 * performance.
 *
 * \param[in]  data A pointer to the data array
 * \param[in]  n    The number of data items in the array
 *
 * \return The energy of the input data
 */
float energy(float *data, const uint32_t n)
{
    float energy = 0.0f;

    for(uint32_t i=0; i<n; ++i)
    {
        energy += (data[i] * data[i]);
    }

    return energy;
}

/*!
 * \brief Computes the maximum peak-to-peak value of the input data
 *
 * The feature peak-to-peak computes the maximum peak-to-peak value of the
 * input data.
 *
 *        max
 *  - - - -X- - - - - - - - - - - - - - - - - - - - - - - - -    -
 *       .   .                                                   ^     peak
 *     .       .                                                 |     -to-
 *    .          .                        .                      |     peak
 *                .                    .     .           . .     |
 *                 .                .           .      .     .   |      =
 *                  .            .                  .            |
 *                    .       .                                  |  max - min
 *                      .   .                                    v
 *  - - - - - - - - - - - X - - - - - - - - - - - - - - - - -    -
 *                       min
 *  ---------------------------------------------------------- ->t
 *    |                                                   |
 *    |<------------------- n samples ------------------->|
 *
 * Input parameters are not checked for validity in order to maximize
 * performance.
 *
 * \param[in]  data A pointer to the data array
 * \param[in]  n    The number of data items in the array
 *
 * \return The maximum peak-to-peak value of the input data
 */
float peak_to_peak(float *data, const uint32_t n)
{
    float min = data[0];
    float max = data[0];

    for(uint32_t i=1; i<n; ++i)
    {
        min = (data[i] < min) ? data[i] : min;
        max = (data[i] > max) ? data[i] : max;
    }

    return max - min;
}
