/*! ***************************************************************************
 *
 * \brief     Library of functions for calculating data filters
 * \file      filters.c
 * \author    Jeroen Veen - HAN Embedded Systems Engineering
 * \author    Hugo Arends - HAN Embedded Systems Engineering
 * \date      March 2024
 *
 * \see       Wikipedia contributors. (2023, August 24). Finite impulse
 *            response. In Wikipedia, The Free Encyclopedia.
 *            Retrieved March 22, 2024, from
 *            https://en.wikipedia.org/wiki/Finite_impulse_response
 *
 * \copyright 2024 HAN University of Applied Sciences. All Rights Reserved.
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

#include "filters.h"

/*!
 * \brief FIR filtered data
 *
 * The Finite Impulse Response filter is calculated with the following formula:
 * x' = c_0*x[n-0] + c_1*x[n-1] + c_2*x[n-2] + ... + c_m*x[n-m], where n is the
 * nth data sample, c_ is the coefficient, and m is the total number of
 * coefficients.
 *
 * Coefficients can be designed by using tools such as the firwin() method from
 * the scipy Signal processing library. An example for a low pass filter is
 * provided in ./tools/preprocessing/filter_selection/for_coefs_calculator.py
 *
 * Input parameters are not checked for validity in order to maximize
 * performance.
 *
 * \param[in]     data   Data sample
 * \param[in]     coefs  The coefficients of the FIR filter
 * \param[inout]  x      Pointer to an array to store the intermediate results
 * \param[in]     n      The number of coefficients of the FIR filter
 *
 * \return FIR filtered data sample
 */
float fir(const float data, const float *coefs, float *x, const uint32_t n)
{
    float y = 0.0f;
    
    // Shift the data samples and calculate its contribution
    for(int i = (n - 1); i > 0; --i)
    {
        x[i] = x[i-1];
        y = y + (coefs[i] * x[i]);
    }
    
    // Add new data sample and calculate its contribution
    x[0] = data;
    y = y + (coefs[0] * x[0]);
    
    return y;
}
