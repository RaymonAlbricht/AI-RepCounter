/*! ***************************************************************************
 *
 * \brief     Library of functions for calculating data filters
 * \file      filters.h
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

#ifdef __cplusplus
extern "C" {
#endif

/// Include guard to prevent recursive inclusion
#ifndef _FILTERS_H_
#define _FILTERS_H_

#include <stdint.h>

// Functions are documented in the source file

float fir(const float data, const float *coefs, float *x, const uint32_t n);

#endif // _FILTERS_H_

#ifdef __cplusplus
}
#endif
