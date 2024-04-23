/*! ***************************************************************************
 *
 * \brief     Library of functions for calculating data normalization
 * \file      normalizations.h
 * \author    Jeroen Veen - HAN Embedded Systems Engineering
 * \author    Hugo Arends - HAN Embedded Systems Engineering
 * \date      March 2024
 *
 * \see       Google developers. (n.d.). Machine Learning - Normalization. 
 *            Retrieved March 20, 2024, from
 *            https://developers.google.com/machine-learning/data-prep/transform/normalization
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
#ifndef _NORMALIZATION_H_
#define _NORMALIZATION_H_

#include <stdint.h>

// Functions are documented in the source file

float rescale(const float data, const float from[2], const float to[2]);
float clip(const float data, const float min[1], const float max[1]);

#endif // _NORMALIZATION_H_

#ifdef __cplusplus
}
#endif
