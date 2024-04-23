/*! ***************************************************************************
 *
 * \brief     Library of functions for calculating data normalization
 * \file      normalizations.c
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

#include "normalizations.h"

/*!
 * \brief Normalizes the input data by rescaling it
 *
 * Rescaling is also known as min-max scaling. It converts data values from
 * their natural range into a standard range with the following formula:
 * x' = x * (to_max - to_min)/(from_max - from_min) where x is the input data,
 * to_max and to_min define the output range, and from_max and from_min define
 * the input range.
 *
 * One advantage of range scaling is that it is relative easy to calculate. A
 * disadvantage of range scaling is that it does not handle outliers very well.
 *
 * Input parameters are not checked for validity in order to maximize
 * performance.
 *
 * \param[in]  data Data item to be rescaled
 * \param[in]  from A pointer to two values: min and max value of the natural
 *                  range. The first value in the array is min.
 * \param[in]  to   A pointer to two values: min and max value of the output 
 *                  range. The first value in the array is min.
 *
 * \return Rescaling normalized data
 */
float rescale(const float data, const float from[2], const float to[2])
{
    float rescale_factor = (to[1] - to[0]) / (from[1] - from[0]);
    return ((data - from[0]) * rescale_factor) + to[0];
}

/*!
 * \brief Normalizes the input data by clipping it
 *
 * Clipping is used to cap outliers in the data.
 *
 * Input parameters are not checked for validity in order to maximize
 * performance.
 *
 * \param[in]  data Data item to be rescaled
 * \param[in]  min  A pointer to one value: minimum data value
 * \param[in]  max  A pointer to one value: maximum data value
 *
 * \return Clipped normalized data
 */
float clip(const float data, const float min[1], const float max[1])
{
    float val = (data < *min) ? *min : data;
    return (val > *max) ? *max : val;
}
