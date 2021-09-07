/* Copyright (C) 2013-2020, The Regents of The University of Michigan.
 * All rights reserved.
 * This software was developed in the Biped Lab (https://www.biped.solutions/)
 * under the direction of Jessy Grizzle, grizzle@umich.edu. This software may
 * be available under alternative licensing terms; contact the address above.
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * The views and conclusions contained in the software and documentation are those
 * of the authors and should not be interpreted as representing official policies,
 * either expressed or implied, of the Regents of The University of Michigan.
 *
 * AUTHOR: Bruce JK Huang (bjhuang@umich.edu)
 * WEBSITE: https://www.BrucebotStudio.com/
 */
/**
 * \file     float_comparison.h
 * \author   Bruce JK Huang, Collin Johnson
 *
 * float_comparison.h contains functions for doing fuzzy comparisons of floating
 * points numbers. These functions are useful for operations sensitive to floating
 * point error.
 */

#ifndef FLOAT_COMPARISON_H
#define FLOAT_COMPARISON_H

#include <cmath>

namespace bipedlab
{

const double MAX_ABSOLUTE_ERROR_FLOAT = 0.0001;
const double MAX_RELATIVE_ERROR_FLOAT = 0.0001;
namespace lazy_equal
{
    

/**
 * absolute_fuzzy_equal compares two floating point for absolute error. Absolute
 * error means the difference between two values is within a fixed threshold.
 *
 * See http://www.cygnus-software.com/papers/comparingfloats/comparingfloats.htm for
 * fantastic detail.
 */
template <typename T, typename U>
inline bool absolute_fuzzy_equal(T x, U y, double error = MAX_ABSOLUTE_ERROR_FLOAT)
{
    return fabs(x - y) < error;
}

/**
 * relative_fuzzy_equal compares the relative error between two floating point
 * numbers. If the difference is less than some threshold, the numbers are considered
 * to be equal.
 *
 * See http://www.cygnus-software.com/papers/comparingfloats/comparingfloats.htm for
 * fantastic detail.
 */
template <typename T, typename U>
inline bool relative_fuzzy_equal(T x, U y, double error = MAX_RELATIVE_ERROR_FLOAT)
{
    if (x == y) {
        return true;
    }

    return (fabs(x - y) / y <= error) || (fabs(x - y) / x <= error);
}

/**
 * round_in_tolerance takes the ceiling a value if it is within some tolerance of the next integer. Otherwise, the
 * floor is returned.
 */
template <typename T>
inline T round_in_tolerance(T value, double error = MAX_ABSOLUTE_ERROR_FLOAT)
{
    T ceiling = ceil(value);
    if (std::abs(value - ceiling) < MAX_ABSOLUTE_ERROR_FLOAT) {
        return ceiling;
    }

    return floor(value);
}

} /* laze_equal */ 
}   // namespace bipedlab

#endif   // FLOAT_COMPARISON_H

