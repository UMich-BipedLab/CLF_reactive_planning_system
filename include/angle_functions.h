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
#ifndef ANGLE_FUNCTIONS_H
#define ANGLE_FUNCTIONS_H

#include <cmath>

namespace bipedlab
{
inline double deg_to_rad(double angle)
{
    return angle * M_PI / 180.0;
}

inline double rad_to_deg(double angle)
{
    return angle * 180.0 / M_PI;
}

/**
 * wrap_to_pi takes an angle of arbitrary size and reduces it to the range [-PI, PI].
 *
 * \param    angle           Angle to wrap
 * \return   Equivalent angle in range [-PI, PI].
 */
inline float wrap_to_pi(float angle)
{
    if (angle < -M_PI) {
        for (; angle < -M_PI; angle += 2.0 * M_PI)
            ;
    } else if (angle > M_PI) {
        for (; angle > M_PI; angle -= 2.0 * M_PI)
            ;
    }

    return angle;
}

/**
 * wrap_to_2pi takes an angle of arbitrary sizes and reduces it to the range [0, 2PI].
 *
 * \param    angle           Angle to wrap
 * \return   Equivalent angle in range [0, 2PI].
 */
inline float wrap_to_2pi(float angle)
{
    if (angle < 0) {
        for (; angle < 0; angle += 2.0 * M_PI)
            ;
    } else if (angle > 2 * M_PI) {
        for (; angle > 2 * M_PI; angle -= 2.0 * M_PI)
            ;
    }

    return angle;
}

/**
 * wrap_to_pi_2 takes an arbitrary angle and wraps it to the range [-pi/2,pi/2]. This function is intended
 * for use with lines where the direction doesn't matter, e.g. where something like 3pi/4 == -pi/4 like the
 * slope of a line.
 *
 * \param    angle           Angle to wrap
 * \return   Angle in the range [-pi/2,pi/2].
 */
inline float wrap_to_pi_2(float angle)
{
    float wrapped = wrap_to_pi(angle);

    if (wrapped < -M_PI_2) {
        wrapped += M_PI;
    } else if (wrapped > M_PI_2) {
        wrapped -= M_PI;
    }

    return wrapped;
}

/**
 * angle_diff finds the difference in radians between two angles and ensures that the differences
 * falls in the range of [-PI, PI].
 *
 * \param    leftAngle           Angle on the left-handside of the '-'
 * \param    rightAngle          Angle on the right-handside of the '-'
 * \return   The difference between the angles, leftAngle - rightAngle, in the range [-PI, PI].
 */
inline double angle_diff(double leftAngle, double rightAngle)
{
    double diff = leftAngle - rightAngle;
    if (fabs(diff) > M_PI) {
        diff -= (diff > 0) ? M_PI * 2 : M_PI * -2;
    }

    return diff;
}

/**
 * angle_diff_abs finds the absolute value of the difference in radians between two angles and ensures that the
 * differences falls in the range of [0, PI].
 *
 * \param    leftAngle           Angle on the left-handside of the '-'
 * \param    rightAngle          Angle on the right-handside of the '-'
 * \return   The absolute value of the difference between the angles, leftAngle - rightAngle, in the range [0, PI].
 */
inline double angle_diff_abs(double leftAngle, double rightAngle)
{
    return fabs(angle_diff(leftAngle, rightAngle));
}


/**
 * angle_diff_abs_pi_2 finds the difference in radians between two angles where the range is [0, PI/2].
 * The calculation is the angle_diff. If the abs(diff) > M_PI/2, then the result is PI - abs(diff). This
 * function is intended for cases where the angles represent line slopes rather than vectors. The slope
 * only can differ by at most PI/2.
 *
 * \param    lhs         Angle on left of '-'
 * \param    rhs         Angle on right of '-'
 * \return   lhs - rhs, in the range [0, PI/2].
 */
inline double angle_diff_abs_pi_2(double lhs, double rhs)
{
    double diff = std::abs(angle_diff(lhs, rhs));

    return (diff < M_PI / 2.0) ? diff : M_PI - diff;
}


/**
 * angle_sum finds the sum in radians of two angles and ensures the value is in the range [-PI, PI].
 *
 * \param    angleA              First angle in the sum
 * \param    angleB              Second angle in the sum
 * \return   The sum of the two angles, angleA + angleB, in the range [-PI, PI].
 */
inline double angle_sum(double angleA, double angleB)
{
    double sum = angleA + angleB;

    if (fabs(sum) > M_PI) {
        sum -= (sum > 0) ? M_PI * 2 : M_PI * -2;
    }

    return sum;
}


}   // namespace bipedlab

#endif   // ANGLE_FUNCTIONS_H
