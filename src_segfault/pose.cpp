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
 * \file     pose.cpp
 * \author   Bruce JK Huang, Collin Johnnson
 *
 * Definition of helper functions for pose2d_t.
 */

#include "pose.h"
#include "angle_functions.h"
#include "float_comparision.h"
#include "pose_distribution.h"
#include <cassert>

namespace bipedlab
{

pose2d_t::pose2d_t(const pose_distribution_t& distribution)
: timestamp(distribution.timestamp)
, x(distribution.x)
, y(distribution.y)
, theta(distribution.theta)
{
}


pose2d_t pose2d_t::transformToNewFrame(const pose2d_t& newFrame) const
{
    point2d_t<float> newPosition = transform(toPoint(), -newFrame.x, -newFrame.y, -newFrame.theta);

    return pose2d_t(newPosition.x, newPosition.y, angle_diff(theta, newFrame.theta));
}


pose2d_t pose2d_t::compound(const pose2d_t& origin) const
{
    point2d_t<float> newPosition = homogeneous_transform(toPoint(), origin.x, origin.y, origin.theta);
    return pose2d_t(timestamp, newPosition.x, newPosition.y, angle_sum(theta, origin.theta));
}


bool operator==(const pose2d_t& lhs, const pose2d_t& rhs)
{
    return lazy_equal::absolute_fuzzy_equal(lhs.x, rhs.x) && 
           lazy_equal::absolute_fuzzy_equal(lhs.y, rhs.y) && 
           lazy_equal::absolute_fuzzy_equal(lhs.theta, rhs.theta);
}


bool operator!=(const pose2d_t& lhs, const pose2d_t& rhs)
{
    return !(lhs == rhs);
}


std::ostream& operator<<(std::ostream& out, const pose2d_t& pose)
{
    out << '(' << pose.x << ',' << pose.y << ',' << pose.theta << ')';
    return out;
}


std::ostream& operator<<(std::ostream& out, const pose_6dof_t& pose)
{
    out << '(' << pose.x << ',' << pose.y << ',' << pose.z << ',' << pose.phi << ',' << pose.rho << ',' << pose.theta
        << ')';
    return out;
}


pose2d_t interpolate_pose(const pose2d_t& priorPose, const pose2d_t& currentPose, int64_t desiredPoseTime)
{
    assert((priorPose.timestamp <= desiredPoseTime) && (desiredPoseTime <= currentPose.timestamp));

    if (priorPose.timestamp == currentPose.timestamp) {
        return priorPose;
    }

    double scale =
      static_cast<double>(desiredPoseTime - priorPose.timestamp) / (currentPose.timestamp - priorPose.timestamp);

    pose2d_t interpolatedPose;

    interpolatedPose.timestamp = desiredPoseTime;
    interpolatedPose.x = priorPose.x + (currentPose.x - priorPose.x) * scale;
    interpolatedPose.y = priorPose.y + (currentPose.y - priorPose.y) * scale;
    interpolatedPose.theta = wrap_to_pi(priorPose.theta + angle_diff(currentPose.theta, priorPose.theta) * scale);

    return interpolatedPose;
}

}   // namespace cassie

