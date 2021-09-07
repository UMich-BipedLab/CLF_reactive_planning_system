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
 * \file     lyapunov_distance.cpp
 * \author   Bruce JK Huang, Jong Jin Park
 *
 * Definition of LyapunovDistance.
 */

#include "lyapunov_distance.h"
//#include "nonholonomic_distance.h"

namespace bipedlab
{

LyapunovDistance::LyapunovDistance(const pose_t& target_pose,
                                   const lyapunov_distance_params_t& params)
: params_(params), 
  target_pose_(target_pose),
  local_chart_(target_pose)
{ }

LyapunovDistance::LyapunovDistance(const lyapunov_distance_params_t& params)
: params_(params) { }



LyapunovDistance::LyapunovDistance(void) { }

void LyapunovDistance::assignNewTargetPose(const pose_t& new_target_pose)
{
    target_pose_ = new_target_pose;
    local_chart_.assignNewTargetPose(new_target_pose);
}


double LyapunovDistance::stabilizeDeltaStar(const point2d_t<float> point) const
{
    reduced_egocentric_polar_coords_t coords = 
        local_chart_.point2rp(point);

    return stabilizing_delta_star(coords.r, coords.phi, params_.k_phi, 
            params_.vector_field_type, params_.small_radius);
}

double LyapunovDistance::stabilizeDeltaStarWithTargetPose(
        const point2d_t<float> point, const pose_t& target_pose) 
{
    LyapunovDistance::assignNewTargetPose(target_pose);
    return LyapunovDistance::stabilizeDeltaStar(point);
}


double LyapunovDistance::stabilizeHeading(const point2d_t<float> point) const
{
    reduced_egocentric_polar_coords_t coords = 
        local_chart_.point2rp(point);
    double deltaStar =
      stabilizing_delta_star(coords.r, coords.phi, params_.k_phi, 
              params_.vector_field_type, params_.small_radius);

    return deltaStar + coords.lineOfSightAngle;
}

double LyapunovDistance::stabilizeHeadingWithTargetPose(
        const point2d_t<float> point, const pose_t& target_pose) 
{
    LyapunovDistance::assignNewTargetPose(target_pose);

    return LyapunovDistance::stabilizeHeading(point);
}


double LyapunovDistance::computeDistanceFromPoint(
        const point2d_t<float>& point) const
{
    reduced_egocentric_polar_coords_t coords = 
        local_chart_.point2rp(point);

    return distance_on_manifold(coords.r, coords.phi, params_.k_phi);
}

double LyapunovDistance::computeDistanceFromPointWithTargetPose(
        const point2d_t<float>& point, const pose_t& target_pose) 
{

    LyapunovDistance::assignNewTargetPose(target_pose);

    return LyapunovDistance::computeDistanceFromPoint(point);
}

double LyapunovDistance::computeDistanceFromPose(const pose_t& pose) const
{
    egocentric_polar_coords_t coords = local_chart_.pose2rpd(pose);
    debugger::debugOutput("[Lyap Dis] target pose: ", target_pose_, 0);
    debugger::debugOutput("[Lyap Dis] coords.r: ", coords.r, 0);
    debugger::debugOutput("[Lyap Dis] coords.phi: ", coords.phi, 0);
    debugger::debugOutput("[Lyap Dis] coords.delta: ", coords.delta, 0);
    debugger::debugOutput("[Lyap Dis] coords.line_of_sight_angle: ", 
            coords.lineOfSightAngle, 0);

    return nonholonomic_distance(coords.r,
                                 coords.phi,
                                 coords.delta,
                                 params_.k_phi,
                                 params_.k_delta,
                                 params_.vector_field_type,
                                 params_.small_radius);
}

double LyapunovDistance::computeDistanceFromPoseWithTargetPose(
        const pose_t& pose, const pose_t& target_pose) 
{
    LyapunovDistance::assignNewTargetPose(target_pose);
    
    return LyapunovDistance::computeDistanceFromPose(pose);
}


}   // namespace bipedlab

