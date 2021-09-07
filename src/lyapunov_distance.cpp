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
: params_(params) 
{ 
    // debugger::debugOutput("[Lyap Dis] params.clf_model: ", params_.clf_model, 5);
    // debugger::debugOutput("[Lyap Dis] params.k_phi: ", params_.k_phi, 5);
    // debugger::debugOutput("[Lyap Dis] params.k_delta: ", params_.k_delta, 5);
}



LyapunovDistance::LyapunovDistance(void) { }

void LyapunovDistance::assignNewTargetPose(const pose_t& new_target_pose)
{
    target_pose_ = new_target_pose;

    if (params_.clf_model == 0)
    {
        local_chart_.assignNewTargetPose(new_target_pose);
    }
    else if (params_.clf_model == 1)
    {
        omni_local_chart_.assignNewTargetPosition(new_target_pose.to2DPosition());
    }
}




control_variables_t LyapunovDistance::stabilizeControl(const pose_t& start_pose) const
{
    if (params_.clf_model == 0)
    {
        reduced_egocentric_polar_coords_t coords = 
            local_chart_.point2rp(start_pose.to2DPosition());
        double delta_star =
            stabilizing_delta_star(coords.r, coords.phi, params_.k_phi, 
                    params_.vector_field_type, params_.small_radius);
        double total_heading = delta_star + coords.lineOfSightAngle;

        return control_variables_t(std::cos(total_heading), 
                std::sin(total_heading), 0, 0, 0, total_heading);
    }
    else if (params_.clf_model == 1)
    {
        // In the case of robot's heading being outside the equilibria,
        // the robot will rotate in-place until the heading is inside the equilibria
        // Therefore, we dont need to consider this case and assume the heading is inside
        // the equilibria.
        reduced_omni_ego_polar_coords_t coords = 
            omni_local_chart_.convertPose2rdFoV(start_pose, params_.beta);

        control_variables_t optimal_control = 
            computeOmniStabilizingControl(coords.r, coords.delta,
                                          params_.alpha, params_.beta,
                                          params_.k_r1, params_.k_r2,
                                          params_.k_delta1, params_.k_delta2,
                                          params_.omni_CLF_solution);
        // optimal_control.heading = start_pose.theta;
        optimal_control.heading = 
            coords.line_of_sight_angle - coords.delta; // change to clipped delta (FOV)

        return optimal_control;
    }
}

control_variables_t 
LyapunovDistance::stabilizeOmniControl(
        const reduced_omni_ego_polar_coords_t& coords) const
{
    return computeOmniStabilizingControl(coords.r, coords.delta,
            params_.alpha, params_.beta,
            params_.k_r1, params_.k_r2,
            params_.k_delta1, params_.k_delta2,
            params_.omni_CLF_solution);
}

control_variables_t LyapunovDistance::stabilizeControlWithTargetPose(
        const pose_t& start_pose, const pose_t& target_pose) 
{
    LyapunovDistance::assignNewTargetPose(target_pose);

    return LyapunovDistance::stabilizeControl(start_pose);
}


double LyapunovDistance::computeDistanceFromPoint(
        const pose_t& start_pose) const
{
    if (params_.clf_model == 0)
    {
        reduced_egocentric_polar_coords_t coords = 
            local_chart_.point2rp(start_pose.to2DPosition());

        return distance_on_manifold(coords.r, coords.phi, params_.k_phi);
    }
    else if (params_.clf_model == 1)
    {
        reduced_omni_ego_polar_coords_t coords = 
            omni_local_chart_.convertPose2rd(start_pose);

        return computeOmniDistanceOnManifold(coords.r, coords.delta, params_.gamma, 
                params_.beta, params_.omni_CLF_model);
    }

}

double LyapunovDistance::computeDistanceFromPointWithTargetPose(
        const pose_t& start_pose, const pose_t& target_pose) 
{

    LyapunovDistance::assignNewTargetPose(target_pose);

    return LyapunovDistance::computeDistanceFromPoint(start_pose);
}

double LyapunovDistance::computeDistanceFromPose(const pose_t& pose) const
{
    if (params_.clf_model == 0)
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
    else if (params_.clf_model == 1)
    {
        reduced_omni_ego_polar_coords_t coords = 
            omni_local_chart_.convertPose2rd(pose);

        return computeOmniNonholonomicDistance(coords.r, coords.delta,
                params_.gamma, params_.beta, params_.k_delta_to_manifold, 
                params_.omni_CLF_model);
    }
}

double LyapunovDistance::computeDistanceFromPoseWithTargetPose(
        const pose_t& pose, const pose_t& target_pose) 
{
    LyapunovDistance::assignNewTargetPose(target_pose);

    return LyapunovDistance::computeDistanceFromPose(pose);
}


double LyapunovDistance::computeOmniEquilibria()
{
    return computeOmniEquilibriaStandalone(params_.beta);
}


/* NOT USED SO FAR*/
control_variables_t LyapunovDistance::stabilizeDeltaStar(const pose_t& start_pose) const
{
    if (params_.clf_model == 0)
    {
        reduced_egocentric_polar_coords_t coords = 
            local_chart_.point2rp(start_pose.to2DPosition());

        double delta_star =  stabilizing_delta_star(coords.r, coords.phi, params_.k_phi, 
                params_.vector_field_type, params_.small_radius);

        return control_variables_t(std::cos(delta_star), 
                std::sin(delta_star), 0, 0, 0, delta_star);
    }
    else if (params_.clf_model == 1)
    {
        reduced_omni_ego_polar_coords_t coords = 
            omni_local_chart_.convertPose2rd(start_pose);

        // return omni_stabilizing_delta_star(coords.r, coords.delta, params_.k_phi, 
        //         params_.omni_CLF_model, params_.small_radius);
    }
}

control_variables_t LyapunovDistance::stabilizeDeltaStarWithTargetPose(
        const pose_t& start_pose, const pose_t& target_pose) 
{
    LyapunovDistance::assignNewTargetPose(target_pose);
    return LyapunovDistance::stabilizeDeltaStar(start_pose);
}

}   // namespace bipedlab

