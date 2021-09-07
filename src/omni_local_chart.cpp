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
 * \file     local_chart.cpp
 * \author   Bruce JK Huang, Jong Jin Park
 *
 * Definition of OmniLocalChart, a collection of (bijective) mappings between
 * Cartesian and egocentric polar coordinates, anchored around a pose on a
 * plane.
 */

#include "omni_local_chart.h"
#include "utils/utils.h"
#include "standalone_omni_local_chart.h"
#include "standalone_omni_lyapunov_distance.h"

// #include "angle_functions.h"

namespace bipedlab
{

OmniLocalChart::OmniLocalChart(const position_t& target_position, double small_radius)
: target_position_(target_position)
, small_radius_(small_radius)
{
}

OmniLocalChart::OmniLocalChart(void)
: small_radius_(0.001)
{
}

void OmniLocalChart::assignNewTargetPosition(const position_t& new_target_position)
{
    target_position_ = new_target_position;
}

omni_line_of_sight_t 
OmniLocalChart::lineOfSight(const pose_t& observer_pose) const
{

    return omni_local_chart::computeLineOfSight(
            observer_pose, target_position_, small_radius_);
}

reduced_omni_ego_polar_coords_t 
OmniLocalChart::convertPose2rd(const pose_t& start_pose) const
{

    return omni_local_chart::convertPose2rd(start_pose, target_position_);
}

reduced_omni_ego_polar_coords_t 
OmniLocalChart::convertPose2rdFoV(const pose_t& start_pose, const double beta) const
{

    reduced_omni_ego_polar_coords_t rd = 
        omni_local_chart::convertPose2rd(start_pose, target_position_);
    // debugger::debugColorOutput("==========", "", 5);
    // debugger::debugColorOutput("start: ", start_pose, 5);
    // debugger::debugColorOutput("end: ", target_position_, 5);
    // debugger::debugColorOutput("beta: ", beta, 5);
    // debugger::debugColorOutput("old r: ", rd.r, 5);
    // debugger::debugColorOutput("old theta: ", rd.delta, 5);

    std::pair<bool, double> res = 
        checkOutsideFovReturnModifiedDelta(rd.delta, beta);
    rd.delta = res.second;
    // debugger::debugColorOutput("status: ", res.first, 5);
    // debugger::debugColorOutput("new r: ", rd.r, 5);
    // debugger::debugColorOutput("new theta: ", rd.delta, 5);
    // if (res.first)
    //     utils::pressEnterToContinue();

    return rd;
    // double equilibria = computeOmniEquilibriaStandalone(beta);

    // if ((rd.delta >= 0) && (rd.delta > equilibria))
    //     rd.delta = equilibria;
    // else if ((rd.delta < 0) && (rd.delta < -equilibria))
    //     rd.delta = -equilibria;
    // return rd;
}


// use another end pose to compute polar coords but do not assign the target to
// the target_position_ in this class
reduced_omni_ego_polar_coords_t 
OmniLocalChart::convertPose2rdFoVGivenTargetNoAssign(
        const pose_t& start_pose, const pose_t& end_pose, 
        const double beta) const
{

    reduced_omni_ego_polar_coords_t rd = 
        omni_local_chart::convertPose2rd(start_pose, end_pose.to2DPosition());
    std::pair<bool, double> res = 
        checkOutsideFovReturnModifiedDelta(rd.delta, beta);
    rd.delta = res.second;

    return rd;
    // double equilibria = computeOmniEquilibriaStandalone(beta);
    // if ((rd.delta >= 0) && (rd.delta > equilibria))
    //     rd.delta = equilibria;
    // else if ((rd.delta < 0) && (rd.delta < -equilibria))
    //     rd.delta = -equilibria;
    // return rd;
}

// point2d_t<float> OmniLocalChart::rp2point(double r, double phi) const
// {
// 
//     // return omni_local_chart::rp2point(r, phi, target_position_);
// }
// 
// point2d_t<float> 
// OmniLocalChart::rp2point(const reduced_omni_ego_polar_coords_t& coords) const
// {
//     //return OmniLocalChart::rp2point(coords.r, coords.phi);
// }
// 
// omni_ego_polar_coords_t OmniLocalChart::pose2rpd(const pose_t& robot_pose) const
// {
// 
//     // return omni_local_chart::pose2rpd(robot_pose, target_position_);
// }
// 
// pose_t OmniLocalChart::rpd2pose(double r, double phi, double delta) const
// {
// 
//     // return omni_local_chart::rpd2pose(r, phi, delta, target_position_);
// }
// 
// pose_t OmniLocalChart::rpd2pose(const omni_ego_polar_coords_t& coords) const
// {
//     // return OmniLocalChart::rpd2pose(coords.r, coords.phi, coords.delta);
// }
// 
// pose_t OmniLocalChart::rpd2target(
//         double r, double phi, double delta, const pose_t& robot_pose) const
// {
// 
//     // return omni_local_chart::rpd2target(r, phi, delta, robot_pose);
// }
// 
// pose_t OmniLocalChart::rpd2target(
//         const omni_ego_polar_coords_t& coords, const pose_t& robot_pose) const
// {
//     // return omni_local_chart::rpd2target(coords.r, coords.phi, coords.delta, robot_pose);
// }

}   // namespace bipedlab
