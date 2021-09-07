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
 * Definition of LocalChart, a collection of (bijective) mappings between
 * Cartesian and egocentric polar coordinates, anchored around a pose on a
 * plane.
 */

#include "local_chart.h"
#include "standalone_local_chart.h"

// #include "angle_functions.h"

namespace bipedlab
{

LocalChart::LocalChart(const pose_t& target_pose, double small_radius)
: target_pose_(target_pose)
, small_radius_(small_radius)
{
}

LocalChart::LocalChart(void)
: small_radius_(0.001)
{
}

void LocalChart::assignNewTargetPose(const pose_t& new_target_pose)
{
    target_pose_ = new_target_pose;
}

line_of_sight_t 
LocalChart::lineOfSight(const point2d_t<float>& observerPosition) const
{

    return local_chart::lineOfSight(observerPosition, target_pose_, small_radius_);
}

reduced_egocentric_polar_coords_t 
LocalChart::point2rp(const point2d_t<float>& point) const
{

    return local_chart::point2rp(point, target_pose_);
}

point2d_t<float> LocalChart::rp2point(double r, double phi) const
{

    return local_chart::rp2point(r, phi, target_pose_);
}

point2d_t<float> 
LocalChart::rp2point(const reduced_egocentric_polar_coords_t& coords) const
{
    return LocalChart::rp2point(coords.r, coords.phi);
}

egocentric_polar_coords_t LocalChart::pose2rpd(const pose_t& robot_pose) const
{

    return local_chart::pose2rpd(robot_pose, target_pose_);
}

pose_t LocalChart::rpd2pose(double r, double phi, double delta) const
{

    return local_chart::rpd2pose(r, phi, delta, target_pose_);
}

pose_t LocalChart::rpd2pose(const egocentric_polar_coords_t& coords) const
{
    return LocalChart::rpd2pose(coords.r, coords.phi, coords.delta);
}

pose_t LocalChart::rpd2target(
        double r, double phi, double delta, const pose_t& robot_pose) const
{

    return local_chart::rpd2target(r, phi, delta, robot_pose);
}

pose_t LocalChart::rpd2target(
        const egocentric_polar_coords_t& coords, const pose_t& robot_pose) const
{
    return rpd2target(coords.r, coords.phi, coords.delta, robot_pose);
}

}   // namespace bipedlab

