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

#include "standalone_local_chart.h"
#include "angle_functions.h"

// #define SMALL_RADIOUS 0.001

namespace bipedlab
{

namespace local_chart
{
        

line_of_sight_t 
lineOfSight(const point2d_t<float>& observerPosition,
            const pose_t& end_pose, const double& small_radious)
{
    line_of_sight_t los;

    float x = end_pose.x - observerPosition.x;
    float y = end_pose.y - observerPosition.y;

    los.range = sqrt(x * x + y * y);

    if (los.range < small_radious)   // avoid numerical unstability at r = 0.
    {
        los.angle = end_pose.theta;
    } else {
        los.angle = atan2(y, x);
    }

    return los;
}

reduced_egocentric_polar_coords_t 
point2rp(const point2d_t<float>& point,
         const pose_t& end_pose,
         const double small_radious) 
{
    reduced_egocentric_polar_coords_t coords;

    line_of_sight_t los = lineOfSight(point, end_pose, small_radious);

    coords.r = los.range;
    coords.phi = wrap_to_pi(end_pose.theta - los.angle);
    coords.lineOfSightAngle = los.angle;
    // std::cout << "pi: " <<  coords.phi << std::endl;

    return coords;
}

point2d_t<float> rp2point(double r, double phi, const pose_t& end_pose)
{
    point2d_t<float> point;

    float angle = end_pose.theta - phi;

    point.x = end_pose.x - r * cos(angle);
    point.y = end_pose.y - r * sin(angle);

    return point;
}

point2d_t<float> rp2point(
        const reduced_egocentric_polar_coords_t& coords,
        const pose_t& end_pose) 
{
    return rp2point(coords.r, coords.phi, end_pose);
}

egocentric_polar_coords_t pose2rpd(
        const pose_t& pose,
        const pose_t& target) 
{
    egocentric_polar_coords_t coords;

    // reduced_egocentric_polar_coords_t reducedCoords = 
    //     point2rp(target.toPoint(), target);
    reduced_egocentric_polar_coords_t reducedCoords = 
        point2rp(pose.toPoint(), target);

    coords.r = reducedCoords.r;
    coords.phi = reducedCoords.phi;
    coords.delta = wrap_to_pi(pose.theta - reducedCoords.lineOfSightAngle);
    coords.lineOfSightAngle = reducedCoords.lineOfSightAngle;

    return coords;
}

pose_t rpd2pose(double r, double phi, double delta, const pose_t& end_pose)
{
    pose_t pose;

    double angle = end_pose.theta - phi;

    pose.x = end_pose.x - r * cos(angle);
    pose.y = end_pose.y - r * sin(angle);
    pose.theta = wrap_to_pi(angle + delta);

    return pose;
}

pose_t rpd2pose(
        const egocentric_polar_coords_t& coords, 
        const pose_t& end_pose) 
{
    return rpd2pose(coords.r, coords.phi, coords.delta, end_pose);
}

pose_t rpd2target(double r, double phi, double delta, const pose_t& robotPose) 
{
    pose_t pose;

    double angle = robotPose.theta - delta;

    pose.x = robotPose.x + r * cos(angle);
    pose.y = robotPose.y + r * sin(angle);
    pose.theta = wrap_to_pi(angle + phi);

    return pose;
}

pose_t rpd2target(const egocentric_polar_coords_t& coords, const pose_t& robotPose) 
{
    return rpd2target(coords.r, coords.phi, coords.delta, robotPose);
}

} /* local_chart */ 
}   // namespace bipedlab

