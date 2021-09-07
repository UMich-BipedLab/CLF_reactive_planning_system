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
 * \file     local_chart.h
 * \author   Bruce JK Huang, Jong Jin Park
 *
 * Declaration of egocentric polar coordinates and related mapping functions from/to Cartesian coordinates.
 */

// #pragma once

#ifndef LOCAL_CHART_H
#define LOCAL_CHART_H

#include "pose.h"
#include "egocentric_coordinates.h"


namespace bipedlab
{

/**
 * LocalChart is a collection of bijective mappings between egocentric polar
 * coordinates and Cartesian cooridnates, anchored around a target pose.
 * This mapping is particularly useful for unicycle-type vehicles converging to
 * a target pose on a plane. See [Park-15] for exact definition.
 */
class LocalChart
{
public:
    /**
     * Constructor for unicycle chart.
     *
     * \param    targetPose      Target pose, around which a chart is constructed.
     * \param    smallRadius     Small radius threshold, used to avoid numerical instability at r = 0.
     */
    LocalChart(const pose_t& target_pose, double smallRadius = 0.001);
    LocalChart(void);

    // simple getters
    pose_t getTargetPose(void) const { return target_pose_; };
    double getSmallRadius(void) const { return small_radius_; };
    void assignNewTargetPose(const pose_t& new_target_pose);

    /**
     * lineOfSight finds a line_of_sight_t to a targetPose. If the range is too
     * small the angle of observation aligns with target orientation.
     *
     * \param    observerPosition        Starting point of the line of sight
     * \return   line of sight (range, angle)
     */
    line_of_sight_t lineOfSight(const point2d_t<float>& observer_position) const;

    /**
     * point2rp is a mapping from a Cartesian point to egocentric polar coords.
     * (1) phi is zero if the line of sight aligns with the target orientation
     * (2) Be careful when r is very small or phi is close to +- pi!
     *
     * \param    point           Cartesian point
     * \return   point in egocentric polar coordinates(r, phi)
     */
    reduced_egocentric_polar_coords_t point2rp(const point2d_t<float>& point) const;

    /**
     * rp2point is a mapping from a point in egocentric polar coordinates (r,phi)
     * to a point in Cartesian coordinates.
     *
     * \param    coords           point in egocentric polar coordinates
     * or (overloaded)
     * \param    r                radial distance to the target
     * \param    phi              orientaion of the target measured from the line of sight
     *
     * \return   point in Cartesian coordinates
     */
    point2d_t<float> rp2point(double r, double phi) const;
    point2d_t<float> rp2point(const reduced_egocentric_polar_coords_t& coords) const;

    /**
     * pose2rpd is a mapping from a pose in egocentric polar coordinates (r,phi)
     * to a pose in Cartesian coordinates.
     *
     * \param    pose            Cartesian pose
     * \return   pose in egocentc polar coordinates
     */
    egocentric_polar_coords_t pose2rpd(const pose_t& robot_pose) const;

    /**
     * rpd2pose is a mapping from a pose in egocentric polar coordinates (r,phi,delta)
     * to a robot pose in Cartesian coordinates.
     *
     * \param    coords           pose in egocentric polar coordinates
     * or (overloaded)
     * \param    r                radial distance to the target
     * \param    phi              orientaion of the target measured from the line of sight
     * \param    delta            orientaion of the robot measured from the line of sight
     *
     * \return   robot pose in Cartesian coordinates
     */
    pose_t rpd2pose(double r, double phi, double delta) const;
    pose_t rpd2pose(const egocentric_polar_coords_t& coords) const;

    /**
     * rpd2target recovers the target pose in Cartesian coordinates forom a pose
     * in egocentric polar coordinates (r,phi,delta) and a robot pose in
     * Cartesian coordinates.
     *
     * \param    coords           pose in egocentric polar coordinates
     * \param    robotPose        pose of a robot in Cartesian coordinates
     * or (overloaded)
     * \param    r                radial distance to the target
     * \param    phi              orientaion of the target measured from the line of sight
     * \param    delta            orientaion of the robot measured from the line of sight
     * \param    robotPose        pose of a robot in Cartesian cooridnates
     *
     * \return   target pose in Cartesian coordinates
     */
    pose_t rpd2target(double r, double phi, double delta, 
                      const pose_t& robot_pose) const;
    pose_t rpd2target(const egocentric_polar_coords_t& coords, 
                      const pose_t& robot_pose) const;

private:
    pose_t target_pose_;    // target pose
    double small_radius_;   // small radius
};

}   // namespace bipedlab

#endif   // LOCAL_CHART_H

