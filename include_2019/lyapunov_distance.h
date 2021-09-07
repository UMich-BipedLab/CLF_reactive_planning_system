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
 * \file     lyapunov_distance.h
 * \author   Bruce JK Huang, Jong Jin Park
 *
 * Declaration of LyapunovDistance, implementation of control-Lyapunov
 * distance functions.
 */

//#pragma once

#ifndef LYAPUNOV_DISTANCE_H
#define LYAPUNOV_DISTANCE_H 


#include "pose.h"
#include "standalone_lyapunov_distance.h"
#include "local_chart.h"
#include "point.h"
#include "local_map.h"

namespace bipedlab
{



struct lyapunov_distance_params_t
{
    double k_phi;
    double k_delta;
    double small_radius;
    stabilizing_vector_field_type_t vector_field_type;
    lyapunov_distance_params_t(void) : 
            k_phi(1.2), k_delta(3.0), small_radius(0.001), 
            vector_field_type(GRADIENT_DESCENT) { }
};

// LyapunovDistance encodes distance-to-target-pose.
class LyapunovDistance
{
public:
    /**
     * Constructor for LyapunovDistance
     *
     * \param    targetPose        target pose
     * \param    params            parameters
     */
    LyapunovDistance(const pose_t& target_pose,
                     const lyapunov_distance_params_t& param,
                     const LocalMap& local_map);
    LyapunovDistance(const lyapunov_distance_params_t& param,
                     const LocalMap& local_map);
    LyapunovDistance(const LocalMap& local_map);


    // returns the target pose
    // pose_t getTargetPose(void) const { return local_chart_.getTargetPose(); };
    void assignNewTargetPose(const pose_t& new_target_pose);

    /**
     * stabilizingDeltaStar returns stabilizing unicycle heaidng in egocentric polar coordinates.
     *
     * \param    point           a point on a plane
     * \return   stabilizing heading in egocentric polar coordinates
     */
    double stabilizeDeltaStar(
            const point2d_t<float> point) const;
    double stabilizeDeltaStarWithTargetPose(
            const point2d_t<float> point, const pose_t& target_pose);

    /**
     * stabilizingHeading returns stabilizing unicycle heaidng in Cartesian coordinates.
     *
     * \param    point           a point on a plane
     * \return   stabilizing heading in Cartesian coordinates.
     */
    double stabilizeHeading(
            const point2d_t<float> point) const;
    double stabilizeHeadingWithTargetPose(
            const point2d_t<float> point, const pose_t& target_pose);

    /**
     * distanceFromPoint computes distance-to-go to the taget pose from a point, assuming forward motion
     *
     * \param    point           a point on a plane
     * \return   orientation-weighted distance to the target pose
     */
    double computeDistanceFromPoint(
        const point2d_t<float>& point) const;
    double computeDistanceFromPointWithTargetPose(
        const point2d_t<float>& point, const pose_t& target_pose);


    /**
     * distanceFromPose computes distance-to-go to the taget pose from a pose, assuming forward motion
     *
     * \param    pose            a pose on a plane
     * \return   control-lyapunov distance-to-go to the target pose
     */
    double computeDistanceFromPose(
            const pose_t& pose) const;
    double computeDistanceFromPoseWithTargetPose(
            const pose_t& pose, const pose_t& target_pose);



private:
    friend class LyapunovPath;
    pose_t target_pose_;

    LocalChart local_chart_;   // mapping around the target pose
    LocalMap local_map_;

    lyapunov_distance_params_t params_;
};
}   // namespace bipedlab



#endif /* ifndef LYAPUNOV_DISTANCE_H */
