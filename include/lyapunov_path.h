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
 * \file     lyapunov_path.h
 * \author   Bruce JK Huang, Jong Jin Park
 *
 * Declaration of LyapunovPath, which implements numerically stable
 * steering-along-vector-field via control-Lyapunov function.
 */

#ifndef LYAPUNOV_PATH_H
#define LYAPUNOV_PATH_H

#include <vector>
#include <cmath>

#include "pose.h"
#include "lyapunov_distance.h"
#include "standalone_local_chart.h"
#include "utils/debugger.h"
#include "local_map.h"
#include "map_cost.h"
#include "robot_motion.h"

namespace bipedlab
{

// class LyapunovDistance;

// struct containing a path segment
struct path_segment_t
{
    std::vector<pose_t> steps;
    std::vector<point2d_t<float>> path;
    cost_t cost_along_path;

    path_segment_t(const std::vector<pose_t>& steps,
                   const std::vector<point2d_t<float>>& path, 
                   const cost_t& cost_along_path) :
        steps(steps), path(path), cost_along_path(cost_along_path) { }

    path_segment_t(const std::vector<pose_t>& steps,
                   const std::vector<point2d_t<float>>& path) :
        steps(steps), path(path), cost_along_path(0) { }
};

// parameters for steering
struct lyapunov_path_params_t
{
    double incrementalRotation = 0.175;
    double incrementalTranslation = 0.2;
};

class LyapunovPath
{
public:
    /**
     * Constructor for LyapunovPath
     */
    explicit LyapunovPath(const lyapunov_path_params_t& params, 
                          LyapunovDistance& lyap_dis,
                          const LocalMap& local_map,
                          const MapCost& cost,
                          const size_t& cost_mode);

    explicit LyapunovPath(LyapunovDistance& lyap_dis, 
                          const LocalMap& local_map,
                          const MapCost& cost,
                          const size_t& cost_mode);
    explicit LyapunovPath(LyapunovDistance& lyap_dis, const size_t& cost_mode);


    // LyapunovPath(void){};

    path_segment_t extend(const pose_t& from_node, 
                          const pose_t& to_node,
                          double maxExtension) const;

    path_segment_t steer(const pose_t& robot_pose, const pose_t& end_pose) const;

    std::vector<point2d_t<float>> 
        steerAway(const point2d_t<float>& point, double maxExtension) const;

private:
    // differential driven model
    path_segment_t extendUsingDifferentialDrivenModel_(
            const pose_t& from_node, 
            const pose_t& to_node,
            double maxExtension) const;

    pose_t incrementalRotationForDifferentialDrivenModel_(
            const pose_t& robotPose,
            double maxRotation,
            bool* isAligned) const;

    pose_t incrementalTranslationForDifferentialDrivenModel_(
            const pose_t& robotPose,
            double maxTranslation,
            std::vector<point2d_t<float>>* points,
            bool* isConverged) const;

    // omni-directional model
    path_segment_t extendUsingOmniDirectionalModel_(
            const pose_t& from_node, 
            const pose_t& to_node,
            double maxExtension) const;

    pose_t incrementalTranslationForOmniDirectionalModel_(
            const pose_t& robotPose,
            double maxTranslation,
            std::vector<point2d_t<float>>* points,
            bool* isConverged) const;




    LyapunovDistance* lyap_dis_;
    const LocalMap* local_map_;
    const MapCost* map_cost_;
    lyapunov_path_params_t params_;

    const size_t cost_mode_; // 0: vanilla, 1: height 
    const size_t clf_model_;
};

}   // namespace bipedlab

#endif   // LYAPUNOV_PATH_H

