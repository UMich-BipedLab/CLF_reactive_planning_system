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
// #pragma once
#ifndef SAMPLE_POSE_H
#define SAMPLE_POSE_H


#include "pose.h"
#include "local_map.h"
#include "robot_state.h"




namespace bipedlab
{

typedef struct pose_sampler_params
{
    float goal_bias;           // percentile of samples at goal pose
    float distance_threshold;   // margin from obstacles when determining freespace


    pose_sampler_params(void) : goal_bias(0.2), distance_threshold(0) { }
    pose_sampler_params(float goal_bias, float distance_threshold) : 
        goal_bias(goal_bias), distance_threshold(distance_threshold) { }
} pose_sampler_params_t;


class SamplePose
{
private:
    void sampleGoalOnBoundary(pose_t& sampled_pose);


    LocalMap local_map_;
    pose_t goal_pose_;
    robot_state_t robot_state_;
    pose_sampler_params_t params_;
    

public:
    SamplePose(const pose_sampler_params_t& params, 
               const LocalMap& local_map,
               const pose_t& goal_pose,
               const robot_state_t& robot_state);

    pose_t sampleRandomPoseWithGoalBiasedWithinLocalMap(bool& is_goal);
    pose_t sampleRandomPoseWithinLocalMap(void);
    virtual ~SamplePose();
};


} /* bipedlab */ 
#endif /* SAMPLE_POSE_H */
