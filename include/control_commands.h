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
#ifndef CONTROL_COMMANDS_H
#define CONTROL_COMMANDS_H 


#include "planner_info_to_controller_t.h"
#include "controller_info_to_planner_t.h"
#include "point.h"
#include "inekf_msgs/State.h"
#include "planner_msgs/State.h"
#include "utils/plane.h"



namespace bipedlab
{
namespace control_commands
{



planner_info_to_controller_t assignInfo(
        const double& behavior,
        const std::vector<double>& velocity,
        const double& roll, const double& pitch, const double& yaw,
        const std::vector<point_t<double>>* terrain,
        const std::vector<point2d_t<double>>* path_points_ptr,
        const std::vector<double>* foot_placement,
        const double& x, const double& y, const double& z,
        const double& qw, const double& qx, const double& qy, const double& qz);

planner_info_to_controller_t assignInfoReduced(
        const double& behavior,
        const std::vector<double>& velocity,
        const double& roll, const double& pitch, const double& yaw);

planner_info_to_controller_t assignInfoReducedWithInEKFMsg(
        const double& behavior,
        const std::vector<double>& velocity,
        const double& roll, const double& pitch, const double& yaw,
        const inekf_msgs::State& inekf_state);

planner_info_to_controller_t assignInfoWithInEKFMsg(
        const double& behavior,
        const std::vector<double>& velocity,
        const double& roll, const double& pitch, const double& yaw,
        const std::vector<point_t<double>>* terrain,
        const std::vector<point2d_t<double>>* path_points_ptr,
        const std::vector<double>* foot_placement, 
        const inekf_msgs::State& inekf_state);

planner_msgs::State
convertToPlannerMsg(const planner_info_to_controller_t& data,
                    const ros::Time& time,
                    const std::string& frame_id);
planner_msgs::State
convertToPlannerMsgWithPlane(const planner_info_to_controller_t& data,
        const std::shared_ptr<std::vector<plane::terrain_info_t>> terrain_plane_ptr,
        const ros::Time& time,
        const std::string& frame_id);

planner_msgs::State
convertToPlannerMsgWithPlane(const planner_info_to_controller_t& data,
        const std::vector<plane::terrain_info_t>& terrain_plane,
        const ros::Time& time,
        const std::string& frame_id);




planner_info_to_controller_t assignWalkInPlaceWithInEKFMsg(
        const inekf_msgs::State& inekf_state);
planner_info_to_controller_t assignWalkInPlace();

planner_info_to_controller_t assignRotateInPlaceWithInEKFMsg(
        const inekf_msgs::State& inekf_state, const double angular_velocity);


std::string returnPublishData(const planner_info_to_controller_t& data);
std::string returnReceivedData(const controller_info_to_planner_t& data);

} /* control_commands */
} /* bipedlab */ 
#endif /* ifndef CONTROL_COMMANDS_H */
