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
#include "control_commands.h"
#include "utils/debugger.h"
#include "utils/utils.h"
#include "robot_state.h"

namespace bipedlab
{
namespace control_commands
{

// walk in-place without robot current position
planner_info_to_controller_t
assignWalkInPlace()
{
    const std::vector<double> velocity(3, 0);
    return assignInfo(
            0, // behavior
            velocity, // velocity
            0, 0, 0, // troso
            nullptr, nullptr, nullptr, //
            0, 0, 0, // xyz
            1, 0, 0, 0);// quaternion
}

planner_info_to_controller_t
assignRotateInPlaceWithInEKFMsg(
        const inekf_msgs::State& inekf_state, const double angular_velocity)
{
    tf2::Quaternion q(inekf_state.orientation.x,
                      inekf_state.orientation.y,
                      inekf_state.orientation.z, 
                      inekf_state.orientation.w);
    tf2::Matrix3x3 m(q);
    double roll;
    double pitch;
    double yaw;
    m.getRPY(roll, pitch, yaw); // in radian

    return assignInfo(
            0, // behavior
            {0, 0, angular_velocity}, // velocity
            roll, pitch, yaw, // troso
            nullptr, nullptr, nullptr, //
            inekf_state.position.x,
            inekf_state.position.y,
            inekf_state.position.z,
            inekf_state.orientation.w,
            inekf_state.orientation.x,
            inekf_state.orientation.y,
            inekf_state.orientation.z);
}

planner_info_to_controller_t
assignWalkInPlaceWithInEKFMsg(const inekf_msgs::State& inekf_state)
{
    tf2::Quaternion q(inekf_state.orientation.x,
                      inekf_state.orientation.y,
                      inekf_state.orientation.z, 
                      inekf_state.orientation.w);
    tf2::Matrix3x3 m(q);
    double roll;
    double pitch;
    double yaw;
    m.getRPY(roll, pitch, yaw); // in radian

    const std::vector<double> velocity(3, 0);
    return assignInfo(
            0, // behavior
            velocity, // velocity
            roll, pitch, yaw, // troso
            nullptr, nullptr, nullptr, //
            inekf_state.position.x,
            inekf_state.position.y,
            inekf_state.position.z,
            inekf_state.orientation.w,
            inekf_state.orientation.x,
            inekf_state.orientation.y,
            inekf_state.orientation.z);
}


planner_info_to_controller_t assignInfoReduced(
        const double& behavior,
        const std::vector<double>& velocity,
        const double& roll, const double& pitch, const double& yaw)
{
    return assignInfo(
            behavior, // behavior
            velocity, roll, pitch, yaw, // control
            nullptr, nullptr, nullptr, //
            0, 0, 0, // x, y, z
            1, 0, 0, 0); // quaternion
}

planner_info_to_controller_t assignInfoReducedWithInEKFMsg(
        const double& behavior,
        const std::vector<double>& velocity,
        const double& roll, const double& pitch, const double& yaw,
        const inekf_msgs::State& inekf_state)
{
    return assignInfo(
            behavior, // behavior
            velocity, roll, pitch, yaw, // control
            nullptr, nullptr, nullptr, //
            inekf_state.position.x,
            inekf_state.position.y,
            inekf_state.position.z,
            inekf_state.orientation.w,
            inekf_state.orientation.x,
            inekf_state.orientation.y,
            inekf_state.orientation.z);
}


planner_info_to_controller_t assignInfo(
        const double& behavior,
        const std::vector<double>& velocity,
        const double& roll, const double& pitch, const double& yaw,
        const std::vector<point_t<double>>* terrain,
        const std::vector<point2d_t<double>>* path_points_ptr,
        const std::vector<double>* foot_placement,
        const double& x, const double& y, const double& z,
        const double& qw, const double& qx, const double& qy, const double& qz)
{
    // debugger::debugOutput("[assignInfo]", "1", 5);
    planner_info_to_controller_t planner_info_to_controller_udp;
    memset(&planner_info_to_controller_udp, 0, sizeof(planner_info_to_controller_t));

    // assign behavior
    planner_info_to_controller_udp.behavior = behavior;

    // assign control commands
    planner_info_to_controller_udp.velocity[0] = velocity[0];
    planner_info_to_controller_udp.velocity[1] = velocity[1];
    planner_info_to_controller_udp.velocity[2] = velocity[2];
    planner_info_to_controller_udp.torso.roll = roll;
    planner_info_to_controller_udp.torso.pitch = pitch;
    planner_info_to_controller_udp.torso.yaw = yaw;

    // debugger::debugOutput("[assignInfo]", "2", 5);

    // assign waypoint to follow (nothing for now)
    if (terrain != nullptr)
    {
        for (const auto& terrain_info : *terrain)
        {

        }
    }


    if (path_points_ptr != nullptr)
    {
        for (const auto& point : *path_points_ptr)
        {

        }
    }


    // assign foot placement (nothing for now)
    if (foot_placement != nullptr)
    {
        for (const auto& foot_pos : *foot_placement)
        {

        }
    }


    // assign pose
    planner_info_to_controller_udp.pose[0] = x;
    planner_info_to_controller_udp.pose[1] = y;
    planner_info_to_controller_udp.pose[2] = z;
    planner_info_to_controller_udp.pose[3] = qw;
    planner_info_to_controller_udp.pose[4] = qx;
    planner_info_to_controller_udp.pose[5] = qy;
    planner_info_to_controller_udp.pose[6] = qz;


    // debugger::debugOutput("[assignInfo]", "3", 5);
    return planner_info_to_controller_udp;



    // size_t precision = 3;
    // planner_info_to_controller_t planner_info_to_controller_udp;
    // memset(&planner_info_to_controller_udp, 0, sizeof(planner_info_to_controller_t));

    // // assign behavior
    // planner_info_to_controller_udp.behavior[0] =
    //     std::stod(utils::toStringWithPrecision(behavior, precision));

    // // assign control commands
    // planner_info_to_controller_udp.velocity[0] =
    //     std::stod(utils::toStringWithPrecision(velocity, precision));
    // planner_info_to_controller_udp.roll[0] =
    //     std::stod(utils::toStringWithPrecision(roll, precision));
    // planner_info_to_controller_udp.pitch[0] =
    //     std::stod(utils::toStringWithPrecision(pitch, precision));
    // planner_info_to_controller_udp.yaw[0] =
    //     std::stod(utils::toStringWithPrecision(yaw, precision));

    // // assign waypoint to follow (nothing for now)
    // if (path_points_ptr != nullptr)
    // {
    //     for (const auto& : *path_points_ptr)
    //     {

    //     }
    // }


    // // assign foot placement (nothing for now)
    // if (foot_placement != nullptr)
    // {
    //     for (const auto& : *foot_placement)
    //     {

    //     }
    // }


    // // assign pose
    // planner_info_to_controller_udp.pose[0] =
    //     std::stod(utils::toStringWithPrecision(x, precision));
    // planner_info_to_controller_udp.pose[1] =
    //     std::stod(utils::toStringWithPrecision(y, precision));
    // planner_info_to_controller_udp.pose[2] =
    //     std::stod(utils::toStringWithPrecision(z, precision));
    // planner_info_to_controller_udp.pose[3] =
    //     std::stod(utils::toStringWithPrecision(qw, precision));
    // planner_info_to_controller_udp.pose[4] =
    //     std::stod(utils::toStringWithPrecision(qx, precision));
    // planner_info_to_controller_udp.pose[5] =
    //     std::stod(utils::toStringWithPrecision(qy, precision));
    // planner_info_to_controller_udp.pose[6] =
    //     std::stod(utils::toStringWithPrecision(qz, precision));
}

std::string returnPublishData(const planner_info_to_controller_t& data)
{
    size_t precision = 2;
    std::string s_control = "\n(behavior, vx, vy, omega, roll, pitch, yaw): (" +
        utils::toStringWithPrecision(data.behavior, precision) + "; " +
        utils::toStringWithPrecision(data.velocity[0], precision) + ", " +
        utils::toStringWithPrecision(data.velocity[1], precision) + ", " +
        utils::toStringWithPrecision(data.velocity[2], precision) + "; " +
        utils::toStringWithPrecision(data.torso.roll, precision) + ", " +
        utils::toStringWithPrecision(data.torso.pitch, precision) + ", " +
        utils::toStringWithPrecision(data.torso.yaw, precision) + ")";

    std::string s_pose = "\n(x, y, z, qw, qx, qy, qz): (" +
        utils::toStringWithPrecision(data.pose[0], precision) + ", " +
        utils::toStringWithPrecision(data.pose[1], precision) + ", " +
        utils::toStringWithPrecision(data.pose[2], precision) + ", " +
        utils::toStringWithPrecision(data.pose[3], precision) + ", " +
        utils::toStringWithPrecision(data.pose[4], precision) + ", " +
        utils::toStringWithPrecision(data.pose[5], precision) + ", " +
        utils::toStringWithPrecision(data.pose[6], precision) + ")";
    return s_control + "; " + s_pose;
}

std::string returnReceivedData(const controller_info_to_planner_t& data)
{
    size_t precision = 2;
    std::string s_control = "";
    // std::string s_control = "(behavior, velocity, roll, pitch, yaw): (" +
    //     utils::toStringWithPrecision(data.behavior, precision) + "; " +
    //     utils::toStringWithPrecision(data.velocity[0], precision) + ", " +
    //     utils::toStringWithPrecision(data.velocity[1], precision) + "; " +
    //     utils::toStringWithPrecision(data.torso.roll, precision) + ", " +
    //     utils::toStringWithPrecision(data.torso.pitch, precision) + ", " +
    //     utils::toStringWithPrecision(data.torso.yaw, precision) + ")";

    std::string s_pose = "\n(x, y, z, qw, qx, qy, qz): (" +
        utils::toStringWithPrecision(data.pose[0], precision) + ", " +
        utils::toStringWithPrecision(data.pose[1], precision) + ", " +
        utils::toStringWithPrecision(data.pose[2], precision) + ", " +
        utils::toStringWithPrecision(data.pose[3], precision) + ", " +
        utils::toStringWithPrecision(data.pose[4], precision) + ", " +
        utils::toStringWithPrecision(data.pose[5], precision) + ", " +
        utils::toStringWithPrecision(data.pose[6], precision) + ")";

    robot_state_t state;
    inekf_msgs::State inekf_state;
    inekf_state.position.x  = data.pose[0];
    inekf_state.position.y  = data.pose[1];
    inekf_state.position.z  = data.pose[2];
    inekf_state.orientation.w = data.pose[3];
    inekf_state.orientation.x = data.pose[4];
    inekf_state.orientation.y = data.pose[5];
    inekf_state.orientation.z = data.pose[6];
    state.setPose(inekf_state);
    std::string s_pose_rpy =  "\n(x, y, z, r, p, h): (" +
        utils::toStringWithPrecision(state.pose.x, precision) + ", " +
        utils::toStringWithPrecision(state.pose.y, precision) + ", " +
        utils::toStringWithPrecision(state.pose.z, precision) + ", " +
        utils::toStringWithPrecision(state.pose.rho, precision) + ", " +
        utils::toStringWithPrecision(state.pose.phi, precision) + ", " +
        utils::toStringWithPrecision(state.pose.theta, precision) + ")";
        

    return s_control + "; " + s_pose + "; " + s_pose_rpy;
}

planner_msgs::State
convertToPlannerMsg(const planner_info_to_controller_t& data, 
                    const ros::Time& time,
                    const std::string& frame_id)
{
    planner_msgs::State state;

    state.header.stamp = time;
    state.header.frame_id = frame_id;

    state.behavior = (int) data.behavior;
    state.velocity.x = data.velocity[0]; 
    state.velocity.y = data.velocity[1]; 
    state.velocity.z = data.velocity[2];  // omega


    state.torso.roll = data.torso.roll;
    state.torso.pitch = data.torso.pitch;
    state.torso.yaw = data.torso.yaw;

    // terrain

    // waypoints

    // footplacement

    // pose
    state.pose.position.x = data.pose[0];
    state.pose.position.y = data.pose[1];
    state.pose.position.z = data.pose[2];

    state.pose.orientation.w = data.pose[3];
    state.pose.orientation.x = data.pose[4];
    state.pose.orientation.y = data.pose[5];
    state.pose.orientation.z = data.pose[6];

    return state;
}

} /* control_commands */ 
} /* bipedlab */ 
