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
#pragma once

#ifndef ROBOT_MOTION
#define ROBOT_MOTION

#include <eigen3/Eigen/Dense> // SVD
#include "pose.h"
#include <math.h>

#define GRAVITY 9.8
#define ROBOT_HEIGHT 0.8

namespace bipedlab
{
        
    template <class T>
    pose_t moveRobotInWorldFrame(const T vx, const T vy, const T omega, 
            const float robot_x, const float robot_y, const float robot_yaw, 
            const T time_interval)
    {
        Eigen::Vector2d velocity_robot_frame;
        velocity_robot_frame << vx, vy;

        // compute translation and change of robot orientation in robot frame
        Eigen::Vector2d translation_robot_frame = 
            velocity_robot_frame * time_interval; 
        double delta_theta_robot_frame = omega * time_interval;


        // rotation matrix to transform to world frame
        double cos_theta = std::cos(robot_yaw);
        double sin_theta = std::sin(robot_yaw);
        Eigen::Matrix2d R;
        R << cos_theta, -sin_theta,
          sin_theta, cos_theta;

        // transform to world frame
        Eigen::Vector2d translation_world_frame = R * translation_robot_frame;

        return pose_t(robot_x + translation_world_frame(0),
                robot_y + translation_world_frame(1),
                robot_yaw - delta_theta_robot_frame);
    }


    inline double computeFootPosition(const double robot_x0, const double robot_v0,
            const double robot_vk, const double omega, double step_interval)
    {
        double omega_t = omega * step_interval;
        double p = (robot_vk - omega * std::sinh(omega_t) * robot_x0 - 
                std::cosh(omega_t) * robot_v0) / (-omega * std::sinh(omega_t));
        return p;
    }

    inline double computeCoMPosition(const double robot_x0, const double robot_v0, 
            const double p, const double omega, const double step_interval)
    {
        double omega_t = omega * step_interval;
        double x = robot_x0 * std::cosh(omega_t) + robot_v0 / omega * 
            std::sinh(omega_t) + (1 - cosh(omega_t)) * p;
        return x;
    }

    template<class T> pose_t moveRobotWithALIPModelInWorldFrame(
            const T target_vx, const T target_vy,  // desire speed
            const double vx_0, const double vy_0, 
            const double heading_omega, // current speed
            const pose_t& robot, const double step_interval)
    {
        // transform vx, vy to world frame
        double robot_theta = robot.theta;
        double cos_theta = std::cos(robot_theta);
        double sin_theta = std::sin(robot_theta);


        // initial velocity at world frame
        double vx_w0 = vx_0 * cos_theta - vy_0 * sin_theta;
        double vy_w0 = vx_0 * sin_theta + vy_0 * cos_theta;

        // target velocity at world frame
        double target_vx_w = target_vx * cos_theta - target_vy * sin_theta;
        double target_vy_w = target_vx * sin_theta + target_vy *cos_theta;

        static double omega = std::sqrt(GRAVITY / ROBOT_HEIGHT);

        // foot placement and CoM at world frame
        double px = computeFootPosition(robot.x, vx_w0, 
                target_vx_w,  omega, step_interval);
        double CoM_x = computeCoMPosition(robot.x, vx_w0, px, omega, step_interval);
        double py = computeFootPosition(robot.y, vy_w0, 
                target_vy_w, omega, step_interval);
        double CoM_y = computeCoMPosition(robot.y, vy_w0, py, omega, step_interval);


        double delta_theta_robot_frame = heading_omega * step_interval;
        double delta_theta_world_frame = robot_theta - delta_theta_robot_frame;
        return pose_t(CoM_x, CoM_y, delta_theta_world_frame);
    }


} /* bipedlab */ 
#endif /* ifndef ROBOT_MOTION */
