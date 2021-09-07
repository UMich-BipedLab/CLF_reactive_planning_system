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
#ifndef ROBOT_STATE_H
#define ROBOT_STATE_H 

#include <vector>


// ros stuff
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/buffer_core.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// #include <tf/transform_datatypes.h>
#include "inekf_msgs/State.h"


#include "pose.h"

namespace bipedlab 
{

typedef struct velocity 
{
    double vx;
    double vy;
    double vz;
    velocity(double vx, double vy, double vz) : vx(vx), vy(vy), vz(vz) { }
    velocity(double vx, double vy) : vx(vx), vy(vy), vz(0) { }
    velocity(const velocity &v) : vx(v.vx), vy(v.vy), vz(v.vz) { }
    velocity(void) : vx(0), vy(0), vz(0) { }


} velocity_t;

typedef struct robot_state
{
    // tf::StampedTransform robot_tf_pose;
    inekf_msgs::State inekf_state;
    ros::Time time;

    pose_6dof_t pose;


    double roll; // rad
    double pitch; // rad
    double yaw; // rad

    velocity_t velocity; // real robot velocity
    velocity_t dir_velocity; // robot direction velocity
    std::vector<double> contacts;

    robot_state(void) : pose(), velocity(), roll(0), pitch(0), yaw(0) 
    { 
        updateInEKFWithPose();
    }

    robot_state(const double& x, const double& y, const double& z) : 
        pose(x, y, z), velocity(), roll(0), pitch(0), yaw(0) 
    { 
        updateInEKFWithPose();
    }

    robot_state(const double& x, const double& y, const double& z, 
                const double& vx, const double& vy, const double& vz ) : 
        pose(x, y, z), velocity(vx, vy, vz), roll(0), pitch(0), yaw(0) 
    { 
        updateInEKFWithPose();
    }


    // robot_state(const inekf_msgs::State& inekf_state)
    // {
    //     pose(inekf_state->pose.position.x,
    //          inekf_state->pose.position.y,
    //          inekf_state->pose.position.z); 

    //     velocity(inekf_state->velocity.x,
    //              inekf_state->velocity.y,
    //              inekf_state->velocity.z);
    // }


    robot_state(pose_t& pose) : pose(pose), velocity(), roll(0), pitch(0), yaw(0) 
    { 
        updateInEKFWithPose();
    }

    robot_state(pose_t& pose, velocity_t& velocity) : 
        pose(pose), velocity(velocity) , roll(0), pitch(0), yaw(0)
    { 
        updateInEKFWithPose();
    }

    robot_state(robot_state& new_state) : 
        pose(new_state.pose), velocity(new_state.velocity), 
        roll(0), pitch(0), yaw(0)
    { 
        updateInEKFWithPose();
    }

    void setPose(const inekf_msgs::State& inekf_state)
    {
        this->inekf_state = inekf_state;
        setPoseStandaloneFromQuaternion(inekf_state.position.x,
                                        inekf_state.position.y,
                                        inekf_state.position.z,
                                        inekf_state.orientation.w,
                                        inekf_state.orientation.x,
                                        inekf_state.orientation.y,
                                        inekf_state.orientation.z);
    }

    void setFullPose(const inekf_msgs::State& inekf_state)
    {

        setPose(inekf_state);

        velocity.vx = inekf_state.velocity.x;
        velocity.vy = inekf_state.velocity.y;
        velocity.vz = inekf_state.velocity.z;

        dir_velocity.vx = std::cos(yaw); 
        dir_velocity.vy = std::sin(yaw);
        dir_velocity.vz = 0;

        time = inekf_state.header.stamp;
    }

    void setPoseStandaloneFromQuaternion(const double& x, const double& y, const double& z,
            const double& qw, const double& qx, const double& qy, const double& qz)
    {
        tf2::Quaternion q(qx, qy, qz, qw);
        tf2::Matrix3x3 m(q);
        m.getRPY(roll, pitch, yaw); // in radian

        this->pose.x = x;
        this->pose.y = y;
        this->pose.z = z;

        this->pose.phi = pitch;
        this->pose.rho = roll;
        this->pose.theta = yaw;

        this->roll = roll;
        this->pitch = pitch;
        this->yaw = yaw;


        geometry_msgs::Quaternion q_msg = tf2::toMsg(q);
        this->inekf_state.position.x = x;
        this->inekf_state.position.y = y;
        this->inekf_state.position.z = z;
        this->inekf_state.orientation = q_msg;
        this->inekf_state.header.stamp = ros::Time::now();
        this->inekf_state.header.frame_id = "map";

        this->time = this->inekf_state.header.stamp;
    }


    void updateInEKFWithPose(void)
    {
        this->inekf_state.position.x = this->pose.x;
        this->inekf_state.position.y = this->pose.y;
        this->inekf_state.position.z = this->pose.z;

        tf2::Quaternion q;
        q.setRPY(this->pose.rho, this->pose.phi, this->pose.theta);
        geometry_msgs::Quaternion q_msg = tf2::toMsg(q);
        this->inekf_state.orientation = q_msg;

        this->roll = this->pose.rho;
        this->pitch = this->pose.phi;
        this->yaw = this->pose.theta;
    }

    void updatePoseWithInEKF(void)
    {
        this->pose.x = this->inekf_state.position.x; 
        this->pose.y = this->inekf_state.position.y; 
        this->pose.z = this->inekf_state.position.z; 

        tf2::Quaternion q;
        tf2::convert(this->inekf_state.orientation, q);
        tf2::Matrix3x3 m(q); 
        m.getRPY(roll, pitch, yaw); // in radian
        this->pose.phi = pitch;
        this->pose.rho = roll;
        this->pose.theta = yaw;

        this->roll = roll;
        this->pitch = pitch;
        this->yaw = yaw;
    }


    void printPose()
    {
        std::cout << "pose: " << pose  << "; "
                  << "velocity: (" << velocity.vx << ", " 
                                   << velocity.vy << ", "
                                   << velocity.vz << ") "
                  << "roll: " << roll << ", "
                  << "pitch: " << pitch << ", "
                  << "yaw: " << yaw << std::endl;
    }
} robot_state_t;



} // bipedlab
#endif /* ifndef ROBOT_STATE_H */
