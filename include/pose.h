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
 * \file     pose.h
 * \author   Bruce JK Huang, Collin Johnson
 *
 * Declaration of pose2d_t, pose_distribution_t, and associated helper functions.  
 * 
*/

#pragma once

#ifndef POSE_H
#define POSE_H

#include <cstdint>
#include <iosfwd>
#include <string>
#include <iostream>

#include <eigen3/Eigen/Dense>

#include "angle_functions.h"
#include "position.h"

namespace bipedlab
{

struct pose_distribution_t;

/**
 * pose2d_t represents the robot pose in Cartesian coordinates in the current global
 * frame of reference.
 */
struct pose2d_t
{
    int64_t timestamp;

    float x;
    float y;
    float theta;
    Eigen::Vector2d pos2d_v;

    pose2d_t(void) : timestamp(0), x(0), y(0), theta(0) { }

    pose2d_t(float x, float y, float theta) : timestamp(0), 
    x(x), y(y), theta(theta), pos2d_v(x, y) { }

    explicit pose2d_t(position_t position, float theta = 0.0f) : 
        timestamp(0), x(position.x), y(position.y), theta(theta),
        pos2d_v(position.x, position.y) { }

    explicit pose2d_t(std::pair<float, float> position, float theta = 0.0f) : 
        timestamp(0), x(position.first), y(position.second), theta(theta),
        pos2d_v(position.first, position.second) { }

    pose2d_t(int64_t timestamp, float x, float y, float theta) : 
        timestamp(timestamp), x(x), y(y), theta(theta),
        pos2d_v(x, y) { }

    pose2d_t(const pose_distribution_t& distribution);

    // Some helper functions for converting reference frames, etc.
    /**
     * toPoint converts the pose to a point. Simply chop off the theta.
     */
    position_t toPoint(void) const { return position_t(x, y); }


    Eigen::Vector2d getPosition2d(void) const { return pos2d_v; }


    /**
     * flip rotates the pose by 180 degrees.
     */
    pose2d_t flip(void) const { return pose2d_t(x, y, angle_sum(theta, M_PI)); }

    /**
     * transformToNewFrame applies a transform to the pose and returns the resulting pose.
     *
     * \param    newFrame    New reference frame for the pose (newFrame is the offset from (0,0,0) in the current frame)
     * \return   Pose transformed into the new reference frame.
     */
    pose2d_t transformToNewFrame(const pose2d_t& newFrame) const;

    /**
     * compound applies the compound operator to transform this pose to a new location relative to the provided origin.
     *
     * \param    origin          Origin pose of the compound, i.e. on the right of the circle plus
     * \return   this circle-plus origin.
     */
    pose2d_t compound(const pose2d_t& origin) const;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

/**
 * pose_6dof_t is a full 6-DOF pose. While most of the robot functionality only needs the simpler 3-DOF
 * pose, some sensors, like the laser rangefinders, exist at different heights and rotations, so the full
 * description of their pose is needed when converting values into the robot's frame of reference.
 */
struct pose_6dof_t : pose2d_t
{
    // pose_6dof_t() {}

    float z;
    float phi; // pitch
    float rho; // roll
    
    pose_6dof_t(void) : pose2d_t(), z(0), phi(0), rho(0) { }

    pose_6dof_t(float x, float y, float theta) : pose2d_t(x, y, theta), z(0), 
    phi(0), rho(0){ }

    // x, y, z, pitck, roll, yaw
    pose_6dof_t(int64_t timestamp, float x, float y, float z, float phi, float rho, float theta)
    : pose2d_t(timestamp, x, y, theta), z(z), phi(phi), rho(rho)
    {
    }

    // x, y, z, pitch, roll, yaw
    template <class T>
    pose_6dof_t(const T& x, const T& y, const T& z, 
                const T& phi, const T& rho, const T& theta)
    : pose2d_t(x, y, theta), z(z), phi(phi), rho(rho)
    {
    }

    // pose_6dof_t(float x, float y, float z, float phi, float rho, float theta)
    // : pose2d_t(x, y, theta), z(z), phi(phi), rho(rho), pos_v(x, y, z)
    // {
    // }

    // Allow implicit conversion from a pose
    pose_6dof_t(const pose2d_t& pose) : pose_6dof_t(pose.timestamp, pose.x, pose.y, 0.0f, 0.0f, 0.0f, pose.theta) { }
    // pose_6dof_t(const pose2d_t& pose) : pose_6dof_t(pose.timestamp, pose.x, pose.y, 0.0f, 0.0f, 0.0f, pose.theta) { }

    /**
     * to2DPosition retrieves the 2D position, (x,y), of the 6dof pose.
     */
    position_t to2DPosition(void) const { return position_t(x, y); }

    Eigen::Vector3d getPosition(void) const 
    { 
        return Eigen::Vector3d((double) x, (double) y, (double) z); 
    }
    Eigen::Vector2d get2DPosition(void) const 
    { 
        return Eigen::Vector2d((double) x, (double) y); 
    }

    /**
     * toPose retrieves the 3-dof pose (x, y, theta) of the full 6dof pose.
     */
    pose2d_t toPose(void) const { return pose2d_t(timestamp, x, y, theta); }

    void print(void) const
    {
        std::cout << "x: " << this->x << ", y: " << this->y << ", z: " << this->z 
                  << ", roll: " << this->rho 
                  << ", pitch: "<< this->phi 
                  << ", yaw: " << this->theta << std::endl;
    }

    std::string returnMemberString(void) const

        {
            return "x: " + std::to_string(this->x)+
                   ", y: " + std::to_string(this->y) +
                   ", z: " + std::to_string(this->z) + 
                   ", roll: " + std::to_string(this->rho) + 
                   ", pitch: " + std::to_string(this->phi) + 
                   ", yaw: " + std::to_string(this->theta);
        }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

typedef pose_6dof_t pose_t;

// struct pose_6dof_t
// {
//     int64_t timestamp;
// 
//     float x;
//     float y;
//     float z;
// 
//     float phi;     ///< Pitch
//     float rho;     ///< Roll
//     float theta;   ///< Yaw
// 
//     pose_6dof_t(void) : timestamp(0), x(0), y(0), z(0), phi(0), rho(0), theta(0) { }
// 
//     pose_6dof_t(float x, float y, float theta) : timestamp(0), x(x), y(y), z(0), phi(0), rho(0), theta(theta) { }
// 
//     pose_6dof_t(int64_t timestamp, float x, float y, float z, float phi, float rho, float theta)
//     : timestamp(timestamp)
//     , x(x)
//     , y(y)
//     , z(z)
//     , phi(phi)
//     , rho(rho)
//     , theta(theta)
//     {
//     }
// 
//     // Allow implicit conversion from a pose
//     pose_6dof_t(const pose2d_t& pose) : pose_6dof_t(pose.timestamp, pose.x, pose.y, 0.0f, 0.0f, 0.0f, pose.theta) { }
// 
//     /**
//      * to2DPosition retrieves the 2D position, (x,y), of the 6dof pose.
//      */
//     position_t to2DPosition(void) const { return position_t(x, y); }
// 
//     /**
//      * toPose retrieves the 3-dof pose (x, y, theta) of the full 6dof pose.
//      */
//     pose2d_t toPose(void) const { return pose2d_t(timestamp, x, y, theta); }
// };

// Various operator overloads
bool operator==(const pose2d_t& lhs, const pose2d_t& rhs);
bool operator!=(const pose2d_t& lhs, const pose2d_t& rhs);
std::ostream& operator<<(std::ostream& out, const pose2d_t& pose);
std::ostream& operator<<(std::ostream& out, const pose_6dof_t& pose);

/**
 * transform_pose2d_to_new_frame transforms a pose to a new reference frame.
 *
 * \param    pose        Pose to be transformed
 * \param    newFrame    New reference frame for the pose (newFrame is the offset from (0,0,0) in the current frame)
 * \return   Pose transformed into the new reference frame.
 */
pose2d_t transform_pose_to_new_frame(const pose2d_t& pose, const pose2d_t& newFrame);

/**
 * interpolate_pose calculates an estimate of the robot pose given two robot poses
 * and a time in the time interval between the poses.
 *
 * \pre  priorPose.timestamp < desiredPoseTime < currentPose.timestamp
 */
pose2d_t interpolate_pose(const pose2d_t& priorPose, const pose2d_t& currentPose, int64_t desiredPoseTime);

// Serialization support -- both types are just PODs
template <class Archive>
void serialize(Archive& ar, pose2d_t& pose)
{
    ar(pose.timestamp, pose.x, pose.y, pose.theta);
}

template <class Archive>
void serialize(Archive& ar, pose_6dof_t& pose, const unsigned int version)
{
    ar(pose.timestamp, pose.x, pose.y, pose.z, pose.theta, pose.rho, pose.phi);
}

}   // namespace bipedlab

// DEFINE_SERIALIZATION_MSG_TRAITS(pose2d_t, ("ROBOT_POSE"), old_message_tag)

#endif   // POSE_H
