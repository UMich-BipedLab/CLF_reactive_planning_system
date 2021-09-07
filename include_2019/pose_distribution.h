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
 * \file     pose_distribution.h
 * \author   Bruce JK Huang, Collin Johnson
 *
 * Declaration of pose_distrubtion_t.
 */

#ifndef POSE_DISTRIBUTION_H
#define POSE_DISTRIBUTION_H

#include "multivariate_gaussian.h"
#include "pose.h"
// #include "system/message_traits.h"

namespace bipedlab
{

/**
 * pose_distribution_t represents the robot pose and uncertainty in Cartesian coordinates in the current LPM reference
 * frame. The pose is also stored outside the Gaussian for ease of access.
 */
struct pose_distribution_t
{
    int64_t timestamp;

    float x;
    float y;
    float theta;

    MultivariateGaussian uncertainty;

    pose_distribution_t(void) : timestamp(0), x(0), y(0), theta(0), uncertainty(3)
    {
        Eigen::Vector3d pose = Eigen::Vector3d::Zero(3, 3);
        Eigen::Matrix3d cov = Eigen::Matrix3d::Identity(3, 3);
        cov *= 0.0000001;
        uncertainty.setDistributionStatistics(pose, cov);
    }

    pose_distribution_t(int64_t timestamp, float x, float y, float theta, const MultivariateGaussian& uncertainty)
    : timestamp(timestamp)
    , x(x)
    , y(y)
    , theta(theta)
    , uncertainty(uncertainty)
    {
        assert(uncertainty.dimensions() == 3);
    }

    pose_distribution_t(float x, float y, float theta, float varX, float varY, float varTheta)
    : timestamp(0)
    , x(x)
    , y(y)
    , theta(theta)
    , uncertainty(3)
    {
        assert(uncertainty.dimensions() == 3);
        Eigen::Vector3d pose(x, y, theta);
        Eigen::Matrix3d cov;
        cov << varX, 0.0, 0.0, 0.0, varY, 0.0, 0.0, 0.0, varTheta;

        uncertainty.setDistributionStatistics(pose, cov);
    }

    pose_distribution_t(const pose2d_t& pose, const MultivariateGaussian& uncertainty)
    : timestamp(pose.timestamp)
    , x(pose.x)
    , y(pose.y)
    , theta(pose.theta)
    , uncertainty(uncertainty)
    {
        assert(uncertainty.dimensions() == 3);
    }

    pose_distribution_t(const MultivariateGaussian& uncertainty) : timestamp(0), uncertainty(uncertainty)
    {
        assert(uncertainty.dimensions() == 3);

        x = uncertainty[0];
        y = uncertainty[1];
        theta = uncertainty[2];
    }

    pose_distribution_t(int64_t timestamp, const MultivariateGaussian& uncertainty)
    : timestamp(timestamp)
    , uncertainty(uncertainty)
    {
        assert(uncertainty.dimensions() == 3);

        Eigen::Vector3d pose = uncertainty.getMean();
        x = pose(0);
        y = pose(1);
        theta = pose(2);
    }

    // Create with default noise
    explicit pose_distribution_t(const pose2d_t& pose)
    : timestamp(pose.timestamp)
    , x(pose.x)
    , y(pose.y)
    , theta(pose.theta)
    , uncertainty(3)
    {
        Eigen::Vector3d p(pose.x, pose.y, pose.theta);
        Eigen::Matrix3d cov = Eigen::Matrix3d::Identity();
        cov *= 0.0000001;
        uncertainty.setDistributionStatistics(p, cov);
    }

    /**
     * Apply the compound operator to transform this pose relative to another origin. The covariance is correctly
     * adjusted in the translation.
     */
    pose_distribution_t compound(const pose_distribution_t& origin) const;

    /**
     * toPose converts the pose_distribution_t to a simple pose, which is used in most places in the code.
     */
    pose2d_t toPose(void) const { return pose2d_t(timestamp, x, y, theta); }

    /**
     * Convert to just the position.
     */
    point2d_t<float> toPoint(void) const { return point2d_t<float>(x, y); }
};

bool operator==(const pose_distribution_t& lhs, const pose_distribution_t& rhs);
bool operator!=(const pose_distribution_t& lhs, const pose_distribution_t& rhs);

// Serialization support
template <class Archive>
void serialize(Archive& ar, pose_distribution_t& pose)
{
    ar& pose.timestamp;
    ar& pose.x;
    ar& pose.y;
    ar& pose.theta;
    ar& pose.uncertainty;
}

}   // namespace bipedlab

// DEFINE_SYSTEM_MESSAGE(pose_distribution_t, ("HSSH_LOCAL_POSE_DISTRIBUTION"))

#endif   // POSE_DISTRIBUTION_H

