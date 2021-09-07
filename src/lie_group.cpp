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
/* ----------------------------------------------------------------------------
 * Copyright 2018, Ross Hartley <m.ross.hartley@gmail.com>
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 *  @file   LieGroup.cpp
 *  @author Bruce JK Huang, Ross Hartley
 *  @brief  Source file for various Lie Group functions 
 *  @date   September 25, 2018
 **/

#include "lie_group.h"

namespace bipedlab {
namespace lie_group
{

using namespace std;

const double TOLERANCE = 1e-8;

Eigen::Matrix3d computeSkew(const Eigen::Vector3d& v) {
    // Convert vector to skew-symmetric matrix
    Eigen::Matrix3d M = Eigen::Matrix3d::Zero();
    M << 0, -v[2], v[1],
         v[2], 0, -v[0], 
        -v[1], v[0], 0;
        return M;
}

Eigen::Vector3d computeUnskew(const Eigen::Matrix3d& m)
{
    Eigen::Vector3d v;
    v << m(2, 1), m(0, 2), m(1, 0);

    return v;
}

Eigen::Matrix3d computeExpSO3(const Eigen::Vector3d& w) {
    // Computes the vectorized exponential map for SO(3)
    Eigen::Matrix3d A = computeSkew(w);
    double theta = w.norm();
    if (theta < TOLERANCE) {
        return Eigen::Matrix3d::Identity();
    } 
    Eigen::Matrix3d R =  Eigen::Matrix3d::Identity() + (sin(theta)/theta)*A + ((1-cos(theta))/(theta*theta))*A*A;
    return R;
}

Eigen::Vector3d computeLogSO3(const Eigen::Matrix3d& m) {
    // LOG_SO3 Computes the vectorized log map for SO(3)
    double theta = std::acos((m.trace() - 1) / 2);
    // std::cout << "in computeLogSo3 function, theta: " << theta << std::endl;

    if (theta < TOLERANCE) {
        return Eigen::Vector3d::Zero();
    } 
    
    Eigen::Matrix3d m_ =  theta * (m - m.transpose()) / (2 * std::sin(theta));
    // std::cout << "in computeLogSo3 function, m: " << m << std::endl;

    return computeUnskew(m_);
}


Eigen::MatrixXd computeExpSEK3(const Eigen::VectorXd& v) {
    // Computes the vectorized exponential map for SE_K(3)
    int K = (v.size()-3)/3;
    Eigen::MatrixXd X = Eigen::MatrixXd::Identity(3+K,3+K);
    Eigen::Matrix3d R;
    Eigen::Matrix3d Jl;
    Eigen::Vector3d w = v.head(3);
    double theta = w.norm();
    Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
    if (theta < TOLERANCE) {
        R = I;
        Jl = I;
    } else {
        Eigen::Matrix3d A = computeSkew(w);
        double theta2 = theta*theta;
        double stheta = sin(theta);
        double ctheta = cos(theta);
        double oneMinusCosTheta2 = (1-ctheta)/(theta2);
        Eigen::Matrix3d A2 = A*A;
        R =  I + (stheta/theta)*A + oneMinusCosTheta2*A2;
        Jl = I + oneMinusCosTheta2*A + ((theta-stheta)/(theta2*theta))*A2;
    }
    X.block<3,3>(0,0) = R;
    for (int i=0; i<K; ++i) {
        X.block<3,1>(0,3+i) = Jl * v.segment<3>(3+3*i);
    }
    return X;
}

Eigen::MatrixXd computeAdjointSEK3(const Eigen::MatrixXd& X) {
    // Compute Adjoint(X) for X in SE_K(3)
    int K = X.cols()-3;
    Eigen::MatrixXd Adj = Eigen::MatrixXd::Zero(3+3*K, 3+3*K);
    Eigen::Matrix3d R = X.block<3,3>(0,0);
    Adj.block<3,3>(0,0) = R;
    for (int i=0; i<K; ++i) {
        Adj.block<3,3>(3+3*i,3+3*i) = R;
        Adj.block<3,3>(3+3*i,0) = computeSkew(X.block<3,1>(0,3+i))*R;
    }
    return Adj;
}

} 
}
