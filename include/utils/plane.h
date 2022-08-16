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

#ifndef PLANE_H
#define PLANE_H 

#include <cmath>
#include <utility> // pair
#include <memory> // shared_ptr
#include <float.h> // FLT_MAX
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/QR>
#include "point.h"
#include "pose.h"

namespace bipedlab
{
namespace plane
{
    // PLANE REPRESENTATION 
    //          normal_vector
    //               \
    //                \
    //       ----------\---------------------     
    //      /           \               o    /    
    //     /                   fixed_point  /     
    //    /                                /     
    //   /                                /       
    //  /                                /        
    //  ---------------------------------         
    //
    struct plane_t
    {
        // normal vector zof this plane
        Eigen::Vector3f normal_vector; // x, y, z

        // a point on the plane
        Eigen::Vector3f fixed_point; // x, y, z

        int status; // if the plane is estimated (0/1)

        plane_t(const Eigen::Vector3f &normal_vector, 
                const Eigen::Vector3f &fixed_point):
                normal_vector(normal_vector), fixed_point(fixed_point) { }

        plane_t(const plane_t &copy):
                normal_vector(copy.normal_vector), fixed_point(copy.fixed_point),
                status(copy.status){ }

        plane_t(void):
                normal_vector(0, 0, 1), 
                fixed_point(Eigen::Vector3f::Zero(3)), status(0) { }

        std::vector<float>
            getPlaneCoefficients(void) 
        {
            return std::vector<float>{this->normal_vector(0), this->normal_vector(1), 
                                 this->normal_vector(2), this->fixed_point(2)};
        }

        void print(void) {
            std::cout << "normal_vector: \n" << this->normal_vector << std::endl;
            std::cout << "fixed_point: \n" << this->fixed_point << std::endl;
        }



        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };


    typedef struct terrain
    {
        int terrain_type;
        double probability;
        double friction;

        terrain(void) : terrain_type(-1), probability(-1), friction(-1) { }
    } terrain_t;

    // TERRAIN PLANE REPRESENTATION 
    //          normal_vector
    //               \
    //                \
    //       ----------\---------------------     ^
    //      /           \               o    /    |
    //     /                   fixed_point  /     |
    //    /                                /      | length
    //   /            / \                 /       |
    //  /              | robot_pose      /        |
    //  ---------------------------------         v
    //
    //  <------------------------------->
    //               width
    // 
    // 
    struct terrain_info_t : plane_t
    {
        // length of the plane (extending from robot pose toward its heading
        // direction)
        double length;

        // width of the plane (extending from robot pose toward perpendicularly 
        // toward it heading directing)
        double width;

        // robot pose when computing this plane
        pose_t robot_pose;

        // terrain information
        terrain_t terrain;

        // points
        Eigen::MatrixXf points;

        terrain_info_t(const Eigen::Vector3f &normal_vector, 
                        const Eigen::Vector3f &fixed_point,
                        const double &length, const double &width,
                        const pose_t &robot_pose):
                plane_t(normal_vector, fixed_point),
                length(length), width(width), robot_pose(robot_pose)
        { }

        terrain_info_t(const plane_t &plane,
                        const double &length, const double &width,
                        const pose_t &robot_pose):
                plane_t(plane),
                length(length), width(width), robot_pose(robot_pose)
        { }

        terrain_info_t(void): plane_t(), length(0), width(0), robot_pose()
        {
            // std::cout << "In terrain_plane_t default constructor\n";
        }

        // terrain_plane_t(const plane_t& copy) {
        //     std::cout << "In terrain_plane_t copy constructor\n";
        // }

        terrain_info_t& operator=(const plane_t& copy) {
            this->normal_vector = copy.normal_vector;
            this->fixed_point = copy.fixed_point;
            return *this;
        }

        void print(void) {
            std::cout << "normal_vector: \n" << this->normal_vector << std::endl;
            std::cout << "fixed_point: \n" << this->fixed_point << std::endl;
            std::cout << "(l, w): " << this->length << ", " << this->width << std::endl;
            std::cout << "robot_pose: "; this->robot_pose.print(); 
        }

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };

    // Points should be 3 x m format, where m is the number of points
    std::shared_ptr<plane_t> fitPlaneViaLeastSquaresPtr(const Eigen::MatrixXf &points);
    plane_t fitPlaneViaLeastSquares(const Eigen::MatrixXf &points);
    // plane_t fitPlaneViaLeastSquares(const Eigen::MatrixXf &points);
    // plane_t fitPlaneViaLeastSquares2(const Eigen::MatrixXf &points);
    // int fitPlaneViaLeastSquares2(const Eigen::MatrixXf &points);


} /* plane */ 
} /* bipedlab */ 
#endif /* ifndef PLANE_H */
