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
#include "utils/plane.h"
#include <Eigen/SVD>


namespace bipedlab
{
namespace plane 
{

    // Points should be 3 x m format, where m is the number of points
    std::shared_ptr<plane_t> fitPlaneViaLeastSquaresPtr(const Eigen::MatrixXf &points)
    {
        // points = [xs
        //           ys
        //           zs]
        std::shared_ptr<plane_t> fitted_plane(new plane_t);
        size_t num_points = points.cols();
        Eigen::MatrixXf A = Eigen::MatrixXf::Ones(num_points, 3); // 1s, xs, ys
        A.col(1) = points.row(0); // xs
        A.col(2) = points.row(1); // ys
        // std::cout << "A: \n"  << A << std::endl;
        Eigen::VectorXf z = points.row(2).transpose().eval(); // zs
        Eigen::MatrixXf b = A.bdcSvd(Eigen::ComputeThinU |
                     Eigen::ComputeThinV).solve(z);
        // std::cout << "b: \n"  << b << std::endl;

        // b1 * x + b2 * y - z + b0 = 0
        fitted_plane->normal_vector = Eigen::Vector3f(b(1), b(2), -1);
        fitted_plane->fixed_point = Eigen::Vector3f(0, 0, b(0));

        return fitted_plane;
    }


    // Points should be 3 x m format, where m is the number of points
    // std::shared_ptr<plane_t> 
    plane_t
    fitPlaneViaLeastSquares(const Eigen::MatrixXf &points)
    {
        // points = [xs
        //           ys
        //           zs]
        // std::shared_ptr<plane_t> fitted_plane(new plane_t);
        // td::cout << "In fitPlaneViaLeastSquares2\n";
        plane_t fitted_plane;

        size_t num_points = points.cols();
        Eigen::MatrixXf A = Eigen::MatrixXf::Ones(num_points, 3); // 1s, xs, ys
        A.col(1) = points.row(0); // xs
        A.col(2) = points.row(1); // ys
        Eigen::VectorXf z = points.row(2).transpose().eval(); // zs
        Eigen::MatrixXf b = A.bdcSvd(Eigen::ComputeThinU |
                     Eigen::ComputeThinV).solve(z);



        // Eigen::MatrixXf A = Eigen::MatrixXf::Random(2, 10); // 1s, xs, ys
        // Eigen::MatrixXf b = Eigen::MatrixXf::Random(1, 10); // ys
        // Eigen::VectorXf z = Eigen::MatrixXf::Random(1, 10).transpose().eval(); // zs
        // Eigen::MatrixXf b = A.bdcSvd(Eigen::ComputeThinU |
        //              Eigen::ComputeThinV).solve(z);

        // Eigen::JacobiSVD<Eigen::MatrixXf> svd(A, Eigen::ComputeThinU | Eigen::ComputeThinV);
        // Eigen::MatrixXf b = svd.solve(z); 

        // Eigen::MatrixXf pinv = A.completeOrthogonalDecomposition().pseudoInverse();
        // Eigen::MatrixXf b = pinv * z;

        // std::cout << "A: \n"  << A << std::endl;
        // std::cout << "b: \n"  << b << std::endl;

        // b1 * x + b2 * y - z + b0 = 0
        fitted_plane.normal_vector = Eigen::Vector3f(b(1), b(2), -1);
        fitted_plane.fixed_point = Eigen::Vector3f(0, 0, b(0));

        return fitted_plane;
        // return 0;
        // return std::make_shared<plane_t>(fitted_plane);
    }



} // plane
} // bipedlab


