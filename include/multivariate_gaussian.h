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
 * \file     multivariate_gaussian.h
 * \author   Bruce JK Huang, Collin Johnson
 *
 * Declaration of a simple MultivariateGaussian implementation.
 */

#ifndef MULTIVARIATE_GAUSSIAN_H
#define MULTIVARIATE_GAUSSIAN_H

#include <cereal/access.hpp>
#include <Eigen/Dense>
#include <Eigen/Cholesky>

namespace bipedlab
{

/**
 * MultivariateGaussian is a multi-dimensional Gaussian distribution described by a mean and covariance.
 *
 * Samples can be drawn from the distribution using the sample() method.
 */
class MultivariateGaussian
{
public:
    /**
     * Default constructor for MultivariateGaussian.
     *
     * \param    dimensions      Number of dimensions for the Gaussian (default = 2)
     */
    explicit MultivariateGaussian(std::size_t dimensions = 2);

    /**
     * Constructor for MultivariateGaussian.
     */
    MultivariateGaussian(const Eigen::VectorXd& mean, 
                         const Eigen::MatrixXd& covariance,
                         const double height = 1);

    /**
     * setDistributionStatistics sets statistics for the distribution. Both the mean
     * and covariance are required because it isn't a distribution otherwise!
     */
    void setDistributionStatistics(const Eigen::VectorXd& mean, const Eigen::MatrixXd& covariance);

    // Accessors

    Eigen::VectorXd getMean(void) const { return mean; }
    Eigen::MatrixXd getCovariance(void) const { return covariance; }
    std::size_t dimensions(void) const { return mean.rows(); }

    /**
     * sample draws a sample from the distribution.
     */
    Eigen::VectorXd sample(void) const;

    /**
     * prepareForSampling needs to be called before the first call to sample().
     */
    void prepareForSampling(void);

    /**
     * probability evaluates the probability of the PDF at the specified value.
     */
    double probability(const Eigen::VectorXd& value) const;


    /**
     * distribution evaluates the distribution of the PDF at the specified value.
     */
    double distribution(const Eigen::VectorXd& vector) const;


    /**
     * logPosterior evaluates the natural log of probability of the PDF at the specified value.
     */
    double logPosterior(const Eigen::VectorXd& value) const;

    // Operator overloads:

    /**
     * operator[] provides access to the corresponding element of the mean vector for the Gaussian.
     *
     * \pre  0 <= n < dimensions()
     */
    double operator[](int n) const { return mean(n); }
    double& operator[](int n) { return mean(n); }

    /**
     * operator() provides access to the corresponding element of the covariance matrix for the Gaussian.
     *
     * \pre  0 <= row < dimensions()
     * \pre  0 <= col < dimensions()
     */
    double operator()(int row, int col) const { return covariance(row, col); }
    double& operator()(int row, int col) { return covariance(row, col); }
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
    double height;
    Eigen::VectorXd mean;
    Eigen::MatrixXd covariance;

    // Cholesky factorization of covariance -- used for sampling
    Eigen::MatrixXd covariance_cholesky;   

    friend class ::cereal::access;

    template <class Archive>
    void serialize(Archive& ar, const unsigned int version)
    {
        ar& mean;
        ar& covariance;
    }
};

}   // namespace bipedlab

#endif   // MULTIVARIATE_GAUSSIAN_H

