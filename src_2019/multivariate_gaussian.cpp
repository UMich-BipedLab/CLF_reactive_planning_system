/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     multivariate_gaussian.cpp
 * \author   Collin Johnson
 *
 * Definition of MultivariateGaussian.
 */

#include "multivariate_gaussian.h"
#include <cassert>

namespace bipedlab
{

MultivariateGaussian::MultivariateGaussian(size_t dimensions) : 
    mean(dimensions), covariance(dimensions, dimensions)
{
}


MultivariateGaussian::MultivariateGaussian(
        const Eigen::VectorXd& mean, const Eigen::MatrixXd& covariance, 
        const double height)
: mean(mean), 
  covariance(covariance),
  height(height)
{
    assert(covariance.allFinite());
    assert(mean.allFinite());
}


void MultivariateGaussian::setDistributionStatistics(
        const Eigen::VectorXd& mean, const Eigen::MatrixXd& covariance)
{
    this->mean = mean;
    this->covariance = covariance;

    // If the cholesky has already been computed for the distribution, 
    // chances are it will be used again, so do the calculation right away
    if (covariance_cholesky.rows() > 0) {
        MultivariateGaussian::prepareForSampling();
        // covariance_cholesky = Eigen::chol(covariance);
        // assert(covariance_cholesky.isFinite()());
        // assert(covariance_cholesky.size() > 0);
    }
}


Eigen::VectorXd MultivariateGaussian::sample(void) const
{
    Eigen::VectorXd sample_mean = 
        covariance_cholesky * Eigen::MatrixXd::Random(mean.rows(), mean.cols());
    return sample_mean + mean;
}


void MultivariateGaussian::prepareForSampling(void)
{
    Eigen::LLT<Eigen::MatrixXd> lltOfCov(covariance);
    covariance_cholesky = lltOfCov.matrixL();
    assert(covariance_cholesky.allFinite());
    assert(covariance_cholesky.size() > 0);
    if(lltOfCov.info() == Eigen::NumericalIssue)
    {
        throw std::runtime_error("Possibly non semi-positive definitie matrix!");
    }
}


double MultivariateGaussian::probability(const Eigen::VectorXd& vector) const
{
    Eigen::VectorXd error = vector - mean;
    Eigen::MatrixXd inv = covariance.inverse();
    Eigen::MatrixXd value = -0.5 * (error.transpose() * inv * error); // a scalar
    return height * exp(value(0));
}

double MultivariateGaussian::distribution(const Eigen::VectorXd& vector) const
{
    return MultivariateGaussian::probability(vector) / 
        std::sqrt(std::pow(2*M_PI, mean.rows()) * covariance.determinant());
}


double MultivariateGaussian::logPosterior(const Eigen::VectorXd& vector) const
{
    Eigen::VectorXd error = vector - mean;
    Eigen::MatrixXd value = 
        -0.5 * (error.transpose() * covariance.inverse()* error); // a scalar
    return height * value(0);
}

}   // namespace bipedlab

