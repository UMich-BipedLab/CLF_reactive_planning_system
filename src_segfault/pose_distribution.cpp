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
 * \file     pose_distribution.cpp
 * \author   Bruce JK Huang, Collin Johnson
 *
 * Definition of pose_distribution_t.
 */

#include "pose_distribution.h"
#include "float_comparison.h"

namespace bipedlab
{

pose_distribution_t pose_distribution_t::compound(const pose_distribution_t& origin) const
{
    pose_distribution_t newDist;
    newDist.x = origin.x + (x * std::cos(origin.theta)) - (y * std::sin(origin.theta));
    newDist.y = origin.y + (x * std::sin(origin.theta)) + (y * std::cos(origin.theta));
    newDist.theta = angle_sum(origin.theta, theta);

    Matrix jThis = {{std::cos(origin.theta), -std::sin(origin.theta), 0.0},
                    {std::sin(origin.theta), std::cos(origin.theta), 0.0},
                    {0.0, 0.0, 1.0}};

    Matrix jOrigin = {{1.0, 0.0, (-x * std::sin(origin.theta)) - (y * std::cos(origin.theta))},
                      {0.0, 1.0, (x * std::cos(origin.theta)) - (y * std::sin(origin.theta))},
                      {0.0, 0.0, 1.0}};

    Vector mean = {newDist.x, newDist.y, newDist.theta};
    Matrix cov(3, 3);
    cov.zeros();

    cov += jOrigin * origin.uncertainty.getCovariance() * arma::trans(jOrigin);
    cov += jThis * uncertainty.getCovariance() * arma::trans(jThis);
    newDist.uncertainty.setDistributionStatistics(mean, cov);

    return newDist;
}


bool operator==(const pose_distribution_t& lhs, const pose_distribution_t& rhs)
{
    return absolute_fuzzy_equal(lhs.x, rhs.x) && absolute_fuzzy_equal(lhs.y, rhs.y)
      && absolute_fuzzy_equal(lhs.theta, rhs.theta);
}


bool operator!=(const pose_distribution_t& lhs, const pose_distribution_t& rhs)
{
    return !(lhs == rhs);
}

}   // namespace bipedlab

