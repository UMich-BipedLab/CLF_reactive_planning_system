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
 * \file     unicycle_nonholonomic_distance.cpp
 * \author   Bruce JK Huang, Jong Jin Park
 *
 * Definitions for nonholonomic distance functions
 */

#pragma once
#ifndef NONHOLONOMIC_DISTANCE_H
#define NONHOLONOMIC_DISTANCE_H 

#include "egocentric_coordinates.h"
#include <cassert>
#include <cmath>
#include <iostream>

namespace bipedlab
{

enum stabilizing_vector_field_type_t
{
    GRADIENT_DESCENT,
    SMOOTH_DESCENT
};


// distance to target in position space
double distance_on_manifold(double r, double phi, double kPhi)
{
    double weightedPhi = kPhi * phi;

    return sqrt(r * r + weightedPhi * weightedPhi);
}

// stabilizing vector fields
double gradient_descent_on_manifold(double r, double phi, double kPhi, double rangeEpsilon)
{
    if (r > rangeEpsilon) {
        return atan(-kPhi * phi / r / r);
    } else {
        return 0;   // avoid numerical error and align with target orientation
    }
}

double smooth_descent_on_manifold(double r, double phi, double kPhi, double rangeEpsilon)
{
    if (r > rangeEpsilon) {
        return atan(-kPhi * phi);
    } else {
        return 0;   // avoid numerical error and align with target orientation
    }
}

double stabilizing_delta_star(double r,
                              double phi,
                              double kPhi,
                              stabilizing_vector_field_type_t vectorFieldType,
                              double rangeEpsilon)
{
    double deltaStar;
    switch (vectorFieldType) {
    case GRADIENT_DESCENT:
        deltaStar = gradient_descent_on_manifold(r, phi, kPhi, rangeEpsilon);
        break;
    case SMOOTH_DESCENT:
        deltaStar = smooth_descent_on_manifold(r, phi, kPhi, rangeEpsilon);
        break;
    default:
        std::cout << "ERROR: UnicycleLyapunovChart: Unknown vector field type.\n";
        assert(false);
        break;
    }

    return deltaStar;
}


// distance to a target pose over the manifold equipped with stabilizing vector field
double distance_to_manifold(double r,
                            double phi,
                            double delta,
                            double kPhi,
                            double kDelta,
                            stabilizing_vector_field_type_t vectorFieldType,
                            double rangeEpsilon)
{
    double deltaStar = stabilizing_delta_star(r, phi, kPhi, vectorFieldType, rangeEpsilon);

    return kDelta * fabs(delta - deltaStar);
}


// distance to target pose in egocentric polar coordinates
double nonholonomic_distance(double r,
                                      double phi,
                                      double delta,
                                      double kPhi,
                                      double kDelta,
                                      stabilizing_vector_field_type_t vectorFieldType,
                                      double rangeEpsilon)
{
    double distanceOnManifold = distance_on_manifold(r, phi, kPhi);
    double distanceTOManifold = distance_to_manifold(r, phi, delta, kPhi, kDelta, vectorFieldType, rangeEpsilon);

    return distanceOnManifold + distanceTOManifold;
}

}   // namespace bipedlab

#endif /* ifndef NONHOLONOMIC_DISTANCE_H */
