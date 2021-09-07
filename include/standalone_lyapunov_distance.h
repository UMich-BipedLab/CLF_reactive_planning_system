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
 * \file     unicycle_nonholonomic_distance.h
 * \author   Jong Jin Park
 *
 * Declaration of distance functions relavent for unicycle-type vehicles with
 * non-holonomic constraints (that can rotate in place but cannot move sideways).
 * Supports two (analytically describable) stabilizing vector field types.
 * Uses egocentric polar coordinates.
 */

#ifndef STANDALONE_LYAPUNOV_DISTANCE_H
#define STANDALONE_LYAPUNOV_DISTANCE_H

namespace bipedlab
{

enum stabilizing_vector_field_type_t
{
    GRADIENT_DESCENT,
    SMOOTH_DESCENT
};

// distance to target in position space
double distance_on_manifold(double r, double phi, double kPhi);

// stabilizing vector fields
double gradient_descent_on_manifold(double r, double phi, double kPhi, double rangeEpsilon = 0.05);

double smooth_descent_on_manifold(double r, double phi, double kPhi, double rangeEpsilon = 0.05);

double stabilizing_delta_star(double r,
                              double phi,
                              double kPhi,
                              stabilizing_vector_field_type_t vectorFieldType,
                              double rangeEpsilon = 0.05);


double computeOmniEquilibriaStandalone(double beta);


// distance to a target pose over the manifold equipped with stabilizing vector field
double distance_to_manifold(double r,
                            double phi,
                            double delta,
                            double kPhi,
                            double kDelta,
                            stabilizing_vector_field_type_t vectorFieldType,
                            double rangeEpsilon = 0.05);

// distance to target pose in egocentric polar coordinates
double nonholonomic_distance(double r,
                             double phi,
                             double delta,
                             double kPhi,
                             double kDelta,
                             stabilizing_vector_field_type_t vectorFieldType,
                             double rangeEpsilon = 0.05);

void decideStabilizingVectorFieldType(
        stabilizing_vector_field_type_t& vector_field_type, int mode);


}   // namespace bipedlab

#endif   // STANDALONE_LYAPUNOV_DISTANCE_H

