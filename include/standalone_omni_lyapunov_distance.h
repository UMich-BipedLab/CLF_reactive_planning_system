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

#ifndef STANDALONE_OMNI_LYAPUNOV_DISTANCE_H
#define STANDALONE_OMNI_LYAPUNOV_DISTANCE_H

#include "control_variables.h"
#include "utils/debugger.h"


namespace bipedlab
{
enum omni_CLF_models_t
{
    VANILLA, // 0
    SMOOTH_VANILLA, // 1
    SMOOTH_WEIGHTED_VANILLA // 2
};

enum omni_CLF_solutions_t
{
    VANILLA_SOL1, // 0
    VANILLA_SOL2, // 1
    SMOOTH_VANILLA_SOL1, // 2
    SMOOTH_VANILLA_SOL2, // 3
    SMOOTH_WEIGHTED_VANILLA_SOL1, //4
    SMOOTH_WEIGHTED_VANILLA_SOL2 // 5
};


// distance to target in position space
double computeOmniDistanceOnManifold(const double r, const double delta,
        const double gamma, const double beta, const omni_CLF_models_t& model);


// chnage v_r and v_delta to v_x and v_y
control_variables_t 
changeOfControlVariables(const double r, const double delta, 
        const double v_r, const double v_delta, const double alpha);


control_variables_t 
computeOmniStabilizingControl(const double r,
                         const double delta,
                         const double alpha,
                         const double beta,
                         const double k_r1,
                         const double k_r2,
                         const double k_delta1,
                         const double k_delta2,
                         const omni_CLF_solutions_t& solution_type);

// first: 
// +1: rotated to pos equilibria
// 0: original inside the FOV
// -1: rotated to neg quilibria
// second: resulting delta
std::pair<int, double>
checkOutsideFovReturnModifiedDelta(const double delta, const double beta);

double computeOmniEquilibriaStandalone(const double beta);


// distance to a target pose over the manifold equipped with stabilizing vector field
double computeOmniDistanceToManifold(const double delta,
                            const double beta,
                            const double k_delta_to_manifold);


// distance to target pose in egocentric polar coordinates
double computeOmniNonholonomicDistance(const double r,
                            const double delta,
                            const double gamma,
                            const double beta,
                            const double k_delta_to_manifold,
                            const omni_CLF_models_t& model);

void decideOmniCLFModel(omni_CLF_models_t& clf_model, int model);

void decideOmniCLFSolution(omni_CLF_solutions_t& clf_solution, int solution_type);


}   // namespace bipedlab

#endif   // STANDALONE_LYAPUNOV_DISTANCE_H

