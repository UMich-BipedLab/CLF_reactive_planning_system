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
#include "angle_functions.h"
#include "standalone_omni_lyapunov_distance.h"
#include "omni_ego_coordinates.h"
#include <cassert>
#include <cmath>
#include <algorithm>
#include <iostream>

namespace bipedlab
{
        

// distance to target in position space
double computeOmniDistanceOnManifold(const double r, const double delta, 
        const double gamma, const double beta, const omni_CLF_models_t& model)
{
    switch (model)
    {
        double weighted_delta;
        case VANILLA:
            weighted_delta = gamma * delta;
            return std::sqrt(r * r + weighted_delta * weighted_delta);
        case SMOOTH_VANILLA:
            weighted_delta = gamma * std::sin(delta);
            return 0.5 * (r * r + weighted_delta * weighted_delta);
        case SMOOTH_WEIGHTED_VANILLA:
            weighted_delta = gamma * std::sin(beta * delta);
            return 0.5 * (r * r + weighted_delta * weighted_delta);
        default:
            std::string error_msg = 
                "ERROR: in standalone omnilyapunov diatance and " 
                "computeOmniDistanceOnManifold(): "  "Unknown CLF model (" + 
                std::to_string(model) + ")";
            debugger::debugColorOutput(error_msg, "", 10, BR, BOLD);
            assert(false);
            exit(-1);
            break;
    }
}


control_variables_t 
changeOfControlVariables(const double r, const double delta, 
        const double v_r, const double v_delta, const double alpha)
{
    // prepare some useful values for faster computation
    double cos_delta = std::cos(delta);
    double sin_delta = std::sin(delta);
    double r_squared = r * r;
    double cos_delta_squared = cos_delta * cos_delta;

    // construct A matrix
    // Eigen::Matrix2d A;
    // A << cos_delta,  r * sin_delta, 
    //      sin_delta, -r * cos_delta;

    double omega_star = 
        -(r * cos_delta * (v_r * sin_delta - r * cos_delta * v_delta)) / 
        (alpha + r_squared *cos_delta_squared);

    double vx_star = 
        ((alpha * cos_delta * v_r) + (cos_delta * r_squared * v_r) + 
         (alpha * r * v_delta * sin_delta)) / 
        (alpha + cos_delta_squared * r_squared);

    double vy_star = (alpha * (v_r * sin_delta - r * cos_delta * v_delta)) / 
        (alpha + r_squared * cos_delta_squared);

    return control_variables_t(vx_star, vy_star, v_r, v_delta, omega_star, 0);
}


control_variables_t computeOmniStabilizingControl(const double r,
                             const double delta,
                             const double alpha,
                             const double beta,
                             const double k_r1,
                             const double k_r2,
                             const double k_delta1,
                             const double k_delta2,
                             const omni_CLF_solutions_t& solution_type)
{
    double v_r;
    double v_delta;
    switch (solution_type)
    {
        case VANILLA_SOL1: // 0
            v_r = r;
            v_delta = -delta;
            break;
        case VANILLA_SOL2: // 1
            v_r = k_r1 * (r / (k_r2 + r));
            v_delta = -k_delta1 * (r/ (k_delta2 + r)) * delta;
            break;
        case SMOOTH_VANILLA_SOL1: // 2
            v_r = k_r1 * (r / (k_r2 + r));
            v_delta = -k_delta1 * (r/ (k_delta2 + r)) * std::sin(2 * delta);
            break;
        case SMOOTH_VANILLA_SOL2: // 3
            v_r = k_r1 * (r / (k_r2 + r));
            v_delta = -k_delta1 * (r * r/ (k_delta2 + r * r)) * std::sin(2 * delta);
            break;
        case SMOOTH_WEIGHTED_VANILLA_SOL1: // 4
            v_r = k_r1 * (r / (k_r2 + r));
            v_delta = -(2.0/beta) * k_delta1 * 
                (r/ (k_delta2 + r)) * std::sin(2 * beta * delta);
            break;
        case SMOOTH_WEIGHTED_VANILLA_SOL2: // 5
            v_r = k_r1 * (r / (k_r2 + r));
            v_delta = -(2.0/beta) * 
                k_delta1 * (r * r / (k_delta2 + r * r)) * 
                std::sin(2 * beta * delta);
            break;
        default:
            debugger::debugColorOutput("SMOOTH_WEIGHTED_VANILLA_SOL1: ", 
                    SMOOTH_WEIGHTED_VANILLA_SOL1, 10);
            std::string error_msg = 
                "ERROR: in standalone omnilyapunov diatance and " 
                "computeOmniStabilizingControl(): Unknown CLF solution (" + 
                std::to_string(solution_type) + ")";
            debugger::debugExitColor(error_msg, __LINE__, __FILE__);
            break;
    }

    return changeOfControlVariables(r, delta, v_r, v_delta, alpha);
}


std::pair<int, double>
checkOutsideFovReturnModifiedDelta(const double delta, const double beta)
{
    double equilibria = computeOmniEquilibriaStandalone(beta);
    int outside_FOV = 0;
    double new_delta = delta;

    if ((delta >= 0) && (delta > equilibria))
    {
        new_delta = equilibria;
        outside_FOV = 1;
    }
    else if ((delta < 0) && (delta < -equilibria))
    {
        new_delta = -equilibria;
        outside_FOV = -1;
    }
    return {outside_FOV, new_delta};

}


double computeOmniEquilibriaStandalone(const double beta)
{
    return M_PI / (2 * beta);
}

// distance to a target pose over the manifold equipped with stabilizing vector field
double computeOmniDistanceToManifold(const double delta,
                            const double beta,
                            const double k_delta_to_manifold)
{
    double equilibria = computeOmniEquilibriaStandalone(beta);

    return k_delta_to_manifold * std::max(std::abs(delta) - equilibria, 0.0);
}


// distance to target pose in egocentric polar coordinates
double computeOmniNonholonomicDistance(const double r,
                            const double delta,
                            const double gamma,
                            const double beta,
                            const double k_delta_to_manifold,
                            const omni_CLF_models_t& model)
{
    double distance_on_manifold = computeOmniDistanceOnManifold(
            r, delta, gamma, beta, model);
    double distance_to_manifold = computeOmniDistanceToManifold(
            delta, beta, k_delta_to_manifold);

    return distance_on_manifold + distance_to_manifold;
}

void decideOmniCLFModel(omni_CLF_models_t& clf_model, int model)
{
    // switch (model)
    // {
    //     case 0:
    //         clf_model = VANILLA;
    //     case 1:
    //         clf_model = SMOOTH_VANILLA;
    //     case 2:
    //         clf_model = SMOOTH_WEIGHTED_VANILLA;
    //     default:
    //         std::string error_msg = 
    //             "ERROR: in standalone omnilyapunov diatance and " 
    //             "decideOmniCLFModel(): Unknown CLF model (" + 
    //             std::to_string(model) + ")";
    //         debugger::debugColorOutput(error_msg, "", 10, BR, BOLD);
    //         assert(false);
    //         exit(-1);
    //         break;
    // }

    if (model == 0)
        clf_model = VANILLA;
    else if (model == 1)
        clf_model = SMOOTH_VANILLA;
    else if (model == 2)
        clf_model = SMOOTH_WEIGHTED_VANILLA;
    else
    {
        std::string error_msg = 
            "ERROR: in standalone omnilyapunov diatance and " 
            "decideOmniCLFModel(): Unknown CLF model (" + 
            std::to_string(model) + ")";
        debugger::debugColorOutput(error_msg, "", 10, BR, BOLD);
        exit(-1);
    }
}

void decideOmniCLFSolution(omni_CLF_solutions_t& clf_solution, 
                           int solution_type)
{
    // switch does not work for some reason...
    // switch (solution_type)
    // {
    //     case 0:
    //         clf_solution = VANILLA_SOL1;
    //     case 1:
    //         clf_solution = VANILLA_SOL2;
    //     case 2:
    //         clf_solution = SMOOTH_VANILLA_SOL1;
    //     case 3: 
    //         clf_solution = SMOOTH_VANILLA_SOL2;
    //     case 4:
    //         clf_solution = SMOOTH_WEIGHTED_VANILLA_SOL1;
    //     case 5:
    //         clf_solution = SMOOTH_WEIGHTED_VANILLA_SOL2;
    //     default:
    //         std::string error_msg = 
    //             "ERROR: in standalone omnilyapunov diatance and " 
    //             "decideOmniCLFSolution(): Unknown CLF solution (" + 
    //             std::to_string(solution_type) + ")";
    //         debugger::debugColorOutput(error_msg, "", 10, BR, BOLD);
    //         assert(false);
    //         exit(-1);
    //         break;
    // }

    if (solution_type == 0)
        clf_solution = VANILLA_SOL1;
    else if (solution_type == 1)
        clf_solution = VANILLA_SOL2;
    else if (solution_type == 2)
        clf_solution = SMOOTH_VANILLA_SOL1;
    else if (solution_type == 3)
        clf_solution = SMOOTH_VANILLA_SOL2;
    else if (solution_type == 4)
        clf_solution = SMOOTH_WEIGHTED_VANILLA_SOL1;
    else if (solution_type == 5)
        clf_solution = SMOOTH_WEIGHTED_VANILLA_SOL2;
    else
    {
        std::string error_msg = 
            "ERROR: in standalone omnilyapunov diatance and " 
            "decideOmniCLFSolution(): Unknown CLF solution (" + 
            std::to_string(solution_type) + ")";
        debugger::debugColorOutput(error_msg, "", 10, BR, BOLD);
        exit(-1);
    }
}


}   // namespace bipedlab

