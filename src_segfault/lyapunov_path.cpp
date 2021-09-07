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
 * \file     unicycle_lyapunov_steering.cpp
 * \author   Jong Jin Park
 *
 * Definition of numerically stable steering via lyapunov function.
 */

#include "lyapunov_path.h"
// #include "lyapunov_distance.h"

namespace bipedlab
{
LyapunovPath::LyapunovPath(const lyapunov_path_params_t& params,
                           LyapunovDistance& lyap_dis,
                           const LocalMap& local_map,
                           const MapCost& cost,
                           const size_t& mode) : 
params_(params), mode_(mode)
{
    lyap_dis_ = &lyap_dis;
    local_map_ = &local_map;
    map_cost_ = &cost;
}

LyapunovPath::LyapunovPath(LyapunovDistance& lyap_dis, 
                           const LocalMap& local_map,
                           const MapCost& cost,
                           const size_t& mode):
mode_(mode)
{
    lyap_dis_ = &lyap_dis;
    local_map_ = &local_map;
    map_cost_ = &cost;
}

LyapunovPath::LyapunovPath(LyapunovDistance& lyap_dis,
                           const size_t& mode) :
mode_(mode)
{
    local_map_ = nullptr;
    map_cost_ = nullptr;
    lyap_dis_ = &lyap_dis;
}


path_segment_t LyapunovPath::extend(const pose_t& robot_pose,
                                    const pose_t& end_pose,
                                    double max_extension) const
{

    // debugger::debugTextOutput("[LyapPath] assinging new pose", 3);
    lyap_dis_->assignNewTargetPose(end_pose);

    std::vector<pose_t> steps;
    // debugger::debugTextOutput("[LyapPath] pushing the pose to steps vector", 3);
    steps.push_back(robot_pose);

    std::vector<point2d_t<float>> path;
    // debugger::debugTextOutput("[LyapPath] convering it to 2D points", 3);
    path.push_back(robot_pose.to2DPosition());
    // debugger::debugOutput("[LyapPath] start: ", robot_pose, 3);
    // debugger::debugOutput("[LyapPath] end: ", end_pose, 3);
    // debugger::debugOutput("[LyapPath] max extension: ", max_extension, 3);

    cost_t cost_along_path;

    // rotate onto the vector field
    bool is_aligned = false;
    while (!is_aligned) {
        // rotation onto vector field (isAligned is updated here)
        pose_t rotated_pose = incrementalRotation_(steps.back(), 
                params_.incrementalRotation, &is_aligned);
        // std::cout<< "is rotating at: "<< rotated_pose << "    with increments: " <<
        //     params_.incrementalRotation <<'\n';

        steps.push_back(rotated_pose);
        // path doesn't have to be updated since only rotating-in-place
    }
    // debugger::debugOutput("[LyapPath] Finished rotated pose: ", steps.back(), 3);
    // debugger::debugOutput("[LyapPath] translation param: ", params_.incrementalTranslation, 3);

    // once successfully rotated onto the vector field, move along the vector field
    bool is_converged = false;
    pose_t translated_pose;
    cost_t pre_cost = map_cost_->computePositionMapCost(robot_pose);
    cost_t current_cost;
    double distance_traveled = 0.0;
    // debugger::debugOutput("[LyapPath] mode:", mode_, 3);
    while (!is_converged) {
        // translation along vector field (path and isConverged are updated here)
        translated_pose = incrementalTranslation_(
                  steps.back(), params_.incrementalTranslation, 
                  &path, &is_converged);
        // std::cout<< "is translating at: "<< translated_pose << "    with increments: " << params_.incrementalTranslation <<'\n';

        steps.push_back(translated_pose);
        if (mode_ != 0)
        {
            // compute the difference
            current_cost = map_cost_->computePositionMapCost(translated_pose);
            cost_along_path = cost_along_path + 
                pre_cost.takeAbsoluteDifference(current_cost);
            // debugger::debugTextOutput("[Lyap Path] ===========", 4);
            // debugger::debugOutput("[Lyap Path] Current cost: ", 
            //                       current_cost, 4);
            // debugger::debugOutput("[Lyap Path] cost along path: ", cost_along_path, 4);
            pre_cost = current_cost;
        }

        if (mode_ != 0 || max_extension > 0.0) {
            egocentric_polar_coords_t coords = 
                local_chart::pose2rpd(robot_pose, translated_pose);

            // LyapunovDistance traveled(translated_pose, lyap.params_);
            // double distanceTraveled = traveled.distanceFromPose(translated_pose);
            distance_traveled = 
                 nonholonomic_distance(coords.r,
                                       coords.phi,
                                       coords.delta,
                                       lyap_dis_->params_.k_phi,
                                       lyap_dis_->params_.k_delta,
                                       lyap_dis_->params_.vector_field_type,
                                       lyap_dis_->params_.small_radius);
            cost_along_path.distance = distance_traveled;
            // when max_extension is positive
            if (max_extension > 0.0)   
            {
                if (distance_traveled > max_extension)   // terminate if moved sufficiently far away
                {
                    break;
                }
            }
        }

    }
    // debugger::debugOutput("[Lyap Path] Finished translated pose: ", steps.back(), 3);

    return path_segment_t{steps, path, cost_along_path};
}


path_segment_t LyapunovPath::steer(const pose_t& robot_pose, 
                                   const pose_t& end_pose) const
{
    return this->extend(robot_pose, end_pose, -1);   // extend without length constraint
}


std::vector<point2d_t<float>> 
LyapunovPath::steerAway(const point2d_t<float>& point, double max_extension) const
{
    // TODO: Implement this!
    std::vector<point2d_t<float>> temp;
    return temp;
}


pose_t LyapunovPath::incrementalRotation_(const pose_t& robot_pose,
                                         double maxRotation,
                                         bool* isAligned) const
{
    pose_t nextPose = robot_pose;

    // find reference heading induced by the laypunov function
    double referenceHeading = lyap_dis_->stabilizeHeading(robot_pose.toPoint());
    // debugger::debugOutput("[lyap path/rot] point: ", robot_pose.toPoint(), 3);
    // debugger::debugOutput("[lyap path/rot] reference heading: ", referenceHeading, 3);
    double headingError = angle_diff(referenceHeading, robot_pose.theta);

    if (fabs(headingError) > maxRotation) {
        *isAligned = false;
        nextPose.theta = nextPose.theta + copysign(maxRotation, headingError);
    } else {
        *isAligned = true;
        nextPose.theta = wrap_to_2pi(referenceHeading);
    }

    return nextPose;
}


pose_t LyapunovPath::incrementalTranslation_(const pose_t& robot_pose, // point
                                            double maxTranslation,
                                            std::vector<point2d_t<float>>* points,
                                            bool* isConverged) const
{
    // hard-coded parameters
    const double kIntegrationStepSize = 0.01;
    const double kConvergedRadius = 0.05;
    // size_t kMaxNumSteps = maxTranslation / kIntegrationStepSize * 1000;
    size_t kMaxNumSteps = maxTranslation / kIntegrationStepSize;
    size_t numSteps = 0;

    // map to reduced egocentric polar coords
    reduced_egocentric_polar_coords_t coords = 
        lyap_dis_->local_chart_.point2rp(robot_pose.toPoint());
    // debugger::debugOutput("Lyap Path: target pose: ", lyap_dis_->getTargetPose(), 3);
    // debugger::debugOutput("Lyap Path: point: ", robot_pose.toPoint(), 3);
    // debugger::debugOutput("Lyap Path: point2rp.r: ", coords.r, 3);
    // debugger::debugOutput("Lyap Path: point2rp.phi: ", coords.phi, 3);
    // debugger::debugOutput("Lyap Path: rp2point.phi: ", lyap_dis_->local_chart_.rp2point(coords), 3);


    float deltaStar = stabilizing_delta_star(coords.r,
            coords.phi,
            lyap_dis_->params_.k_phi,
            lyap_dis_->params_.vector_field_type,
            lyap_dis_->params_.small_radius);
    // debugger::debugOutput("Lyap Path: k_pi: ", lyap_dis_->params_.k_phi , 3);
    // debugger::debugOutput("Lyap Path: vector_field_type: ", lyap_dis_->params_.vector_field_type, 3);
    // debugger::debugOutput("Lyap Path: small_radius: ", lyap_dis_->params_.small_radius, 3);
    // debugger::debugOutput("Lyap Path: deltaStar: ", deltaStar, 3);



    while (true) {
        if (points) {
            points->push_back(lyap_dis_->local_chart_.rp2point(coords));   // store trajectory in Cartesian coords
            // std::cout << "translated point: " << lyap_dis_->local_chart_.rp2point(coords) << std::endl;
        }

        // termination conditions
        numSteps++;
        double distance = distance_on_manifold(
                                               coords.r, coords.phi, 
                                               lyap_dis_->params_.k_phi);
        if (coords.r == 0)
        {
            *isConverged = true;
            break;
        }

        *isConverged =  distance < kConvergedRadius;
        if (*isConverged || numSteps >= kMaxNumSteps) {
            break;
        }

        // align heading along the vector field
        float deltaStar = stabilizing_delta_star(coords.r,
                                                 coords.phi,
                                                 lyap_dis_->params_.k_phi,
                                                 lyap_dis_->params_.vector_field_type,
                                                 lyap_dis_->params_.small_radius);

        // step forward along the heading
        // x-axis crossing and passing-the-origin are numerical errors, so modify step sizes to prevent that.
        double ds = std::min(kIntegrationStepSize, coords.r);   // prevents passing-the-origin
        double dr = -ds * std::cos(deltaStar);
        double dphi = ds / coords.r * std::sin(deltaStar);   // compute ds/r first for numerical stability

        // debugger::debugTextOutput ("===== 1", 3);
        // debugger::debugOutput("distance: ", distance, 3);
        // debugger::debugOutput("ds: ", ds, 3);
        // debugger::debugOutput("dr: ", dr, 3);
        // debugger::debugOutput("dphi: ", dphi, 3);

        if (coords.phi * (coords.phi + dphi) < 0)   // detect r-axis crossing
        {
            dr = std::fabs(coords.phi / dphi) * dr;   // scale change in r
            dphi = -coords.phi;                  // to snap to x-axis
        }

        // debugger::debugTextOutput ("===== 2", 3);
        // debugger::debugOutput("ds: ", ds, 3);
        // debugger::debugOutput("dr: ", dr, 3);
        // debugger::debugOutput("dphi: ", dphi, 3);

        // if (!std::isfinite(coords.r) || !std::isfinite(coords.phi))
        // {
        //     debugger::debugTextOutput ("================", 3);
        //     debugger::debugOutput("isfinite: ", std::isfinite(coords.r), 3);
        //     debugger::debugOutput("isfinite: ", std::isfinite(coords.phi), 3);
        //     debugger::debugOutput("coords.r: ", coords.r, 3);
        //     debugger::debugOutput("coords.phi: ", coords.phi, 3);
        //     debugger::debugOutput("ds: ", ds, 3);
        //     debugger::debugOutput("dr: ", dr, 3);
        //     debugger::debugOutput("dphi: ", dphi, 3);
        //     debugger::debugTextCerr("numerical error", 4);
        //     debugger::debugTextOutput ("================", 3);
        // }

        // update
        coords.r += dr;
        coords.phi += dphi;

        // debugger::debugTextOutput ("===== 3", 3);
        // debugger::debugOutput("coords.r: ", coords.r, 3);
        // debugger::debugOutput("coords.phi: ", coords.phi, 3);
    }

    // final pose
    point2d_t translated_point = points->back(); 
    reduced_egocentric_polar_coords_t new_coords = 
        lyap_dis_->local_chart_.point2rp(translated_point);
    double deltaAtEndPoint = stabilizing_delta_star(new_coords.r,
                                                    new_coords.phi,
                                                    lyap_dis_->params_.k_phi,
                                                    lyap_dis_->params_.vector_field_type,
                                                    lyap_dis_->params_.small_radius);
    pose_t endPose = pose_t(translated_point.x, translated_point.y, 
                            deltaAtEndPoint + new_coords.lineOfSightAngle);
    // std::cout << "delta end_point: " << deltaAtEndPoint << std::endl;

    // egocentric_polar_coords_t extendedCoords = {coords.r, deltaAtEndPoint + coords.lineOfSightAngle, coords.phi, coords.lineOfSightAngle};
    // pose_t endPose = lyap_dis_->local_chart_.rpd2pose(extendedCoords);

    return endPose;
}
} // namespace bipedlab

