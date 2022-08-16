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
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>
#include "utils/plane.h"
#include "pose.h"



namespace bipedlab
{
    
namespace map_operation
{

/*!
 * Function to add noise to the elevation map. Noise added is white noise from a uniform distribution [-1:1] multiplied by
 * the amount of noise wanted specified by noise_on_map.
 *
 * @param map: grid map to which add the layers that contains the errors.
 * @param gridMapSize: Dimensions of grid map, passed as parameter to not being calculated every time.
 * @param noise_on_map: Amount of noise wanted in meters, can be set as an argument in the roslaunch phase.
 */
void addNoiseToMap(grid_map::GridMap& map,
        const grid_map::Size& gridMapSize,
        const double& noise_on_map,
        const std::string& add_to_layer,
        const std::string& output_layer);



void addNANToMap(grid_map::GridMap& map,
        const grid_map::Size& gridMapSize,
        const double& percentage,
        const std::string& add_to_layer,
        const std::string& output_layer);

void addNANHoleToMap(grid_map::GridMap& map,
        const grid_map::Size& gridMapSize,
        const double& percentage,
        const std::string& add_to_layer,
        const std::string& output_layer);

// output number of nan corrected
size_t removeNAN(grid_map::GridMap& map,
        const float nan_value,
        const std::string apply_to_layer);




/*!
 * Function to add outliers to the elevation map. Outliers are point where the elevation value is set to infinity.
 * It has to be performed after mapAddNoise.
 *
 * @param map: grid map to which add the layers that contains the errors.
 * @param gridMapSize: Dimensions of grid map, passed as parameter to not being calculated every time.
 * @param outlierPercentage: Amount of outliers wanted percentage, can be set as an argument in the roslaunch phase.
 */
// void addOutliersToMap(grid_map::GridMap& map, 
//                       const grid_map::Size& gridMapSize, 
//                       const double outlierPercentage,
//                       const std::string& add_to_layer);


/*!
 * Function to add outliers to the elevation map. Outliers are point where the elevation value is set to infinity.
 * It has to be performed after mapAddNoise.
 *
 * @param map: grid map to which add the layers that contains the errors.
 * @param gridMapSize: Dimensions of grid map, passed as parameter to not being calculated every time.
 * @param filterRadius: Radius of the wanted shifting average filter.
 */
void averageMapFiltering(grid_map::GridMap& map,
        const grid_map::Size& gridMapSize,
        const double filterRadius,
        const std::string apply_to_layer,
        const std::string& output_layer);



// average a cell by searching a circle around the cell
// skip any NAN value in the circle
// if percentage of nan in the circle is larger than the percentage, 
// the cell will become the nan_value
void averageNANFiltering(grid_map::GridMap& map,
        const grid_map::Size& gridMapSize,
        const double filterRadius,
        const std::string apply_to_layer,
        std::string output_layer,
        const double percentage,
        const double nan_value);


size_t checkNAN(grid_map::GridMap& map, const std::string check_layer);


void computeSlopeWRTRobotPose(grid_map::GridMap& map,
        const grid_map::Size& gridMapSize,
        const std::string apply_to_layer,
        std::string output_layer,
        const pose_t& robot_pose);


void assignObstacles(grid_map::GridMap& map,
        const grid_map::Size& gridMapSize,
        const std::string compute_on_layer,
        std::string output_layer,
        const double obstacle_threshold, bool& has_nan);

void computeSignedDistnaceField(grid_map::GridMap& map, 
        const grid_map::Size& gridMapSize, 
        const double search_radius,
        const std::string compute_on_layer, 
        const std::string apply_to_layer);


void assignUnknownBehindObstacles(grid_map::GridMap& map,
        const grid_map::Index& robot_indx,
        const double unknown_cost,
        const std::string compute_on_layer,
        const std::string apply_to_layer);

std::vector<plane::terrain_info_t>
averageMapFilteringAndcomputeTerrainPlaneInformation(grid_map::GridMap& map,
        const grid_map::Size& gridMapSize,
        const double filterRadius,
        const std::string apply_to_layer,
        std::string output_layer,
        const double percentage, // larger, the more be corrected
        const double nan_value,
        const pose_t& robot_pose,
        const double delta_x,
        const double delta_y,
        const size_t num_planes, 
        const size_t num_points_for_fitting);

std::vector<plane::terrain_info_t>
averageMapFilteringAndcomputeTerrainPlaneInformationAndTerrainInformation(
        grid_map::GridMap& map,
        const grid_map::Size& gridMapSize,
        const double filterRadius,
        const std::string apply_to_layer,
        std::string output_layer,
        const double percentage, // larger, the more be corrected
        const double nan_value,
        const pose_t& robot_pose,
        const double delta_x,
        const double delta_y,
        const size_t num_planes, 
        const size_t num_points_for_fitting);

// std::shared_ptr<std::vector<plane::terrain_info_t>>
// averageMapFilteringAndcomputeTerrainPlaneInformation(grid_map::GridMap& map,
//         const grid_map::Size& gridMapSize,
//         const double filterRadius,
//         const std::string apply_to_layer,
//         std::string output_layer,
//         const double percentage, // larger, the more be corrected
//         const double nan_value,
//         const pose_t& robot_pose,
//         const double delta_x,
//         const double delta_y,
//         const double num_planes);
// 
// std::shared_ptr<std::vector<plane::terrain_info_t>>
// averageMapFilteringAndcomputeTerrainPlaneInformationAndTerrainInformation(
//         grid_map::GridMap& map,
//         const grid_map::Size& gridMapSize,
//         const double filterRadius,
//         const std::string apply_to_layer,
//         std::string output_layer,
//         const double percentage, // larger, the more be corrected
//         const double nan_value,
//         const pose_t& robot_pose,
//         const double delta_x,
//         const double delta_y,
//         const double num_planes);


    
} /* map_operation */ 
} /* bipedlab */ 
