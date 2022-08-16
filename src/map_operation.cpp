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
#include "utils/debugger.h"
#include "utils/utils.h"
#include "utils/line.h"
#include <eigen3/Eigen/Dense>
#include "map_operation.h"
#include "terrain_properties.h"
#include "robot_state.h"
#include <cmath>

namespace bipedlab
{
namespace map_operation
{


void addNoiseToMap(grid_map::GridMap& map, 
        const grid_map::Size& gridMapSize, 
        const double& noise_on_map,
        const std::string& add_to_layer,
        const std::string& output_layer) 
{
    // Add noise (using Eigen operators).
    if (!map.exists(output_layer))
        map.add(output_layer,  Eigen::MatrixXf::Zero(gridMapSize(0), gridMapSize(1)));
    const grid_map::Matrix noise = noise_on_map * 
                Eigen::MatrixXf::Random(gridMapSize(0), gridMapSize(1));
    map.add(output_layer, map.get(add_to_layer) + noise);
}

void addNANToMap(grid_map::GridMap& map, 
        const grid_map::Size& gridMapSize, 
        const double& percentage,
        const std::string& add_to_layer,
        const std::string& output_layer) 
{
    // Add percentage (using Eigen operators).
    if (!map.exists(output_layer))
        map.add(output_layer, Eigen::MatrixXf::Zero(gridMapSize(0), gridMapSize(1)));

    // int num_nan = 0;
    auto& data_to = map[output_layer];
    for (grid_map::GridMapIterator it(map); !it.isPastEnd(); ++it) 
    {
        const size_t i = it.getLinearIndex();
        // grid_map::Position position;
        // map.getPosition(*it, position);
        if (utils::genInclusiveRandomNumber(0, 1) < percentage)
        {
            data_to(i) = std::numeric_limits<double>::quiet_NaN();
            // map.at(output_layer, *it) = std::numeric_limits<double>::quiet_NaN();
            // num_nan++;
        }
    }
    // debugger::debugOutput("num nan: ", num_nan, 5);
    // debugger::debugOutput("percentage: ", 
    // (double)num_nan/ (gridMapSize(0) * gridMapSize(1)), 5);

    // const grid_map::Matrix noise = map["noise"];
    // map.add("noisy_" + add_to_layer, map.get(add_to_layer) + noise);
    // map.add("noisy_" + add_to_layer, map.get(add_to_layer) + map["noise"]);
}

void addNANHoleToMap(grid_map::GridMap& map, 
        const grid_map::Size& gridMapSize, 
        const double& percentage,
        const std::string& add_to_layer,
        const std::string& output_layer) 
{
    if (!map.exists(output_layer))
        map.add(output_layer, Eigen::MatrixXf::Zero(gridMapSize(0), gridMapSize(1)));

    for (grid_map::GridMapIterator it(map); !it.isPastEnd(); ++it) 
    {
        grid_map::Position position;
        map.getPosition(*it, position);
        if (utils::genInclusiveRandomNumber(0, 1) < percentage)
        {
            double radius = utils::genInclusiveRandomNumber(0, 0.3);
            for (grid_map::CircleIterator circleIt(map, position, radius); 
                    !circleIt.isPastEnd(); ++circleIt) 
            {
                map.at(output_layer, *circleIt) = std::numeric_limits<double>::quiet_NaN();
            }
        }
    }
}

// void addOutliersToMap(grid_map::GridMap& map, 
//                     const grid_map::Size& gridMapSize, 
//                     const double outlierPercentage,
//                     const std::string& add_to_layer) 
// {
//     // Adding outliers at infinite height (accessing cell by position).
//     const double numberInfPoints = 
//         outlierPercentage * gridMapSize(0) * gridMapSize(1);
// 
//     for (int i = 0; i < static_cast<int>(numberInfPoints); ++i) 
//     {
//         grid_map::Position randomPosition = grid_map::Position::Random();
//         if (map.isInside(randomPosition)) 
//         {
//             map.atPosition("noisy_" + add_to_layer, randomPosition) = 
//                 std::numeric_limits<float>::infinity();
//         }
//     }
// }


// output number of nan corrected
size_t removeNAN(grid_map::GridMap& map, 
        const float nan_value,
        const std::string apply_to_layer)
{
    size_t num_NAN = 0;
    auto& data_from = map[apply_to_layer];

    // Iterate over whole map.
    for (grid_map::GridMapIterator it(map); !it.isPastEnd(); ++it) 
    {
        const size_t i = it.getLinearIndex();
        if (!std::isfinite(data_from(i)))
        {
            data_from(i) = nan_value;
            num_NAN++;
        }
    }

    return num_NAN;
}

void averageMapFiltering(grid_map::GridMap& map, 
        const grid_map::Size& gridMapSize, 
        const double filterRadius,
        const std::string apply_to_layer,
        const std::string& output_layer) 
{
    // grid_map::Index startIndex(0, 0);
    // grid_map::SubmapIterator it(map, startIndex, gridMapSize);

    if (!map.exists(output_layer))
        map.add(output_layer, Eigen::MatrixXf::Zero(gridMapSize(0), gridMapSize(1)));

    // Iterate over whole map.
    auto& data_to = map[output_layer];
    // for (; !it.isPastEnd(); ++it) {
    for (grid_map::GridMapIterator it(map); !it.isPastEnd(); ++it) 
    {
        grid_map::Position currentPosition;
        map.getPosition(*it, currentPosition);
        double mean = 0.0;
        double sumOfWeights = 0.0;

        // Compute weighted mean.
        for (grid_map::CircleIterator circleIt(map, currentPosition, filterRadius); 
                !circleIt.isPastEnd(); ++circleIt) 
        {
            if (!map.isValid(*circleIt, apply_to_layer)) 
            {
                continue;
            }
            grid_map::Position currentPositionInCircle;
            map.getPosition(*circleIt, currentPositionInCircle);

            // Computed weighted mean based on Euclidian distance.
            double distance = (currentPosition - currentPositionInCircle).norm();
            double weight = pow(filterRadius - distance, 2);
            mean += weight * map.at(apply_to_layer, *circleIt);
            sumOfWeights += weight;
        }

        // apply filter
        const size_t i = it.getLinearIndex();
        if (sumOfWeights != 0) 
        {
            data_to(i) = mean / sumOfWeights;
        } 
        else 
        {
            data_to(i) = 1e5;
            debugger::debugColorTextOutput("[map_operation]/[averageMapFiltering] "
                    "sumOfWeights is zero", 10, Y);
        }
    }
}

void averageNANFiltering(grid_map::GridMap& map, 
        const grid_map::Size& gridMapSize, 
        const double filterRadius,
        const std::string apply_to_layer,
        std::string output_layer,
        const double percentage, // larger, the more be corrected
        const double nan_value) 
{
    // grid_map::Index startIndex(0, 0);
    // grid_map::SubmapIterator it(map, startIndex, gridMapSize);

    if (output_layer.size()==0)
        output_layer  = "filtered_" + apply_to_layer;

    if (!map.exists(output_layer))
        map.add(output_layer, Eigen::MatrixXf::Zero(gridMapSize(0), gridMapSize(1)));


    // Iterate over whole map.
    auto& data_to = map[output_layer];
    // for (; !it.isPastEnd(); ++it) {
    for (grid_map::GridMapIterator it(map); !it.isPastEnd(); ++it) 
    {
        grid_map::Position currentPosition;
        map.getPosition(*it, currentPosition);

        // assign NAN with nan_value
        if (!map.isValid(*it, apply_to_layer))
            map.atPosition(output_layer, currentPosition) = nan_value;

        double mean = 0.0;
        double sumOfWeights = 0.0;

        // Compute weighted mean.
        size_t num_cell_in_circle = 0;
        size_t num_nan_cell = 0;
        for (grid_map::CircleIterator circleIt(map, currentPosition, filterRadius); 
                !circleIt.isPastEnd(); ++circleIt) 
        {
            num_cell_in_circle++;
            if (!map.isValid(*circleIt, apply_to_layer)) 
            {
                num_nan_cell++;
               continue; 
            }
            grid_map::Position currentPositionInCircle;
            map.getPosition(*circleIt, currentPositionInCircle);

            // Computed weighted mean based on Euclidian distance.
            double distance = (currentPosition - currentPositionInCircle).norm();
            double weight = pow(filterRadius - distance, 2);
            mean += weight * map.at(apply_to_layer, *circleIt);
            sumOfWeights += weight;
        }
        double ratio = (double) num_nan_cell / num_cell_in_circle;

        // apply filter
        const size_t i = it.getLinearIndex();
        if (sumOfWeights != 0 && ratio <= percentage) 
        {
            data_to(i) = mean / sumOfWeights;
        } 
        else 
        {
            data_to(i) = nan_value;
            debugger::debugColorTextOutput("[map_operation]/[averageNANFiltering] "
                    "hole too big, remain large value", 4, Y);
        }
    }
}

size_t checkNAN(grid_map::GridMap& map, 
        const std::string check_layer)
{
    size_t num_NAN = 0;
    // Iterate over whole map.
    auto& data_from = map[check_layer];
    for (grid_map::GridMapIterator it(map); !it.isPastEnd(); ++it) 
    {
        const size_t i = it.getLinearIndex();
        if (!std::isfinite(data_from(i)))
            num_NAN++;
    }
    return num_NAN;
}


void computeSlopeWRTRobotPose(grid_map::GridMap& map, 
        const grid_map::Size& gridMapSize, 
        const std::string compute_on_layer,
        std::string output_layer,
        const pose_t& robot_pose)
{
    if (!map.exists(output_layer))
    {
        map.add(output_layer, 
                Eigen::MatrixXf::Zero(gridMapSize(0), gridMapSize(1)));
    }

    auto& data_from = map[compute_on_layer];
    auto& data_to = map[output_layer];
    for (grid_map::GridMapIterator it(map); !it.isPastEnd(); ++it) 
    {
        const size_t i = it.getLinearIndex();
        grid_map::Position current_position;
        map.getPosition(*it, current_position);
        double height = data_from(i);
        // double distance = (robot_pose.getPosition() - 
        //                    Eigen::Vector3d(current_position(0), 
        //                                    current_position(1), 
        //                                    height)).norm();
        double distance = (robot_pose.get2DPosition() - current_position).norm();
        double slope = height - robot_pose.z;
        data_to(i) = slope;

        // grid_map::Position3 current_position;
        // map.getPosition3(compute_on_layer, *it, current_position);
        // double distance = (robot_pose.getPosition() - current_position).norm();
        // double slope = current_position(2) - robot_pose.z;
    }
}

// assign 1 if occupied
void assignObstacles(grid_map::GridMap& map, 
        const grid_map::Size& gridMapSize, 
        const std::string compute_on_layer,
        std::string output_layer, 
        const double obstacle_threshold, bool& has_nan)
{
    if (!map.exists(output_layer))
    {
        map.add(output_layer, 
                Eigen::MatrixXf::Zero(gridMapSize(0), gridMapSize(1)));
    }

    auto& data_from = map[compute_on_layer];
    auto& data_to = map[output_layer];
    for (grid_map::GridMapIterator it(map); !it.isPastEnd(); ++it) 
    {
        const size_t i = it.getLinearIndex();
        grid_map::Position current_position;
        map.getPosition(*it, current_position);


        double height = data_from(i);
        if (height < obstacle_threshold)
            data_to(i) = 0;
        else
            data_to(i) = 1;


        if (!std::isfinite(height))
            has_nan = true;
    }
}

void computeSignedDistnaceField(grid_map::GridMap& map, 
        const grid_map::Size& gridMapSize, 
        const double search_radius,
        const std::string compute_on_layer, 
        const std::string apply_to_layer)
{
    // Iterate over whole map.
    auto& data_from = map[compute_on_layer];
    auto& data_to = map[apply_to_layer];
    for (grid_map::GridMapIterator it(map); !it.isPastEnd(); ++it) 
    {
        grid_map::Position currentPosition;
        map.getPosition(*it, currentPosition);

        // Compute SDF 
        for (grid_map::SpiralIterator circleIt(map, currentPosition, search_radius); 
                !circleIt.isPastEnd(); ++circleIt) 
        {
            if (!map.isValid(*circleIt, apply_to_layer)) 
            {
                continue;
            }

            if (map.at(compute_on_layer, *circleIt) == 1)
            {
                grid_map::Position currentPositionInCircle;
                map.getPosition(*circleIt, currentPositionInCircle);

                // Computed weighted mean based on Euclidian distance.
                double distance = (currentPosition - currentPositionInCircle).norm();
                const size_t i = it.getLinearIndex();
                data_to(i) = 1.0 / std::min(distance, 0.01);
                break;
            }
        }
    }
}

void assignUnknownBehindObstacles(grid_map::GridMap& map, 
        const grid_map::Index& robot_indx,
        const double unknown_cost, 
        const std::string compute_on_layer, 
        const std::string apply_to_layer)
{
    // Iterate over whole map.
    auto& data_from = map[compute_on_layer];
    auto& data_to = map[apply_to_layer];


    // debugger::debugColorOutput(
    //         "[map_operation] "
    //         "robot index \n", robot_indx, 10, Y);

    for (grid_map::GridMapIterator it(map); !it.isPastEnd(); ++it) 
    {
        const size_t i = it.getLinearIndex();
        if (!std::isfinite(data_to(i)))
            continue;

       //  grid_map::Position robot_pos;
       //  map.getPosition(robot_indx, robot_pos);
       //  debugger::debugColorOutput(
       //          "[map_operation] "
       //          "robot pos \n", robot_pos, 10, Y);

       //  grid_map::Position end_point;
       //  map.getPosition(*it, end_point);
       //  debugger::debugColorOutput(
       //          "[map_operation] "
       //          "end point \n", end_point, 10, Y);
            
        bool encounter_obstalce = false;
        // bool ignore_itself = false;
        for (grid_map::LineIterator iterator(map, robot_indx, *it);
                !iterator.isPastEnd(); ++iterator) {
            // if (!ignore_itself)
            // {
            //     ignore_itself = true;
            //     continue;
            // }

            if (map.at(compute_on_layer, *iterator) == 1.0)
            {
                encounter_obstalce = true;
                // debugger::debugColorOutput(
                //         "[map_operation] "
                //         "face obstacle \n", *iterator, 10, Y);
            }


            if (encounter_obstalce)
            {
                // debugger::debugColorOutput(
                //         "[map_operation] "
                //         "assign everything behind as unknown \n", *iterator, 10, Y);
                map.at(apply_to_layer, *iterator) = unknown_cost;

                // due to unknown, assign occupied
                map.at(compute_on_layer, *iterator) = 1.0; 
            }
            // else
            // {
            //     debugger::debugColorOutput(
            //             "[map_operation] "
            //             "no obs \n", *iterator, 10, Y);
            // }
            // utils::pressEnterToContinue();
        }
    }
}


// return a vector of terrain plane information
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
        const size_t num_planes, const size_t num_points_for_fitting) 
{
    if (output_layer.size()==0)
        output_layer  = "filtered_" + apply_to_layer;

    if (!map.exists(output_layer))
        map.add(output_layer, Eigen::MatrixXf::Zero(gridMapSize(0), gridMapSize(1)));


    // prepare for plane information
    Eigen::Vector2f robot_pose_vec = Eigen::Vector2f(std::cos(robot_pose.theta),
            std::sin(robot_pose.theta));
    // grid_map::Size submap_size = map.getSize();
    // size_t num_points = submap_size(0) * submap_size(1);
    // std::vector<plane::terrain_info_t> terrain_plane_vec(num_planes);
    std::vector<plane::terrain_info_t> terrain_plane = std::vector<plane::terrain_info_t>(num_planes);
    std::vector<std::vector<Eigen::Vector3f>> segment_points(num_planes);



    // Iterate over whole map.
    auto& data_to = map[output_layer];
    for (grid_map::GridMapIterator it(map); !it.isPastEnd(); ++it) 
    {
        grid_map::Position currentPosition;
        map.getPosition(*it, currentPosition);
        bool assign_nan = true;

        /////////////////////////
        // average nan filtering
        ////////////////////////
        {
            // assign NAN with nan_value
            if (!map.isValid(*it, apply_to_layer))
                map.atPosition(output_layer, currentPosition) = nan_value;

            double mean = 0.0;
            double sumOfWeights = 0.0;

            // Compute weighted mean.
            size_t num_cell_in_circle = 0;
            size_t num_nan_cell = 0;
            for (grid_map::CircleIterator circleIt(map, currentPosition, filterRadius); 
                    !circleIt.isPastEnd(); ++circleIt) 
            {
                num_cell_in_circle++;
                if (!map.isValid(*circleIt, apply_to_layer)) 
                {
                    num_nan_cell++;
                    continue; 
                }
                grid_map::Position currentPositionInCircle;
                map.getPosition(*circleIt, currentPositionInCircle);

                // Computed weighted mean based on Euclidian distance.
                double distance = (currentPosition - currentPositionInCircle).norm();
                double weight = pow(filterRadius - distance, 2);
                mean += weight * map.at(apply_to_layer, *circleIt);
                sumOfWeights += weight;
            }
            double ratio = (double) num_nan_cell / num_cell_in_circle;

            // apply filter
            const size_t i = it.getLinearIndex();
            if (sumOfWeights != 0 && ratio <= percentage) 
            {
                data_to(i) = mean / sumOfWeights;
                assign_nan = false;
            } 
            else 
            {
                data_to(i) = nan_value;
                debugger::debugColorTextOutput("[map_operation]/[averageNANFiltering] "
                        "hole too big, remain large value", 4, Y);
            }
        }


        /////////////////////////////
        // terrain plane computation 
        ////////////////////////////
        // if current value is nan (large value), skipped!
        if (assign_nan)
            continue;

        const size_t i = it.getLinearIndex();
        Eigen::Vector2f v = currentPosition.cast<float>() - Eigen::Vector2f(robot_pose.x, robot_pose.y);
        double distance = v.norm();
        double x_distance = robot_pose_vec.dot(v);

        double theta = std::acos(x_distance / std::max(distance, 1e-5));
        double y_distance = distance * std::sin(theta);
        if (y_distance > delta_y || x_distance < 0)
            continue;

        int k = std::floor(x_distance / delta_x);
        if (k < num_planes)
        {
            Eigen::Vector3f lidar_point(currentPosition(0), currentPosition(1), data_to(i));
            // debugger::debugColorOutputPrecision(
            //         "[map_operation]/[plane_fitting] assining points: \n",
            //         lidar_point, 6, 3, WHITE);
            segment_points[k].push_back(lidar_point.cast<float>());
        }
    }

    ////////////////////////
    // terrain plane fitting
    ////////////////////////
    // debugger::debugColorTextOutput("[map_operation]/[averageNANFiltering] "
    //                     "Plane Fitting", 6, Y);
    // debugger::debugOutput("[map_operation]/[averageNANFiltering] num_planes: ",
    //                 num_planes, 6);
    for (int seg = 0; seg < num_planes; ++seg)
    {
        size_t num_seg_points = segment_points[seg].size();

        // default status is -1 and assume flat ground
        if (num_seg_points <= 0 || num_seg_points < num_points_for_fitting)
        {
            
            debugger::debugWarningOutput("[map_operation]/[plane_fitting] "
                    "Number of points lower than threshold (" + 
                    std::to_string(num_points_for_fitting)+ "): ", num_seg_points, 6);
            continue;
        }

        debugger::debugOutput("[map_operation]/[plane_fitting] num_seg_points: ",
                    num_seg_points, 6);
        Eigen::MatrixXf seg_points_mat = Eigen::MatrixXf(3, num_seg_points);

        for (int i = 0; i < num_seg_points; ++i)
        {
            seg_points_mat.col(i) = segment_points[seg][i];
        }
        terrain_plane.at(seg).points = seg_points_mat;
        auto plane_params = 
            plane::fitPlaneViaLeastSquares(terrain_plane.at(seg).points);
        // std::cout << "=============================================\n";
        std::vector<float> plane_coefficients = plane_params.getPlaneCoefficients();
        terrain_plane.at(seg).status = 1;
        terrain_plane.at(seg) = plane_params;
        terrain_plane.at(seg).robot_pose = robot_pose;
        terrain_plane.at(seg).length = delta_x;
        terrain_plane.at(seg).width = delta_y;
    }

    return terrain_plane;
}


// return a vector of terrain plane information
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
        const size_t num_planes, const size_t num_points_for_fitting) 
{
    if (output_layer.size()==0)
        output_layer  = "filtered_" + apply_to_layer;

    if (!map.exists(output_layer))
        map.add(output_layer, Eigen::MatrixXf::Zero(gridMapSize(0), gridMapSize(1)));

    // plane information
    std::vector<plane::terrain_info_t> terrain_plane = 
        averageMapFilteringAndcomputeTerrainPlaneInformation(
            map, gridMapSize, filterRadius, apply_to_layer, output_layer,
            percentage, nan_value, robot_pose, delta_x, delta_y, 
            num_planes, num_points_for_fitting);

    // std::vector<plane::terrain_info_t> terrain_plane(num_planes);
    // prepare for terrain information
    grid_map::Position robot_position(robot_pose.x, robot_pose.y);
    grid_map::Index cell_index;
    map.getIndex(robot_position, cell_index);


    // taking terrain information 
    std::vector< std::string > terrain_types = map.getLayers();
    int terrain_type = -1;
    float terrain_value = -1;
    float terrain_friction = -1;
    for (int i = 0; i < terrain_types.size(); ++i)
    {
        terrain_value = map.at(terrain_types[i], cell_index);
        // debugger::debugColorOutput("[Driver]/[map_operation] terrain: ",
        //         terrain_types[i],  3, B);
        // debugger::debugColorOutput("[Driver]/[map_operation] terrain value: ",
        //         terrain_value,  3, B);
        // if (terrain_value > 0)
        if (!std::isnan(terrain_value))
        {
            terrain_type = i;
            terrain_friction = terrain_properties::terrain_friction[i];
            debugger::debugColorOutput("[map_operation]/[Terrain] "
                    "Terrain Assigned as " + 
                    std::string(terrain_properties::terrain_type[i]), "!!",6, BM);
            break;
        }
            
    }

    //if (terrain_value < 0)
    if (terrain_value == -1)
    {
        std::string txt = "[WARNING]: TERRAIN INFORMATION NOT ASSIGNED!!!!";
        debugger::debugColorTextOutput(txt,  10, Y);
    }


    // assign terrain information
    for (int i = 0; i < num_planes; ++i)
    {
        terrain_plane.at(i).terrain.terrain_type = terrain_type;
        terrain_plane.at(i).terrain.probability = terrain_value;
        terrain_plane.at(i).terrain.friction = terrain_friction;
    }

    return terrain_plane;
}
    

// // return a vector of terrain plane information
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
//         const double num_planes) 
// {
//     if (output_layer.size()==0)
//         output_layer  = "filtered_" + apply_to_layer;
// 
//     if (!map.exists(output_layer))
//         map.add(output_layer, Eigen::MatrixXf::Zero(gridMapSize(0), gridMapSize(1)));
// 
// 
//     // prepare for plane information
//     Eigen::Vector2f robot_pose_vec = Eigen::Vector2f(std::cos(robot_pose.theta),
//                                                      std::sin(robot_pose.theta));
//     // grid_map::Size submap_size = map.getSize();
//     // size_t num_points = submap_size(0) * submap_size(1);
//     // std::vector<plane::terrain_info_t> terrain_plane_vec(num_planes);
//     auto terrain_plane_ptr = 
//         std::make_shared<std::vector<plane::terrain_info_t> >(num_planes);
//     std::vector<std::vector<Eigen::Vector3f>> segment_points(num_planes);
// 
// 
// 
//     // Iterate over whole map.
//     auto& data_to = map[output_layer];
//     for (grid_map::GridMapIterator it(map); !it.isPastEnd(); ++it) 
//     {
//         grid_map::Position currentPosition;
//         map.getPosition(*it, currentPosition);
//         bool assign_nan = true;
// 
//         /////////////////////////
//         // average nan filtering
//         ////////////////////////
//         {
//             // assign NAN with nan_value
//             if (!map.isValid(*it, apply_to_layer))
//                 map.atPosition(output_layer, currentPosition) = nan_value;
// 
//             double mean = 0.0;
//             double sumOfWeights = 0.0;
// 
//             // Compute weighted mean.
//             size_t num_cell_in_circle = 0;
//             size_t num_nan_cell = 0;
//             for (grid_map::CircleIterator circleIt(map, currentPosition, filterRadius); 
//                     !circleIt.isPastEnd(); ++circleIt) 
//             {
//                 num_cell_in_circle++;
//                 if (!map.isValid(*circleIt, apply_to_layer)) 
//                 {
//                     num_nan_cell++;
//                     continue; 
//                 }
//                 grid_map::Position currentPositionInCircle;
//                 map.getPosition(*circleIt, currentPositionInCircle);
// 
//                 // Computed weighted mean based on Euclidian distance.
//                 double distance = (currentPosition - currentPositionInCircle).norm();
//                 double weight = pow(filterRadius - distance, 2);
//                 mean += weight * map.at(apply_to_layer, *circleIt);
//                 sumOfWeights += weight;
//             }
//             double ratio = (double) num_nan_cell / num_cell_in_circle;
// 
//             // apply filter
//             const size_t i = it.getLinearIndex();
//             if (sumOfWeights != 0 && ratio <= percentage) 
//             {
//                 data_to(i) = mean / sumOfWeights;
//                 assign_nan = false;
//             } 
//             else 
//             {
//                 data_to(i) = nan_value;
//                 debugger::debugColorTextOutput("[map_operation]/[averageNANFiltering] "
//                         "hole too big, remain large value", 4, Y);
//             }
//         }
// 
// 
//         /////////////////////////////
//         // terrain plane computation 
//         ////////////////////////////
//         // if current value is nan (large value), skipped!
//         if (assign_nan)
//             continue;
// 
//         const size_t i = it.getLinearIndex();
//         Eigen::Vector2f v = currentPosition.cast<float>() - Eigen::Vector2f(robot_pose.x, robot_pose.y);
//         double distance = v.norm();
//         double x_distance = robot_pose_vec.dot(v);
// 
//         double theta = std::acos(x_distance / std::max(distance, 1e-5));
//         double y_distance = distance * std::sin(theta);
//         if (y_distance > delta_y || x_distance < 0)
//             continue;
// 
//         int k = std::floor(x_distance / delta_x);
//         if (k < num_planes)
//         {
//             Eigen::Vector3f lidar_point(currentPosition(0), currentPosition(2), data_to(i));
//             segment_points[k].push_back(lidar_point.col(i).cast<float>());
//         }
//     }
// 
//     ////////////////////////
//     // terrain plane fitting
//     ////////////////////////
//     for (int seg = 0; seg < num_planes; ++seg)
//     {
//         size_t num_seg = segment_points[seg].size();
//         Eigen::MatrixXf seg_points = Eigen::MatrixXf(3, num_seg);
//         for (int i = 0; i < num_seg; ++i)
//         {
//             seg_points.col(i) = segment_points[seg][i];
//         }
//         terrain_plane_ptr->at(seg).points = seg_points;
//         auto plane_params = 
//             plane::fitPlaneViaLeastSquares(terrain_plane_ptr->at(seg).points);
//         std::vector<float> plane_coefficients = plane_params->getPlaneCoefficients();
//         terrain_plane_ptr->at(seg) = *plane_params;
//         terrain_plane_ptr->at(seg).robot_pose = robot_pose;
//         terrain_plane_ptr->at(seg).length = delta_x;
//         terrain_plane_ptr->at(seg).width = delta_y;
//     }
// 
//     return terrain_plane_ptr;
// }
// 
// 
// // return a vector of terrain plane information
// std::shared_ptr<std::vector<plane::terrain_info_t>>
// averageMapFilteringAndcomputeTerrainPlaneInformationAndTerrainInformation(grid_map::GridMap& map, 
//         const grid_map::Size& gridMapSize, 
//         const double filterRadius,
//         const std::string apply_to_layer,
//         std::string output_layer,
//         const double percentage, // larger, the more be corrected
//         const double nan_value, 
//         const pose_t& robot_pose, 
//         const double delta_x,
//         const double delta_y,
//         const double num_planes) 
// {
//     if (output_layer.size()==0)
//         output_layer  = "filtered_" + apply_to_layer;
// 
//     if (!map.exists(output_layer))
//         map.add(output_layer, Eigen::MatrixXf::Zero(gridMapSize(0), gridMapSize(1)));
// 
//     // plane information
//     auto terrain_info_ptr = 
//         std::make_shared<std::vector<plane::terrain_info_t> >(num_planes);
//     terrain_info_ptr = averageMapFilteringAndcomputeTerrainPlaneInformation(
//             map, gridMapSize, filterRadius, apply_to_layer, output_layer,
//             percentage, nan_value, robot_pose, delta_x, delta_y, num_planes);
// 
//     // prepare for terrain information
//     grid_map::Position robot_position(robot_pose.x, robot_pose.y);
//     grid_map::Index cell_index;
//     map.getIndex(robot_position, cell_index);
// 
// 
//     // taking terrain information 
//     std::vector< std::string > terrain_types = map.getLayers();
//     float terrain_type = -1;
//     float terrain_value = -1;
//     float terrain_friction = -1;
//     for (int i = 0; i < terrain_types.size(); ++i)
//     {
//         terrain_value = map.at(terrain_types[i], cell_index);
//         if (terrain_value > 0)
//         {
//             terrain_type = i;
//             terrain_friction = terrain_friction::terrain_friction[i];
//         }
//             
//     }
// 
//     if (terrain_value < 0)
//     {
//         std::string txt = "[WARNING]: TERRAIN INFORMATION NOT ASSIGNED!!!!";
//         debugger::debugColorTextOutput(txt,  10, Y);
//     }
// 
// 
//     // assign terrain information
//     for (int i = 0; i < num_planes; ++i)
//     {
//         terrain_info_ptr->at(i).terrain.terrain_type = terrain_type;
//         terrain_info_ptr->at(i).terrain.probability = terrain_value;
//         terrain_info_ptr->at(i).terrain.friction = terrain_friction;
//     }
// 
//     return terrain_info_ptr;
// }
    
} /* map_operation */ 
} /* bipedlab */ 

