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
#include "cmath"
#include "fake_map.h"
#include "map_utils.h"
#include "utils/debugger.h"
#include "utils/utils.h"
#include "map_operation.h"

namespace bipedlab {
    FakeMap::FakeMap(int map_num)
    {
        switch (map_num) {
            case 0:
                FakeMap::loadTestMap1_();
                break;
            case 1:
                FakeMap::loadTestNoisyMap1_();
                break;
            case 2:
                FakeMap::loadLevelOneWaveField_();
                break;
            case 3:
                FakeMap::loadNoisyLevelOneWaveField_();
                break;
            case 4:
                FakeMap::loadLevelTwoWaveField_();
                break;
            case 5:
                FakeMap::loadNoisyLevelTwoWaveField_();
                break;
            case 100:
                FakeMap::loadSimpleCorridorIndoorScene1_();
                break;
            case 101:
                FakeMap::loadNoisySimpleCorridorIndoorScene1_();
                break;
            case 102:
                FakeMap::loadSimpleCorridorIndoorScene2_();
                break;
            case 103:
                FakeMap::loadNoisySimpleCorridorIndoorScene2_();
                break;
            case 110:
                FakeMap::loadSimpleClutterIndoorScene1_();
                break;
            case 111:
                FakeMap::loadNoisySimpleClutterIndoorScene1_();
                break;
            case 112:
                FakeMap::loadSimpleClutterIndoorScene2_();
                break;
            case 113:
                FakeMap::loadNoisySimpleClutterIndoorScene2_();
                break;
            case 120:
                FakeMap::loadRetancularIndoorScene1_();
                break;
            case 121:
                FakeMap::loadNoisyRetancularIndoorScene1_();
                break;
            case 900:
                FakeMap::loadLevelThreeWaveField_();
                break;
            case 901:
                FakeMap::loadNoisyLevelThreeWaveField_();
                break;
            default:
                debugger::debugColorOutput("[FakeMap] Unknown map_num: ", 
                        map_num, 10);
                exit(-1);
        }
    }
    FakeMap::~FakeMap() {}

    void FakeMap::loadTestMap1_() 
    {
        // map size and resolution
        global_map_info = map_info_t(-15, 15, -15, 15, 0.1);
        map = new grid_map::GridMap();
        map->add("elevation_map", 0.0);
        map->add("traversability_map", 3.0);
        // map->add("noisy_elevation_map", 3.0);
        // map->add("noise", 3.0);
        map->setFrameId("odom");
        map->setGeometry(
                grid_map::Length(global_map_info.len_x, global_map_info.len_y), 
                global_map_info.grid_size);
        // (*map)["occupancy_map"].setConstant(1.0); // all occupied 
        ROS_INFO("Created map with size %f x %f m (%i x %i cells).",
                map->getLength().x(), map->getLength().y(),
                map->getSize()(0), map->getSize()(1));

        // targets
        // start = pose_t(1, 9, -M_PI/4);
        // goal = pose_t(8, 4, 0);
        // start = pose_t(-4, 4, -M_PI/4);
        // goal = pose_t(3, -1, 0);


        // height distribution
        Eigen::Vector2d mean1(-4, -4);
        Eigen::Matrix2d cov1; 
        cov1 << 1.5, 0, 0, 3;
        MultivariateGaussian *sub_map1 = new MultivariateGaussian(mean1, cov1, 10);


        Eigen::Vector2d mean2(0, 0);
        Eigen::Matrix2d cov2; 
        cov2 << 1.5, 0, 0, 3;
        MultivariateGaussian *sub_map2 = new MultivariateGaussian(mean2, cov2, 10);


        Eigen::Vector2d mean3(1, 5);
        Eigen::Matrix2d cov3; 
        cov3 << 1.5, 0, 0, 1.5;
        MultivariateGaussian *sub_map3 = new MultivariateGaussian(mean3, cov3, 10);


        Eigen::Vector2d mean4(4, 4);
        Eigen::Matrix2d cov4; 
        cov4 << 2, 0.5, 0.5, 3;
        MultivariateGaussian *sub_map4 = new MultivariateGaussian(mean4, cov4, 20);

        // traversability map
        // (*map)["traversability_map"].setConstant(1);
        // (*map)["occupancy_map"].setConstant(1.0); // all occupied 



        // grid_map::Matrix& data = map["layer"];
        for (grid_map::GridMapIterator it(*map); !it.isPastEnd(); ++it) {
            grid_map::Position position;
            map->getPosition(*it, position);

                Eigen::Vector2d current_x_y(position.x(), position.y());
                double height = 
                    sub_map1->distribution(current_x_y) + 
                    sub_map2->distribution(current_x_y) + 
                    sub_map3->distribution(current_x_y) + 
                    sub_map4->distribution(current_x_y);
            map->at("elevation_map", *it) = height; 
            map->at("traversability_map", *it) = 1; 

            // if (height >= obstacle_threshold_)
            //     map->at("occupancy_map", *it) = 1; 
            // else
            //     map->at("occupancy_map", *it) = 0; 

        }


        // create untraversable area
        grid_map::Position center(-1, -5); // 0,-5
        double radius = 0.4;

        for (grid_map::CircleIterator iterator(*map, center, radius);
                !iterator.isPastEnd(); ++iterator) {
            map->at("traversability_map", *iterator) = 0.0;
        }
    }

    void FakeMap::loadTestNoisyMap1_() 
    {
        // map size and resolution
        global_map_info = map_info_t(-15, 15, -15, 15, 0.1);
        map = new grid_map::GridMap();
        map->add("elevation_map", 0.0);
        map->add("traversability_map", 3.0);
        map->setFrameId("odom");
        map->setGeometry(
                grid_map::Length(global_map_info.len_x, global_map_info.len_y), 
                global_map_info.grid_size);
        ROS_INFO("Created map with size %f x %f m (%i x %i cells).",
                map->getLength().x(), map->getLength().y(),
                map->getSize()(0), map->getSize()(1));

        // height distribution
        Eigen::Vector2d mean1(-4, -4);
        Eigen::Matrix2d cov1; 
        cov1 << 1.5, 0, 0, 3;
        MultivariateGaussian *sub_map1 = new MultivariateGaussian(mean1, cov1, 10);


        Eigen::Vector2d mean2(0, 0);
        Eigen::Matrix2d cov2; 
        cov2 << 1.5, 0, 0, 3;
        MultivariateGaussian *sub_map2 = new MultivariateGaussian(mean2, cov2, 10);


        Eigen::Vector2d mean3(1, 5);
        Eigen::Matrix2d cov3; 
        cov3 << 1.5, 0, 0, 1.5;
        MultivariateGaussian *sub_map3 = new MultivariateGaussian(mean3, cov3, 10);


        Eigen::Vector2d mean4(4, 4);
        Eigen::Matrix2d cov4; 
        cov4 << 2, 0.5, 0.5, 3;
        MultivariateGaussian *sub_map4 = new MultivariateGaussian(mean4, cov4, 20);


        // grid_map::Matrix& data = map["layer"];
        for (grid_map::GridMapIterator it(*map); !it.isPastEnd(); ++it) {
            grid_map::Position position;
            map->getPosition(*it, position);

                Eigen::Vector2d current_x_y(position.x(), position.y());
                double height = 
                    sub_map1->distribution(current_x_y) + 
                    sub_map2->distribution(current_x_y) + 
                    sub_map3->distribution(current_x_y) + 
                    sub_map4->distribution(current_x_y);
            map->at("elevation_map", *it) = height; 
            map->at("traversability_map", *it) = 1; 
        }


        // create untraversable area
        grid_map::Position center(-1, -5); // 0,-5
        double radius = 0.4;

        for (grid_map::CircleIterator iterator(*map, center, radius);
                !iterator.isPastEnd(); ++iterator) {
            map->at("traversability_map", *iterator) = 0.0;
        }

        // create NAN area
        // grid_map::Position nan_center(-5, -5); // 0,-5
        // double nan_radius = 1;

        // for (grid_map::CircleIterator iterator(*map, nan_center, nan_radius);
        //         !iterator.isPastEnd(); ++iterator) 
        // {
        //     map->at("elevation_map", *iterator) = 
        //         std::numeric_limits<double>::quiet_NaN();
        // }
        // num_NAN = map_operation::checkNAN(*map, "elevation_map");
        // debugger::debugOutput("num_nan at elevation: ", num_NAN, 5);

        // create NAN dots
        size_t num_NAN = 0;
        map_operation::addNANToMap(*map, map->getSize(), 0.01, "elevation_map", "elevation_map");
        num_NAN = map_operation::checkNAN(*map, "elevation_map");
        debugger::debugOutput("num_nan for small holes: ", num_NAN, 5);


        // create NAN holes
        map_operation::addNANHoleToMap(*map, map->getSize(), 0.005, "elevation_map", "elevation_map");
        num_NAN = map_operation::checkNAN(*map, "elevation_map");
        debugger::debugOutput("num_nan for big holes: ", num_NAN, 5);


        // add random noise
        map_operation::addNoiseToMap(*map, map->getSize(), 0.1, "elevation_map", "elevation_map");



        // filtering
        // double filter_radius = 0.4;
        // double percentage = 0.6;
        // double nan_value = 0.2;
        // map_operation::averageNANFiltering(*map, map->getSize(), 
        //         filter_radius, "elevation_map", "filtered_elevation_map", 
        //         percentage, nan_value);
        // num_NAN = map_operation::checkNAN(*map, "filtered_elevation_map");
        // debugger::debugOutput("num_nan after averaging: ", num_NAN, 5);
    }


    // wavefield level one
    void FakeMap::loadLevelOneWaveField_() 
    {
        // map size and resolution
        global_map_info = map_info_t(-15, 15, -15, 15, 0.05);
        map = new grid_map::GridMap();
        map->add("elevation_map", 0.0);
        map->setFrameId("odom");
        map->setGeometry(
                grid_map::Length(global_map_info.len_x, global_map_info.len_y), 
                global_map_info.grid_size);
        ROS_INFO("Created map with size %f x %f m (%i x %i cells).",
                map->getLength().x(), map->getLength().y(),
                map->getSize()(0), map->getSize()(1));


        std::vector<MultivariateGaussian> terrain;
        FakeMap::loadLevelOneWaveFieldTerrain_(terrain);

        grid_map::Matrix& data = map->get("elevation_map");
        for (grid_map::GridMapIterator it(*map); !it.isPastEnd(); ++it) {
            grid_map::Position position;
            map->getPosition(*it, position);

            Eigen::Vector2d current_x_y(position.x(), position.y());

            double height = 0;
            for (const auto terrain_it : terrain)
            {
                height += terrain_it.distribution(current_x_y);
            }
            size_t i = it.getLinearIndex();
            data(i) = height; 
            // map->at("elevation_map", *it) = height; 
            // map->at("traversability_map", *it) = 1; 
        }


        // create untraversable area
        // grid_map::Position center(-1, -5); // 0,-5
        // double radius = 0.4;

        // for (grid_map::CircleIterator iterator(*map, center, radius);
        //         !iterator.isPastEnd(); ++iterator) {
        //     map->at("traversability_map", *iterator) = 0.0;
        // }
    }

    void FakeMap::loadNoisyLevelOneWaveField_() 
    {
        // due to different resolution, 
        // we can't directly loadWaveField and add noise
        global_map_info = map_info_t(-15, 15, -15, 15, 0.1);
        map = new grid_map::GridMap();
        map->add("elevation_map", 0.0);
        map->setFrameId("odom");
        map->setGeometry(
                grid_map::Length(global_map_info.len_x, global_map_info.len_y), 
                global_map_info.grid_size);
        ROS_INFO("Created map with size %f x %f m (%i x %i cells).",
                map->getLength().x(), map->getLength().y(),
                map->getSize()(0), map->getSize()(1));


        std::vector<MultivariateGaussian> terrain;
        FakeMap::loadLevelOneWaveFieldTerrain_(terrain);

        grid_map::Matrix& data = map->get("elevation_map");
        for (grid_map::GridMapIterator it(*map); !it.isPastEnd(); ++it) {
            grid_map::Position position;
            map->getPosition(*it, position);

            Eigen::Vector2d current_x_y(position.x(), position.y());

            double height = 0;
            for (const auto terrain_it : terrain)
            {
                height += terrain_it.distribution(current_x_y);
            }
            size_t i = it.getLinearIndex();
            data(i) = height; 
            // map->at("elevation_map", *it) = height; 
            // map->at("traversability_map", *it) = 1; 
        }


        // create NAN dots
        size_t num_NAN = 0;
        map_operation::addNANToMap(*map, map->getSize(), 0.01, "elevation_map", "elevation_map");
        num_NAN = map_operation::checkNAN(*map, "elevation_map");
        debugger::debugOutput("num_nan for small holes: ", num_NAN, 5);


        // create NAN holes
        map_operation::addNANHoleToMap(*map, map->getSize(), 0.005, "elevation_map", "elevation_map");
        num_NAN = map_operation::checkNAN(*map, "elevation_map");
        debugger::debugOutput("num_nan for big holes: ", num_NAN, 5);


        // add random noise
        map_operation::addNoiseToMap(*map, map->getSize(), 0.1, "elevation_map", "elevation_map");
    }

    void FakeMap::loadLevelOneWaveFieldTerrain_(
            std::vector<MultivariateGaussian>& terrain)
    {
        // height distribution
        // left most at x-axis
        FakeMap::genTerrain(-12.5, 7.5, 2, 0.5, 0.5, 3, 10, terrain, -1, 1, -1, 1);
        FakeMap::genTerrain(-8.5,  3.5, 2, 0.5, 0.5, 3, 10, terrain, -1, 1, -1, 1);
        FakeMap::genTerrain(-4.5,-0.5, 2, 0.5, 0.5, 3, 10, terrain, -1, 1, -1, 1);
        FakeMap::genTerrain(-0.5,-4.5, 2, 0.5, 0.5, 3, 10, terrain, -1, 1, -1, 1);
        FakeMap::genTerrain(3.5, -8.5, 2, 0.5, 0.5, 3, 10, terrain, -1, 1, -1, 1);
        FakeMap::genTerrain(7.5, -12.5, 2, 0.5, 0.5, 3, 10, terrain, -1, 1, -1, 1);


        // top most at y-axis 
        FakeMap::genTerrain(-7.5, 12.5, 2, 0.5, 0.5, 3, 10, terrain, -1, 1, -1, 1);
        FakeMap::genTerrain(-3.5, 8.5,  2, 0.5, 0.5, 3, 10, terrain, -1, 1, -1, 1);
        FakeMap::genTerrain(0.5, 4.5,   2, 0.5, 0.5, 3, 10, terrain, -1, 1, -1, 1);
        FakeMap::genTerrain(4.5, 0.5,   2, 0.5, 0.5, 3, 10, terrain, -1, 1, -1, 1);
        FakeMap::genTerrain(8.5, -3.5,  2, 0.5, 0.5, 3, 10, terrain, -1, 1, -1, 1);
        FakeMap::genTerrain(12.5, -7.5, 2, 0.5, 0.5, 3, 10, terrain, -1, 1, -1, 1);


        // little humps in the valley
        double random_height = 1;
        FakeMap::genTerrain(-12.5, 12.5, 2, 0.5, 0.5, 3, random_height, terrain, -0.5, 0.5, -0.5, 0.5);
        FakeMap::genTerrain(-8.5,  8.5,  2, 0.5, 0.5, 3, random_height, terrain, -0.5, 0.5, -0.5, 0.5);
        FakeMap::genTerrain(-4.5, 4.5,   2, 0.5, 0.5, 3, random_height, terrain, -0.5, 0.5, -0.5, 0.5);
        FakeMap::genTerrain(-0.5, 0.5,   2, 0.5, 0.5, 3, random_height, terrain, -0.5, 0.5, -0.5, 0.5);
        FakeMap::genTerrain(3.5, -3.5,   2, 0.5, 0.5, 3, random_height, terrain, -0.5, 0.5, -0.5, 0.5);
        FakeMap::genTerrain(7.5, -7.5,   2, 0.5, 0.5, 3, random_height, terrain, -0.5, 0.5, -0.5, 0.5);
        FakeMap::genTerrain(11.5, -11.5, 2, 0.5, 0.5, 3, random_height, terrain, -0.5, 0.5, -0.5, 0.5);
    }

    // wavefield level two
    void FakeMap::loadLevelTwoWaveField_() 
    {
        // map size and resolution
        global_map_info = map_info_t(-15, 15, -15, 15, 0.05);
        map = new grid_map::GridMap();
        map->add("elevation_map", 0.0);
        map->setFrameId("odom");
        map->setGeometry(
                grid_map::Length(global_map_info.len_x, global_map_info.len_y), 
                global_map_info.grid_size);
        ROS_INFO("Created map with size %f x %f m (%i x %i cells).",
                map->getLength().x(), map->getLength().y(),
                map->getSize()(0), map->getSize()(1));


        std::vector<MultivariateGaussian> terrain;
        FakeMap::loadLevelTwoWaveFieldTerrain_(terrain);

        grid_map::Matrix& data = map->get("elevation_map");
        for (grid_map::GridMapIterator it(*map); !it.isPastEnd(); ++it) {
            grid_map::Position position;
            map->getPosition(*it, position);

            Eigen::Vector2d current_x_y(position.x(), position.y());

            double height = 0;
            for (const auto& terrain_it : terrain)
            {
                height += terrain_it.distribution(current_x_y);
            }
            size_t i = it.getLinearIndex();
            data(i) = height; 
            // map->at("elevation_map", *it) = height; 
            // map->at("traversability_map", *it) = 1; 
        }


        // create untraversable area
        // grid_map::Position center(-1, -5); // 0,-5
        // double radius = 0.4;

        // for (grid_map::CircleIterator iterator(*map, center, radius);
        //         !iterator.isPastEnd(); ++iterator) {
        //     map->at("traversability_map", *iterator) = 0.0;
        // }
    }

    void FakeMap::loadNoisyLevelTwoWaveField_() 
    {
        // due to different resolution, 
        // we can't directly loadWaveField and add noise
        global_map_info = map_info_t(-15, 15, -15, 15, 0.1);
        map = new grid_map::GridMap();
        map->add("elevation_map", 0.0);
        map->setFrameId("odom");
        map->setGeometry(
                grid_map::Length(global_map_info.len_x, global_map_info.len_y), 
                global_map_info.grid_size);
        ROS_INFO("Created map with size %f x %f m (%i x %i cells).",
                map->getLength().x(), map->getLength().y(),
                map->getSize()(0), map->getSize()(1));


        std::vector<MultivariateGaussian> terrain;
        FakeMap::loadLevelTwoWaveFieldTerrain_(terrain);

        grid_map::Matrix& data = map->get("elevation_map");
        for (grid_map::GridMapIterator it(*map); !it.isPastEnd(); ++it) {
            grid_map::Position position;
            map->getPosition(*it, position);

            Eigen::Vector2d current_x_y(position.x(), position.y());

            double height = 0;
            for (const auto terrain_it : terrain)
            {
                height += terrain_it.distribution(current_x_y);
            }
            size_t i = it.getLinearIndex();
            data(i) = height; 
            // map->at("elevation_map", *it) = height; 
            // map->at("traversability_map", *it) = 1; 
        }


        // create NAN dots
        size_t num_NAN = 0;
        map_operation::addNANToMap(*map, map->getSize(), 0.01, "elevation_map", "elevation_map");
        num_NAN = map_operation::checkNAN(*map, "elevation_map");
        debugger::debugOutput("num_nan for small holes: ", num_NAN, 5);


        // create NAN holes
        map_operation::addNANHoleToMap(*map, map->getSize(), 0.005, "elevation_map", "elevation_map");
        num_NAN = map_operation::checkNAN(*map, "elevation_map");
        debugger::debugOutput("num_nan for big holes: ", num_NAN, 5);


        // add random noise
        map_operation::addNoiseToMap(*map, map->getSize(), 0.1, "elevation_map", "elevation_map");
    }

    void FakeMap::loadLevelTwoWaveFieldTerrain_(
            std::vector<MultivariateGaussian>& terrain)
    {
        // height distribution
        // left most at x-axis
        FakeMap::genTerrain(-12.5, 7.5, 2, 0.5, 0.5, 3, 10, terrain, -1, 1, -1, 1);
        FakeMap::genTerrain(-8.5,  3.5, 2, 0.5, 0.5, 3, 10, terrain, -1, 1, -1, 1);
        FakeMap::genTerrain(-4.5,-0.5, 2, 0.5, 0.5, 3, 10, terrain, -1, 1, -1, 1);
        FakeMap::genTerrain(-0.5,-4.5, 2, 0.5, 0.5, 3, 10, terrain, -1, 1, -1, 1);
        FakeMap::genTerrain(3.5, -8.5, 2, 0.5, 0.5, 3, 10, terrain, -1, 1, -1, 1);
        FakeMap::genTerrain(7.5, -12.5, 2, 0.5, 0.5, 3, 10, terrain, -1, 1, -1, 1);


        // top most at y-axis 
        FakeMap::genTerrain(-7.5, 12.5, 2, 0.5, 0.5, 3, 10, terrain, -1, 1, -1, 1);
        FakeMap::genTerrain(-3.5, 8.5,  2, 0.5, 0.5, 3, 10, terrain, -1, 1, -1, 1);
        FakeMap::genTerrain(0.5, 4.5,   2, 0.5, 0.5, 3, 10, terrain, -1, 1, -1, 1);
        FakeMap::genTerrain(4.5, 0.5,   2, 0.5, 0.5, 3, 10, terrain, -1, 1, -1, 1);
        FakeMap::genTerrain(8.5, -3.5,  2, 0.5, 0.5, 3, 10, terrain, -1, 1, -1, 1);
        FakeMap::genTerrain(12.5, -7.5, 2, 0.5, 0.5, 3, 10, terrain, -1, 1, -1, 1);


        // little humps in the valley
        double random_height = 1;
        FakeMap::genTerrain(-12.5, 12.5, 2, 0.5, 0.5, 3, random_height, terrain, -0.5, 0.5, -0.5, 0.5);
        FakeMap::genTerrain(-8.5,  8.5,  2, 0.5, 0.5, 3, random_height, terrain, -0.5, 0.5, -0.5, 0.5);
        FakeMap::genTerrain(-4.5, 4.5,   2, 0.5, 0.5, 3, random_height, terrain, -0.5, 0.5, -0.5, 0.5);
        FakeMap::genTerrain(-0.5, 0.5,   2, 0.5, 0.5, 3, random_height, terrain, -0.5, 0.5, -0.5, 0.5);
        FakeMap::genTerrain(3.5, -3.5,   2, 0.5, 0.5, 3, random_height, terrain, -0.5, 0.5, -0.5, 0.5);
        FakeMap::genTerrain(7.5, -7.5,   2, 0.5, 0.5, 3, random_height, terrain, -0.5, 0.5, -0.5, 0.5);
        FakeMap::genTerrain(11.5, -11.5, 2, 0.5, 0.5, 3, random_height, terrain, -0.5, 0.5, -0.5, 0.5);



        // random humps
        // +x, +y 
        FakeMap::genTerrain(0, 0, 2, 0.5, 0.5, 3, 10, terrain, 0, 15, 0, 15);
        FakeMap::genTerrain(0, 0, 2, 0.5, 0.5, 3, 10, terrain, 0, 15, 0, 15);
        FakeMap::genTerrain(0, 0, 2, 0.5, 0.5, 3, 10, terrain, 0, 15, 0, 15);
        FakeMap::genTerrain(0, 0, 2, 0.5, 0.5, 3, 10, terrain, 0, 15, 0, 15);
        FakeMap::genTerrain(0, 0, 2, 0.5, 0.5, 3, 10, terrain, 0, 15, 0, 15);

        // -x, +y
        FakeMap::genTerrain(0, 0, 2, 0.5, 0.5, 3, 10, terrain, -15, 0, 0, 15);
        FakeMap::genTerrain(0, 0, 2, 0.5, 0.5, 3, 10, terrain, -15, 0, 0, 15);
        FakeMap::genTerrain(0, 0, 2, 0.5, 0.5, 3, 10, terrain, -15, 0, 0, 15);
        FakeMap::genTerrain(0, 0, 2, 0.5, 0.5, 3, 10, terrain, -15, 0, 0, 15);
        FakeMap::genTerrain(0, 0, 2, 0.5, 0.5, 3, 10, terrain, -15, 0, 0, 15);

        // +x, -y
        FakeMap::genTerrain(0, 0, 2, 0.5, 0.5, 3, 10, terrain, 0, 15, -15, 0);
        FakeMap::genTerrain(0, 0, 2, 0.5, 0.5, 3, 10, terrain, 0, 15, -15, 0);
        FakeMap::genTerrain(0, 0, 2, 0.5, 0.5, 3, 10, terrain, 0, 15, -15, 0);
        FakeMap::genTerrain(0, 0, 2, 0.5, 0.5, 3, 10, terrain, 0, 15, -15, 0);
        FakeMap::genTerrain(0, 0, 2, 0.5, 0.5, 3, 10, terrain, 0, 15, -15, 0);
        FakeMap::genTerrain(0, 0, 2, 0.5, 0.5, 3, 10, terrain, 0, 15, -15, 0);


        // -x, -y
        FakeMap::genTerrain(0, 0, 2, 0.5, 0.5, 3, 10, terrain, -15, 0, -15, 0);
        FakeMap::genTerrain(0, 0, 2, 0.5, 0.5, 3, 10, terrain, -15, 0, -15, 0);
        FakeMap::genTerrain(0, 0, 2, 0.5, 0.5, 3, 10, terrain, -15, 0, -15, 0);
        FakeMap::genTerrain(0, 0, 2, 0.5, 0.5, 3, 10, terrain, -15, 0, -15, 0);
        FakeMap::genTerrain(0, 0, 2, 0.5, 0.5, 3, 10, terrain, -15, 0, -15, 0);
    }

    // wavefield level three
    void FakeMap::loadLevelThreeWaveField_() 
    {
        // map size and resolution
        global_map_info = map_info_t(-15, 15, -15, 15, 0.2);
        map = new grid_map::GridMap();
        map->add("elevation_map", 0.0);
        map->setFrameId("odom");
        map->setGeometry(
                grid_map::Length(global_map_info.len_x, global_map_info.len_y), 
                global_map_info.grid_size);
        ROS_INFO("Created map with size %f x %f m (%i x %i cells).",
                map->getLength().x(), map->getLength().y(),
                map->getSize()(0), map->getSize()(1));


        std::vector<MultivariateGaussian> terrain;
        FakeMap::loadLevelThreeWaveFieldTerrain_(terrain);

        grid_map::Matrix& data = map->get("elevation_map");
        for (grid_map::GridMapIterator it(*map); !it.isPastEnd(); ++it) {
            grid_map::Position position;
            map->getPosition(*it, position);

            Eigen::Vector2d current_x_y(position.x(), position.y());

            double height = 0;
            for (const auto& terrain_it : terrain)
            {
                height += terrain_it.distribution(current_x_y);
            }
            size_t i = it.getLinearIndex();
            data(i) = height; 
            // map->at("elevation_map", *it) = height; 
            // map->at("traversability_map", *it) = 1; 
        }


        // create untraversable area
        // grid_map::Position center(-1, -5); // 0,-5
        // double radius = 0.4;

        // for (grid_map::CircleIterator iterator(*map, center, radius);
        //         !iterator.isPastEnd(); ++iterator) {
        //     map->at("traversability_map", *iterator) = 0.0;
        // }
    }

    void FakeMap::loadNoisyLevelThreeWaveField_() 
    {
        // due to different resolution, 
        // we can't directly loadWaveField and add noise
        global_map_info = map_info_t(-15, 15, -15, 15, 0.1);
        map = new grid_map::GridMap();
        map->add("elevation_map", 0.0);
        map->setFrameId("odom");
        map->setGeometry(
                grid_map::Length(global_map_info.len_x, global_map_info.len_y), 
                global_map_info.grid_size);
        ROS_INFO("Created map with size %f x %f m (%i x %i cells).",
                map->getLength().x(), map->getLength().y(),
                map->getSize()(0), map->getSize()(1));


        std::vector<MultivariateGaussian> terrain;
        FakeMap::loadLevelThreeWaveFieldTerrain_(terrain);

        grid_map::Matrix& data = map->get("elevation_map");
        for (grid_map::GridMapIterator it(*map); !it.isPastEnd(); ++it) {
            grid_map::Position position;
            map->getPosition(*it, position);

            Eigen::Vector2d current_x_y(position.x(), position.y());

            double height = 0;
            for (const auto terrain_it : terrain)
            {
                height += terrain_it.distribution(current_x_y);
            }
            size_t i = it.getLinearIndex();
            data(i) = height; 
            // map->at("elevation_map", *it) = height; 
            // map->at("traversability_map", *it) = 1; 
        }


        // create NAN dots
        size_t num_NAN = 0;
        map_operation::addNANToMap(*map, map->getSize(), 0.01, "elevation_map", "elevation_map");
        num_NAN = map_operation::checkNAN(*map, "elevation_map");
        debugger::debugOutput("num_nan for small holes: ", num_NAN, 5);


        // create NAN holes
        map_operation::addNANHoleToMap(*map, map->getSize(), 0.005, "elevation_map", "elevation_map");
        num_NAN = map_operation::checkNAN(*map, "elevation_map");
        debugger::debugOutput("num_nan for big holes: ", num_NAN, 5);


        // add random noise
        map_operation::addNoiseToMap(*map, map->getSize(), 0.1, "elevation_map", "elevation_map");
    }

    void FakeMap::loadLevelThreeWaveFieldTerrain_(
            std::vector<MultivariateGaussian>& terrain)
    {
        // height distribution
        // left most at x-axis
        double left_start_x = 12.5;
        double left_start_y = -12.5;
        FakeMap::genTerrain(left_start_x, left_start_y, 2, 0.5, 0.5, 3, 10, terrain, 0, 0, 0, 0);
        FakeMap::genTerrain(left_start_x - 4, left_start_y, 2, 0.5, 0.5, 3, 10, terrain, 0, 0, 0,0);
        FakeMap::genTerrain(left_start_x - 8, left_start_y, 2, 0.5, 0.5, 3, 10, terrain, 0, 0, 0,0);
        FakeMap::genTerrain(left_start_x - 12, left_start_y, 2, 0.5, 0.5, 3, 10, terrain, 0, 0, 0,0);
        FakeMap::genTerrain(left_start_x - 16, left_start_y, 2, 0.5, 0.5, 3, 10, terrain, 0, 0, 0,0);
        FakeMap::genTerrain(left_start_x - 20, left_start_y, 2, 0.5, 0.5, 3, 10, terrain, 0, 0, 0,0);


        // top most at y-axis 
        double right_start_x = 12.5;
        double right_start_y = -5.5;
        FakeMap::genTerrain(right_start_x, right_start_y, 2, 0.5, 0.5, 3, 10, terrain, 0,0,0,0);
        FakeMap::genTerrain(right_start_x - 4, right_start_y, 2, 0.5, 0.5, 3, 10, terrain, 0,0,0,0);
        FakeMap::genTerrain(right_start_x - 8, right_start_y, 2, 0.5, 0.5, 3, 10, terrain, 0,0,0,0);
        FakeMap::genTerrain(right_start_x - 12, right_start_y, 2, 0.5, 0.5, 3, 10, terrain, 0,0,0,0);
        FakeMap::genTerrain(right_start_x - 16, right_start_y, 2, 0.5, 0.5, 3, 10, terrain, 0,0,0,0);
        FakeMap::genTerrain(right_start_x - 20, right_start_y, 2, 0.5, 0.5, 3, 10, terrain, 0,0,0,0);
    }


    void FakeMap::loadSimpleCorridorIndoorScene1_()
    {
        // due to different resolution, 
        // we can't directly loadWaveField and add noise
        global_map_info = map_info_t(-15, 15, -10, 10, 0.1);
        map = new grid_map::GridMap();
        map->add("elevation_map", 50.0);
        // map->add("traversability_map", 1);
        map->setFrameId("odom");
        map->setGeometry(
                grid_map::Length(global_map_info.len_x, global_map_info.len_y), 
                global_map_info.grid_size);
        ROS_INFO("Created map with size %f x %f m (%i x %i cells).",
                map->getLength().x(), map->getLength().y(),
                map->getSize()(0), map->getSize()(1));


        // std::vector<MultivariateGaussian> terrain;
        // FakeMap::loadLevelThreeWaveFieldTerrain_(terrain);

        // grid_map::Position center(-1, -5); // 0,-5
        // double radius = 0.4;
        // for (grid_map::CircleIterator iterator(*map, center, radius);
        //         !iterator.isPastEnd(); ++iterator) {
        //     map->at("traversability_map", *iterator) = 0.0;
        // }

        grid_map::Matrix& data = map->get("elevation_map");
        for (grid_map::GridMapIterator it(*map); !it.isPastEnd(); ++it) {
            size_t i = it.getLinearIndex();
            data(i) = 0; 
        }



        // Note:-25, 25, -15, 15
        // Box: fixed point is top-right corner of the box
        double corridor_width = 1.2;
        double wall_height = 1;
        double map_resolution = map->getResolution();

        // top-right box
        FakeMap::genSolidBox(
                map->getLength().x()/2, 
                map->getLength().y()/2,
                map->getLength().x()/2 - corridor_width/2, 
                map->getLength().y()/2 - corridor_width/2,
                wall_height, map, "elevation_map");


        // bottom-right box
        FakeMap::genSolidBox(
                map->getLength().x()/2, 
                -corridor_width/2,
                map->getLength().x()/2 - corridor_width/2, 
                map->getLength().y()/2 - corridor_width/2,
                wall_height, map, "elevation_map");

        // top-left box
        FakeMap::genSolidBox(
                -corridor_width/2, 
                map->getLength().y()/2,
                map->getLength().x()/2 - corridor_width/2, 
                map->getLength().y()/2 - corridor_width/2,
                wall_height, map, "elevation_map");

        // bottom-left box
        FakeMap::genSolidBox(
                -corridor_width/2, 
                -corridor_width/2,
                map->getLength().x()/2 - corridor_width/2,
                map->getLength().y()/2 - corridor_width/2,
                wall_height, map, "elevation_map");

        // assigning boundary
        double boundary_height = 2;
        for (int i = 0; i < map->getSize()(0); ++i)
        {
            grid_map::Index index(i, 0);
            map->at("elevation_map", index) = boundary_height; 

            index(1) = map->getSize()(1) - 1;
            map->at("elevation_map", index) = boundary_height; 
        }

        for (int i = 0; i < map->getSize()(1); ++i)
        {
            grid_map::Index index(0, i);
            map->at("elevation_map", index) = boundary_height; 

            index(0) = map->getSize()(0) - 1;
            map->at("elevation_map", index) = boundary_height; 
        }
    }

    void FakeMap::loadNoisySimpleCorridorIndoorScene1_()
    {
        FakeMap::loadSimpleCorridorIndoorScene1_();

        // create NAN dots
        size_t num_NAN = 0;
        map_operation::addNANToMap(*map, map->getSize(), 0.01, "elevation_map", "elevation_map");
        num_NAN = map_operation::checkNAN(*map, "elevation_map");
        debugger::debugOutput("num_nan for small holes: ", num_NAN, 5);


        // create NAN holes
        map_operation::addNANHoleToMap(*map, map->getSize(), 0.005, "elevation_map", "elevation_map");
        num_NAN = map_operation::checkNAN(*map, "elevation_map");
        debugger::debugOutput("num_nan for big holes: ", num_NAN, 5);
        
        // add random noise
        map_operation::addNoiseToMap(*map, map->getSize(), 0.1, "elevation_map", "elevation_map");
    } 

    void FakeMap::loadSimpleCorridorIndoorScene2_()
    {
        // due to different resolution, 
        // we can't directly loadWaveField and add noise
        global_map_info = map_info_t(-15, 15, -10, 10, 0.1);
        map = new grid_map::GridMap();
        map->add("elevation_map", 50.0);
        // map->add("traversability_map", 1);
        map->setFrameId("odom");
        map->setGeometry(
                grid_map::Length(global_map_info.len_x, global_map_info.len_y), 
                global_map_info.grid_size);
        ROS_INFO("Created map with size %f x %f m (%i x %i cells).",
                map->getLength().x(), map->getLength().y(),
                map->getSize()(0), map->getSize()(1));


        // std::vector<MultivariateGaussian> terrain;
        // FakeMap::loadLevelThreeWaveFieldTerrain_(terrain);

        // grid_map::Position center(-1, -5); // 0,-5
        // double radius = 0.4;
        // for (grid_map::CircleIterator iterator(*map, center, radius);
        //         !iterator.isPastEnd(); ++iterator) {
        //     map->at("traversability_map", *iterator) = 0.0;
        // }

        grid_map::Matrix& data = map->get("elevation_map");
        for (grid_map::GridMapIterator it(*map); !it.isPastEnd(); ++it) {
            size_t i = it.getLinearIndex();
            data(i) = 0; 
        }



        // Note:-25, 25, -15, 15
        // Box: fixed point is top-right corner of the box
        double corridor_width = 1.8;
        double align_parms = 1.2; // boundary will be occupied...
        double wall_height = 2;
        double map_resolution = map->getResolution();

        // top-right box
        FakeMap::genSolidBox(
                map->getLength().x()/2 - align_parms * corridor_width, 
                map->getLength().y()/2 - align_parms * corridor_width,
                map->getLength().x()/2 - 3 * corridor_width/2 - (align_parms - 0.9) * corridor_width, 
                map->getLength().y()/2 - 3 * corridor_width/2 - (align_parms - 0.9) * corridor_width, 
                wall_height, map, "elevation_map");


        // bottom-right box
        FakeMap::genSolidBox(
                map->getLength().x()/2 - align_parms * corridor_width, 
                -corridor_width/2,
                map->getLength().x()/2 - 3 * corridor_width/2 - (align_parms - 0.9) * corridor_width, 
                map->getLength().y()/2 - 3 * corridor_width/2 - (align_parms - 1.1) * corridor_width, 
                wall_height, map, "elevation_map");

        // top-left box
        FakeMap::genSolidBox(
                -corridor_width/2, 
                map->getLength().y()/2 - align_parms * corridor_width,
                map->getLength().x()/2 - 3 * corridor_width/2 - (align_parms - 0.9) * corridor_width, 
                map->getLength().y()/2 - 3 * corridor_width/2 - (align_parms - 0.9) * corridor_width, 
                wall_height, map, "elevation_map");

        // bottom-left box
        FakeMap::genSolidBox(
                -corridor_width/2, 
                -corridor_width/2,
                map->getLength().x()/2 - 3 * corridor_width/2 - (align_parms - 0.9) * corridor_width, 
                map->getLength().y()/2 - 3 * corridor_width/2 - (align_parms - 1.1) * corridor_width, 
                wall_height, map, "elevation_map");

        // assigning boundary
        double boundary_height = 2;
        for (int i = 0; i < map->getSize()(0); ++i)
        {
            grid_map::Index index(i, 0);
            map->at("elevation_map", index) = boundary_height; 

            index(1) = map->getSize()(1) - 1;
            map->at("elevation_map", index) = boundary_height; 
        }

        for (int i = 0; i < map->getSize()(1); ++i)
        {
            grid_map::Index index(0, i);
            map->at("elevation_map", index) = boundary_height; 

            index(0) = map->getSize()(0) - 1;
            map->at("elevation_map", index) = boundary_height; 
        }
    }

    void FakeMap::loadNoisySimpleCorridorIndoorScene2_()
    {
        FakeMap::loadSimpleCorridorIndoorScene2_();

        // create NAN dots
        size_t num_NAN = 0;
        map_operation::addNANToMap(*map, map->getSize(), 0.01, "elevation_map", "elevation_map");
        num_NAN = map_operation::checkNAN(*map, "elevation_map");
        debugger::debugOutput("num_nan for small holes: ", num_NAN, 5);


        // create NAN holes
        map_operation::addNANHoleToMap(*map, map->getSize(), 0.005, "elevation_map", "elevation_map");
        num_NAN = map_operation::checkNAN(*map, "elevation_map");
        debugger::debugOutput("num_nan for big holes: ", num_NAN, 5);
        
        // add random noise
        map_operation::addNoiseToMap(*map, map->getSize(), 0.1, "elevation_map", "elevation_map");
    } 

    void FakeMap::loadSimpleClutterIndoorScene1_()
    {
        // due to different resolution, 
        // we can't directly loadWaveField and add noise
        global_map_info = map_info_t(-25, 25, -15, 15, 0.1);
        map = new grid_map::GridMap();
        map->add("elevation_map", 50.0);
        // map->add("traversability_map", 1);
        map->setFrameId("odom");
        map->setGeometry(
                grid_map::Length(global_map_info.len_x, global_map_info.len_y), 
                global_map_info.grid_size);
        ROS_INFO("Created map with size %f x %f m (%i x %i cells).",
                map->getLength().x(), map->getLength().y(),
                map->getSize()(0), map->getSize()(1));


        // std::vector<MultivariateGaussian> terrain;
        // FakeMap::loadLevelThreeWaveFieldTerrain_(terrain);

        // grid_map::Position center(-1, -5); // 0,-5
        // double radius = 0.4;
        // for (grid_map::CircleIterator iterator(*map, center, radius);
        //         !iterator.isPastEnd(); ++iterator) {
        //     map->at("traversability_map", *iterator) = 0.0;
        // }

        grid_map::Matrix& data = map->get("elevation_map");
        for (grid_map::GridMapIterator it(*map); !it.isPastEnd(); ++it) {
            size_t i = it.getLinearIndex();
            data(i) = 0; 
        }



        // Note:-25, 25, -15, 15
        // Box: fixed point is top-right corner of the box
        double corridor_width = 1;
        double obs_height = 0.5;
        double map_resolution = map->getResolution();

        // +x, +y
        FakeMap::genSolidBox(
                2.5, 3, 4, 1.5,
                obs_height, map, "elevation_map");
        FakeMap::genSolidBox(
                10, 0, 3.5, 1.5,
                obs_height, map, "elevation_map");
        FakeMap::genSolidBox(
                12, 13, 1, 3,
                obs_height, map, "elevation_map");
        FakeMap::genSolidBox(
                19, 2, 2, 3,
                obs_height, map, "elevation_map");
        FakeMap::genSolidBox(
                3, 13, 5, 2,
                obs_height, map, "elevation_map");
        FakeMap::genSolidBox(
                10, 7, 4, 7,
                obs_height, map, "elevation_map");
        FakeMap::genSolidBox(
                18, 12, 3, 1.5,
                obs_height, map, "elevation_map");
        FakeMap::genSolidBox(
                22, 14, 1.5, 1.5,
                obs_height, map, "elevation_map");
        FakeMap::genSolidBox(
                24, 13, 1.5, 4,
                obs_height, map, "elevation_map");
        FakeMap::genSolidBox(
                14, 1, 2, 0.5,
                obs_height, map, "elevation_map");
        FakeMap::genSolidBox(
                13, 8, 0.5, 2.5,
                obs_height, map, "elevation_map");

        // +x, -y
        FakeMap::genSolidBox(
                22, -1, 1, 4,
                obs_height, map, "elevation_map");
        FakeMap::genSolidBox(
                18, -3, 2, 1,
                obs_height, map, "elevation_map");
        FakeMap::genSolidBox(
                15, -8, 4, 1,
                obs_height, map, "elevation_map");
        FakeMap::genSolidBox(
                6, -9, 2, 5,
                obs_height, map, "elevation_map");
        FakeMap::genSolidBox(
                23, -9, 1, 5,
                obs_height, map, "elevation_map");
        FakeMap::genSolidBox(
                15, -13, 3, 1,
                obs_height, map, "elevation_map");
        FakeMap::genSolidBox(
                10, -2, 1.5, 4,
                obs_height, map, "elevation_map");
        FakeMap::genSolidBox(
                7, -5, 3, 1,
                obs_height, map, "elevation_map");
        FakeMap::genSolidBox(
                2, -4, 1, 3,
                obs_height, map, "elevation_map");



        // -x, +y
        FakeMap::genSolidBox(
                -1, 7, 1, 3,
                obs_height, map, "elevation_map");
        // FakeMap::genSolidBox(
        //         -3, 7, 1, 1,
        //         obs_height, map, "elevation_map");
        // FakeMap::genSolidBox(
        //         -5, 5, 1, 1,
        //         obs_height, map, "elevation_map");
        FakeMap::genSolidBox(
                -8, 6, 4, 1,
                obs_height, map, "elevation_map");
        FakeMap::genSolidBox(
                -9, 8, 1, 1,
                obs_height, map, "elevation_map");
        FakeMap::genSolidBox(
                -10, 10, 1, 1,
                obs_height, map, "elevation_map");
        FakeMap::genSolidBox(
                -5, 0, 4, 1,
                obs_height, map, "elevation_map");
        FakeMap::genSolidBox(
                -12, 3, 3, 1,
                obs_height, map, "elevation_map");
        FakeMap::genSolidBox(
                -13, 13, 1, 2,
                obs_height, map, "elevation_map");
        FakeMap::genSolidBox(
                -14, 5, 2, 1,
                obs_height, map, "elevation_map");
        FakeMap::genSolidBox(
                -19, 10, 2.5, 1,
                obs_height, map, "elevation_map");
        FakeMap::genSolidBox(
                -20, 13, 3, 1,
                obs_height, map, "elevation_map");
        FakeMap::genSolidBox(
                -22, 7, 1, 4,
                obs_height, map, "elevation_map");
        FakeMap::genSolidBox(
                -22, 1, 1, 3,
                obs_height, map, "elevation_map");

        // -x, -y
        FakeMap::genSolidBox(
                -2, -6, 1, 3,
                obs_height, map, "elevation_map");
        FakeMap::genSolidBox(
                -0, -2, 2, 1,
                obs_height, map, "elevation_map");
        FakeMap::genSolidBox(
                -6, -3, 1, 2,
                obs_height, map, "elevation_map");
        FakeMap::genSolidBox(
                -6, -9, 1, 3,
                obs_height, map, "elevation_map");
        FakeMap::genSolidBox(
                -8, -3, 5, 3,
                obs_height, map, "elevation_map");
        FakeMap::genSolidBox(
                -13, -3, 4, 1,
                obs_height, map, "elevation_map");
        FakeMap::genSolidBox(
                -15, -10, 1, 4,
                obs_height, map, "elevation_map");
        FakeMap::genSolidBox(
                -20, -5, 1, 3,
                obs_height, map, "elevation_map");
        FakeMap::genSolidBox(
                -20, -10, 4, 1,
                obs_height, map, "elevation_map");

        // assigning boundary
        double boundary_height = 2;
        for (int i = 0; i < map->getSize()(0); ++i)
        {
            grid_map::Index index(i, 0);
            map->at("elevation_map", index) = boundary_height; 

            index(1) = map->getSize()(1) - 1;
            map->at("elevation_map", index) = boundary_height; 
        }

        for (int i = 0; i < map->getSize()(1); ++i)
        {
            grid_map::Index index(0, i);
            map->at("elevation_map", index) = boundary_height; 

            index(0) = map->getSize()(0) - 1;
            map->at("elevation_map", index) = boundary_height; 
        }
    }

    void FakeMap::loadNoisySimpleClutterIndoorScene1_()
    {
        FakeMap::loadSimpleClutterIndoorScene1_();

        // create NAN dots
        size_t num_NAN = 0;
        map_operation::addNANToMap(*map, map->getSize(), 0.01, "elevation_map", "elevation_map");
        num_NAN = map_operation::checkNAN(*map, "elevation_map");
        debugger::debugOutput("num_nan for small holes: ", num_NAN, 5);


        // create NAN holes
        map_operation::addNANHoleToMap(*map, map->getSize(), 0.005, "elevation_map", "elevation_map");
        num_NAN = map_operation::checkNAN(*map, "elevation_map");
        debugger::debugOutput("num_nan for big holes: ", num_NAN, 5);
        
        // add random noise
        map_operation::addNoiseToMap(*map, map->getSize(), 0.1, "elevation_map", "elevation_map");
    } 

    void FakeMap::loadSimpleClutterIndoorScene2_()
    {
        // due to different resolution, 
        // we can't directly loadWaveField and add noise
        global_map_info = map_info_t(-25, 25, -15, 15, 0.1);
        map = new grid_map::GridMap();
        map->add("elevation_map", 50.0);
        // map->add("traversability_map", 1);
        map->setFrameId("odom");
        map->setGeometry(
                grid_map::Length(global_map_info.len_x, global_map_info.len_y), 
                global_map_info.grid_size);
        ROS_INFO("Created map with size %f x %f m (%i x %i cells).",
                map->getLength().x(), map->getLength().y(),
                map->getSize()(0), map->getSize()(1));


        // std::vector<MultivariateGaussian> terrain;
        // FakeMap::loadLevelThreeWaveFieldTerrain_(terrain);

        // grid_map::Position center(-1, -5); // 0,-5
        // double radius = 0.4;
        // for (grid_map::CircleIterator iterator(*map, center, radius);
        //         !iterator.isPastEnd(); ++iterator) {
        //     map->at("traversability_map", *iterator) = 0.0;
        // }

        grid_map::Matrix& data = map->get("elevation_map");
        for (grid_map::GridMapIterator it(*map); !it.isPastEnd(); ++it) {
            size_t i = it.getLinearIndex();
            data(i) = 0; 
        }



        // Note:-25, 25, -15, 15
        // Box: fixed point is top-right corner of the box
        double corridor_width = 1;
        double obs_height = 0.5;
        double map_resolution = map->getResolution();

        // +x, +y
        FakeMap::genSolidBox(
                2.5, 3, 4, 1.5,
                obs_height, map, "elevation_map");
        FakeMap::genSolidBox(
                10, 0, 3.5, 1.5,
                obs_height, map, "elevation_map");
        FakeMap::genSolidBox(
                12, 13, 1, 3,
                obs_height, map, "elevation_map");
        FakeMap::genSolidBox(
                19, 2, 2, 3,
                obs_height, map, "elevation_map");
        FakeMap::genSolidBox(
                3, 13, 5, 2,
                obs_height, map, "elevation_map");
        FakeMap::genSolidBox(
                10, 7, 4, 7,
                obs_height, map, "elevation_map");
        FakeMap::genSolidBox(
                18, 12, 3, 1.5,
                obs_height, map, "elevation_map");
        FakeMap::genSolidBox(
                22, 14, 1.5, 1.5,
                obs_height, map, "elevation_map");
        FakeMap::genSolidBox(
                24, 13, 1.5, 4,
                obs_height, map, "elevation_map");
        FakeMap::genSolidBox(
                14, 1, 2, 0.5,
                obs_height, map, "elevation_map");
        FakeMap::genSolidBox(
                13, 8, 0.5, 2.5,
                obs_height, map, "elevation_map");

        // +x, -y
        FakeMap::genSolidBox(
                22, -1, 1, 4,
                obs_height, map, "elevation_map");
        FakeMap::genSolidBox(
                18, -3, 2, 1,
                obs_height, map, "elevation_map");
        FakeMap::genSolidBox(
                15, -8, 4, 1,
                obs_height, map, "elevation_map");
        FakeMap::genSolidBox(
                6, -9, 2, 5,
                obs_height, map, "elevation_map");
        FakeMap::genSolidBox(
                23, -9, 1, 5,
                obs_height, map, "elevation_map");
        FakeMap::genSolidBox(
                15, -13, 3, 1,
                obs_height, map, "elevation_map");
        FakeMap::genSolidBox(
                14, -2, 5.5, 8,
                obs_height, map, "elevation_map");
        FakeMap::genSolidBox(
                7, -5, 3, 1,
                obs_height, map, "elevation_map");
        FakeMap::genSolidBox(
                2, -4, 1, 3,
                obs_height, map, "elevation_map");



        // -x, +y
        FakeMap::genSolidBox(
                -1, 7, 1, 3,
                obs_height, map, "elevation_map");
        // FakeMap::genSolidBox(
        //         -3, 7, 1, 1,
        //         obs_height, map, "elevation_map");
        // FakeMap::genSolidBox(
        //         -5, 5, 1, 1,
        //         obs_height, map, "elevation_map");
        FakeMap::genSolidBox(
                -8, 6, 4, 1,
                obs_height, map, "elevation_map");
        FakeMap::genSolidBox(
                -9, 8, 1, 1,
                obs_height, map, "elevation_map");
        FakeMap::genSolidBox(
                -10, 10, 1, 1,
                obs_height, map, "elevation_map");
        FakeMap::genSolidBox(
                -5, 0, 4, 1,
                obs_height, map, "elevation_map");
        FakeMap::genSolidBox(
                -12, 3, 3, 1,
                obs_height, map, "elevation_map");
        FakeMap::genSolidBox(
                -13, 13, 1, 2,
                obs_height, map, "elevation_map");
        FakeMap::genSolidBox(
                -14, 5, 2, 1,
                obs_height, map, "elevation_map");
        FakeMap::genSolidBox(
                -19, 10, 2.5, 1,
                obs_height, map, "elevation_map");
        FakeMap::genSolidBox(
                -20, 13, 3, 1,
                obs_height, map, "elevation_map");
        FakeMap::genSolidBox(
                -22, 7, 1, 4,
                obs_height, map, "elevation_map");
        FakeMap::genSolidBox(
                -22, 1, 1, 3,
                obs_height, map, "elevation_map");

        // -x, -y
        FakeMap::genSolidBox(
                -2, -6, 1, 3,
                obs_height, map, "elevation_map");
        FakeMap::genSolidBox(
                -0, -2, 2, 1,
                obs_height, map, "elevation_map");
        FakeMap::genSolidBox(
                -6, -3, 1, 2,
                obs_height, map, "elevation_map");
        FakeMap::genSolidBox(
                -6, -9, 1, 3,
                obs_height, map, "elevation_map");
        FakeMap::genSolidBox(
                -8, -5, 5, 1,
                obs_height, map, "elevation_map");
        FakeMap::genSolidBox(
                -13, -3, 4, 1,
                obs_height, map, "elevation_map");
        FakeMap::genSolidBox(
                -15, -10, 1, 4,
                obs_height, map, "elevation_map");
        FakeMap::genSolidBox(
                -20, -5, 1, 3,
                obs_height, map, "elevation_map");
        FakeMap::genSolidBox(
                -20, -10, 4, 1,
                obs_height, map, "elevation_map");

        // assigning boundary
        double boundary_height = 2;
        for (int i = 0; i < map->getSize()(0); ++i)
        {
            grid_map::Index index(i, 0);
            map->at("elevation_map", index) = boundary_height; 

            index(1) = map->getSize()(1) - 1;
            map->at("elevation_map", index) = boundary_height; 
        }

        for (int i = 0; i < map->getSize()(1); ++i)
        {
            grid_map::Index index(0, i);
            map->at("elevation_map", index) = boundary_height; 

            index(0) = map->getSize()(0) - 1;
            map->at("elevation_map", index) = boundary_height; 
        }
    }

    void FakeMap::loadNoisySimpleClutterIndoorScene2_()
    {
        FakeMap::loadSimpleClutterIndoorScene2_();

        // create NAN dots
        size_t num_NAN = 0;
        map_operation::addNANToMap(*map, map->getSize(), 0.01, "elevation_map", "elevation_map");
        num_NAN = map_operation::checkNAN(*map, "elevation_map");
        debugger::debugOutput("num_nan for small holes: ", num_NAN, 5);


        // create NAN holes
        map_operation::addNANHoleToMap(*map, map->getSize(), 0.005, "elevation_map", "elevation_map");
        num_NAN = map_operation::checkNAN(*map, "elevation_map");
        debugger::debugOutput("num_nan for big holes: ", num_NAN, 5);
        
        // add random noise
        map_operation::addNoiseToMap(*map, map->getSize(), 0.1, "elevation_map", "elevation_map");
    } 

    void FakeMap::loadRetancularIndoorScene1_()
    {
        // due to different resolution, 
        // we can't directly loadWaveField and add noise
        double length = 30;
        global_map_info = map_info_t(-length/2, length/2, -length/2, length/2, 0.1);
        map = new grid_map::GridMap();
        map->add("elevation_map", 50.0);
        // map->add("traversability_map", 1);
        map->setFrameId("odom");
        map->setGeometry(
                grid_map::Length(global_map_info.len_x, global_map_info.len_y), 
                global_map_info.grid_size);
        ROS_INFO("Created map with size %f x %f m (%i x %i cells).",
                map->getLength().x(), map->getLength().y(),
                map->getSize()(0), map->getSize()(1));


        // std::vector<MultivariateGaussian> terrain;
        // FakeMap::loadLevelThreeWaveFieldTerrain_(terrain);

        // grid_map::Position center(-1, -5); // 0,-5
        // double radius = 0.4;
        // for (grid_map::CircleIterator iterator(*map, center, radius);
        //         !iterator.isPastEnd(); ++iterator) {
        //     map->at("traversability_map", *iterator) = 0.0;
        // }

        grid_map::Matrix& data = map->get("elevation_map");
        for (grid_map::GridMapIterator it(*map); !it.isPastEnd(); ++it) {
            size_t i = it.getLinearIndex();
            data(i) = 0; 
        }



        // Note:-25, 25, -15, 15
        // Box: fixed point is top-right corner of the box
        double corridor_width = 2.5;
        double box_width = 3;
        double obs_height = 2;
        double map_resolution = map->getResolution();

        // 1 (x)
        FakeMap::genSolidBox(
                length/2 - corridor_width, length/2 - corridor_width, 
                length - corridor_width, 
                box_width,
                obs_height, map, "elevation_map");

        // 2 (y)
        FakeMap::genSolidBox(
                length/2 - corridor_width, 
                length/2 - corridor_width - box_width, 
                box_width, 
                length - 2*corridor_width - 2*box_width,
                obs_height, map, "elevation_map");

        // 3 (x)
        FakeMap::genSolidBox(
                length/2 - corridor_width, 
                -length/2 + corridor_width + box_width, 
                length - 2*corridor_width, 
                box_width,
                obs_height, map, "elevation_map");

        // 4 (y)
        FakeMap::genSolidBox(
                -length/2 + corridor_width + box_width, 
                length/2 - 2*corridor_width - box_width, 
                box_width, 
                length - 2*box_width - 2*corridor_width,
                obs_height, map, "elevation_map");

        // 5 (x)
        FakeMap::genSolidBox(
                length/2 - 2*corridor_width - 2*box_width, 
                length/2 - 2*corridor_width - box_width, 
                length - 3*corridor_width - 3*box_width,
                box_width,
                obs_height, map, "elevation_map");

        // 6 (y)
        FakeMap::genSolidBox(
                length/2 - 2*corridor_width - box_width, 
                length/2 - 2*corridor_width - box_width, 
                box_width, 
                length - 4*corridor_width - 2*box_width,
                obs_height, map, "elevation_map");

        // 7 (x)
        FakeMap::genSolidBox(
                length/2 - 2*corridor_width - 2*box_width, 
                length/2 - 2*corridor_width - box_width - 
                (length - 4*corridor_width - 2*box_width - box_width), 
                length - 4*corridor_width - 3*box_width,
                box_width,
                obs_height, map, "elevation_map");

        // 8 (y) 
        FakeMap::genSolidBox(
                length/2 - 2*corridor_width - box_width - 
                (length - 4*corridor_width - 3*box_width), 
                length/2 - 3*corridor_width - 2*box_width, 
                box_width, 
                length - 5*corridor_width - 4*box_width,
                obs_height, map, "elevation_map");

        // 9 (x)
        FakeMap::genSolidBox(
                length/2 - 3*corridor_width - 3*box_width, 
                length/2 - 3*corridor_width - 2*box_width, 
                length - 5*corridor_width - 5*box_width,
                box_width,
                obs_height, map, "elevation_map");

        // 10 (y)
        FakeMap::genSolidBox(
                length/2 - 3*corridor_width - 2*box_width, 
                length/2 - 3*corridor_width - 2*box_width, 
                box_width, 
                length - 6*corridor_width - 4*box_width,
                obs_height, map, "elevation_map");

        // 11 (x)
        FakeMap::genSolidBox(
                length/2 - 3*corridor_width - 3*box_width, 
                length/2 - 3*corridor_width - 2*box_width -  
                (length - 6*corridor_width - 5*box_width),
                length - 6*corridor_width - 5*box_width,
                box_width,
                obs_height, map, "elevation_map");

        // assigning boundary
        double boundary_height = 2;
        for (int i = 0; i < map->getSize()(0); ++i)
        {
            grid_map::Index index(i, 0);
            map->at("elevation_map", index) = boundary_height; 

            index(1) = map->getSize()(1) - 1;
            map->at("elevation_map", index) = boundary_height; 
        }

        for (int i = 0; i < map->getSize()(1); ++i)
        {
            grid_map::Index index(0, i);
            map->at("elevation_map", index) = boundary_height; 

            index(0) = map->getSize()(0) - 1;
            map->at("elevation_map", index) = boundary_height; 
        }
    }

    void FakeMap::loadNoisyRetancularIndoorScene1_()
    {
        FakeMap::loadRetancularIndoorScene1_();

        // create NAN dots
        size_t num_NAN = 0;
        map_operation::addNANToMap(*map, map->getSize(), 0.01, "elevation_map", "elevation_map");
        num_NAN = map_operation::checkNAN(*map, "elevation_map");
        debugger::debugOutput("num_nan for small holes: ", num_NAN, 5);


        // create NAN holes
        map_operation::addNANHoleToMap(*map, map->getSize(), 0.005, "elevation_map", "elevation_map");
        num_NAN = map_operation::checkNAN(*map, "elevation_map");
        debugger::debugOutput("num_nan for big holes: ", num_NAN, 5);
        
        // add random noise
        map_operation::addNoiseToMap(*map, map->getSize(), 0.1, "elevation_map", "elevation_map");
    } 
    void FakeMap::genTerrain(double mean_x, double mean_y, 
            double cov11, double cov12, double cov21, double cov22,
            double height, 
            std::vector<MultivariateGaussian>& terrain, 
            double noise_min_x, double noise_max_x,
            double noise_min_y, double noise_max_y) 
    {
        double random_x = utils::genInclusiveRandomNumber(noise_min_x, noise_max_x);
        double random_y = utils::genInclusiveRandomNumber(noise_min_y, noise_max_y);
        Eigen::Vector2d mean(mean_x + random_x, mean_y + random_y);
        Eigen::Matrix2d cov; 
        cov << cov11, cov12, cov21, cov22; 

        double random_h = utils::genInclusiveRandomNumber(-2, 2);
        MultivariateGaussian sub_map1 = 
            MultivariateGaussian(mean, cov, height + random_h);
        terrain.push_back(sub_map1);
    } 


    void FakeMap::genSolidBox(double anchor_x, double anchor_y, 
                         double len_x, double len_y, 
                         double height,
                         grid_map::GridMap* s_map, std::string layer)
    {
        double map_resolution = s_map->getResolution();
        grid_map::Position position(anchor_x, anchor_y);
        grid_map::Index submapStartIndex;
        s_map->getIndex(position, submapStartIndex);
        grid_map::Index submapBufferSize(len_x / map_resolution, 
                                         len_y / map_resolution);
        for (grid_map::SubmapIterator iterator(*s_map, submapStartIndex, 
                                               submapBufferSize);
                !iterator.isPastEnd(); ++iterator) {
            s_map->at("elevation_map", *iterator) = height;
        }
    }
}
