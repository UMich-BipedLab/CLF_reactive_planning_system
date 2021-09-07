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
#include "fake_map.h"
#include "map_utils.h"

namespace bipedlab {
    FakeMap::FakeMap(int map_num)
    {
        switch (map_num) {
            case 0:
                FakeMap::loadWaveField_();
                
                break;
            case 1:
                FakeMap::loadTestMap1_();
                
                break;
            case 2:
                FakeMap::loadTestMap2_();
                break;
            //default:
        }
    }
    FakeMap::~FakeMap() {}

    void FakeMap::loadWaveField_() {
    }

    void FakeMap::loadTestMap1_() {
    }

    void FakeMap::loadTestMap2_() {
        // map size and resolution
        global_map_info = map_info_t(-15, 15, -15, 15, 0.5);
        map = new grid_map::GridMap();
        map->add("elevation_map", 0.0);
        map->add("traversability_map", 3.0);
        map->setFrameId("map");
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
        start = pose_t(-4, 4, -M_PI/4);
        goal = pose_t(3, -1, 0);


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
        grid_map::Position center(0.0, -5);
        double radius = 0.4;

        for (grid_map::CircleIterator iterator(*map, center, radius);
                !iterator.isPastEnd(); ++iterator) {
            map->at("traversability_map", *iterator) = 0.0;
        }
    }

    // void FakeMap::loadTestMap2_() {
    //     // map size and resolution
    //     global_map_info = map_info_t(-20, 20, -20, 20, 0.1);

    //     // targets
    //     // start = pose_t(1, 9, -M_PI/4);
    //     // goal = pose_t(8, 4, 0);
    //     start = pose_t(-4, 4, -M_PI/4);
    //     goal = pose_t(3, -1, 0);


    //     // height distribution
    //     Eigen::Vector2d mean1(-4, -4);
    //     Eigen::Matrix2d cov1; 
    //     cov1 << 1.5, 0, 0, 3;
    //     MultivariateGaussian *sub_map1 = new MultivariateGaussian(mean1, cov1, 10);


    //     Eigen::Vector2d mean2(0, 0);
    //     Eigen::Matrix2d cov2; 
    //     cov2 << 1.5, 0, 0, 3;
    //     MultivariateGaussian *sub_map2 = new MultivariateGaussian(mean2, cov2, 10);


    //     Eigen::Vector2d mean3(1, 5);
    //     Eigen::Matrix2d cov3; 
    //     cov3 << 1.5, 0, 0, 1.5;
    //     MultivariateGaussian *sub_map3 = new MultivariateGaussian(mean3, cov3, 10);


    //     Eigen::Vector2d mean4(4, 4);
    //     Eigen::Matrix2d cov4; 
    //     cov4 << 2, 0.5, 0.5, 3;
    //     MultivariateGaussian *sub_map4 = new MultivariateGaussian(mean4, cov4, 20);

    //     size_t n_rows = global_map_info.n_rows;
    //     size_t n_cols = global_map_info.n_cols;
    //     this->height_map_mat = Eigen::MatrixXd(n_rows, n_cols);

    //     pcl::PointCloud<pcl::PointXYZI>::Ptr 
    //         msg (new pcl::PointCloud<pcl::PointXYZI>);
    //     msg->header.frame_id = "map";

    //     // compute height and fill in to a matrix
    //     for (int row = 0; row < n_rows; ++row)  // y
    //     {
    //         // double y = global_map_info.y_vec[n_rows - row - 1];
    //         double y = global_map_info.y_vec[row];
    //         for (int col = 0; col < n_cols; ++col) // x
    //         {
    //             double x = global_map_info.x_vec[col]; 
    //             Eigen::Vector2d current_x_y(x, y);
    //             double height = 
    //                 sub_map1->distribution(current_x_y) + 
    //                 sub_map2->distribution(current_x_y) + 
    //                 sub_map3->distribution(current_x_y) + 
    //                 sub_map4->distribution(current_x_y);
    //             this->height_map_mat(row, col) = height;

    //             msg->points.push_back(
    //                 pcl::PointXYZI( 
    //                     (float) x, (float) y, (float) height, (float) height * 10));

    //             // std::pair<float, float> world_xy = 
    //             //     map_utils::mapToWorld<float>(global_map_info.grid_size, col, row);
    //             // msg->points.push_back(
    //             //     pcl::PointXYZI( 
    //             //         (float) world_xy.first, 
    //             //         (float) world_xy.second, (float) height, (float) height * 10));
    //             // std::cout << "(x, y, z)" << x << ", " << y << ", " << height << std::endl;
    //         }
    //     }
    //     this->height_map_xyz = msg;
    //     // std::cout << "length:" << this->height_map_xyz.size()<< std::endl;
    // }

}
