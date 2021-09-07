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
#include <iostream>
#include <Eigen/Dense>
#include <cmath>
#include <vector>

#include "multivariate_gaussian.h"
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/impl/point_types.hpp>

#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>

#include "point.h"
#include "pose.h"

#ifndef FAKE_MAP_H
#define FAKE_MAP_H

namespace bipedlab{


typedef struct map_info
{
    // map resolution
    double grid_size;

    // map boundary
    double min_x;
    double max_x;
    double min_y;
    double max_y;

    // length
    double len_x;
    double len_y;

    // map x-y coordinate
    // std::vector<std::vector<std::pair<double, double>>> map_mesh_grid_v;
    std::vector<double> x_vec;
    std::vector<double> y_vec;


    size_t n_cols; // size of x
    size_t n_rows; // size of y

    map_info(void) : grid_size(0.1), 
        min_x(-2.5), max_x(2.5), min_y(-2.5), max_y(2.5) 
    { 
        computeMeshGrid();
    }

    map_info(double min_x, double max_x, double min_y, double max_y, double grid_size) :
        min_x(min_x), max_x(max_x), min_y(min_y), max_y(max_y), grid_size(grid_size)
    {
        computeMeshGrid();
    }

    void computeMeshGrid() 
    {
        len_x = max_x - min_x;
        len_y = max_y - min_y;

        double x_val = min_x;
        while (x_val <= max_x) 
        {
            x_vec.push_back(x_val);
            x_val += grid_size;
        }
        n_cols = x_vec.size();

        double y_val = min_y;
        while (y_val <= max_y) 
        {
            y_vec.push_back(y_val);
            y_val += grid_size;
        }
        n_rows = y_vec.size();
    }
} map_info_t;


class FakeMap
{
private:
    double obstacle_threshold_;
    void loadTestMap2_();
    void loadTestMap1_();
    void loadWaveField_();
    double sumSubMaps_() { }

    // Variadic function Template that takes 
    // variable number of arguments and prints
    // all of them.
    // template <typename T, typename... Types>
    // double sumSubMaps_(T* var1, Types*... var2)
    // {
    //     var1->probablity()
    //     sumSubMaps_(var2...) ;
    // }

    

public:
    FakeMap(int map_num, double obstacle_threshold);

    map_info_t global_map_info;
    grid_map::GridMap* map;

    pcl::PointCloud<pcl::PointXYZI>::Ptr height_map_xyz; // a vector of x, y, z, which z is height
    Eigen::MatrixXd height_map_mat; // contains height as a matrix

    pose_t start;
    pose_t goal;

    ~FakeMap();
};
}

#endif /* FAKE_MAP_H */



