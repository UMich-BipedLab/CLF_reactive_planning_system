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
#include <stdlib.h>
#include <iostream>

#include <ros/ros.h>

#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h> // Marker

#include <pcl_conversions/pcl_conversions.h>
// #include <velodyne_pointcloud/point_types.h>
// #include <velodyne_pointcloud/pointcloudXYZIR.h>



#include <eigen3/Eigen/Dense> // SVD
#include <eigen3/Eigen/StdVector>
#include <eigen3/Eigen/Eigenvalues>


#include "fake_map.h"
#include "local_map.h"
#include "utils/debugger.h"
#include "utils/utils.h"



// 0-4 general debugging purposes
// 5-8: algorithm status/process
int DEBUG_LEVEL = 3; 

using namespace bipedlab;
// typedef velodyne_pointcloud::PointXYZIR PointXYZRI;


int main(int argc, char *argv[]) {
    // int scene = 2; // which map to use
    // double length_of_local_map = 6; // 6: 3 front and 3 back
    // double obstacle_threshold = 1; // beyond this cost, they will be considered


    // // load map
    // FakeMap* fake_map = new FakeMap(scene, obstacle_threshold);
    // pose_t pose1 = fake_map->start;

    // LocalMap* local_map = new LocalMap(&pose1, (fake_map->map), length_of_local_map);
    // debugger::debugTextOutput("[main] updateLocalMap()", 3);
    // local_map->updateLocalMap(pose_t());
    // debugger::debugTitleTextOutput("[main] ","all done", 3);


    int scene = 2; // which map to use

    // load map
    FakeMap* fake_map = new FakeMap(scene);
    pose_t pose1 = fake_map->start;


    int mode = 0; // default mode, no fill-in holes
    double length_of_local_map = 6; // 6: 3 front and 3 back
    double obstacle_threshold = 1; // beyond this cost, they will be considered
    local_map_params_t local_map_info(mode, obstacle_threshold, length_of_local_map);
    LocalMap* local_map = new LocalMap(&pose1, (fake_map->map), local_map_info);
    debugger::debugTextOutput("[main] updateLocalMap()", 3);
    local_map->updateLocalMap(pose_t());

    debugger::debugTitleTextOutput("[main] ","all done", 3);



    return 0;
}
