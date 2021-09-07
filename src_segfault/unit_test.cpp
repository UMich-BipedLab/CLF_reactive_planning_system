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
#include "map_cost.h"
#include "utils/plotting.h"
#include "utils/debugger.h"
#include "utils/utils.h"
#include "clf_rrt.h"



// 0-4 general debugging purposes
// 5-8: algorithm status/process
int DEBUG_LEVEL = 3; 

using namespace bipedlab;
// typedef velodyne_pointcloud::PointXYZIR PointXYZRI;


int main(int argc, char *argv[]) {
    // ros
    ros::init(argc, argv, "test_rrt");
    ros::NodeHandle nh;
    // std::cout << DEBUG_LEVEL << std::endl;
    
    // publishers
    ros::Publisher map_pub = nh.advertise<grid_map_msgs::GridMap>(
            "world_map", 1, true);
    ros::Publisher height_pub = nh.advertise<grid_map_msgs::GridMap>(
            "local_map", 1, true);
    // ros::Publisher debug_pub = nh.advertise<grid_map_msgs::GridMap>(
    //         "debug", 1, true);
    ros::Publisher path_pub = 
        nh.advertise<pcl::PointCloud<pcl::PointXYZI>> ("path_points", 1);
    ros::Publisher search_pub = 
        nh.advertise<pcl::PointCloud<pcl::PointXYZI>> ("search_points", 1);
    ros::Publisher marker_pub = 
        nh.advertise<visualization_msgs::MarkerArray>("results", 10);


    // markers
    visualization_msgs::Marker marker;
    visualization_msgs::MarkerArray marker_array;
    pcl::PointCloud<pcl::PointXYZI>::Ptr 
        path_points (new pcl::PointCloud<pcl::PointXYZI>);
    path_points->header.frame_id = "map";

    pcl::PointCloud<pcl::PointXYZI>::Ptr 
        search_points (new pcl::PointCloud<pcl::PointXYZI>);
    search_points->header.frame_id = "map";


    // map parameters
    int scene = 2; // which map to use

    // sampling params
    double goal_bias = 0.2;
    double distance_threshold_for_sampling = 0;

    pose_sampler_params_t pose_sampler_params(goal_bias, 
                                              distance_threshold_for_sampling);

    // rrt params
    size_t mode = 1;
    size_t num_samples = SIZE_MAX;
    double allowed_computation_time = 120; // in seconds
    bool terminate_if_path = false; 


    rrt_params_t rrt_params;
    // rrt_params.num_samples = num_samples;
    // rrt_params.allowed_computation_time = allowed_computation_time;
    // rrt_params.terminate_if_path = terminate_if_path;
    rrt_params.mode = mode;

    // load map
    FakeMap* fake_map = new FakeMap(scene);




    debugger::debugTitleTextOutput("[Main]", "Pose Setting", 5, Y);
    pose_t pose1 = fake_map->start;
    // pose_t pose2(-0.5, -2.5, M_PI/4);
    pose_t pose2(0, 0, -M_PI/4);
    pose_t pose3 = fake_map->goal;
    debugger::debugOutput("[Main] Pose1: ", pose1, 5);
    debugger::debugOutput("[Main] Pose2: ", pose2, 5);
    debugger::debugOutput("[Main] Pose3: ", pose3, 5);

    size_t local_map_mode = 0; // default mode, no fill-in holes
    double length_of_local_map = 19; // 6: 3 front and 3 back
    double obstacle_threshold = 1; // beyond this cost, they will be considered
    local_map_params_t local_map_info(local_map_mode, obstacle_threshold, length_of_local_map);



    LyapunovDistance* lyap_dist = new LyapunovDistance();
    LocalMap* local_map = new LocalMap(&pose1, (fake_map->map), 
                                       local_map_info);
    exit(-1);
    MapCost* map_cost = new MapCost(local_map, rrt_params.mode);
    LyapunovPath* lyap_path = new LyapunovPath(*lyap_dist, *local_map, 
                                               *map_cost, rrt_params.mode);

    // LyapunovDistance() is declared here for speed
    // whichever class (lyapunovPath, CassieRRTTree)
    // wants to change the target pose of the local chart inside of
    // the LyapunovDistance(), CHANGE ON ITS OWN!


    // test pose distance on manifold
    debugger::debugTitleTextOutput("[Main]", "Pose Distance", 0, Y);
    double dis_pose1_pose2 = lyap_dist->computeDistanceFromPoseWithTargetPose(
            pose1, pose2);
    debugger::debugOutput("[Main] pose1 and pose2: ", dis_pose1_pose2, 0);
    //debugger::debugOutput("[Main] Target pose in lypa_dist: ", lyap_dist->getTargetPose(), 0);
    double dis_pose2_pose3 = lyap_dist->computeDistanceFromPoseWithTargetPose(
            pose2, pose3);
    debugger::debugOutput("[Main] pose2 and pose3: ", dis_pose2_pose3, 0);
    //debugger::debugOutput("[Main] Target pose in lypa_dist: ", lyap_dist->getTargetPose(), 0);
    double dis_pose1_pose3 = lyap_dist->computeDistanceFromPoseWithTargetPose(
            pose1, pose3);
    //debugger::debugOutput("[Main] Target pose in lypa_dist: ", lyap_dist->getTargetPose(), 0);
    debugger::debugOutput("[Main] pose1 and pose3: ", dis_pose1_pose3, 0);



    // test path generation
    std::vector<point2d_t<double>> planned_path;
    debugger::debugTitleTextOutput("[Main]", "Path Generation and Cost", 5, Y);
    path_segment_t path_segment1 =
        lyap_path->steer(pose1, pose2);
    debugger::debugOutput("[Main] Target pose in lypa_dist: ", lyap_dist->getTargetPose(), 0);
    // const cost_t test_cost(0);
    debugger::debugOutput("[Main] cost: ", path_segment1.cost_along_path, 5);


    path_segment_t path_segment2 =
        lyap_path->steer(pose2, pose3);
    debugger::debugOutput("[Main] Target pose in lypa_dist: ", lyap_dist->getTargetPose(), 0);
    debugger::debugOutput("[Main] cost: ", path_segment2.cost_along_path, 5);

    planned_path.insert(planned_path.begin(),
            path_segment2.path.begin(),
            path_segment2.path.end());

    planned_path.insert(planned_path.begin(),
            path_segment1.path.begin(),
            path_segment1.path.end());


    // std::cout << std::setprecision(2) << fake_map->height_map_mat << std::endl;

    // load rrt
    // robot_state_t robot_state(fake_map->start);
    // CLFRRTStarPlanner* planner = new CLFRRTStarPlanner(
    //         *(fake_map->map), length_of_local_map, 
    //         pose_6dof_t(fake_map->start), pose_6dof_t(fake_map->goal),
    //         robot_state, pose_sampler_params, rrt_params);
    
    // grid_map::GridMap local_map({"test"});
    // local_map.setFrameId(planner->getLocalMap().getFrameId());
    // local_map["test"] = (planner->getLocalMap())["elevation"];
    // grid_map::GridMap local_map = planner->getLocalMap();
    // grid_map::GridMap debug_local_map = planner->debug_local_map;
    // local_map.add("local_map", local_map["elevation"]);
    // local_map.clear("elevation");



    // show targets
    plotting::addMarkerWithPose(marker, visualization_msgs::Marker::ARROW, 
              "pose1", 
              0, 1, 0, 1, // color
              pose1, // pose
              0, 0 // count, time
              );
    marker_array.markers.push_back(marker);
    plotting::addMarkerWithPose(marker, visualization_msgs::Marker::ARROW, 
              "pose2", 
              0, 1, 1, 1, // color
              pose2,
              0, 0 // count, time
              );
    marker_array.markers.push_back(marker);
    plotting::addMarkerWithPose(marker, visualization_msgs::Marker::ARROW, 
              "pose3", 
              0, 1, 1, 1, // color
              pose3,
              0, 0 // count, time
              );
    marker_array.markers.push_back(marker);
    marker_pub.publish(marker_array);

    ros::Time time = ros::Time::now();
    fake_map->map->setTimestamp(time.toNSec());
    grid_map_msgs::GridMap message;
    grid_map::GridMapRosConverter::toMessage(*(fake_map->map), message);
    map_pub.publish(message);

    grid_map_msgs::GridMap message2;
    grid_map::GridMapRosConverter::toMessage((local_map->local_map), message2);
    height_pub.publish(message2);


    double radius = 2 * local_map->local_map.getResolution();
    for (const auto& it : planned_path)
    {
        // bool is_locally_free = local_map->isNeighborObstacleFree(
        //         it.x, it.y, rrt_params.distance_threshold);

        // search locally
        bool is_locally_free = true;
        grid_map::Position center(it.x, it.y);
        
        if (local_map->local_map.atPosition("occupancy_map", center) == 1)
            is_locally_free = false;
        else
        {
            search_points->points.clear();
            for (grid_map::CircleIterator iterator(local_map->local_map, center, radius);
                    !iterator.isPastEnd(); ++iterator) 
            {
                grid_map::Position position;
                local_map->local_map.getPosition(*iterator, position);
                if (local_map->local_map.at("occupancy_map", *iterator) == 1) 
                {
                    is_locally_free = false;
                    search_points->points.push_back(pcl::PointXYZI(
                                (float) position.x(), (float) position.y(), 1.2, 0));
                }
                else
                {
                    search_points->points.push_back(pcl::PointXYZI(
                                (float) position.x(), (float) position.y(), 1.2, 255));
                }
            }
        }
        pcl_conversions::toPCL(time, 
                               search_points->header.stamp);
        path_pub.publish(search_points);
        search_pub.publish(search_points);


        if (!is_locally_free) {
            path_points->points.push_back(pcl::PointXYZI(
                (float) it.x, (float) it.y, 1.5, 0));
            debugger::debugTextOutput("[main] point occupied", 0);
        }
        else
        {
            path_points->points.push_back(pcl::PointXYZI(
                (float) it.x, (float) it.y, 1.5, 255));
            debugger::debugTextOutput("[main] point free", 0);
        }
        pcl_conversions::toPCL(time, 
                               path_points->header.stamp);
        path_pub.publish(path_points);
        // utils::pressEnterToContinue();
    }

    // plotting::addMarkerWithPose(marker, visualization_msgs::Marker::ARROW, 
    //           "sample pose", 
    //           0, 1, 1, 1, // color
    //           planner->sample_pose_testing,
    //           0, 0 // count, time
    //           );
    // marker_array.markers.push_back(marker);

    // if (planner->getPathStatus())
    // {
    //     std::vector<point2d_t<double>> path = planner->getPlannedPath(); 
    //     std::vector<std::pair<pose_t, bool>> waypoints = 
    //         planner->getPlannedWaypoints(); 

    //     for (const auto& it : path)
    //     {
    //         // plotting::addMarker(marker, visualization_msgs::Marker::SPHERE, 
    //         //         "planned path", 
    //         //         0, 0, 0, 0, // color
    //         //         it.x, it.y, 0, // location
    //         //         0, 0, 0, 1, // quaternion
    //         //         count, 0.05 // count, time
    //         //         );
    //         // count ++;
    //         // marker_array.markers.push_back(marker);
    //         path_points->points.push_back(pcl::PointXYZI(
    //                 (float) it.x, (float) it.y, 0.1, 0));

    //     }

    //     size_t count = 0;
    //     for (const auto& it : waypoints)
    //     {
    //         plotting::addMarkerWithPose(marker, visualization_msgs::Marker::ARROW, 
    //                 "waypoints", 
    //                 1, 1, 0, 1, // color
    //                 pose_t(it.first),
    //                 count, 0 // count, time
    //                 );
    //         count ++;
    //         marker_array.markers.push_back(marker);
    //     }
    // }

    std::cout << "all done" << std::endl;
    ros::Rate loop_rate(1);
    while (nh.ok())
    {
        ros::Time time = ros::Time::now();
        pcl_conversions::toPCL(time, 
                               path_points->header.stamp);
        path_pub.publish(path_points);

        fake_map->map->setTimestamp(time.toNSec());
        grid_map_msgs::GridMap message;
        grid_map::GridMapRosConverter::toMessage(*(fake_map->map), message);
        map_pub.publish(message);

        grid_map_msgs::GridMap message2;
        grid_map::GridMapRosConverter::toMessage((local_map->local_map), message2);
        height_pub.publish(message2);


        // debug_local_map.setTimestamp(time.toNSec());
        // grid_map_msgs::GridMap message3;
        // grid_map::GridMapRosConverter::toMessage(debug_local_map, message3);
        // debug_pub.publish(message3);

        marker_pub.publish(marker_array);
        ros::spinOnce();
        loop_rate.sleep();
    }
    
    return 0;
}
