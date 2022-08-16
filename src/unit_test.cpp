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
#include <assert.h>     /* assert */

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

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>



#include "fake_map.h"
#include "lie_group.h"
#include "map_cost.h"
#include "utils/plotting.h"
#include "utils/debugger.h"
#include "utils/utils.h"
#include "utils/line.h"
#include "utils/plane.h"
#include "clf_rrt.h"
#include "point.h"
#include "csv.h"
#include "point.h"

#include "rviz_visual_tools/rviz_visual_tools.h"



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

   // debugger::debugOutput("[Unit test] ", "Least Squares", 5);
   // Eigen::MatrixXf A = Eigen::MatrixXf::Random(3, 2);
   // std::cout << "Here is the matrix A:\n" << A << std::endl;
   // Eigen::VectorXf b = Eigen::VectorXf::Random(3);
   // std::cout << "Here is the right hand side b:\n" << b << std::endl;
   // std::cout << "The least-squares solution is:\n"
   //      << A.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b) << std::endl;



   // For visualizing things in rviz


   debugger::debugTitleTextOutput("[Unit test] ", "Plane Fitting", 5);
   // plane::plane_t test_plane;
   // test_plane.print();
   int dataset = 2;

   Eigen::MatrixXd lidar_points; // input points
   double y_width = 2;
   if (dataset == 1)
   {
       Eigen::MatrixXd test_points(3, 8); // input points
       test_points <<
           0.014047, 0.53454, 0.5782, 0.06592, 0.00070, 0.42795, 0.73865, 0.84611,
           0.047974, 0.83191, 0.4617, 0.45222, 0.33814, 0.67403, 0.52019, 0.45547,
           0.677590, 0.27622, 0.7249, 0.80713, 0.34365, 0.47471, 0.29546, 0.20986;
       lidar_points = test_points;
   }
   else if (dataset == 2)
   {
       size_t num_test_points = 10000;
       auto xs = utils::genListOfInclusiveRandomNumbers<float>(num_test_points, -1, 3);
       auto ys = utils::genListOfInclusiveRandomNumbers<float>(num_test_points, 
                                                               -y_width/2, y_width/2);
       auto z_noise = utils::genListOfInclusiveRandomNumbers<float>(num_test_points, 
                                                               -0.05, 0.05);
       std::vector<float> zs(num_test_points, 0);
       double k1 = 1;
       double k2 = 0;
       for (int i = 0; i < num_test_points; ++i)
       {
           zs[i] = std::cos(k1 * xs[i]) * std::cos(k2 * ys[i]) + z_noise[i];
       }

       Eigen::MatrixXf test_points(3, num_test_points);
       float* xs_ptr = &xs[0];
       test_points.row(0) = Eigen::Map<Eigen::VectorXf, Eigen::Unaligned>(xs.data(), xs.size());

       float* ys_ptr = &ys[0];
       test_points.row(1) = Eigen::Map<Eigen::VectorXf, Eigen::Unaligned>(ys.data(), ys.size());

       float* zs_ptr = &zs[0];
       test_points.row(2) = Eigen::Map<Eigen::VectorXf, Eigen::Unaligned>(zs.data(), zs.size());
       lidar_points = test_points.cast<double>();
   }

   // robot pose
   pose_t robot_pose(0, 0, deg_to_rad(0));
   Eigen::Vector2f robot_pose_vec = Eigen::Vector2f(std::cos(robot_pose.theta),
                                                    std::sin(robot_pose.theta));




   // plane fitting
   int max_step = 5;
   double increment = 0.3;
   double delta_y = 0.3;
   size_t num_points = lidar_points.cols();
   std::vector<plane::terrain_info_t> terrain_plane_vec(max_step);
   std::vector<std::vector<Eigen::Vector3f>> segment_points(max_step);

   for (int i = 0; i < num_points; ++i)
   {
       Eigen::Vector2f v = lidar_points.block(0, i, 2, 1).cast<float>() - Eigen::Vector2f(robot_pose.x, robot_pose.y);
       double distance = v.norm();
       double x_distance = robot_pose_vec.dot(v);

       double theta = std::acos(x_distance / std::max(distance, 1e-5));
       double y_distance = distance * std::sin(theta);
       if (y_distance > delta_y || x_distance < 0)
           continue;


       // int k = -1;
       // if (x_distance >= 0 && x_distance <= increment)
       // {
       //     k = 0;
       // }
       // else if (x_distance >= 1 * increment && x_distance <= 2 * increment)
       //     k = 1;
       // else if (x_distance >= 2 * increment && x_distance <= 3 * increment)
       //     k = 2;
       // else if (x_distance >= 3 * increment && x_distance <= 4 * increment)
       //     k = 3;
       // else if (x_distance >= 4 * increment && x_distance <= 5 * increment)
       //     k = 4;
       // else
       // {
       //     continue;
       //     // std::string error_msg = "No such distnace: " + std::to_string(k);
       //     // debugger::debugExitColor(error_msg, __LINE__, __FILE__);
       // }
       int k = std::floor(x_distance / increment);
       if (k < max_step)
       {
           segment_points[k].push_back(lidar_points.col(i).cast<float>());
       }
   }

   // plane fitting
   
   for (int seg = 0; seg < max_step; ++seg)
   {
       size_t num_seg = segment_points[seg].size();
       Eigen::MatrixXf seg_points = Eigen::MatrixXf(3, num_seg);
       for (int i = 0; i < num_seg; ++i)
       {
           seg_points.col(i) = segment_points[seg][i];
       }
       terrain_plane_vec[seg].points = seg_points;
       auto plane_params = plane::fitPlaneViaLeastSquares(terrain_plane_vec[seg].points);
       std::cout << "===================================\n";
       std::vector<float> plane_coefficients = plane_params.getPlaneCoefficients();
       terrain_plane_vec[seg].normal_vector = plane_params.normal_vector;
       terrain_plane_vec[seg].fixed_point = plane_params.fixed_point;

       // terrain_plane_vec[seg] = *plane_params;
   }
   // lidar_points = seg_points;


   // plane_params->print(); // from matlab: 1.7764e-15          1.5           -1  -1.1552e-15
   // std::vector<float> plane_coefficients = plane_params->getPlaneCoefficients();


   // viz
   rviz_visual_tools::RvizVisualToolsPtr visual_tools_;
   visual_tools_.reset(new rviz_visual_tools::RvizVisualTools("base","/rviz_visual_markers"));
   visual_tools_->loadMarkerPub();  // create publisher before waiting

   // Clear messages
   visual_tools_->deleteAllMarkers();
   visual_tools_->enableBatchPublishing();

   // publish all points
   for (int i = 0; i < lidar_points.cols(); ++i) {
       geometry_msgs::Vector3 scale = visual_tools_->getScale(rviz_visual_tools::MEDIUM);
       std_msgs::ColorRGBA color = visual_tools_->getColor(rviz_visual_tools::WHITE);
       visual_tools_->publishSphere(lidar_points.col(i), color, scale, "all points");
   }

   // publish plane information
   for (int seg = 0; seg < max_step; ++seg)
   {
       geometry_msgs::Vector3 scale = visual_tools_->getScale(rviz_visual_tools::MEDIUM);
       std_msgs::ColorRGBA color = visual_tools_->getColorScale((float) seg / max_step);
       for (int i = 0; i < terrain_plane_vec[seg].points.cols(); ++i)
       {
           visual_tools_->publishSphere(
                   terrain_plane_vec[seg].points.col(i).cast<double>(),
                   color, scale, "seg points");
       }
       std::vector<float> plane_coefficients = 
           terrain_plane_vec[seg].getPlaneCoefficients();
       Eigen::Vector3d center = terrain_plane_vec[seg].points.rowwise().mean().cast<double>();
       visual_tools_->publishABCDPlaneWithCenter(plane_coefficients[0],
               plane_coefficients[1],
               plane_coefficients[2],
               plane_coefficients[3], center, color, increment, 2*delta_y);
   }
   visual_tools_->trigger();
   exit(0);




    // debugger::debugOutput("[Unit test] Quaternion", "", 5);
    // tf2::Quaternion q;
    // double yaw = 0.1;
    // double pitch = 0.2;
    // double roll = 0.3;
    // q.setRPY(yaw, pitch, roll);  // Create this quaternion from roll/pitch/yaw (in radians)
    // std::cout << "q_ypr: " << q.getX() << ", " << q.getY() << ", " << q.getZ() << ", " << q.getW() << std::endl;

    // q.setRPY(yaw, pitch, roll);  // Create this quaternion from roll/pitch/yaw (in radians)
    // std::cout << "q: " << q.getX() << ", " << q.getY() << ", " << q.getZ() << ", " << q.getW() << std::endl;


    // q.setRPY(yaw, pitch, roll);  // Create this quaternion from roll/pitch/yaw (in radians)
    // std::cout << "q: " << q.getX() << ", " << q.getY() << ", " << q.getZ() << ", " << q.getW() << std::endl;


    // q.setRPY(yaw, pitch, roll);  // Create this quaternion from roll/pitch/yaw (in radians)
    // std::cout << "q: " << q.getX() << ", " << q.getY() << ", " << q.getZ() << ", " << q.getW() << std::endl;

    // q.setRPY(yaw, pitch, roll);  // Create this quaternion from roll/pitch/yaw (in radians)
    // std::cout << "q: " << q.getX() << ", " << q.getY() << ", " << q.getZ() << ", " << q.getW() << std::endl;


    // q.setRPY(yaw, pitch, roll);  // Create this quaternion from roll/pitch/yaw (in radians)
    // std::cout << "q: " << q.getX() << ", " << q.getY() << ", " << q.getZ() << ", " << q.getW() << std::endl;

    // q.setRPY(yaw, pitch, roll);  // Create this quaternion from roll/pitch/yaw (in radians)


    // tf2::Quaternion q(0        , 0,   -0.3827, 0.9239         );
    // double roll, pitch, yaw;
    // tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

    // std::cout << "q: " << q.getX() << ", " << q.getY() << ", " << q.getZ() << ", " << q.getW() << std::endl;
    // std::cout << "(roll, pitch, yaw): " << roll << ", " << pitch << ", " << yaw << std::endl;


    // debugger::debugOutput("[Unit test] Eigen rotation test", "", 5);
    // double yaw = 0.1;
    // double pitch = 0.2;
    // double roll = 0.3;
    // debugger::debugColorOutput("roll1: ", roll, 5);
    // debugger::debugColorOutput("pitch1: ", pitch, 5);
    // debugger::debugColorOutput("yaw1: ", yaw, 5);

    // Eigen::Matrix3d m;
    // m = Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()) *
    //     Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
    //     Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());
    // debugger::debugColorOutput("m: \n", m, 5);
    // debugger::debugColorOutput("unskew: \n", lie_group::computeUnskew(m), 5);
    // Eigen::Vector3d v = lie_group::computeLogSO3(m);
    // debugger::debugColorOutput("v: \n", v, 5);
    // Eigen::Matrix3d new_m = lie_group::computeExpSO3(v);
    // debugger::debugColorOutput("new_m: \n", new_m, 5);
    // Eigen::Vector3d rpy = new_m.eulerAngles(0, 1, 2);

    // debugger::debugColorOutput("roll2: ", rpy(0), 5);
    // debugger::debugColorOutput("pitch2: ", rpy(1), 5);
    // debugger::debugColorOutput("yaw2: ", rpy(2), 5);


    // test random numbers
    // CSVFile log_csv;
    // try
    // {
    //     std::string date = utils::getTimeNDate();
    //     std::string path = utils::getCurrentDirectory();
    //     std::string name = std::string(path) + "/" + date + "_random_numbers.csv";

    //     log_csv.open(name); // throws exceptions!
    //     debugger::debugColorOutput("saved path: " + std::string(name), "", 5, W);


    //     // Header
    //     log_csv << "iter" << "num" << endrow;
    // }
    // catch (const std::exception& ex)
    // {
    //     std::cout << "Exception was thrown: " << ex.what() << std::endl;
    // }

    // size_t num_rand = 1000;
    // size_t precision = 4;
    // for (int i = 0; i < num_rand; ++i)
    // {
    //     double num = utils::genInclusiveRandomNumber(0.0, 10.0);
    //     // debugger::debugColorOutput("num-" + std::to_string(i) + ": ", num, 5);
    //     log_csv << utils::toStringWithPrecision(i, precision)
    //      << utils::toStringWithPrecision(num, precision) << endrow;

    // }



    // test assignUnknownBehindObstacles


    // test coimputeBresenham
    auto points = line::coimputeBresenham(0, 0, -5, 0);
    points->print();

    points = line::coimputeBresenham(0, 0, 5, 0);
    points->print();

    points = line::coimputeBresenham(0, 0, 0, 5);
    points->print();

    points = line::coimputeBresenham(0, 0, 0, -5);
    points->print();


    exit(0);


    // test Eigen SVD
    Eigen::Matrix3f M; // input matrix
    M <<        0,         0,        0,
        -0.660054, -0.510275, 0.651784,
         0.631292,  0.154421,  0.80545;

    Eigen::Matrix3f U; // save for comparision
    Eigen::Matrix3f V; // save for comparision
    Eigen::Vector3f S; // save for comparision

    // run 100 times to check consistency
    for (int i = 0; i < 100; ++i)
    {
        std::cout << "=============" << std::endl;
        std::cout << "i: " << i << std::endl;
        std::cout << "M: \n" << M << std::endl;
        Eigen::JacobiSVD<Eigen::MatrixXf> svd(
                M, Eigen::ComputeFullU | Eigen::ComputeFullV);


        std::cout << "U: \n" << svd.matrixU() << std::endl;
        std::cout << "S: \n" << svd.singularValues() << std::endl;
        std::cout << "V: \n" << svd.matrixV() << std::endl;

        // take the first results and then compare them with the other
        if (i == 0)
        {
            U = svd.matrixU();
            S = svd.singularValues();
            V = svd.matrixV();
        }

        std::cout << "norm(U): \n" << (U - svd.matrixU()).norm() << std::endl;
        std::cout << "norm(S): \n" << (S - svd.singularValues()).norm() << std::endl;
        std::cout << "norm(V): \n" << (V - svd.matrixV()).norm() << std::endl;

        // raise error if not consistent
        assert((U - svd.matrixU()).norm() < 1e-5);
        assert((S - svd.singularValues()).norm() < 1e-5);
        assert((V - svd.matrixV()).norm() < 1e-5);
    }




    // test angle from std
    double p_x = 0;
    double p_y = 0;

    // I
    double p1_x = 1;
    double p1_y = 3;
    double dis_p1_p = std::sqrt((p1_x - p_x) * (p1_x - p_x) + (p1_y - p_y) * (p1_y - p_y));

    // II
    double p2_x = -1;
    double p2_y = 3;
    double dis_p2_p = std::sqrt((p2_x - p_x) * (p2_x - p_x) + (p2_y - p_y) * (p2_y - p_y));

    // III
    double p3_x = -1;
    double p3_y = -3;
    double dis_p3_p = std::sqrt((p3_x - p_x) * (p3_x - p_x) + (p3_y - p_y) * (p3_y - p_y));

    // IV
    double p4_x = 1;
    double p4_y = -3;
    double dis_p4_p = std::sqrt((p4_x - p_x) * (p4_x - p_x) + (p4_y - p_y) * (p4_y - p_y));


    // +x-axis
    double p5_x = 1;
    double p5_y = 0;

    // -x-axis
    double p6_x = -1;
    double p6_y = 0;

    // +y-axis
    double p7_x = 0;
    double p7_y = 1;

    // -y-axis
    double p8_x = 0;
    double p8_y = -1;


    // double theta1 = std::asin((p1_x - p_x) / dis_p1_p);
    // double theta2 = M_PI + std::asin((p2_x - p_x) / dis_p2_p);
    // double theta3 = -M_PI - std::asin((p3_x - p_x) / dis_p3_p);
    // double theta4 = -std::asin((p4_x - p_x) / dis_p4_p);
    // debugger::debugOutput("theta1: ", theta1, 5);
    // debugger::debugOutput("theta2: ", theta2, 5);
    // debugger::debugOutput("theta3: ", theta3, 5);
    // debugger::debugOutput("theta4: ", theta4, 5);

    debugger::debugOutput("theta1 from pi function: ",
            angle_between_two_points_and_x_axis_pi(p_x, p_y, p1_x, p1_y), 5);
    debugger::debugOutput("theta2 from pi function: ",
            angle_between_two_points_and_x_axis_pi(p_x, p_y, p2_x, p2_y), 5);
    debugger::debugOutput("theta3 from pi function: ",
            angle_between_two_points_and_x_axis_pi(p_x, p_y, p3_x, p3_y), 5);
    debugger::debugOutput("theta4 from pi function: ",
            angle_between_two_points_and_x_axis_pi(p_x, p_y, p4_x, p4_y), 5);

    debugger::debugOutput("theta5 from pi function: ",
            angle_between_two_points_and_x_axis_pi(p_x, p_y, p5_x, p5_y), 5);
    debugger::debugOutput("theta6 from pi function: ",
            angle_between_two_points_and_x_axis_pi(p_x, p_y, p6_x, p6_y), 5);
    debugger::debugOutput("theta7 from pi function: ",
            angle_between_two_points_and_x_axis_pi(p_x, p_y, p7_x, p7_y), 5);
    debugger::debugOutput("theta8 from pi function: ",
            angle_between_two_points_and_x_axis_pi(p_x, p_y, p8_x, p8_y), 5);

    debugger::debugOutput("theta1 from 2pi function: ",
            angle_between_two_points_and_x_axis_2pi(p_x, p_y, p1_x, p1_y), 5);
    debugger::debugOutput("theta2 from 2pi function: ",
            angle_between_two_points_and_x_axis_2pi(p_x, p_y, p2_x, p2_y), 5);
    debugger::debugOutput("theta3 from 2pi function: ",
            angle_between_two_points_and_x_axis_2pi(p_x, p_y, p3_x, p3_y), 5);
    debugger::debugOutput("theta4 from 2pi function: ",
            angle_between_two_points_and_x_axis_2pi(p_x, p_y, p4_x, p4_y), 5);

    debugger::debugOutput("theta5 from 2pi function: ",
            angle_between_two_points_and_x_axis_2pi(p_x, p_y, p5_x, p5_y), 5);
    debugger::debugOutput("theta6 from 2pi function: ",
            angle_between_two_points_and_x_axis_2pi(p_x, p_y, p6_x, p6_y), 5);
    debugger::debugOutput("theta7 from 2pi function: ",
            angle_between_two_points_and_x_axis_2pi(p_x, p_y, p7_x, p7_y), 5);
    debugger::debugOutput("theta8 from 2pi function: ",
            angle_between_two_points_and_x_axis_2pi(p_x, p_y, p8_x, p8_y), 5);


    exit(0);


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
    path_points->header.frame_id = "odom";

    pcl::PointCloud<pcl::PointXYZI>::Ptr
        search_points (new pcl::PointCloud<pcl::PointXYZI>);
    search_points->header.frame_id = "odom";


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
