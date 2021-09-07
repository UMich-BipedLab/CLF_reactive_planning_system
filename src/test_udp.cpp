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
#include "clf_rrt.h"
#include "communication.h"

#include "utils/plotting.h"
#include "utils/debugger.h"
#include "utils/utils.h"
#include "utils/ros_utils.h"

// #define SEND_PORT "26000"
// #define SEND_IP "10.10.10.120"
// #define RECEIVE_PORT "28000"



// 0-4 general debugging purposes
// 5-8: algorithm status/process
int DEBUG_LEVEL = 5; 

using namespace bipedlab;

int main(int argc, char *argv[]) {


    // ros
    ros::init(argc, argv, "test_udp");
    ros::NodeHandle nh("test_udp");


    std::string title_name("[test_udp] ");
    std::string port_to_send("26000");
    std::string port_to_receive("28000");
    std::string ip_to_send("10.10.10.120");
    bool received_all = false;
    double pub_rate = 2;;


    ros_utils::checkROSParam(nh, "communication/port_to_send",
            port_to_send, getNameOf(port_to_send), title_name, received_all);
    ros_utils::checkROSParam(nh, "communication/port_to_receive",
            port_to_receive, getNameOf(port_to_receive), title_name, received_all);
    ros_utils::checkROSParam(nh, "communication/ip_to_send", ip_to_send,
            getNameOf(ip_to_send), title_name, received_all);

    ros_utils::checkROSParam(nh, "publishing_rate", pub_rate,
            getNameOf(pub_rate), title_name, received_all);
    if (received_all)
    {
        debugger::debugColorOutput("[Driver] Not enough parameters: ",
                "Using default values", 10, BR, BOLD);

        utils::pressEnterToContinue();
    }



    ros::Rate loop_rate(pub_rate);

    communication_t udp_info(ip_to_send, port_to_send, port_to_receive);

    // UDP setup
    // Communication* communication = new Communication(std::string(SEND_IP), 
    //                                                  std::string(SEND_PORT),
    //                                                  std::string(RECEIVE_PORT));
    Communication* communication = new Communication(udp_info);

    // need to change the function to public
    // boost::thread listen_to_root(&Communication::createListener_, &*communication);


    

    ros::Time time = ros::Time::now();
    tf::Transform robot_pose;
    robot_pose.setIdentity();
    robot_pose.setOrigin(tf::Vector3(-5, 0, 0));
    tf::StampedTransform tf(robot_pose, time, "map", "robot");

    debugger::debugTextOutput("[test_udp] Sending...", 4);
    std::pair<int, controller_info_to_planner_t> received_msg;
    double vx = 2;
    double vy = 1;
    double heading = 0.5;
    while (nh.ok())
    {
        communication->publishToRobotFromTFMsg(tf, vx, vy, heading);
        // received_msg = communication->getReceivedMessage();
        // debugger::debugOutput("[main] received: ", received_msg.first, 5);
        loop_rate.sleep();

    }





    exit(0);






    
    return 0;
}

