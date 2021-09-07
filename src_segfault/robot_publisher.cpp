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
#include <ros/ros.h>
#include <ros/console.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h> // Marker



#include <cmath>

#include "inekf_msgs/State.h"
#include "pose.h"
#include "utils/debugger.h"
#include "angle_functions.h"
#include "utils/plotting.h"



using namespace bipedlab;
int DEBUG_LEVEL = 10;

int main(int argc, char** argv)
{
    // Initialize node and publisher.
    ros::init(argc, argv, "fake_robot_state");
    ros::NodeHandle nh("~");
    ros::Publisher pose_pub = nh.advertise<inekf_msgs::State>("inekf", 1, true);
    ros::Publisher marker_pub = 
        nh.advertise<visualization_msgs::MarkerArray>("fake_robot", 10);


    visualization_msgs::Marker marker;
    visualization_msgs::MarkerArray marker_array;


    float rate = 100;


    // robot state
    float x = 0;
    float y = 0;
    float z = 0;
    float roll = 0;
    float pitch = 0;
    float yaw = 0;

    float vx = 0;
    float vy = 0;
    float vz = 0;



    nh.getParam("rate", rate);
    nh.getParam("fake_robot/x", x);
    nh.getParam("fake_robot/y", y);
    nh.getParam("fake_robot/z", z);
    nh.getParam("fake_robot/roll", roll);
    nh.getParam("fake_robot/pitch", pitch);
    nh.getParam("fake_robot/yaw", yaw);
    nh.getParam("fake_robot/vx", vx);
    nh.getParam("fake_robot/vy", vy);
    nh.getParam("fake_robot/vz", vz);


    debugger::debugTitleTextOutput("[fake robot]", "Robot Publisher", 10);
    debugger::debugColorOutput("[fake robot] Publishing robot at [Hz] ", rate, 10, BB);
    debugger::debugColorOutput("[fake robot] x: ", x, 10, BB);
    debugger::debugColorOutput("[fake robot] y: ", y, 10, BB);
    debugger::debugColorOutput("[fake robot] z: ", z, 10, BB);
    debugger::debugColorOutput("[fake robot] roll: ", roll, 10, BB);
    debugger::debugColorOutput("[fake robot] pitch: ", pitch, 10, BB);
    debugger::debugColorOutput("[fake robot] yaw: ", yaw, 10, BB);


    inekf_msgs::State fake_robot;

    // position
    fake_robot.pose.position.x = x;
    fake_robot.pose.position.y = y;
    fake_robot.pose.position.z = z;

    // orientation
    tf2::Quaternion q_tf;
    q_tf.setRPY(deg_to_rad(roll), deg_to_rad(pitch), deg_to_rad(yaw));
    fake_robot.pose.orientation = tf2::toMsg(q_tf);


    // velocity
    fake_robot.velocity.x = vx;
    fake_robot.velocity.y = vy;
    fake_robot.velocity.z = vz;

    // header
    fake_robot.header.frame_id = "map";




    pose_t pose((double) x, (double) y, (double) yaw);
    plotting::addMarkerWithPose(marker, visualization_msgs::Marker::ARROW,
            "start",
            0, 1, 0, 1, // color
            pose, // pose
            0, 0 // count, time
            );
    marker_array.markers.push_back(marker);


    // Work with grid map in a loop.
    ros::Rate loop_rate(rate);
    while (nh.ok()) {

        fake_robot.header.stamp = ros::Time::now();
        // Add data to grid map.
        pose_pub.publish(fake_robot);
        marker_pub.publish(marker_array);


        ROS_INFO_THROTTLE(1.0, "Fake robot (timestamp %f) published.", fake_robot.header.stamp.toSec());

        // Wait for next cycle.
        loop_rate.sleep();
    }

    return 0;
}
