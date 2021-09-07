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
#ifndef PLOTTING_H
#define PLOTTING_H 

#include <string>


#include <ros/ros.h>
//#include <std_msgs/String.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h> // Marker
#include <limits>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "pose.h"

namespace bipedlab
{
namespace plotting 
{

void addMarkerWithPose(visualization_msgs::Marker &marker,
              const uint32_t shape, const std::string t_namespace,
              const double r, const double g, const double b, const double a,
              const pose_6dof_t &pose,
              const int id_count, 
              const double lasting_time = 0,
              const std::string dis_text = "",
              const double marker_size_x = 0.5, 
              const double marker_size_y = 0.1, 
              const double marker_size_z = 0.1,
              const std::string frame_id = "odom");

void addMarker(visualization_msgs::Marker &marker,
              const uint32_t shape, const std::string t_namespace,
              const double r, const double g, const double b,const double a,
              const double x, const double y, const double z, 
              const double wx, const double wy, const double wz, const double w,
              const int id_count, 
              const double lasting_time = 0,
              const std::string dis_text = "",
              const double marker_size_x = 0.5, 
              const double marker_size_y = 0.1, 
              const double marker_size_z = 0.1,
              const std::string frame_id = "odom");

void addMarkerWithTwoPoints(visualization_msgs::Marker &marker,
              const uint32_t shape, const std::string t_namespace,
              const double r, const double g, const double b,const double a,
              const double x1, const double y1, const double z1,
              const double x2, const double y2, const double z2,
              const int id_count, 
              const double lasting_time = 0,
              const std::string dis_text = "",
              const double marker_size_x = 0.5, 
              const double marker_size_y = 0.1, 
              const double marker_size_z = 0.1,
              const std::string frame_id = "odom");



bool moveRobot(ros::Publisher marker_pub,
        geometry_msgs::Point next_pose, geometry_msgs::Point goal);

// void drawFeasiblePath(
//         const PolynomialTraj_t trajs,
//         const double t_0, const double t_m,
//         const double delta, const int order,
//         ros::Publisher markerArray_pub){
//     visualization_msgs::MarkerArray markerarray_trajs;
//     visualization_msgs::Marker traj;
//     traj.type = visualization_msgs::Marker::ARROW;
//     traj.header.frame_id = "map";
//     traj.header.stamp = ros::Time::now();
//     traj.lifetime = ros::Duration();
//     traj.action = visualization_msgs::Marker::ADD;
//     traj.color.a = 1.0f;
//     traj.color.r = 1.0f;
//     traj.color.g = 0.0f;
//     traj.color.b = 1.0f;
//     traj.scale.x = 1;
//     traj.scale.y = 0.05;
//     traj.scale.z = 0.05;
//     traj.pose.orientation.x = 0.0;
//     traj.pose.orientation.y = 0.0;
//     traj.pose.orientation.w = 1;
//     traj.ns = "feasible path";
// 
//     int k = 0;
//     for (double t=t_0; t<t_m; t+=delta){
//         double x = 0;
//         double y = 0;
//         double theta = 0;
// 
//         for (int i=0; i<order+1; ++i){
//             x = x + trajs.px[i]*std::pow(t, i);
//             y = y + trajs.py[i]*std::pow(t, i);
//             theta = theta + trajs.ptheta[i]*std::pow(t, i);
//         }
//         // if (t==t_0 || t==t_m){
//         //     cout << "(x, y, theta) = " << "(" << x << ", " << y << ", " << theta << ")" << endl;
//         // }
// 
//         // traj.ns = "feasible path" + to_string(k);
//         traj.id = k;
//         traj.pose.orientation.z = theta;
//         traj.pose.position.x = x;
//         traj.pose.position.y = y;
//         traj.pose.position.z = 0.5;
//         markerarray_trajs.markers.push_back(traj);
//         k++;
//     }
//     markerArray_pub.publish(markerarray_trajs);
// }

void drawFinalPath(geometry_msgs::Point p1, 
                   geometry_msgs::Point p2, 
                   ros::Publisher marker_pub);

} // bipedlab
} // plotting

#endif /* ifndef PLOTTING_H */
