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
#pragma once
#ifndef UTILS_H
#define UTILS_H

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h> // Marker
#include <limits>

#include "ctto.h"

using namespace std;

namespace bipedlab{
namespace utils{
template <class T>
void AssignMarker(visualization_msgs::Marker &Marker, 
				  const uint32_t Shape, const string NameSpace,
				  const double r, const double g, const double b,
				  const T &Point, 
				  const int Count, const double Size, const string Text){
	Marker.header.frame_id = "map";
	Marker.header.stamp = ros::Time::now();;
	Marker.ns = NameSpace;
	Marker.id = Count;
	Marker.type = Shape; 
	Marker.action = visualization_msgs::Marker::ADD;
	Marker.pose.position.x = Point.x;
	Marker.pose.position.y = Point.y;
	Marker.pose.position.z = 0.5;
	Marker.pose.orientation.x = 0.0;
	Marker.pose.orientation.y = 0.0;
	Marker.pose.orientation.z = 0.0;
	Marker.pose.orientation.w = 1.0;
	Marker.text = Text;
	Marker.lifetime = ros::Duration(); // should disappear along with updateing rate

	// Set the scale of the marker -- 1x1x1 here means 1m on a side
	Marker.scale.x = Size;
	Marker.scale.y = Size;
	Marker.scale.z = Size;

	// Set the color -- be sure to set alpha to something non-zero!
	Marker.color.r = r;
	Marker.color.g = g;
	Marker.color.b = b;
	Marker.color.a = 1.0; 
}

bool moveRobot(ros::Publisher marker_pub, 
        geometry_msgs::Point next_pose, geometry_msgs::Point goal) {
    static visualization_msgs::Marker rob;
    rob.type = visualization_msgs::Marker::CUBE;


    rob.header.frame_id = "map";
    rob.header.stamp = ros::Time::now();
    rob.ns = "rob";
    rob.id = 0;
    rob.action = visualization_msgs::Marker::ADD;
    rob.lifetime = ros::Duration();

    rob.scale.x = 0.5;
    rob.scale.y = 1;
    rob.scale.z = 0.25;
    rob.pose.orientation.w = 1;
    rob.pose.orientation.x = rob.pose.orientation.y = rob.pose.orientation.z = 0;
    rob.color.r = 1.0f;
    rob.color.g = 0.5f;
    rob.color.b = 0.5f;
    rob.color.a = 1.0;

    //calculate m to change the orientation of the robot
    float m = (next_pose.y - rob.pose.position.y) / (next_pose.x - rob.pose.position.x);

    rob.pose.orientation.z = atan(m) + M_PI / 2;
    rob.pose.position = next_pose;

    marker_pub.publish(rob);

    if ((rob.pose.position.x == goal.x) && (rob.pose.position.y == goal.y)) {
        marker_pub.publish(rob);
        return true;
    }

    return false;
}

void drawFeasiblePath(
        const PolynomialTraj_t trajs, 
        const double t_0, const double t_m,
        const double delta, const int order,
        ros::Publisher markerArray_pub){
    visualization_msgs::MarkerArray markerarray_trajs;
    visualization_msgs::Marker traj;
    traj.type = visualization_msgs::Marker::ARROW;
    traj.header.frame_id = "map";
    traj.header.stamp = ros::Time::now();
    traj.lifetime = ros::Duration();
    traj.action = visualization_msgs::Marker::ADD;
    traj.color.a = 1.0f;
    traj.color.r = 1.0f;
    traj.color.g = 0.0f;
    traj.color.b = 1.0f;
    traj.scale.x = 1;
    traj.scale.y = 0.05;
    traj.scale.z = 0.05;
    traj.pose.orientation.x = 0.0;
    traj.pose.orientation.y = 0.0;
    traj.pose.orientation.w = 1;
    traj.ns = "feasible path";

    int k = 0;
    for (double t=t_0; t<t_m; t+=delta){
        double x = 0;
        double y = 0;
        double theta = 0;

        for (int i=0; i<order+1; ++i){
            x = x + trajs.px[i]*std::pow(t, i);
            y = y + trajs.py[i]*std::pow(t, i);
            theta = theta + trajs.ptheta[i]*std::pow(t, i);
        }
        // if (t==t_0 || t==t_m){
        //     cout << "(x, y, theta) = " << "(" << x << ", " << y << ", " << theta << ")" << endl;
        // }
        
        // traj.ns = "feasible path" + to_string(k);
        traj.id = k;
        traj.pose.orientation.z = theta;
        traj.pose.position.x = x;
        traj.pose.position.y = y;
        traj.pose.position.z = 0.5;
        markerarray_trajs.markers.push_back(traj);
        k++;
    }
    markerArray_pub.publish(markerarray_trajs);
}

void drawFinalPath(geometry_msgs::Point p1, geometry_msgs::Point p2, ros::Publisher marker_pub) {
    static visualization_msgs::Marker edge;
    edge.type = visualization_msgs::Marker::LINE_LIST;
    edge.header.frame_id = "map";
    edge.header.stamp = ros::Time::now();
    edge.ns = "finalPath";
    edge.id = 4;
    edge.action = visualization_msgs::Marker::ADD;
    edge.pose.orientation.w = 1;

    edge.scale.x = 0.04;

    edge.color.g = edge.color.r = 1;
    edge.color.a = 1.0;

    edge.points.push_back(p1);
    edge.points.push_back(p2);

    marker_pub.publish(edge);
}

void addEdge(geometry_msgs::Point p1, geometry_msgs::Point p2, 
        ros::Publisher marker_pub, bool isFinal) {
    static visualization_msgs::Marker edge, vertex;
    vertex.type = visualization_msgs::Marker::POINTS;
    edge.type = visualization_msgs::Marker::LINE_LIST;
    edge.header.frame_id = "map";
    edge.header.stamp = ros::Time::now();
    edge.ns = "edges";
    edge.id = 3;
    edge.action = visualization_msgs::Marker::ADD;
    edge.pose.orientation.w = 1;

    edge.scale.x = 0.02;
    if (!isFinal) {
        edge.color.r = 1.0;
    } else {
        edge.color.g = edge.color.r = 1;
    }
    edge.color.a = 1.0;

    edge.points.push_back(p1);
    edge.points.push_back(p2);

    marker_pub.publish(edge);
}

void nineCellUpdate(int x, int y, int value, Eigen::MatrixXd &costMap,
                    int gridToCostMapRatio){
    costMap(gridToCostMapRatio*x, gridToCostMapRatio*y) = value;
    costMap(gridToCostMapRatio*x-1, gridToCostMapRatio*y) = value;
    costMap(gridToCostMapRatio*x+1, gridToCostMapRatio*y) = value;
    costMap(gridToCostMapRatio*x, gridToCostMapRatio*y+1) = value;
    costMap(gridToCostMapRatio*x, gridToCostMapRatio*y-1) = value;
    costMap(gridToCostMapRatio*x-1, gridToCostMapRatio*y-1) = value;
    costMap(gridToCostMapRatio*x+1, gridToCostMapRatio*y+1) = value;
    costMap(gridToCostMapRatio*x-1, gridToCostMapRatio*y+1) = value;
    costMap(gridToCostMapRatio*x+1, gridToCostMapRatio*y-1) = value;
}

void addObstacles(std::vector<Obstacle_t> &stateObsVec,
		          int value,	
                  int boundary, int gridToCostMapRatio) {
    Obstacle_t obs(0);
    geometry_msgs::Point p;
    p.z = 0;
    for (int i=0; i<3; ++i){
        for (int j=10; j<20; ++j){
            p.x = i;
            p.y = j;
            obs.point = p;
            stateObsVec.push_back(obs);
        }
    }

    for (int i=3; i<5; ++i){
        for (int j=17; j<20; ++j){
            p.x = i;
            p.y = j;
            obs.point = p;
            stateObsVec.push_back(obs);
        }
    }

    for (int i=7; i<9; ++i){
        for (int j=0; j<12; ++j){
            p.x = i;
            p.y = j;
            obs.point = p;
            stateObsVec.push_back(obs);
        }
    }

    // left boundary
    // for (int j=0; j<boundary; ++j){
    //     p.x = 0;
    //     p.y = j;
    //     obs.point = p;
    //     stateObsVec.push_back(obs);
    // }

    // // right boundary
    // for (int j=0; j<boundary; ++j){
    //     p.x = boundary;
    //     p.y = j;
    //     obs.point = p;
    //     stateObsVec.push_back(obs);
    // }

    // // bottom boundary
    // for (int i=0; i<boundary; ++i){
    //     p.x = i;
    //     p.y = 0;
    //     obs.point = p;
    //     stateObsVec.push_back(obs);
    // }

    // // top boundary
    // for (int i=0; i<boundary; ++i){
    //     p.x = 0;
    //     p.y = boundary;
    //     obs.point = p;
    //     stateObsVec.push_back(obs);
    // }
}

void addObstacles(std::vector<visualization_msgs::Marker> &worldObsVec,
                  Eigen::MatrixXd &costMap,
		          int value,	
                  int boundary, int gridToCostMapRatio) {
    visualization_msgs::Marker obs;
    obs.type = visualization_msgs::Marker::CUBE;
    obs.header.frame_id = "map";
    obs.header.stamp = ros::Time::now();
    obs.lifetime = ros::Duration();
    obs.action = visualization_msgs::Marker::ADD;
    obs.color.a = 1.0f;
    obs.color.r = 1.0f;
    obs.color.g = 1.0f;
    obs.color.b = 1.0f;
    obs.scale.x = 1;
    obs.scale.y = 1;
    obs.scale.z = 0.5;
    obs.pose.position.z = 0.25;
    obs.pose.orientation.x = 0.0;
    obs.pose.orientation.y = 0.0;
    obs.pose.orientation.z = 0.0;
    obs.pose.orientation.w = 1;

    int k = 0;
    for (int i=0; i<3; ++i){
        for (int j=10; j<20; ++j){
            obs.ns = "Obstacles6";
            obs.id = k;
            obs.pose.position.x = i + 0.5;
            obs.pose.position.y = j + 0.5;
            costMap(gridToCostMapRatio*i,gridToCostMapRatio*j) = value;
            if (i>0) nineCellUpdate(i, j, value, costMap, gridToCostMapRatio);
            worldObsVec.push_back(obs);
            // stateObsVec.push_back(obs);
            k++;
        }
    }

    k = 0;
    for (int i=3; i<5; ++i){
        for (int j=17; j<20; ++j){
            obs.ns = "Obstacles5";
            obs.id = k;
            obs.pose.position.x = i + 0.5;
            obs.pose.position.y = j + 0.5;
            // costMap(i,j) = value;
            costMap(gridToCostMapRatio*i,gridToCostMapRatio*j) = value;
            nineCellUpdate(i, j, value, costMap, gridToCostMapRatio);
            worldObsVec.push_back(obs);
            // stateObsVec.push_back(obs);
            k++;
        }
    }

    k = 0;
    for (int i=7; i<9; ++i){
        for (int j=0; j<12; ++j){
            obs.ns = "Obstacles1";
            obs.id = k;
            obs.pose.position.x = i + 0.5;
            obs.pose.position.y = j + 0.5;
            // costMap(i,j) = value;
            costMap(gridToCostMapRatio*i,gridToCostMapRatio*j) = value;
            if (j>0) nineCellUpdate(i, j, value, costMap, gridToCostMapRatio);
            worldObsVec.push_back(obs);
            // stateObsVec.push_back(obs);
            k++;
        }
    }

    // obs.id = k;
    // obs.pose.position.x = 7 + 0.5;
    // obs.pose.position.y = 0 - 0.5;
    // worldObsVec.push_back(obs);
    // k++;

    k = 0;
    for (int i=13; i<15; ++i){
        for (int j=10; j<16; ++j){
            obs.ns = "Obstacles2";
            obs.id = k;
            obs.pose.position.x = i + 0.5;
            obs.pose.position.y = j + 0.5;
            // costMap(i,j) = value;
            worldObsVec.push_back(obs);
            k++;
        }
    }

    k = 0;
    for (int i=13; i<17; ++i){
        for (int j=8; j<10; ++j){
            obs.ns = "Obstacles4";
            obs.id = k;
            obs.pose.position.x = i + 0.5;
            obs.pose.position.y = j + 0.5;
            // costMap(i,j) = value;
            worldObsVec.push_back(obs);
            k++;
        }
    }

    k = 0;
    for (int i=13; i<20; ++i){
        for (int j=0; j<4; ++j){
            obs.ns = "Obstacles3";
            obs.id = k;
            obs.pose.position.x = i + 0.5;
            obs.pose.position.y = j + 0.5;
            // costMap(i,j) = value;
            worldObsVec.push_back(obs);
            k++;
        }
    }

    // left boundary
    k = 0;
    for (int j=0; j<boundary; ++j){
        obs.ns = "left boundry";
        obs.id = k;
        obs.pose.position.x = 0 - 0.5;
        obs.pose.position.y = j + 0.5;
        // costMap(0,j) = value;
        costMap(0,gridToCostMapRatio*j) = value;
        costMap(0,gridToCostMapRatio*j+1) = value;

        if (j>0) costMap(0,gridToCostMapRatio*j-1) = value;
        worldObsVec.push_back(obs);
        k++;
    }

    // right boundary
    k = 0;
    for (int j=0; j<boundary; ++j){
        obs.ns = "right boundry";
        obs.id = k;
        obs.pose.position.x = boundary + 0.5;
        obs.pose.position.y = j + 0.5;
        // costMap(boundary-1, j) = value;
        costMap(gridToCostMapRatio*boundary-1, gridToCostMapRatio*j) = value;
        costMap(gridToCostMapRatio*boundary-1, gridToCostMapRatio*j+1) = value;
        if (j>0) costMap(gridToCostMapRatio*boundary-1, gridToCostMapRatio*j-1) = value;
        worldObsVec.push_back(obs);
        k++;
    }

    // bottom boundary
    k = 0;
    for (int i=0; i<boundary; ++i){
        obs.ns = "bottom boundry";
        obs.id = k;
        obs.pose.position.x = i + 0.5;
        obs.pose.position.y = 0 - 0.5;
        // costMap(i, 0) = value;
        costMap(gridToCostMapRatio*i, 0) = value;
        costMap(gridToCostMapRatio*i+1, 0) = value;
        // costMap(gridToCostMapRatio*i+2, 0) = value;
        if (i>0) costMap(gridToCostMapRatio*i-1, 0) = value;
        worldObsVec.push_back(obs);
        k++;
    }

    // top boundary
    k = 0;
    for (int i=0; i<boundary; ++i){
        obs.ns = "bottom boundry";
        obs.id = k;
        obs.pose.position.x = i + 0.5;
        obs.pose.position.y = boundary + 0.5;
        costMap(gridToCostMapRatio*i, gridToCostMapRatio*boundary-1) = value;
        costMap(gridToCostMapRatio*i+1, gridToCostMapRatio*boundary-1) = value;

        costMap(gridToCostMapRatio*i, gridToCostMapRatio*boundary-2) = value;
        costMap(gridToCostMapRatio*i+1, gridToCostMapRatio*boundary-2) = value;
        if (i>0) {
            costMap(gridToCostMapRatio*i-1, gridToCostMapRatio*boundary-1) = value;
            costMap(gridToCostMapRatio*i-1, gridToCostMapRatio*boundary-2) = value;
        }
        worldObsVec.push_back(obs);
        k++;
    }
}

void populateObstacles(std::vector<visualization_msgs::Marker> &stateObsVec,
					   ros::Publisher marker_pub, int boundary, int gridToCostMapRatio) {
    visualization_msgs::Marker obs;
    obs.type = visualization_msgs::Marker::CUBE;
    obs.header.frame_id = "map";
    obs.header.stamp = ros::Time::now();
    obs.action = visualization_msgs::Marker::ADD;
    obs.color.a = 1.0f;
    obs.color.r = 1.0f;
    obs.color.g = 1.0f;
    obs.color.b = 1.0f;
    obs.scale.x = 1;
    obs.scale.y = 1;
    obs.scale.z = 0.5;
    obs.pose.position.z = 0.25;
    obs.pose.orientation.x = 0.0;
    obs.pose.orientation.y = 0.0;
    obs.pose.orientation.z = 0.0;
    obs.pose.orientation.w = 1;
    obs.ns = "world obs";

    int k = 0;
    for (int i=0; i<3; ++i){
        for (int j=10; j<20; ++j){
            obs.id = k;
            obs.pose.position.x = i + 0.5;
            obs.pose.position.y = j + 0.5;
            marker_pub.publish(obs);
            k++;
        }
    }

    for (int i=3; i<5; ++i){
        for (int j=17; j<20; ++j){
            obs.id = k;
            obs.pose.position.x = i + 0.5;
            obs.pose.position.y = j + 0.5;
            marker_pub.publish(obs);
            k++;
        }
    }

    for (int i=7; i<9; ++i){
        for (int j=0; j<13; ++j){
            obs.id = k;
            obs.pose.position.x = i + 0.5;
            obs.pose.position.y = j + 0.5;
            if (j<12) marker_pub.publish(obs);
            k++;
        }
    }

    for (int i=13; i<20; ++i){
        for (int j=0; j<4; ++j){
            obs.id = k;
            obs.pose.position.x = i + 0.5;
            obs.pose.position.y = j + 0.5;
            if (j<12) marker_pub.publish(obs);
            k++;
        }
    }

    for (int i=13; i<15; ++i){
        for (int j=10; j<16; ++j){
            obs.id = k;
            obs.pose.position.x = i + 0.5;
            obs.pose.position.y = j + 0.5;
            marker_pub.publish(obs);
            k++;
        }
    }

    for (int i=13; i<17; ++i){
        for (int j=8; j<10; ++j){
            obs.id = k;
            obs.pose.position.x = i + 0.5;
            obs.pose.position.y = j + 0.5;
            marker_pub.publish(obs);
            k++;
        }
    }

    obs.color.a = 0.3f;
    obs.color.r = 1.0f;
    obs.color.g = 1.0f;
    obs.color.b = 1.0f;
    obs.ns = "steep slope";
    for (int i=13; i<15; ++i){
        for (int j=16; j<20; ++j){
            obs.id = k;
            obs.pose.position.x = i + 0.5;
            obs.pose.position.y = j + 0.5;
            marker_pub.publish(obs);
            k++;
        }
    }


    obs.type = visualization_msgs::Marker::LINE_STRIP;
    obs.header.frame_id = "map";
    obs.header.stamp = ros::Time::now();
    obs.action = visualization_msgs::Marker::ADD;
    obs.color.a = 1.0f;
    obs.color.r = 1.0f;
    obs.color.g = 0;
    obs.color.b = 0;
    obs.scale.x = 0.1;
    obs.scale.y = 0.1;
    obs.scale.z = 0.;
    obs.pose.position.z = 0.25;
    obs.pose.orientation.x = 0.0;
    obs.pose.orientation.y = 0.0;
    obs.pose.orientation.z = 0.0;
    obs.pose.orientation.w = 1;

    // geometry_msgs::Point p;
    // p.x = p.y = p.z = 0;
    // obs.points.push_back(p);

    // p.x = 0;
    // p.y = boundary;
    // p.z = 0;
    // obs.points.push_back(p);

    // obs.points.push_back(p);
    // p.x = boundary;
    // p.y = boundary;
    // p.z = 0;
    // obs.points.push_back(p);

    // obs.points.push_back(p);
    // p.x = boundary;
    // p.y = 0;
    // p.z = 0;
    // obs.points.push_back(p);

    // obs.points.push_back(p);
    // p.x = 0;
    // p.y = 0;
    // p.z = 0;
    // obs.points.push_back(p);
    // marker_pub.publish(obs);
}

void populateObstacles(std::vector<visualization_msgs::Marker> &worldObsVec,
                       ros::Publisher marker_pub) {
    visualization_msgs::Marker obs1, obs2, obs3, obs4, obs5, obs6;
    obs1.type = obs2.type = obs3.type = obs4.type = obs5.type = obs6.type = visualization_msgs::Marker::CUBE;
    obs1.header.frame_id = obs2.header.frame_id = obs3.header.frame_id = obs4.header.frame_id = obs5.header.frame_id = obs6.header.frame_id = "map";
    obs1.header.stamp = obs2.header.stamp = obs3.header.stamp = obs4.header.stamp = obs5.header.stamp = obs6.header.stamp = ros::Time::now();
    obs1.ns = "obstacles1";
    obs2.ns = "obstacles2";
    obs3.ns = "obstacles3";
    obs4.ns = "obstacles4";
    obs5.ns = "obstacles5";
    obs6.ns = "obstacles6";
    obs1.lifetime = obs2.lifetime = obs3.lifetime = obs4.lifetime = obs5.lifetime = obs6.lifetime = ros::Duration();
    obs1.action = obs2.action = obs3.action = obs4.action = obs5.action = obs6.action = visualization_msgs::Marker::ADD;

    obs1.id = 0;
    obs2.id = 1;
    obs3.id = 2;
    obs4.id = 3;
    obs5.id = 4;
    obs6.id = 5;

    obs1.scale.x = obs2.scale.x = 2;
    obs1.scale.y = 12;
    obs2.scale.y = 12;
    obs3.scale.y = 4;
    obs3.scale.x = 7;
    obs4.scale.x = obs4.scale.y = 2;
    obs5.scale.x = 5;
    obs5.scale.y = 3;
    obs6.scale.x = 3;
    obs6.scale.y = 7;

    obs1.scale.z = obs2.scale.z = obs3.scale.z = obs4.scale.z = obs5.scale.z = obs6.scale.z = 0.25;


    obs1.pose.position.x = 8;
    obs1.pose.position.y = 6;
    obs1.pose.position.z = 0.25;
    obs1.pose.orientation.x = 0.0;
    obs1.pose.orientation.y = 0.0;
    obs1.pose.orientation.z = 0.0;
    obs1.pose.orientation.w = 1;
    obs1.color.a = 1;
    obs1.color.r = obs1.color.g = obs1.color.b = 6.6f;

    obs2.pose.position.x = 14;
    obs2.pose.position.y = 14;
    obs2.pose.position.z = 0.25;
    obs2.pose.orientation.x = 0.0;
    obs2.pose.orientation.y = 0.0;
    obs2.pose.orientation.z = 0.0;
    obs2.pose.orientation.w = 1;
    obs2.color.a = 1;
    obs2.color.r = obs2.color.g = obs2.color.b = 6.6f;

    obs3.pose.position.x = 16.5;
    obs3.pose.position.y = 2;
    obs3.pose.position.z = 0.25;
    obs3.pose.orientation.x = 0.0;
    obs3.pose.orientation.y = 0.0;
    obs3.pose.orientation.z = 0.0;
    obs3.pose.orientation.w = 1;
    obs3.color.a = 1;
    obs3.color.r = obs3.color.g = obs3.color.b = 6.6f;

    obs4.pose.position.x = 16;
    obs4.pose.position.y = 9;
    obs4.pose.position.z = 0.25;
    obs4.pose.orientation.x = 0.0;
    obs4.pose.orientation.y = 0.0;
    obs4.pose.orientation.z = 0.0;
    obs4.pose.orientation.w = 1;
    obs4.color.a = 1;
    obs4.color.r = obs4.color.g = obs4.color.b = 6.6f;

    obs5.pose.position.x = 2.5;
    obs5.pose.position.y = 18.5;
    obs5.pose.position.z = 0.25;
    obs5.pose.orientation.x = 0.0;
    obs5.pose.orientation.y = 0.0;
    obs5.pose.orientation.z = 0.0;
    obs5.pose.orientation.w = 1;
    obs5.color.a = 1;
    obs5.color.r = obs5.color.g = obs5.color.b = 6.6f;

    obs6.pose.position.x = 1.5;
    obs6.pose.position.y = 13.5;
    obs6.pose.position.z = 0.25;
    obs6.pose.orientation.x = 0.0;
    obs6.pose.orientation.y = 0.0;
    obs6.pose.orientation.z = 0.0;
    obs6.pose.orientation.w = 1;
    obs6.color.a = 1;
    obs6.color.r = obs6.color.g = obs6.color.b = 6.6f;

    marker_pub.publish(obs1);
    marker_pub.publish(obs2);
    marker_pub.publish(obs3);
    marker_pub.publish(obs4);
    marker_pub.publish(obs5);
    marker_pub.publish(obs6);
}

void populateCurrObstacles(
	// std::vector<geometry_msgs::Point> &currObsVec,
    std::vector<Obstacle_t> &currObsVec,
	ros::Publisher markerArray_pub, string ns) {

    visualization_msgs::MarkerArray obsMarkers;

    visualization_msgs::Marker obs;
    obs.type = visualization_msgs::Marker::CUBE;
    obs.header.frame_id = "map";
    obs.header.stamp = ros::Time::now();
    obs.ns = ns;
    obs.lifetime = ros::Duration();
    obs.action = visualization_msgs::Marker::ADD;
    obs.color.a = 1.0f;
    obs.color.g = 1.0f;
    obs.scale.x = obs.scale.y = 0.5;
    obs.scale.z = 0.5;
    obs.pose.orientation.x = 0.0;
    obs.pose.orientation.y = 0.0;
    obs.pose.orientation.z = 0.0;
    obs.pose.orientation.w = 1;

    for (int i=0; i<currObsVec.size(); ++i){
        obs.id = i;
        obs.pose.position.x = currObsVec[i].point.x;
        obs.pose.position.y = currObsVec[i].point.y;
        obs.pose.position.z = 0.5;
        obsMarkers.markers.push_back(obs);
    }
    if (currObsVec.size()!=0){
        // ROS_INFO("current obs: %i", (int)currObsVec.size());
        markerArray_pub.publish(obsMarkers);
    }
}

void populateCurrObstaclesRGB(
	// std::vector<geometry_msgs::Point> &currObsVec,
    std::vector<Obstacle_t> &currObsVec,
    double a, double r, double g, double b,
	ros::Publisher markerArray_pub, string ns) {

    visualization_msgs::MarkerArray obsMarkers;

    visualization_msgs::Marker obs;
    obs.type = visualization_msgs::Marker::SPHERE;
    obs.header.frame_id = "map";
    obs.header.stamp = ros::Time::now();
    obs.ns = ns;
    obs.lifetime = ros::Duration();
    obs.action = visualization_msgs::Marker::ADD;
    obs.color.a = a;
    obs.color.r = r;
    obs.color.g = g;
    obs.color.b = b;
    obs.scale.x = obs.scale.y = 0.5;
    obs.scale.z = 0.5;
    obs.pose.orientation.x = 0.0;
    obs.pose.orientation.y = 0.0;
    obs.pose.orientation.z = 0.0;
    obs.pose.orientation.w = 1;

    for (int i=0; i<currObsVec.size(); ++i){
        obs.id = i;
        obs.pose.position.x = currObsVec[i].point.x;
        obs.pose.position.y = currObsVec[i].point.y;
        obs.pose.position.z = 0.5;
        obsMarkers.markers.push_back(obs);
    }
    if (currObsVec.size()!=0){
        // ROS_INFO("current obs: %i", (int)currObsVec.size());
        markerArray_pub.publish(obsMarkers);
    }
}

void populatePotentialMapBoundary(
    std::string nameSpace,
    uint32_t shape,
    double size,
    const double a, const double r, const double g, const double b,
    BoundaryPoint_t boundaryPoints,
	ros::Publisher markerArray_pub) {

    visualization_msgs::MarkerArray markers;
    visualization_msgs::Marker marker;
    marker.type = shape;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();
    marker.ns = nameSpace;
    marker.lifetime = ros::Duration();
    marker.action = visualization_msgs::Marker::ADD;
    marker.color.a = a;
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker.scale.x = size;
    marker.scale.y = size;
    marker.scale.z = size;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1;

    marker.id = 0;
    marker.pose.position.x = boundaryPoints.p1.x;
    marker.pose.position.y = boundaryPoints.p1.y;
    marker.pose.position.z = 0.5;
    markers.markers.push_back(marker);

    marker.id = 1;
    marker.pose.position.x = boundaryPoints.p2.x;
    marker.pose.position.y = boundaryPoints.p2.y;
    marker.pose.position.z = 0.5;
    markers.markers.push_back(marker);

    marker.id = 2;
    marker.pose.position.x = boundaryPoints.p3.x;
    marker.pose.position.y = boundaryPoints.p3.y;
    marker.pose.position.z = 0.5;
    markers.markers.push_back(marker);

    marker.id = 3;
    marker.pose.position.x = boundaryPoints.p4.x;
    marker.pose.position.y = boundaryPoints.p4.y;
    marker.pose.position.z = 0.5;
    markers.markers.push_back(marker);

    markers.markers.push_back(marker);

    markerArray_pub.publish(markers);
}

template <class T>
void populateVectorPoints(
    std::string nameSpace,
    uint32_t shape,
    double size,
    const double a, const double r, const double g, const double b,
    std::vector<T> &vectors,
	ros::Publisher markerArray_pub) {

    if (vectors.size()==0) return;
    visualization_msgs::MarkerArray markers;
    visualization_msgs::Marker marker;
    marker.type = shape;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();
    marker.ns = nameSpace;
    marker.lifetime = ros::Duration();
    marker.action = visualization_msgs::Marker::ADD;
    marker.color.a = a;
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker.scale.x = size;
    marker.scale.y = size;
    marker.scale.z = size;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1;

    for (int i=0; i<vectors.size(); ++i){
        marker.id = i;
        marker.pose.position.x = vectors[i].point.x;
        marker.pose.position.y = vectors[i].point.y;
        marker.pose.position.z = 0.5;
        markers.markers.push_back(marker);
    }
    markerArray_pub.publish(markers);
}

template <class T>
void populateVector(
    std::string nameSpace,
    uint32_t shape,
    double size,
    const double a, const double r, const double g, const double b,
    const std::vector<T> &vectors,
	ros::Publisher markerArray_pub) {

    if (vectors.size()==0) return;
    visualization_msgs::MarkerArray markers;
    visualization_msgs::Marker marker;
    marker.type = shape;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();
    marker.ns = nameSpace;
    marker.lifetime = ros::Duration();
    marker.action = visualization_msgs::Marker::ADD;
    marker.color.a = a;
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker.scale.x = size;
    marker.scale.y = size;
    marker.scale.z = size;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1;

    for (int i=0; i<vectors.size(); ++i){
        marker.id = i;
        marker.pose.position.x = vectors[i].x;
        marker.pose.position.y = vectors[i].y;
        marker.pose.position.z = 0.5;
        markers.markers.push_back(marker);
    }
    markerArray_pub.publish(markers);
}

template <class T>
void populateMatrix(
    std::string nameSpace,
    uint32_t shape,
    double size,
    const double a, const double r, const double g, const double b,
    const T &matrix, int ratio,
	ros::Publisher markerArray_pub) {

    if (matrix.size()==0) return;
    visualization_msgs::MarkerArray markers;
    visualization_msgs::Marker marker;
    marker.type = shape;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();
    marker.ns = nameSpace;
    marker.lifetime = ros::Duration();
    marker.action = visualization_msgs::Marker::ADD;
    marker.color.a = a;
    marker.scale.x = size;
    marker.scale.y = size;
    marker.scale.z = size;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1;

    int k = 0;
    for (int i=0; i<matrix.rows(); ++i){
        for (int j=0; j<matrix.cols(); ++j){
            marker.id = k;
            marker.pose.position.x = (i + 0.5)/ ratio;
            marker.pose.position.y = (j + 0.5)/ ratio;
            marker.pose.position.z = 0.5;
                // if (i==27){
                //     if (j==54){
                //     cout << "i: " << i << endl;
                //     cout << "j: " << j << endl;
                //     cout << "d: " << matrix(i,j) << endl;
                //     }
                // }
            marker.color.r = matrix(i, j);
            marker.color.g = matrix(i, j);
            marker.color.b = matrix(i, j);
            markers.markers.push_back(marker);
            k ++;
        }
    }
    markerArray_pub.publish(markers);
}

double getSlope(geometry_msgs::Point p, float x, float y){
    if (std::abs(p.x-x)>1e-5)
        return (p.y-y)/(p.x-x);
    else {
        if ((y-p.y)>0)
            return M_PI/2;
        else
            return -M_PI/2;
    }
}

double getSlope(float x1, float y1, float x2, float y2){
    if (std::abs(x1-x2)>1e-5)
        return (y1-y2)/(x1-x2);
    else{
        if((y2-y1)>0)
            return M_PI/2;
        else 
            return -M_PI/2;
    }
}


void populateRobotandSensor(int sparsity,
                            const geometry_msgs::Point robot, ros::Publisher marker_pub, 
                            const std::vector<Obstacle_t> &currObsVec){
				   // const std::vector<geometry_msgs::Point> &currObsVec){
    visualization_msgs::Marker line;
    visualization_msgs::Marker robotMarker;
    line.type = visualization_msgs::Marker::LINE_LIST;
    line.header.frame_id = "map";
    line.header.stamp = ros::Time::now();
    line.ns = "sensor";
    line.id = 0;
    line.color.a = 1.0f;
    line.color.r = 1.0f;
    line.color.g = 1.0f;
    line.scale.x = 0.02;

	for (int i=0; i<currObsVec.size(); i+=sparsity){
		line.points.push_back(robot);
		line.points.push_back(currObsVec[i].point);
	}

	AssignMarker(robotMarker, visualization_msgs::Marker::SPHERE, 
			     "robot", 
			     0, 1, 1,
			     robot, 0, 0.5, "");

	marker_pub.publish(line);
	marker_pub.publish(robotMarker);
}


void populateRviz(ros::Publisher marker_pub, 
        int boundary_x, int boundary_y,
        float init_x, float init_y, 
        float goal_x, float goal_y) {
    visualization_msgs::Marker v_start, v_end, line;
    v_start.type = v_end.type = visualization_msgs::Marker::POINTS;
    line.type = visualization_msgs::Marker::LINE_LIST;
    v_start.header.frame_id = v_end.header.frame_id = line.header.frame_id = "map";
    v_start.header.stamp = v_end.header.stamp = line.header.stamp = ros::Time::now();
    v_start.ns = v_end.ns = "start/end vertices";
    line.ns = "corner";
    line.id = 0;
    v_start.id = 0;
    v_end.id = 1;
    v_start.action = v_end.action = line.action = visualization_msgs::Marker::ADD;

    line.color.a = 1.0f;
    line.color.r = 1.0f;
    line.scale.x = 0.2;

    v_start.color.a = 1.0f;
    v_start.color.b = 1.0f;
    v_start.scale.x = v_start.scale.y = 0.5;
    v_end.scale.x = v_end.scale.y = 0.5;


    v_end.color.a = 1.0f;
    v_end.color.g = 1.0f;

    geometry_msgs::Point ps, pe, corner;
    ps.x = init_x;
    ps.y = init_y;
    pe.x = goal_x;
    pe.y = goal_y;
    v_start.points.push_back(ps);
    v_end.points.push_back(pe);

    corner.x = 0;
    corner.y = 0;
    line.points.push_back(corner);

    corner.x = 0;
    corner.y = boundary_y;
    line.points.push_back(corner);
    line.points.push_back(corner);

    corner.x = boundary_x;
    corner.y = boundary_y;
    line.points.push_back(corner);
    line.points.push_back(corner);

    corner.x = boundary_x;
    corner.y = 0;
    line.points.push_back(corner);
    line.points.push_back(corner);

    corner.x = 0;
    corner.y = 0;
    line.points.push_back(corner);

    // corner.x = init_x;
    // corner.y = init_y;
    // line.points.push_back(corner);

    marker_pub.publish(v_start);
    marker_pub.publish(v_end);
    marker_pub.publish(line);
};

void pressEntertoContinue(){
    int c;
    printf( "Press ENTER to continue... \n" );
    fflush( stdout );
    do c = getchar(); while ((c != '\n') && (c != EOF));
}


} // namespace utils
} // namespace bipedlab

#endif
