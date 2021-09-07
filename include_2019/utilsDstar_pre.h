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
#ifndef UTILS_DSTAR_H
#define UTILS_DSTAR_H

#include <ros/ros.h>
#include <sstream> // for to_string_precision function
#include <std_msgs/String.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h> // Marker

#include "ctto.h"
#include "Dstar.h"
#include "occupancy_grid_utils.h"


using namespace std;

namespace bipedlab{
namespace dstar_utils{


template <typename T>
std::string toStringWithPrecision(const T a_value, const int n = 6)
{
    std::ostringstream out;
    out.precision(n);
    out << std::fixed << a_value;
    return out.str();
}


void nineCellUpdate(int x, int y, int value, Dstar *dstar,
                    int gridToCostMapRatio){
    dstar->updateCell(gridToCostMapRatio*x, gridToCostMapRatio*y, value);
    dstar->updateCell(gridToCostMapRatio*x-1, gridToCostMapRatio*y, value);
    dstar->updateCell(gridToCostMapRatio*x+1, gridToCostMapRatio*y, value);
    dstar->updateCell(gridToCostMapRatio*x, gridToCostMapRatio*y+1, value);
    dstar->updateCell(gridToCostMapRatio*x, gridToCostMapRatio*y-1, value);
    dstar->updateCell(gridToCostMapRatio*x-1, gridToCostMapRatio*y-1, value);
    dstar->updateCell(gridToCostMapRatio*x+1, gridToCostMapRatio*y+1, value);
    dstar->updateCell(gridToCostMapRatio*x-1, gridToCostMapRatio*y+1, value);
    dstar->updateCell(gridToCostMapRatio*x+1, gridToCostMapRatio*y-1, value);
}

void drawDstar(std::vector<state> path, ros::Publisher markerArray_pub, int
               gridToCostMapRatio, string ns, float r, float g, float b){
    visualization_msgs::MarkerArray obsMarkers;
    visualization_msgs::Marker obs;
    obs.type = visualization_msgs::Marker::SPHERE;
    obs.header.frame_id = "odom";
    obs.header.stamp = ros::Time::now();
    obs.ns = ns;
    obs.action = visualization_msgs::Marker::ADD;
    obs.color.a = 1.0f;
    obs.color.r = r;
    obs.color.g = g;
    obs.color.b = b;
    obs.scale.x = obs.scale.y = obs.scale.z = 0.2;
    obs.pose.orientation.x = 0.0;
    obs.pose.orientation.y = 0.0;
    obs.pose.orientation.z = 0.0;
    obs.pose.orientation.w = 1;

    for (int i=0; i<path.size(); ++i){
        obs.id = i;
        obs.pose.position.x = path[i].x/gridToCostMapRatio;
        obs.pose.position.y = path[i].y/gridToCostMapRatio;
        obs.pose.position.z = 0.5;
        obsMarkers.markers.push_back(obs);
    }
    if (path.size()!=0){
        markerArray_pub.publish(obsMarkers);
    }
}

void drawCell(std::vector<occupancy_grid_utils::Cell> cell, ros::Publisher markerArray_pub, int
               gridToCostMapRatio, string ns, float r, float g, float b, float a, bool text=false){
    if (cell.size()==0){
        return;
    }
    // cell
    visualization_msgs::MarkerArray obsMarkers;
    visualization_msgs::Marker obs;
    obs.type = visualization_msgs::Marker::CUBE;
    obs.header.frame_id = "odom";
    obs.header.stamp = ros::Time::now();
    obs.ns = ns;
    obs.action = visualization_msgs::Marker::ADD;

    if (a>0){
        obs.color.a = a;
    }
    obs.color.r = r;
    obs.color.g = g;
    obs.color.b = b;
    obs.scale.x = obs.scale.y = obs.scale.z = 1;
    obs.pose.orientation.x = 0.0;
    obs.pose.orientation.y = 0.0;
    obs.pose.orientation.z = 0.0;
    obs.pose.orientation.w = 1;

    // text (for some reasons it will overwrite the cell....)
    visualization_msgs::Marker obs_text;
    if (text){
        obs_text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        obs_text.header.frame_id = "odom";
        obs_text.header.stamp = ros::Time::now();
        obs_text.ns = ns + string("_text");
        obs_text.action = visualization_msgs::Marker::ADD;

        if (a>0){
            obs_text.color.a = a+0.3; // make text darker
        }
        obs_text.color.r = r;
        obs_text.color.g = g;
        obs_text.color.b = b;
        obs_text.scale.x = obs_text.scale.y = obs_text.scale.z = 0.2;
        obs_text.pose.orientation.x = 0.0;
        obs_text.pose.orientation.y = 0.0;
        obs_text.pose.orientation.z = 0.0;
        obs_text.pose.orientation.w = 1;
    }

    for (int i=0; i<cell.size(); ++i){
        if (a<=0){
            if (cell[i].cost==0 || cell[i].cost==-1){
                obs.color.a = 0.2;
                obs_text.color.a = 0.5; // make text darker
            }
            else{
                obs.color.a = cell[i].cost/200;
                obs_text.color.a = cell[i].cost/200 + 0.3; // make text darker
            }
        }
        obs.id = i;
        obs.pose.position.x = cell[i].x/gridToCostMapRatio;
        obs.pose.position.y = cell[i].y/gridToCostMapRatio;
        obs.pose.position.z = -0.4;
        obsMarkers.markers.push_back(obs);

        if (text){
            obs_text.id = i; 
            obs_text.pose.position.x = cell[i].x/gridToCostMapRatio;
            obs_text.pose.position.y = cell[i].y/gridToCostMapRatio;
            obs_text.pose.position.z = -0.4;
            obs_text.text = std::to_string(cell[i].index) + std::string("_") + 
                            toStringWithPrecision(cell[i].cost, 2);
            obsMarkers.markers.push_back(obs_text);
        }
    }
    markerArray_pub.publish(obsMarkers);
}

template <class T>
void drawUnorderedSet(T cell, ros::Publisher markerArray_pub, int gridToCostMapRatio, 
                      double occupied_range, int unknown_value,
                      string ns, bool text=false){
    if (cell.size()==0){
        return;
    }
    // cell
    visualization_msgs::MarkerArray obsMarkers;
    visualization_msgs::Marker obs;
    obs.type = visualization_msgs::Marker::CUBE;
    obs.header.frame_id = "odom";
    obs.header.stamp = ros::Time::now();
    obs.ns = ns;
    obs.action = visualization_msgs::Marker::ADD;
    obs.color.a = 0.2;
    obs.scale.x = obs.scale.y = obs.scale.z = 1;
    obs.pose.orientation.x = 0.0;
    obs.pose.orientation.y = 0.0;
    obs.pose.orientation.z = 0.0;
    obs.pose.orientation.w = 1;

    // text (for some reasons it will overwrite the cell....)
    visualization_msgs::Marker obs_text;
    if (text){
        obs_text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        obs_text.header.frame_id = "odom";
        obs_text.header.stamp = ros::Time::now();
        obs_text.ns = ns + string("_text");
        obs_text.action = visualization_msgs::Marker::ADD;
        obs_text.color.a = 0.5;
        obs_text.scale.x = obs_text.scale.y = obs_text.scale.z = 0.2;
        obs_text.pose.orientation.x = 0.0;
        obs_text.pose.orientation.y = 0.0;
        obs_text.pose.orientation.z = 0.0;
        obs_text.pose.orientation.w = 1;
    }

    for (auto iter=cell.begin(); iter!=cell.end(); ++iter){
        if (iter->cost<occupied_range){ // occupied
            obs.color.r = 1;
            obs.color.g = 0;
            obs.color.b = 0;

            obs_text.color.r = 1;
            obs_text.color.g = 0;
            obs_text.color.b = 0;
        }
        else if (iter->cost==unknown_value){ // unknown
            obs.color.r = 1;
            obs.color.g = 1;
            obs.color.b = 0;

            obs_text.color.r = 1;
            obs_text.color.g = 1;
            obs_text.color.b = 0;
        }
        else{
            obs.color.a = iter->cost/200;
            obs_text.color.a = iter->cost/200 + 0.3; // make text darker
            obs.color.r = 0;
            obs.color.g = 1;
            obs.color.b = 0;

            obs_text.color.r = 0;
            obs_text.color.g = 1;
            obs_text.color.b = 0;
        }
        obs.id = std::distance(cell.begin(), iter);
        obs.pose.position.x = iter->x/gridToCostMapRatio;
        obs.pose.position.y = iter->y/gridToCostMapRatio;
        obs.pose.position.z = -0.4;
        obsMarkers.markers.push_back(obs);

        if (text){
            obs_text.id = std::distance(cell.begin(), iter);; 
            obs_text.pose.position.x = iter->x/gridToCostMapRatio;
            obs_text.pose.position.y = iter->y/gridToCostMapRatio;
            obs_text.pose.position.z = -0.4;
            obs_text.text = std::to_string(iter->index) + std::string("_") + 
                            toStringWithPrecision(iter->cost, 2);
            obsMarkers.markers.push_back(obs_text);
        }
    }
    markerArray_pub.publish(obsMarkers);
}

template <class T> 
void assignMarker(ros::Publisher marker_pub, T x, T y, T z, const uint32_t Shape, const string NameSpace,
				  const double r, const double g, const double b,
				  const int Count, const double Size){
    visualization_msgs::Marker obs;
	obs.header.frame_id = "odom";
	obs.header.stamp = ros::Time::now();
	obs.ns = NameSpace;
	obs.id = Count;
	obs.type = Shape; 
	obs.action = visualization_msgs::Marker::ADD;
	obs.pose.position.x = x;
	obs.pose.position.y = y;
	obs.pose.position.z = z;
	obs.pose.orientation.x = 0.0;
	obs.pose.orientation.y = 0.0;
	obs.pose.orientation.z = 0.0;
	obs.pose.orientation.w = 1.0;
	// obs.lifetime = ros::Duration(_sleep_time_for_vis); // should disappear along with updateing rate
	// obs.lifetime = ros::Duration(_sleep_time_for_vis); // should disappear along with updateing rate

	// Set the scale of the marker -- 1x1x1 here means 1m on a side
	obs.scale.x = Size;
	obs.scale.y = Size;
	obs.scale.z = Size;

	// Set the color -- be sure to set alpha to something non-zero!
	obs.color.r = r;
	obs.color.g = g;
	obs.color.b = b;
	obs.color.a = 1.0; 
    marker_pub.publish(obs);
}

void updateDstar(Dstar *dstar, std::vector<visualization_msgs::Marker> &stateObsVec,
		         int value,	
                 Eigen::MatrixXd costMap, float weight,
                 ros::Publisher marker_pub, int boundary, int gridToCostMapRatio) {
    for (int i=0; i<costMap.rows()-1; ++i){
        for (int j=0; j<costMap.cols()-1; ++j){
            // cout << "(" << i << ", " << j << ", " << costMap(i, j) << ")" << endl;
            dstar->updateCell(i, j, weight/costMap(i,j));
        }
    }
    visualization_msgs::Marker obs;
    obs.type = visualization_msgs::Marker::CUBE;
    obs.header.frame_id = "odom";
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
    obs.ns = "Dstar obs";

    int k = 0;
    for (int i=0; i<3; ++i){
        for (int j=10; j<20; ++j){
            // dstar->updateCell(i, j, value);
            nineCellUpdate(i, j, value, dstar, gridToCostMapRatio);
            obs.id = k;
            obs.pose.position.x = i + 0.5;
            obs.pose.position.y = j + 0.5;
            stateObsVec.push_back(obs);
            marker_pub.publish(obs);
            k++;
        }
    }

    for (int i=3; i<5; ++i){
        for (int j=17; j<20; ++j){
            // dstar->updateCell(i, j, value);
            nineCellUpdate(i, j, value, dstar, gridToCostMapRatio);
            obs.id = k;
            obs.pose.position.x = i + 0.5;
            obs.pose.position.y = j + 0.5;
            stateObsVec.push_back(obs);
            marker_pub.publish(obs);
            k++;
        }
    }

    for (int i=7; i<9; ++i){
        for (int j=0; j<13; ++j){
            // dstar->updateCell(i, j, value);
            nineCellUpdate(i, j, value, dstar, gridToCostMapRatio);
            obs.id = k;
            obs.pose.position.x = i + 0.5;
            obs.pose.position.y = j + 0.5;
            stateObsVec.push_back(obs);
            if (j<12) marker_pub.publish(obs);
            k++;
        }
    }

    // left boundary
    for (int j=0; j<boundary; ++j){
        // dstar->updateCell(0, j, value);
        nineCellUpdate(0, j, value, dstar, gridToCostMapRatio);
        obs.id = k;
        obs.pose.position.x = 0 - 0.5;
        obs.pose.position.y = j + 0.5;
        stateObsVec.push_back(obs);
        k++;
    }

    // right boundary
    for (int j=0; j<boundary; ++j){
        // dstar->updateCell(boundary, j, value);
        nineCellUpdate(boundary, j, value, dstar, gridToCostMapRatio);
        obs.id = k;
        obs.pose.position.x = boundary + 0.5;
        obs.pose.position.y = j + 0.5;
        stateObsVec.push_back(obs);
        k++;
    }

    // bottom boundary
    for (int i=0; i<boundary; ++i){
        // dstar->updateCell(i, 0, value);
        nineCellUpdate(i, 0, value, dstar, gridToCostMapRatio);
        obs.id = k;
        obs.pose.position.x = i + 0.5;
        obs.pose.position.y = 0 - 0.5;
        stateObsVec.push_back(obs);
        k++;
    }

    // top boundary
    for (int i=0; i<boundary; ++i){
        // dstar->updateCell(i, boundary, value);
        nineCellUpdate(i, boundary, value, dstar, gridToCostMapRatio);
        obs.id = k;
        obs.pose.position.x = i + 0.5;
        obs.pose.position.y = boundary + 0.5;
        stateObsVec.push_back(obs);
        k++;
    }

    obs.type = visualization_msgs::Marker::LINE_STRIP;
    obs.header.frame_id = "odom";
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
}

void updateDstar(Dstar *dstar, std::vector<visualization_msgs::Marker> &stateObsVec,
		         int value,	
                 ros::Publisher marker_pub, int boundary, int gridToCostMapRatio) {
    visualization_msgs::Marker obs;
    obs.type = visualization_msgs::Marker::CUBE;
    obs.header.frame_id = "odom";
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
    obs.ns = "Dstar obs";

    int k = 0;
    for (int i=0; i<3; ++i){
        for (int j=10; j<20; ++j){
            // dstar->updateCell(i, j, value);
            nineCellUpdate(i, j, value, dstar, gridToCostMapRatio);
            obs.id = k;
            obs.pose.position.x = i + 0.5;
            obs.pose.position.y = j + 0.5;
            stateObsVec.push_back(obs);
            marker_pub.publish(obs);
            k++;
        }
    }

    for (int i=3; i<5; ++i){
        for (int j=17; j<20; ++j){
            // dstar->updateCell(i, j, value);
            nineCellUpdate(i, j, value, dstar, gridToCostMapRatio);
            obs.id = k;
            obs.pose.position.x = i + 0.5;
            obs.pose.position.y = j + 0.5;
            stateObsVec.push_back(obs);
            marker_pub.publish(obs);
            k++;
        }
    }

    for (int i=7; i<9; ++i){
        for (int j=0; j<13; ++j){
            // dstar->updateCell(i, j, value);
            nineCellUpdate(i, j, value, dstar, gridToCostMapRatio);
            obs.id = k;
            obs.pose.position.x = i + 0.5;
            obs.pose.position.y = j + 0.5;
            stateObsVec.push_back(obs);
            if (j<12) marker_pub.publish(obs);
            k++;
        }
    }

    // left boundary
    for (int j=0; j<boundary; ++j){
        // dstar->updateCell(0, j, value);
        nineCellUpdate(0, j, value, dstar, gridToCostMapRatio);
        obs.id = k;
        obs.pose.position.x = 0 - 0.5;
        obs.pose.position.y = j + 0.5;
        stateObsVec.push_back(obs);
        k++;
    }

    // right boundary
    for (int j=0; j<boundary; ++j){
        // dstar->updateCell(boundary, j, value);
        nineCellUpdate(boundary, j, value, dstar, gridToCostMapRatio);
        obs.id = k;
        obs.pose.position.x = boundary + 0.5;
        obs.pose.position.y = j + 0.5;
        stateObsVec.push_back(obs);
        k++;
    }

    // bottom boundary
    for (int i=0; i<boundary; ++i){
        // dstar->updateCell(i, 0, value);
        nineCellUpdate(i, 0, value, dstar, gridToCostMapRatio);
        obs.id = k;
        obs.pose.position.x = i + 0.5;
        obs.pose.position.y = 0 - 0.5;
        stateObsVec.push_back(obs);
        k++;
    }

    // top boundary
    for (int i=0; i<boundary; ++i){
        // dstar->updateCell(i, boundary, value);
        nineCellUpdate(i, boundary, value, dstar, gridToCostMapRatio);
        obs.id = k;
        obs.pose.position.x = i + 0.5;
        obs.pose.position.y = boundary + 0.5;
        stateObsVec.push_back(obs);
        k++;
    }

    obs.type = visualization_msgs::Marker::LINE_STRIP;
    obs.header.frame_id = "odom";
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

bool moveRobot(ros::Publisher marker_pub, 
        geometry_msgs::Point next_pose, geometry_msgs::Point goal) {
    static visualization_msgs::Marker rob;
    rob.type = visualization_msgs::Marker::CUBE;


    rob.header.frame_id = "odom";
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


} // namespace dstar utils
} // namespace bipedlab

#endif
