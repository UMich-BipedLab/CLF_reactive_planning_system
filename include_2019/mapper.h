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
/**
 *  @author Bruce JK Huang 
 *  @date 6/2019
 *  @version 0.1 
 */
#ifndef MAPPER_H
#define MAPPER_H

#include <string>     // std::string, std::to_string
#include <queue>
#include <vector>
#include <algorithm> // for sort function
#include <cmath> // for sort function
#include <random> // for mvn
#include <fstream> // log files
#include <chrono> // timing
// #include <set> // for global map
#include <unordered_set>
#include <array> // array


#include <eigen3/Eigen/Dense> // SVD
#include <eigen3/Eigen/StdVector>
#include <eigen3/Eigen/Eigenvalues>


#include "boost/thread/mutex.hpp"
#include "boost/thread.hpp"

#include <ros/ros.h>

#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h> // Marker


#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include "occupancy_grid_utils.h"
#include "waypoint_data_t.h"
#include "wavefront_frontier_detection.h"

using namespace std;

typedef struct boundaryPoints{
    std::vector<occupancy_grid_utils::Cell> free_cell_vec;
    std::vector<occupancy_grid_utils::Cell> unknown_cell_vec;
    std::vector<occupancy_grid_utils::Cell> occupied_cell_vec;
    std::vector<occupancy_grid_utils::Cell> current_cell_vec;
    std::vector<occupancy_grid_utils::Cell> boundaries_cell_vec;

    std::vector<uint32_t> boundaries_vec;
    occupancy_grid_utils::Cell one;
    occupancy_grid_utils::Cell two;
    occupancy_grid_utils::Cell three;
    occupancy_grid_utils::Cell four;
} boundaryPoints_t;

typedef struct Velocity{
    double v_x;
    double v_y;
    double v_z;
} Velocity_t;

typedef struct robotState{
    tf::StampedTransform robot_pose;
    Velocity_t velocity;
    Velocity_t dir_velocity;
    double roll;
    double pitch;
    double yaw;
    ros::Time time;
    occupancy_grid_utils::Cell pose_in_cell;
    int grid_index;

    robotState& operator=(const robotState &a) {
        robot_pose = a.robot_pose;
        grid_index = a.grid_index;
        pose_in_cell.x = a.pose_in_cell.x;
        pose_in_cell.y = a.pose_in_cell.y;
        pose_in_cell.cost = a.pose_in_cell.cost;
        pose_in_cell.index = a.pose_in_cell.index;

        return *this;
    }
} RobotState_t;

// Custom comparator that compares the cell objects by index
typedef struct CellComparator{
    bool operator()(const occupancy_grid_utils::Cell &obj1, const occupancy_grid_utils::Cell &obj2) const{
        return obj1.index == obj2.index;
    }
} CellComparator_t;

// Custom hasher that return index as key
typedef struct CellHasher{
    size_t operator()(const occupancy_grid_utils::Cell &obj) const{
        return obj.index;
    }
} CellHasher_t;


typedef struct PublishTypes{
        waypoint_data_t waypoints_udp;
        std::string  s_waypoints_udp;
} PublishTypes_t;

// need to be changed... 
typedef struct PlannerStatus {
    bool map_updated;
    bool pose_updated;
    bool goal_updated;
    bool no_path;
    bool no_free_cell;
} PlannerStatus_t;

typedef struct ControlSignal{
    ControlSignal(): normal(0), backward(1), walk_around(2){}
    int normal;
    int backward;
    int walk_around;
} ControlSignal_t;

typedef struct Goal{
    geometry_msgs::Point goal; // goal point
    double distance; // distance to cassie
    double theta; // angle between cassie's yaw and the goal
    occupancy_grid_utils::Cell cell;
} Goal_t;

typedef struct Goals{
    Goal_t *left_goal1;
    Goal_t left_goal2;
    Goal_t *right_goal1;
    Goal_t right_goal2;
    std::vector<Goal_t> vec_goals;
} Goals_t;

namespace bipedlab{
class Mapper {
    public:
        int method; 
        ros::Publisher marker_pub;
        ros::Publisher marker_array_pub;
        ros::Publisher frontier_publisher;
        ros::Publisher side_walk_publisher;
        ros::Subscriber click_point_sub;

        // searching boundaries about robot
        boundaryPoints_t boundaries;

        // robot's current state
        RobotState_t robot_state;
        RobotState_t robot_state_old;

        // goal
        occupancy_grid_utils::Cell goal_cell;
        geometry_msgs::Point goal_point; // real goal of selecting
        Goal_t goal_right; // riht goal point
        Goal_t goal_left; // left goal point
        Goals_t goals; 
        std::vector<std::pair<Goal_t, Goal_t>> goal_segments;
        std::vector<geometry_msgs::Point> circle;
        std::vector<geometry_msgs::Point> free_circle;
        std::vector<geometry_msgs::Point> occupied_circle;
        
        // if updated (TODO: need to pack back to PlannerStatus_t)
        bool map_updated;
        bool pose_updated;
        bool goal_updated;
        bool no_path;
        bool no_free_cell;
        bool no_goal;
        bool not_enough_segments;

        // cost of cells
        double grid_to_costmap_ratio; 
        double occupied_cell; // range of occipied
        int unknown_cell; // single value
        double unknown_cell_cost; // cost for traverse over unknown area
        double occupied_cell_cost; // cost for traverse over occupied area


        // frontiers
        bool frontier;

        // publishers
        bool pub_free;
        bool pub_occupied;
        bool pub_unknown;
        bool pub_all_cells;


        // planner
        float replannning_timeout;
        double update_time;
        double waypoint_distance;
        int dstar_max_step;
        int dstar_unknow_cell_cost;

        // global cost-map
        std::unordered_set<occupancy_grid_utils::Cell, CellHasher_t, CellComparator_t> total_cost_map;

        Mapper(int argc, char** argv): 
            m_data_received(false), m_goal_received(false), 
            map_updated(false), pose_updated(false), goal_updated(false), no_path(false), frontier(false),
            m_look_up_counter(0), m_goal_finder_change_count(0), m_expend_searching_counter(0),
            grid_to_costmap_ratio(1), replannning_timeout(2), m_direction(10), m_goal_distance(3),
            m_queue_size(3), m_goal_counter(0), no_goal(true), not_enough_segments(true),
            m_expend_radious_amount(0)
        {
            tf::Vector3 zero;
            zero.setZero();
            robot_state_old.robot_pose.setOrigin(zero);
            robot_state_old.grid_index = 0;
            robot_state_old.pose_in_cell.x = 0;
            robot_state_old.pose_in_cell.y = 0;
            robot_state_old.velocity.v_x = 0;
            robot_state_old.velocity.v_y = 0;
            robot_state_old.time = ros::Time::now();
            // this->m_control_signal();
            // cout << "control: "<< m_control_signal.normal();
            // exit(0);

            ros::Rate r(10); // 
            Mapper::m_getParameters();

            marker_pub = m_nh.advertise<visualization_msgs::Marker>("visualization_marker", 10);
            marker_array_pub = m_nh.advertise<visualization_msgs::MarkerArray>("MarkerArray", 10);
            // m_map_sub = m_nh.subscribe("/labeled_pc_map_node/occupancy_grid", 10, &Mapper::_octomapCallBack, this);
            m_map_sub = m_nh.subscribe(m_costmap_topic, 10, &Mapper::_octomapCallBack, this);
            click_point_sub = m_nh.subscribe("/clicked_point", 10, &Mapper::_clickPointCallBack, this);
            frontier_publisher = m_nh.advertise<sensor_msgs::PointCloud>("frontiers", 1);
            side_walk_publisher = m_nh.advertise<sensor_msgs::PointCloud>("side_walk", 1);

            // sync_.reset(new Sync(SyncPolicy(10), aprilTagSub_, lidarTagSub_));
            // sync_->registerCallback(boost::bind(&Mapper::callback_, this, _1, _2));
            boost::thread spin(&Mapper::_spin, this);

            if (method==0 || method==1 || method==3){
                ROS_INFO("waiting for data...");
            }
            else if (method==2){
                ROS_INFO("waiting for data and a goal...");
            }
            Mapper::_waitForData();
        }

        void waitForData(){
            _waitForData();
        }

        void waitForNewClick(){
            ROS_INFO("waiting for new click... ");
            this->m_goal_received=false;
            _waitForData();
        }

        template <class T>
        void updateGoal(T a, T b){
            this->goal_point.x = a;
            this->goal_point.y = b;
        }

        template <class T>
        void updateGoalInGrid(T a, T b){
            this->goal_cell.x = a - m_current_map->info.origin.position.x;
            this->goal_cell.y = b - m_current_map->info.origin.position.y;
        }

        bool lookUpPose(){
            // cout << "---------" << endl;
            try{
                // m_listener.lookupTransform(m_cassie_frame, m_map_frame, ros::Time(0), robot_state.robot_pose);
                m_listener.lookupTransform(m_map_frame, m_cassie_frame, ros::Time(0), robot_state.robot_pose);
                ROS_INFO("[LookUp] Looked up robot pose");
                robot_state.time = ros::Time::now();
                pose_updated = true;
                m_look_up_counter ++;
                if (m_look_up_counter<5) return false;
            }
            catch (tf::TransformException ex){
                ROS_WARN("[LookUp] No robot pose");
                ROS_ERROR("%s", ex.what());
                robot_state = robot_state_old;
                ros::Duration(1.0).sleep();
                pose_updated = false;
                if (m_look_up_counter==0) return false;
            }
            // cout << "---------" << endl;
            // cout << "res: " << m_current_map->info.resolution << endl;
            // ROS_DEBUG_STREAM("x in grid:" << robot_state.robot_pose.getOrigin().x()/m_current_map->info.resolution); // the grid map 
            // ROS_DEBUG_STREAM("y in grid:" << robot_state.robot_pose.getOrigin().y()/m_current_map->info.resolution); // the grid map
            Mapper::_robot_velocity();


            // look up robot's pose in the grid map
            occupancy_grid_utils::Cell robot_pose((robot_state.robot_pose.getOrigin().x()
                                                  - m_current_map->info.origin.position.x)/m_current_map->info.resolution, 
                                                  (robot_state.robot_pose.getOrigin().y()
                                                  - m_current_map->info.origin.position.y)/m_current_map->info.resolution);
            robot_state.grid_index = occupancy_grid_utils::cellIndex(m_current_map->info, robot_pose);
            robot_state.pose_in_cell.x = robot_pose.x; // pose in cell
            robot_state.pose_in_cell.y = robot_pose.y; // pose in cell

            robot_state_old = robot_state;

            return true;
            // cout << "index: " << robot_state.grid_index << endl;
            // Mapper::updateBoundries();  
            // Mapper::frontiers();
            
            // Mapper::checkFork();
            // Mapper::goalSelection();
            // cout << "---------" << endl;
            // cout << "Looked up" << endl;
        }

        // working in cell map
        void updateBoundries(){
            if (no_path || no_free_cell){ 
                if (m_expend_searching_counter < m_expend_searching_dim_amount_bound) {
                    m_searching_dim = m_searching_dim + m_expend_searching_dim_amount;
                }
                    no_path = false;
                    no_free_cell = false;
                    m_expend_searching_counter ++;
                    ROS_INFO_STREAM("[Boundary] Updated searching area: " << m_searching_dim);
            }
            else{
                m_searching_dim = m_searching_dim_old;
                m_expend_searching_counter = 0;
            } 

            boundaries.free_cell_vec.clear();
            boundaries.occupied_cell_vec.clear();
            boundaries.unknown_cell_vec.clear();
            boundaries.boundaries_vec.clear();
            boundaries.boundaries_cell_vec.clear();
            boundaries.current_cell_vec.clear();

            boundaries.one.x = robot_state.pose_in_cell.x - m_searching_dim;
            boundaries.one.y = robot_state.pose_in_cell.y + m_searching_dim;
            int index = occupancy_grid_utils::cellIndex(m_current_map->info, boundaries.one);
            boundaries.boundaries_vec.push_back(index);
            boundaries.one.index = index; 

            occupancy_grid_utils::Cell world_one(boundaries.one.x * m_current_map->info.resolution + 
                                                 m_current_map->info.origin.position.x, 
                                                 boundaries.one.y * m_current_map->info.resolution + 
                                                 m_current_map->info.origin.position.y);
            world_one.cost = m_current_map->data[index] * m_current_map->info.resolution;
            world_one.index = index;
            boundaries.boundaries_cell_vec.push_back(world_one);

            boundaries.two.x = robot_state.pose_in_cell.x - m_searching_dim;
            boundaries.two.y = robot_state.pose_in_cell.y - m_searching_dim;
            index = occupancy_grid_utils::cellIndex(m_current_map->info, boundaries.two);
            boundaries.two.index = index;
            boundaries.boundaries_vec.push_back(index);


            occupancy_grid_utils::Cell world_two(boundaries.two.x * m_current_map->info.resolution + 
                                                 m_current_map->info.origin.position.x, 
                                                 boundaries.two.y * m_current_map->info.resolution + 
                                                 m_current_map->info.origin.position.y);
            world_two.cost = m_current_map->data[index] * m_current_map->info.resolution;
            world_two.index = index;
            boundaries.boundaries_cell_vec.push_back(world_two);

            boundaries.three.x = robot_state.pose_in_cell.x + m_searching_dim;
            boundaries.three.y = robot_state.pose_in_cell.y - m_searching_dim;
            index = occupancy_grid_utils::cellIndex(m_current_map->info, boundaries.three);
            boundaries.three.index = index; 
            boundaries.boundaries_vec.push_back(index);

            occupancy_grid_utils::Cell world_three(boundaries.three.x * m_current_map->info.resolution + 
                                                   m_current_map->info.origin.position.x, 
                                                   boundaries.three.y * m_current_map->info.resolution + 
                                                   m_current_map->info.origin.position.y);
            world_three.cost = m_current_map->data[index] * m_current_map->info.resolution;
            world_three.index = index;
            boundaries.boundaries_cell_vec.push_back(world_three);

            boundaries.four.x = robot_state.pose_in_cell.x + m_searching_dim;
            boundaries.four.y = robot_state.pose_in_cell.y + m_searching_dim;
            index = occupancy_grid_utils::cellIndex(m_current_map->info, boundaries.four);
            boundaries.four.index = index; 
            boundaries.boundaries_vec.push_back(index);

            occupancy_grid_utils::Cell world_four(boundaries.four.x * m_current_map->info.resolution + 
                                                  m_current_map->info.origin.position.x, 
                                                  boundaries.four.y * m_current_map->info.resolution + 
                                                  m_current_map->info.origin.position.y);
            world_four.cost = m_current_map->data[index] * m_current_map->info.resolution;
            world_four.index = index;
            boundaries.boundaries_cell_vec.push_back(world_four);

            std::sort(boundaries.boundaries_vec.begin(), boundaries.boundaries_vec.end());

            // for (auto x : boundaries.boundaries_vec) 
            //     ROS_DEBUG_STREAM("boundary index: " << x); 

            ROS_DEBUG_STREAM("[Boundary] robot index: " << (int)robot_state.grid_index);
            // for cost only
            // float distance_left = -1e3;
            // float delta_theta_left = 1e3;

            // float distance_right = -1e3;
            // float delta_theta_right = -1e3;
            // bool got_left_goal = false;
            // bool got_right_goal = false;
            for (int i=-m_searching_dim; i<m_searching_dim+1; ++i){
                for (int j=-m_searching_dim; j<m_searching_dim+1; ++j){
                    occupancy_grid_utils::Cell current_cell((int)std::round(robot_state.pose_in_cell.x) + i, 
                                                            (int)std::round(robot_state.pose_in_cell.y) + j);
                    int index = occupancy_grid_utils::cellIndex(m_current_map->info, current_cell);

                    occupancy_grid_utils::Cell current_cell_test(robot_state.pose_in_cell.x + i, 
                                                                 robot_state.pose_in_cell.y + j);
                    current_cell.index = index; 

                    // distance to the nearest obstacle
                    double value = m_current_map->data[index] * m_current_map->info.resolution; 
                    value = std::pow((m_current_map->data[index] * m_current_map->info.resolution), m_order); 

                    // ROS_DEBUG_STREAM("(i, j) = (" << i << ", " << j << ")");
                    // ROS_DEBUG_STREAM("(x_c, y_c, c) : (" << current_cell.x << ", " 
                    //                  << current_cell.y << ", " << current_cell.cost << ")");

                    // float distance = _getDistanceToRobotInCell(index);
                    
                    // moving back to cost map
                    // cout << "searching index: " << index << endl;
                    // cout << "(x, r, p) : (" << current_cell.x << ", " << m_current_map->info.resolution << ", " << m_current_map->info.origin.position.x << endl;
                    occupancy_grid_utils::Cell potential_goal; // goal point
                    potential_goal.index = index;
                    potential_goal.x = current_cell.x * m_current_map->info.resolution + 
                                     m_current_map->info.origin.position.x;
                    potential_goal.y = current_cell.y * m_current_map->info.resolution + 
                                     m_current_map->info.origin.position.y;

                    m_insert_result = total_cost_map.insert(potential_goal); // all

                    float dis = std::sqrt(std::pow(robot_state.robot_pose.getOrigin().x() - 
                                                   potential_goal.x, 2) + 
                                          std::pow(robot_state.robot_pose.getOrigin().y() - 
                                                   potential_goal.y, 2));
                    // distance checking
                    if (dis < m_blind_horizon) {
                        continue;
                    }

                    // neighbour checking
                    // std::array<int, 24> locations; 
                    // _getNeighbors(locations, index, m_current_map->info.width);
                    // bool neighbor = _neighborsFree(locations);


                    // if no insersion, return
                    // if (m_insert_result.second==false){
                    //     // ROS_WARN("NOT ADDED");
                    //     continue;
                    // }
                    // if (distance < m_planning_horizon){
                    // cout << "dis: " << distance << endl;
                    if (value>occupied_cell){  // free cell
                        // parse distance to cost
                        potential_goal.cost = 1 / (value);
                        boundaries.free_cell_vec.push_back(potential_goal);
                    }
                    else if (value<=occupied_cell && value >= 0){ // occupied
                        // parse distance to cost
                        potential_goal.cost = occupied_cell_cost; 
                        boundaries.occupied_cell_vec.push_back(potential_goal);
                    }
                    else if (value < 0 || 
                             value==unknown_cell*m_current_map->info.resolution){ // unknown
                        // parse distance to cost
                        potential_goal.cost = unknown_cell_cost;
                        // cout << "cost: " << potential_goal.cost << endl;
                        boundaries.unknown_cell_vec.push_back(potential_goal);
                    }

                    if ((i % ((int) std::round(m_searching_dim/4))) == 0 && 
                        (j % ((int) std::round(m_searching_dim/4))) == 0){
                        // cout << "(i, j) = (" << i << ", " << j << ")" << endl;
                        // cout << "cost: " << potential_goal.cost  << endl;
                        boundaries.current_cell_vec.push_back(potential_goal); // current 
                    }

                    // ROS_DEBUG_STREAM("(x, y, c) : (" << potential_goal.x << ", " 
                    //                  << potential_goal.y << ", " << potential_goal.cost << ")");
                    // ROS_DEBUG_STREAM("(x_r, y_r) : (" << this->robot_state.robot_pose.getOrigin().x() << 
                    //                  ", " << this->robot_state.robot_pose.getOrigin().y());

                    // }
                }
            }
        } 
        
        bool checkFork(){
            circle.clear();
            free_circle.clear();
            occupied_circle.clear();
            goal_segments.clear();

            // expending the circle if necessary 
            if (no_goal){ 
                if (m_expend_radious_amount < m_expend_radious_amount_bound) {
                    m_radious = m_radious + m_expend_searching_dim_amount * m_current_map->info.resolution;
                    m_expend_radious_amount ++;
                }
                else{
                    // reset if reach the limit
                    m_radious = m_radious_old;
                    m_expend_radious_amount = 0;
                }
                    ROS_INFO_STREAM("[Circle] Updated circle radius: " << m_radious);
            }
            else {
                m_radious = m_radious_old;
                m_expend_radious_amount = 0;
            }

            // contructing a circle
            float radious = m_radious; // radious of searching
            int searching_range = 360;
            std::array<int, 24> adj; 
            std::array<int, 8> adj_small; 

            // make sure it starts from occupied
            float total_theta = this->robot_state.yaw;
            int offset_theta = 0;
            int start_from_occupied_threshold = 3; // consecutive 3 times
            int start_from_occupied_count = 0; // consecutive 3 times

            while(1){
                double x = this->robot_state.robot_pose.getOrigin().x() -
                           radious * std::cos((total_theta + offset_theta) * M_PI/180);
                double y = this->robot_state.robot_pose.getOrigin().y() + 
                           radious * std::sin((total_theta + offset_theta) * M_PI/180);
                occupancy_grid_utils::Cell cell = Mapper::_worldToMap(x, y);
                _getNeighbors(adj, cell.index, m_current_map->info.width);

                if (_neighborsOccupied(adj)){
                    start_from_occupied_count ++;
                    if (start_from_occupied_count > start_from_occupied_threshold){
                        total_theta = total_theta + offset_theta;
                        ROS_DEBUG_STREAM("[Circle] Start theta: " << total_theta);
                        break;
                    }
                }
                offset_theta += 2;
                if (offset_theta>=360){
                    ROS_WARN("[Circle] No obstacles");
                    break;
                }
            }

            int fork_threshold = 10; // a meter of obstacle
            int path_threshold = m_min_free; //  0.3 meter of path
            int path_count = 0; // how many paths we have now
            bool switch_face = false;

            int occupied_cell_count = 0;
            int free_cell_count = 0;

            Goal_t goal;
            std::pair<Goal_t, Goal_t> goal_seg;
            for (int i=0; i>-searching_range; i-=m_circle_resolution){ // ccw
                double x = this->robot_state.robot_pose.getOrigin().x() -
                           radious * std::cos((total_theta + i) * M_PI/180);
                double y = this->robot_state.robot_pose.getOrigin().y() + 
                           radious * std::sin((total_theta + i) * M_PI/180);

                geometry_msgs::Point point;
                point.x = x;
                point.y = y;
                circle.push_back(point);

                // neighbour checking
                occupancy_grid_utils::Cell cell = Mapper::_worldToMap(x, y);
                _getNeighbors(adj, cell.index, m_current_map->info.width);
                _getNeighbors(adj_small, cell.index, m_current_map->info.width);

                if(_freeCell(x, y)) {
                    if(_neighborsFree(adj)){
                        free_circle.push_back(point);
                        free_cell_count ++;

                        // if (switch_face && occupied_cell_count > fork_threshold) {
                        if (!switch_face) {
                            goal.goal.x = x;
                            goal.goal.y = y;
                            goal.theta = total_theta + i;
                            goal.cell = Mapper::_worldToMap(x, y);
                            goals.vec_goals.push_back(goal);
                            occupied_cell_count = 0;
                            goal_seg.first = goal;
                            switch_face = true;
                        }
                    }
                    else if (free_cell_count > path_threshold && switch_face){
                        switch_face = false;
                        goal.goal.x = x;
                        goal.goal.y = y;
                        goal.theta = total_theta + i;
                        goal.cell = Mapper::_worldToMap(x, y);
                        goals.vec_goals.push_back(goal);
                        free_cell_count = 0;
                        goal_seg.second = goal;
                        goal_segments.push_back(goal_seg);
                        // ROS_INFO("[Circle] Segment added!");
                        path_count ++;
                    }

                }
                else{
                    switch_face = false;
                    free_cell_count = 0;
                }


                if(_neighborsOccupied(adj)){
                    occupied_cell_count ++;
                    occupied_circle.push_back(point);
                }
            }
            ROS_INFO_STREAM("[Circle] path seg: " << goal_segments.size());
        }

        // if num of seg < 2 -> no goal
        void goalSelection(){
            bool no_goal_found = true; // use in this 
            int size = goal_segments.size();
            int direction = 0; // left
            int delta_theta = 1e3;
            bool too_long = false;
            bool has_triggered = false;

            for (int i=0; i<size; ++i){
                // double x = (goal_segments[i].first.goal.x + goal_segments[i].second.goal.x) / 2;
                // double y = (goal_segments[i].first.goal.y + goal_segments[i].second.goal.y) / 2;
                double x =  this->robot_state.robot_pose.getOrigin().x() -
                            m_radious * std::cos((goal_segments[i].first.theta + 
                                                 goal_segments[i].second.theta) / 2 * M_PI/180);
                double y =  this->robot_state.robot_pose.getOrigin().y() +
                            m_radious * std::sin((goal_segments[i].first.theta + 
                                                 goal_segments[i].second.theta) / 2 * M_PI/180);

                geometry_msgs::Point32 vec;
                vec.x = x - robot_state.robot_pose.getOrigin().x();
                vec.y = y - robot_state.robot_pose.getOrigin().y();

                float sin_theta = (robot_state.dir_velocity.v_x * vec.y - 
                                   robot_state.dir_velocity.v_y * vec.x) / 
                                   std::sqrt(std::pow(vec.x, 2) + std::pow(vec.y, 2));
                float cos_theta = (robot_state.dir_velocity.v_x * vec.x + 
                                   robot_state.dir_velocity.v_y * vec.y) / 
                                   std::sqrt(std::pow(vec.x, 2) + std::pow(vec.y, 2)); 
                double theta = std::asin(sin_theta) * 180 / M_PI;
                ROS_DEBUG_STREAM("[Goal] Theta of segment '" << i << "' : " << theta);

                if (cos_theta < 0 && sin_theta > 0) {
                    theta = 180 - theta;
                }

                if (cos_theta < 0 && sin_theta < 0) {
                    theta = - (180 - theta);
                }
                ROS_DEBUG_STREAM("[Goal] Shift theta: " << theta);
                // ROS_DEBUG_STREAM("sin: " << sin_theta);
                // ROS_DEBUG_STREAM("cos: " << cos_theta);
                
                // too wide and find a too long segment, discard
                if (theta < -m_right_fov && too_long){
                    no_goal_found = true;
                    ROS_DEBUG_STREAM("[Goal] TOO WIDE " << theta);
                    continue;
                }

                // pick left
                if (goal_segments.size()==1){
                    if (cos_theta>0){
                        if (std::abs((theta - m_direction)) < (delta_theta)){ 
                            double dis_goal = _distance(goal_segments[i].second.goal.x, 
                                                        goal_segments[i].second.goal.y, x, y);
                            double goal_theta = std::asin(m_away_from_end_of_segment / m_radious) * 180 / M_PI;
                            no_goal_found = false;
                            goal_updated = true;
                            frontier = true;
                            
                            if (dis_goal > 1 || too_long) {
                                ROS_DEBUG_STREAM("[Goal] TOO LONG " << dis_goal);
                                this->goal_point.x =  this->robot_state.robot_pose.getOrigin().x() -
                                                      m_radious * std::cos((goal_segments[i].second.theta + 
                                                                            goal_theta) * M_PI/180);
                                this->goal_point.y =  this->robot_state.robot_pose.getOrigin().y() +
                                                      m_radious * std::sin((goal_segments[i].second.theta + 
                                                                            goal_theta) * M_PI/180);
                            }
                            else {
                                this->goal_point.x = x;
                                this->goal_point.y = y;
                            }


                            delta_theta = std::abs(theta - m_direction);
                            ROS_INFO_STREAM("[Goal] Theta selected: " << theta);
                            ROS_DEBUG_STREAM("[Goal] Delta theta: " << delta_theta);
                        }
                    }
                }
                else if (goal_segments.size()==2){
                    if (cos_theta>0){
                        if (std::abs((theta - m_direction)) < (delta_theta)){ 
                            double dis_goal = _distance(goal_segments[i].second.goal.x, 
                                                        goal_segments[i].second.goal.y, x, y);
                            double goal_theta = std::asin(m_away_from_end_of_segment / m_radious) * 180 / M_PI;
                            no_goal_found = false;
                            goal_updated = true;
                            frontier = true;
                            
                            if (dis_goal > 1 || too_long) {
                                ROS_DEBUG_STREAM("[Goal] TOO LONG " << dis_goal);

                                if (m_only_forward){

                                    // only do it once
                                    if (!has_triggered){
                                        i = -1; // re-search
                                        has_triggered = true;
                                        too_long = true;
                                        ROS_INFO_STREAM("[Goal] Triggered snapping");
                                        continue;
                                    }

                                    this->goal_point.x =  this->robot_state.robot_pose.getOrigin().x() -
                                                          m_radious * std::cos((goal_segments[i].second.theta + 
                                                                                goal_theta) * M_PI/180);
                                    this->goal_point.y =  this->robot_state.robot_pose.getOrigin().y() +
                                                          m_radious * std::sin((goal_segments[i].second.theta + 
                                                                                goal_theta) * M_PI/180);
                                }
                                else{
                                    this->goal_point.x =  this->robot_state.robot_pose.getOrigin().x() -
                                                          m_radious * std::cos((goal_segments[i].second.theta + 
                                                                                goal_theta) * M_PI/180);
                                    this->goal_point.y =  this->robot_state.robot_pose.getOrigin().y() +
                                                          m_radious * std::sin((goal_segments[i].second.theta + 
                                                                                goal_theta) * M_PI/180);
                                }
                            }
                            else {
                                this->goal_point.x = x;
                                this->goal_point.y = y;
                            }


                            delta_theta = std::abs(theta - m_direction);
                            ROS_INFO_STREAM("[Goal] Theta selected: " << theta);
                            ROS_DEBUG_STREAM("[Goal] Delta theta: " << delta_theta);
                        }
                    }
                }
                else if (goal_segments.size()!=2){ // how to choose?
                    // if (sin_theta > 0){ // left side of the robot
                        if (m_only_forward){
                            double fov = std::cos(m_fov * M_PI/180);
                            if (cos_theta>fov){ 
                                if (std::abs((theta - m_direction)) < (delta_theta)){ 
                                    double dis_goal = _distance(goal_segments[i].second.goal.x, 
                                                                goal_segments[i].second.goal.y, x, y);
                                    double goal_theta = std::asin(m_away_from_end_of_segment / m_radious) * 180 / M_PI;
                                    goal_updated = true;
                                    frontier = true;
                                    no_goal_found = false;

                                    if (dis_goal > 1 || too_long) {
                                        ROS_DEBUG_STREAM("[Goal] TOO LONG " << dis_goal);

                                        // only do it once
                                        if (!has_triggered){
                                            i = -1; // re-search
                                            has_triggered = true;
                                            too_long = true;
                                            ROS_INFO_STREAM("[Goal] Triggered snapping");
                                            continue;
                                        }
                                        this->goal_point.x =  this->robot_state.robot_pose.getOrigin().x() -
                                                              m_radious * std::cos((goal_segments[i].second.theta + 
                                                                                    goal_theta) * M_PI/180);
                                        this->goal_point.y =  this->robot_state.robot_pose.getOrigin().y() +
                                                              m_radious * std::sin((goal_segments[i].second.theta + 
                                                                                    goal_theta) * M_PI/180);
                                    }
                                    else {
                                        this->goal_point.x = x;
                                        this->goal_point.y = y;
                                    }
                                    delta_theta = std::abs(theta - m_direction);
                                    ROS_INFO_STREAM("[Goal] Theta selected: " << theta);
                                    ROS_DEBUG_STREAM("[Goal] Delta theta: " << delta_theta);
                                }
                            }
                        }
                        else{
                            if (std::abs((theta - m_direction)) < (delta_theta)){ 
                                no_goal_found = false;
                                this->goal_point.x = x;
                                this->goal_point.y = y;
                                delta_theta = std::abs(theta - m_direction);
                                goal_updated = true;
                                frontier = true;
                            }
                        }
                    // }
                }
            }

            // if no occupied space
            if (size == 0 && no_goal_found){
                for (int i=0; i<free_circle.size(); ++i){
                    double x = free_circle[i].x;
                    double y = free_circle[i].y;

                    geometry_msgs::Point32 vec;
                    vec.x = x - robot_state.robot_pose.getOrigin().x();
                    vec.y = y - robot_state.robot_pose.getOrigin().y();

                    float sin_theta = (robot_state.dir_velocity.v_x * vec.y - 
                                       robot_state.dir_velocity.v_y * vec.x) / 
                                       std::sqrt(std::pow(vec.x, 2) + std::pow(vec.y, 2));
                    float cos_theta = (robot_state.dir_velocity.v_x * vec.x + 
                                       robot_state.dir_velocity.v_y * vec.y) / 
                                       std::sqrt(std::pow(vec.x, 2) + std::pow(vec.y, 2)); 
                    if (cos_theta>0){
                        double theta = std::asin(sin_theta) * 180 / M_PI;
                        if (std::abs((theta)) < (delta_theta)){ 
                            delta_theta = std::abs(theta);
                            this->goal_point.x = x;
                            this->goal_point.y = y;
                            this->no_goal = false;
                            goal_updated = true;
                            frontier = true;
                        }
                    }
                }
            }

            if (size < 2) {
                ROS_WARN_STREAM("[Goal] Not enough segments: " << size);
                this->not_enough_segments = true;
            }

            if (no_goal_found){
                ROS_WARN_STREAM("[Goal] No goals found");
                this->no_goal = true;
            }
            else {
                this->no_goal = false;
            }
         }

        void frontiers(){
            ROS_INFO("Calculating frontiers...");
			float resolution = m_current_map->info.resolution;
			float map_x = m_current_map->info.origin.position.x / resolution;
			float map_y = m_current_map->info.origin.position.y / resolution;
			float x = robot_state.robot_pose.getOrigin().x() - map_x;
			float y = robot_state.robot_pose.getOrigin().y() - map_y;
			// x = robot_state.pose_in_cell.x;
			// y = robot_state.pose_in_cell.y;

			vector<vector<int> > frontiers = wfd(*m_current_map, m_current_map->info.height, 
                                                 m_current_map->info.width, x + (y * m_current_map->info.width), 
                                                 0, -1); // occupied, unknown

			int num_points = 0;
			for(int i = 0; i < frontiers.size(); i++) {
				for(int j = 0; j < frontiers[i].size(); j++) {
					num_points++;
				}
			}

			m_frontier_cloud.points.resize(num_points);
			int pointI = 0;
            int frontier_count = 0;
            float delta_theta = 1e3;
            float distance = -1e3;
			for(int i = 0; i < frontiers.size(); i++) {
				for(int j = 0; j < frontiers[i].size(); j++) {
					m_frontier_cloud.points[pointI].x = ((frontiers[i][j] % m_current_map->info.width) + map_x) * resolution;
					m_frontier_cloud.points[pointI].y = ((frontiers[i][j] / m_current_map->info.width) + map_y) * resolution;
					m_frontier_cloud.points[pointI].z = 0;
                    geometry_msgs::Point32 vec;
                    vec.x = m_frontier_cloud.points[pointI].x - robot_state.robot_pose.getOrigin().x();
                    vec.y = m_frontier_cloud.points[pointI].y - robot_state.robot_pose.getOrigin().y();

                    float sin_theta = (robot_state.dir_velocity.v_x * vec.y - 
                                       robot_state.dir_velocity.v_y * vec.x) / 
                                       std::sqrt(std::pow(vec.x, 2) + std::pow(vec.y, 2));
                    float cos_theta = (robot_state.dir_velocity.v_x * vec.x + 
                                       robot_state.dir_velocity.v_y * vec.y) / 
                                       std::sqrt(std::pow(vec.x, 2) + std::pow(vec.y, 2));
                    // ROS_DEBUG_STREAM("sin: " << sin_theta);
                    // ROS_DEBUG_STREAM("cos: " << cos_theta);
                    // if (sin_theta > 5 && sin_theta < 20 && cos_theta > 0){

                    // must at front
                    if (cos_theta>0){
                        float dis = std::sqrt(std::pow(robot_state.robot_pose.getOrigin().x() - 
                                                       m_frontier_cloud.points[pointI].x, 2) + 
                                              std::pow(robot_state.robot_pose.getOrigin().y() - 
                                                       m_frontier_cloud.points[pointI].y, 2));
                        
                        // find the one that is closest to m_direction with some tolerance (5 degree)
                        // find the one that is most farther with some tolerance (0.5 meter)
                        // if (std::abs(std::asin(sin_theta) * 180 / M_PI - m_direction) < (delta_theta + 5) && 
                        //     dis > (distance + 0.5)){ 
                        if (std::abs(std::asin(sin_theta) * 180 / M_PI - m_direction) < (delta_theta)){ 
                            this->goal_point.x = m_frontier_cloud.points[pointI].x;
                            this->goal_point.y = m_frontier_cloud.points[pointI].y;
                            goal_updated = true;
                            frontier = true;
                            delta_theta = std::asin(sin_theta) * 180 / M_PI;
                            distance = dis;
                        }
                    }
					pointI++;
				}
            }

            m_frontier_cloud.header.frame_id = "odom";
            frontier_publisher.publish(m_frontier_cloud);
            if (num_points==0) {
                ROS_WARN("NO frontier!");
            }
            else {
                ROS_INFO("published frontier, size: %d.", num_points);
            }


            ROS_INFO("Calculating side walk...");
            // calculating side walk
			vector<vector<int> > side_walk = wfd(*m_current_map, m_current_map->info.height, 
                                                 m_current_map->info.width, x + (y * m_current_map->info.width),
                                                 -1, 0);
			int num_side_walk_points = 0;
			for(int i = 0; i < side_walk.size(); i++) {
				for(int j = 0; j < side_walk[i].size(); j++) {
					num_side_walk_points++;
				}
			}
			m_side_walk_cloud.points.resize(num_side_walk_points);

			int side_walk_pointI = 0;
			for(int i = 0; i < side_walk.size(); i++) {
				for(int j = 0; j < side_walk[i].size(); j++) {
					m_side_walk_cloud.points[side_walk_pointI].x = ((side_walk[i][j] % m_current_map->info.width) + map_x) * resolution;
					m_side_walk_cloud.points[side_walk_pointI].y = ((side_walk[i][j] / m_current_map->info.width) + map_y) * resolution;
					m_side_walk_cloud.points[side_walk_pointI].z = 0;
					side_walk_pointI++;
				}
            }

            m_side_walk_cloud.header.frame_id = "odom";
            side_walk_publisher.publish(m_side_walk_cloud);
            if (num_side_walk_points==0) {
                ROS_WARN("NO side walks detected!");
            }
            else {
                ROS_INFO("published side walks, size: %d.", num_side_walk_points);
            }
            /*
            */
        }

        void updateMap(){
            m_lock.lock();
            if (m_map_queue.size()!=0){
                m_current_map = m_map_queue.front();
                m_map_queue.pop();
                map_updated = true;
                grid_to_costmap_ratio = 1 / (m_current_map->info.resolution);
            }
            else {
                map_updated = false;
            }
            m_lock.unlock();
        }

        bool checkReachGoal(){
            double distance = std::sqrt(
                    std::pow(this->robot_state.robot_pose.getOrigin().x() - this->goal_point.x, 2) + 
                    std::pow(this->robot_state.robot_pose.getOrigin().y() - this->goal_point.y, 2));

            return (distance < m_reached_goal_threshold) ? true : false;
        }

        template <class T>
        void newGoal(Dstar *dstar, T a, T b){
            dstar->updateGoal(a, b);
            // Mapper::updateGoal(a, b);
            // occupancy_grid_utils::Cell pose(robot_state.robot_pose, robot_state.robot_pose);
            // cout << "info: " << occupancy_grid_utils::cellIndex(m_current_map->info, test) << endl;
        }

        void updateDstarMap(Dstar *dstar){
            for (int i=0; i<boundaries.occupied_cell_vec.size(); ++i){
                dstar->updateCell((int) std::round(grid_to_costmap_ratio*boundaries.occupied_cell_vec[i].x), 
                                  (int) std::round(grid_to_costmap_ratio*boundaries.occupied_cell_vec[i].y),
                                  occupied_cell_cost);
            }

            for (int i=0; i<boundaries.free_cell_vec.size(); ++i){
                dstar->updateCell((int) std::round(grid_to_costmap_ratio*boundaries.free_cell_vec[i].x), 
                                  (int) std::round(grid_to_costmap_ratio*boundaries.free_cell_vec[i].y),
                                  grid_to_costmap_ratio*boundaries.free_cell_vec[i].cost);
            }

            for (int i=0; i<boundaries.unknown_cell_vec.size(); ++i){
                dstar->updateCell((int) std::round(grid_to_costmap_ratio*boundaries.unknown_cell_vec[i].x), 
                                  (int) std::round(grid_to_costmap_ratio*boundaries.unknown_cell_vec[i].y),
                                  unknown_cell_cost);
            }
        }


        PublishTypes_t publishPath(std::vector<state> path){
            if (path.size()==0){
                PublishTypes_t publish_string;
                publish_string.s_waypoints_udp = "";
                return publish_string;
            }
            
            std::string s_robot_pose = dstar_utils::toStringWithPrecision(robot_state.robot_pose.getOrigin().x(), 3) + 
                                       std::string(", ") + 
                                       dstar_utils::toStringWithPrecision(robot_state.robot_pose.getOrigin().y(), 3) + 
                                       std::string(", ") +
                                       dstar_utils::toStringWithPrecision(robot_state.robot_pose.getOrigin().z(), 3) + 
                                       std::string(", ") +
                                       dstar_utils::toStringWithPrecision(robot_state.robot_pose.getRotation().w(), 3) + 
                                       std::string(", ") +
                                       dstar_utils::toStringWithPrecision(robot_state.robot_pose.getRotation().x(), 3) + 
                                       std::string(", ") +
                                       dstar_utils::toStringWithPrecision(robot_state.robot_pose.getRotation().y(), 3) + 
                                       std::string(", ") +
                                       dstar_utils::toStringWithPrecision(robot_state.robot_pose.getRotation().z(), 3) +
                                       std::string(", ");
            waypoint_data_t waypoints_udp;
            memset(&waypoints_udp, 0, sizeof(waypoint_data_t));

            // if (!no_path && !no_free_cell && !no_goal){
            //     waypoints_udp.controlsignal[0] = m_control_signals.normal;
            // }
            // else {
            //     waypoints_udp.controlsignal[0] = 1;
            // }

            // waypoints_udp.controlsignal[1] = 0; 
            // waypoints_udp.controlsignal[2] = 0;
            // waypoints_udp.controlsignal[3] = 0;
            // waypoints_udp.controlsignal[4] = 0;
            // waypoints_udp.controlsignal[5] = 0;
            // waypoints_udp.controlsignal[6] = 0;
             
            // waypoints_udp.pose[0] = std::stod(dstar_utils::toStringWithPrecision(robot_state.robot_pose.getOrigin().x(), 3));
            // waypoints_udp.pose[1] = std::stod(dstar_utils::toStringWithPrecision(robot_state.robot_pose.getOrigin().y(), 3));
            // waypoints_udp.pose[2] = std::stod(dstar_utils::toStringWithPrecision(robot_state.robot_pose.getOrigin().z(), 3));
            // waypoints_udp.pose[3] = std::stod(dstar_utils::toStringWithPrecision(robot_state.robot_pose.getRotation().w(), 3)); 
            // waypoints_udp.pose[4] = std::stod(dstar_utils::toStringWithPrecision(robot_state.robot_pose.getRotation().x(), 3)); 
            // waypoints_udp.pose[5] = std::stod(dstar_utils::toStringWithPrecision(robot_state.robot_pose.getRotation().y(), 3)); 
            // waypoints_udp.pose[6] = std::stod(dstar_utils::toStringWithPrecision(robot_state.robot_pose.getRotation().z(), 3));
            
            waypoints_udp.pose[1] =0; 
            waypoints_udp.pose[2] =0; 
            waypoints_udp.pose[3] =0; 
            waypoints_udp.pose[4] =0; 
            waypoints_udp.pose[5] =0; 
            waypoints_udp.pose[6] =0; 
            ROS_DEBUG_STREAM ("[wp] robot x: " << std::stod(dstar_utils::toStringWithPrecision(robot_state.robot_pose.getOrigin().x(), 3)));
            ROS_DEBUG_STREAM ("[wp] robot y: " << std::stod(dstar_utils::toStringWithPrecision(robot_state.robot_pose.getOrigin().y(), 3)));

            // if (!this->no_path && !this->no_free_cell && !this->no_goal){
            if (this->no_goal){
                waypoints_udp.pose[0] = m_control_signals.backward;
                ROS_DEBUG_STREAM("[UDP] Backward: " << waypoints_udp.pose[0]);
            }
            else {
                waypoints_udp.pose[0] = m_control_signals.normal;;
                ROS_DEBUG_STREAM("[UDP] Normal: " << waypoints_udp.pose[0]);
            }
            
            std::string s_waypoints = "";
            int count = 0;
            int waypoint_count = 0;
            for (int i=0; i<path.size(); ++i){
                // if too many waypoints
                if ((i % ((int) std::round(waypoint_distance * grid_to_costmap_ratio))) == 0){
                // if ((i % m_waypoint_index) == 0){
                    // if (path[i].x==0 && path[i].y==0){
                    //     continue;
                    // }
                    // ROS_DEBUG_STREAM ("[wp] i: " << i);
                    ROS_DEBUG_STREAM ("[wp] waypoint_count: " << waypoint_count);
                    waypoints_udp.waypoints[count] = std::stod(dstar_utils::toStringWithPrecision(path[i].x/grid_to_costmap_ratio, 3));
                    // ROS_DEBUG_STREAM ("[wp] x_orig: " << path[i].x/grid_to_costmap_ratio);
                    // ROS_DEBUG_STREAM ("[wp] x_orig_precision: " << dstar_utils::toStringWithPrecision(path[i].x/grid_to_costmap_ratio, 3));
                    // ROS_DEBUG_STREAM ("[wp] x_orig_precision_std: " << std::stod(dstar_utils::toStringWithPrecision(path[i].x/grid_to_costmap_ratio, 3)));

                    ROS_DEBUG_STREAM ("[wp] x: " << waypoints_udp.waypoints[count]);
                    count ++;


                    // ROS_DEBUG_STREAM ("[wp] y_orig: " << path[i].y/grid_to_costmap_ratio);
                    // ROS_DEBUG_STREAM ("[wp] y_orig_precision: " << dstar_utils::toStringWithPrecision(path[i].y/grid_to_costmap_ratio, 3));
                    // ROS_DEBUG_STREAM ("[wp] y_orig_precision_std: " << std::stod(dstar_utils::toStringWithPrecision(path[i].y/grid_to_costmap_ratio, 3)));
                    // ROS_DEBUG_STREAM ("[wp] y: " << waypoints_udp.waypoints[count]);
                    waypoints_udp.waypoints[count] = std::stod(dstar_utils::toStringWithPrecision(path[i].y/grid_to_costmap_ratio, 3));
                    ROS_DEBUG_STREAM ("[wp] y: " << waypoints_udp.waypoints[count]);
                    count ++;
                    waypoint_count ++;
                    // ROS_DEBUG_STREAM ("count: " << count);
                }
                else if (i == path.size()-1){
                    // ROS_DEBUG_STREAM ("[wp] i: " << i);
                    // ROS_DEBUG_STREAM ("[wp] waypoint_count: " << waypoint_count);
                    waypoints_udp.waypoints[count] = std::stod(dstar_utils::toStringWithPrecision(path[i].x/grid_to_costmap_ratio, 3));
                    // ROS_DEBUG_STREAM ("[wp] x_orig: " << path[i].x/grid_to_costmap_ratio);
                    // ROS_DEBUG_STREAM ("[wp] x_orig_precision: " << dstar_utils::toStringWithPrecision(path[i].x/grid_to_costmap_ratio, 3));
                    // ROS_DEBUG_STREAM ("[wp] x_orig_precision_std: " << std::stod(dstar_utils::toStringWithPrecision(path[i].x/grid_to_costmap_ratio, 3)));
                    ROS_DEBUG_STREAM ("[wp] x: " << waypoints_udp.waypoints[count]);
                    count ++;
                    waypoints_udp.waypoints[count] = std::stod(dstar_utils::toStringWithPrecision(path[i].y/grid_to_costmap_ratio, 3));
                    // ROS_DEBUG_STREAM ("[wp] y_orig: " << path[i].y/grid_to_costmap_ratio);
                    // ROS_DEBUG_STREAM ("[wp] y_orig_precision: " << dstar_utils::toStringWithPrecision(path[i].y/grid_to_costmap_ratio, 3));
                    // ROS_DEBUG_STREAM ("[wp] y_orig_precision_std: " << std::stod(dstar_utils::toStringWithPrecision(path[i].y/grid_to_costmap_ratio, 3)));
                    ROS_DEBUG_STREAM ("[wp] y: " << waypoints_udp.waypoints[count]);
                    count ++;
                    waypoint_count ++;
                }

                // if(!_checkWaypointsWithRobot(path[i])){
                        // if(!_checkWaypointsWithRobot(path[i])){
                std::string s_waypoint = dstar_utils::toStringWithPrecision(path[i].x/grid_to_costmap_ratio, 3) + 
                                         std::string(", ") + 
                                         dstar_utils::toStringWithPrecision(path[i].y/grid_to_costmap_ratio, 3) + 
                                         std::string(", ");
                s_waypoints = s_waypoints + s_waypoint;

                if (waypoint_count > m_num_waypoints){
                    break;
                }
                    // }
                // }
            }
            std::string msgs = s_robot_pose + s_waypoints;

            PublishTypes_t publish_string;
            publish_string.waypoints_udp = waypoints_udp;
            publish_string.s_waypoints_udp = msgs;

            // ROS_DEBUG_STREAM ("robot_pose: " << s_robot_pose);
            // ROS_DEBUG_STREAM ("way_points: " << s_waypoints);
            // ROS_DEBUG_STREAM ("msgs: " << msgs);


            return publish_string;
            // markerArray_pub.publish(obsMarkers);
        }

		void print(){
			cout << "---------------- printing ----------------" << endl;
		}

    private:
		// ROS handle
        ros::NodeHandle m_nh;

		// publisher/subscriber
        ros::Subscriber m_map_sub;

        // subscribe channel
        std::string m_map_frame;
        std::string m_cassie_frame;
        std::string m_costmap_topic;

		// tf
		tf::TransformBroadcaster m_broadcaster_;
        tf::TransformListener m_listener;

        // data
        bool m_data_received;
        bool m_goal_received;
        boost::mutex m_lock;
        std::queue<nav_msgs::OccupancyGrid::ConstPtr> m_map_queue;

        // map
        nav_msgs::OccupancyGrid::ConstPtr m_current_map;

        // counter 
        int m_look_up_counter;
        int m_expend_searching_counter;
        int m_goal_counter;
        bool m_log;
        std::fstream m_file;

        /***** parameters *****/
        // thresholds
        double m_reached_goal_threshold;
        double m_num_waypoints;
        double m_queue_size;

        // how to find the goal?
        int m_goal_finder;
        int m_goal_finder_change_count;  // 
        int m_goal_finder_count_threshold; // if change several times, then change!
        int m_goal_finder_old;
        int m_goal_finder_1;
        int m_goal_finder_2;
        int m_goal_finder_3;
        int m_goal_finder_4;

        bool m_auto_goal_finder;
        double m_away_from_occupied;

        // goal
        double m_ave_goal_distance; // average distance between left and right goal 
        double m_current_goal_distance; // current distance between left and right goal 
        double m_fork_distance;

        // searching for free space
        int m_order;
        int m_searching_dim;
        int m_searching_dim_old;
        int m_expend_searching_dim_amount;
        int m_expend_searching_dim_amount_bound;
        double m_blind_horizon;
        double m_direction;
        double m_goal_distance;
        double m_goal_distance_threshold;

        sensor_msgs::PointCloud m_frontier_cloud;
        sensor_msgs::PointCloud m_side_walk_cloud;
        std::vector<vector<geometry_msgs::Point32>> m_frontier_cloud_split;

        // goal segment
        int m_min_free;
        double m_circle_resolution;
        double m_away_from_end_of_segment;
        int m_expend_radious_counter;
        double m_expend_radious_amount;
        int m_expend_radious_amount_bound;
        double m_radious;
        double m_radious_old;
        double m_fov;
        double m_right_fov;
        bool m_only_forward;

        // waypoints
        int m_waypoint_index;

        // control signal
        ControlSignal_t m_control_signals;

        // exist or not 
        std::pair<std::unordered_set<occupancy_grid_utils::Cell, CellHasher_t, CellComparator_t>::iterator, 
                  bool> m_insert_result;

        /***** functions *****/

        void _robot_velocity(){
            tf::Quaternion q(robot_state.robot_pose.getRotation().x(),
                             robot_state.robot_pose.getRotation().y(),
                             robot_state.robot_pose.getRotation().z(),
                             robot_state.robot_pose.getRotation().w());
            tf::Matrix3x3 m(q);
            m.getRPY(robot_state.roll, robot_state.pitch, robot_state.yaw);
            // robot_state.roll = robot_state.roll * 180/M_PI;
            // robot_state.pitch = robot_state.pitch * 180/M_PI;
            // robot_state.yaw =  robot_state.yaw * 180/M_PI;
            ROS_DEBUG_STREAM("[Robot] (r,p,y) = "<< "("<< robot_state.roll * 180/M_PI << ", " << 
                                                  robot_state.pitch * 180/M_PI << ", " << 
                                                  robot_state.yaw * 180/M_PI << ")");

            // direction velocity
            robot_state.dir_velocity.v_x = std::cos(robot_state.yaw);
            robot_state.dir_velocity.v_y = std::sin(robot_state.yaw);


            // real velocity
            double duration = robot_state.time.toSec() - robot_state_old.time.toSec();
            robot_state.velocity.v_x = (robot_state.robot_pose.getOrigin().x() - 
                                        robot_state_old.robot_pose.getOrigin().x())/duration;
            robot_state.velocity.v_y = (robot_state.robot_pose.getOrigin().y() - 
                                        robot_state_old.robot_pose.getOrigin().y())/duration;
            ROS_DEBUG_STREAM("[Robot] Dir_vx: " << robot_state.dir_velocity.v_x << ", dir_vy: " << robot_state.dir_velocity.v_y);
            robot_state_old.time = robot_state.time;
        }

        template <class T>
        bool _checkWaypointsWithRobot(const T &point){
            double distance = std::sqrt(
                    std::pow(this->robot_state.robot_pose.getOrigin().x() - point.x, 2) + 
                    std::pow(this->robot_state.robot_pose.getOrigin().y() - point.y, 2));

            return (distance < waypoint_distance) ? true : false;
        }

        template <class T>
        double _distance(T x1, T y1, T x2, T y2){
            return (double) std::sqrt(std::pow(x1 -x2, 2) + std::pow(y1 - y2, 2));
        }

        bool _checkGoalWithRobot(){
            double distance = std::sqrt(
                                std::pow(this->robot_state.robot_pose.getOrigin().x() - this->goal_point.x, 2) + 
                                std::pow(this->robot_state.robot_pose.getOrigin().y() - this->goal_point.y, 2));

            return (distance < m_goal_distance_threshold) ? true : false;
        }

        bool _checkWaypoints(const state &point1, const state &point2){
            double distance = std::sqrt(
                    std::pow(point1.x - point2.x, 2) + 
                    std::pow(point1.y - point2.y, 2));

            return (distance < waypoint_distance) ? true : false;
        }

        void _octomapCallBack(const nav_msgs::OccupancyGrid::ConstPtr& msg){
            m_data_received = true;
            m_lock.lock();
            if (m_map_queue.size() > m_queue_size) {
                Mapper::_clear_queue(m_map_queue);
                ROS_WARN("[Callback] Map queue reset!"); 
            }
            m_map_queue.push(msg);
            ROS_INFO_STREAM("[Callback] queue size: " << m_map_queue.size()); 
            m_lock.unlock();
        }

        void _clickPointCallBack(const geometry_msgs::PointStamped::ConstPtr& msg){
            m_goal_received = true;
            goal_updated = true;
            this->goal_point.x = msg->point.x * grid_to_costmap_ratio;
            this->goal_point.y = msg->point.y * grid_to_costmap_ratio;
            // m_lock.lock();
            // m_goal_queue.push(msg);
            // cout << "queue size: " << m_map_queue.size() << endl; 
            // m_lock.unlock();
        }

        void _waitForData() {
            if (method==0 || method==1){
                while (ros::ok() && !m_data_received){
                    sleep(0.5);
                }
            }
            else if (method==2){ // need to check if goal clicked 
                while (ros::ok() && (!m_data_received || !m_goal_received)){
                    // cout << "-------" << endl;
                    // cout << "1:" << m_data_received << endl;
                    // cout << "2:" << m_goal_received << endl;
                    sleep(0.5);
                }
            }
        }

        void _spin(){
            while (ros::ok()){
                ros::spinOnce();
            }
        }

        template <class T>
        void _clear_queue(std::queue<T> &q ) {
            std::queue<T> empty;
            std::swap( q, empty );
        }


        // in world frame
        template <class T>
        bool _freeCell(T x, T y){
            occupancy_grid_utils::Cell cell = _worldToMap(x, y); // to map
            return (m_current_map->data[cell.index] > 0);
        }

        // distance of robot pose and given index of occupancy grid
        float _getDistance(const int &index){
            int x = occupancy_grid_utils::indexCell(m_current_map->info, index).x;
            int y = occupancy_grid_utils::indexCell(m_current_map->info, index).y;

            return m_current_map->info.resolution*(std::sqrt(
                               std::pow((robot_state.pose_in_cell.x-x), 2) + 
                               std::pow((robot_state.pose_in_cell.y-y), 2))
                   );
        }

        geometry_msgs::Point32 _mapToWorld(occupancy_grid_utils::Cell cell){
            geometry_msgs::Point32 point;
            point.x = cell.x * m_current_map->info.resolution + 
                      m_current_map->info.origin.position.x;
            point.y = cell.y * m_current_map->info.resolution + 
                      m_current_map->info.origin.position.y;
            return point;
        }

        template <class T>
        occupancy_grid_utils::Cell _worldToMap(T x, T y){
            occupancy_grid_utils::Cell cell(std::round((x - m_current_map->info.origin.position.x)/m_current_map->info.resolution), 
                                            std::round((y - m_current_map->info.origin.position.y)/m_current_map->info.resolution));
            cell.index = occupancy_grid_utils::cellIndex(m_current_map->info, cell);

            return cell;
        }

        template<std::size_t SIZE>
        void _getNeighbors(std::array<int, SIZE> &n_array, int position, int map_width) {

            if (n_array.size()==8){
                Mapper::_get8Neighbors(n_array, position, map_width);
            }
            else if (n_array.size()==24){
                Mapper::_getBigNeighbors(n_array, position, map_width);
            }
            else {
                ROS_FATAL("WRONG number of neighbor");
            }
        }

        template<std::size_t SIZE>
        void _get8Neighbors(std::array<int, SIZE> &n_array, int position, int map_width) {
                n_array[0] = position - map_width - 1;
                n_array[1] = position - map_width; 
                n_array[2] = position - map_width + 1; 
                n_array[3] = position - 1;
                n_array[4] = position + 1;
                n_array[5] = position + map_width - 1;
                n_array[6] = position + map_width;
                n_array[7] = position + map_width + 1;
        }

        template<std::size_t SIZE>
        void _getBigNeighbors(std::array<int, SIZE> &n_array, int position, int map_width) {
            n_array[0] = position - map_width - 1;
            n_array[1] = position - map_width; 
            n_array[2] = position - map_width + 1; 
            n_array[3] = position - 1;
            n_array[4] = position + 1;
            n_array[5] = position + map_width - 1;
            n_array[6] = position + map_width;
            n_array[7] = position + map_width + 1;

            n_array[8] = position - (map_width * 2) - 2;
            n_array[9] = position - (map_width * 2) - 1; 
            n_array[10] = position - (map_width * 2); 
            n_array[11] = position - (map_width * 2) + 1;
            n_array[12] = position - (map_width * 2) + 2;
            n_array[13] = position - 2;
            n_array[14] = position + 2;
            n_array[15] = position + (map_width * 2) - 2;
            n_array[16] = position + (map_width * 2) - 1; 
            n_array[17] = position + (map_width * 2); 
            n_array[18] = position + (map_width * 2) + 1;
            n_array[19] = position + (map_width * 2) + 2;
            n_array[20] = position + (map_width) + 2;
            n_array[21] = position + (map_width) - 2;
            n_array[22] = position - (map_width) + 2;
            n_array[23] = position - (map_width) - 2;
        }


        // return true if all free
        template<std::size_t SIZE>
        bool _neighborsFree(std::array<int, SIZE> &n_array){
            for (int i=0; i<n_array.size(); ++i){
                if (n_array[i] >= 0){
                    if (m_current_map->data[n_array[i]] <= 0) return false;
                }
            }
            return true;
        }

        // return true if all occupied
        template<std::size_t SIZE>
        bool _neighborsOccupied(std::array<int, SIZE> &n_array){
            for (int i=0; i<n_array.size(); ++i){
                if (n_array[i] >= 0){
                    if (m_current_map->data[n_array[i]] > 0) return false;
                }
            }
            return true;
        }


		void m_nineCellUpdate(int x, int y, int value, Dstar *dstar){
			dstar->updateCell(grid_to_costmap_ratio*x,   grid_to_costmap_ratio*y, value);
			dstar->updateCell(grid_to_costmap_ratio*x-1, grid_to_costmap_ratio*y, value);
			dstar->updateCell(grid_to_costmap_ratio*x+1, grid_to_costmap_ratio*y, value);
			dstar->updateCell(grid_to_costmap_ratio*x,   grid_to_costmap_ratio*y+1, value);
			dstar->updateCell(grid_to_costmap_ratio*x,   grid_to_costmap_ratio*y-1, value);
			dstar->updateCell(grid_to_costmap_ratio*x-1, grid_to_costmap_ratio*y-1, value);
			dstar->updateCell(grid_to_costmap_ratio*x+1, grid_to_costmap_ratio*y+1, value);
			dstar->updateCell(grid_to_costmap_ratio*x-1, grid_to_costmap_ratio*y+1, value);
			dstar->updateCell(grid_to_costmap_ratio*x+1, grid_to_costmap_ratio*y-1, value);
		}

        void m_getParameters(){
            // methods
			bool got_method = ros::param::get("method", method);
			bool got_pub_free = ros::param::get("pub_free", pub_free);
			bool got_pub_occupied = ros::param::get("pub_occupied", pub_occupied);
			bool got_pub_unknown = ros::param::get("pub_unknown", pub_unknown);
			bool got_pub_all_cells = ros::param::get("pub_all_cells", pub_all_cells);
			bool got_direction = ros::param::get("direction", m_direction);
			bool got_goal_distance = ros::param::get("goal_distance", m_goal_distance);
			bool got_fork_distance = ros::param::get("fork_distance", m_fork_distance);

            // goal segment
			bool got_radious_expend_amount = ros::param::get("expend_radious_amount", m_expend_radious_amount);
			bool got_radious_expend_amount_bound = ros::param::get("expend_radious_amount_bound",
                                                                    m_expend_radious_amount_bound);
			bool got_radious = ros::param::get("radious", m_radious);
			bool got_radious_old = ros::param::get("radious", m_radious_old);
			bool got_min_free = ros::param::get("min_free", m_min_free);
			bool got_circle_resolution = ros::param::get("circle_resolution", m_circle_resolution);
			bool got_away_from_end_of_segment = ros::param::get("away_from_end_of_segment", m_away_from_end_of_segment);
			bool got_only_forward = ros::param::get("only_forward", m_only_forward);
			bool got_fov = ros::param::get("fov", m_fov);
			bool got_right_fov = ros::param::get("right_fov", m_right_fov);

            // goal finder
			bool got_goal_finder = ros::param::get("goal_finder", m_goal_finder);
			bool got_goal_finder_old = ros::param::get("goal_finder", m_goal_finder_old);
			bool got_goal_finder_count_threshold = ros::param::get("goal_finder_count_threshold", 
                                                                    m_goal_finder_count_threshold);
			bool got_auto_goal_finder = ros::param::get("auto_goal_finder", m_auto_goal_finder);
			bool got_away_from_occupied = ros::param::get("away_from_occupied", m_away_from_occupied);

			bool got_goal_finder_1 = ros::param::get("goal_finder_1", m_goal_finder_1);
			bool got_goal_finder_2 = ros::param::get("goal_finder_2", m_goal_finder_2);
			bool got_goal_finder_3 = ros::param::get("goal_finder_3", m_goal_finder_3);
			bool got_goal_finder_4 = ros::param::get("goal_finder_4", m_goal_finder_4);

            // parameters 
			bool got_map_frame = ros::param::get("map_frame", m_map_frame);
			bool got_cassie_frame = ros::param::get("cassie_frame", m_cassie_frame);
			bool got_costmap_topic = ros::param::get("costmap_topic", m_costmap_topic);
			bool got_queue_size = ros::param::get("queue_size", m_queue_size);

			bool got_dstar_max_step = ros::param::get("dstar_max_step", dstar_max_step);
			bool got_dstar_unknow_cell_cost = ros::param::get("dstar_unknow_cell_cost", dstar_unknow_cell_cost);

			bool got_occupied_cell = ros::param::get("occupied_cell", occupied_cell);
			bool got_unknown_cell = ros::param::get("unknown_cell", unknown_cell);
			bool got_unknown_cell_cost = ros::param::get("unknown_cell_cost", unknown_cell_cost);
			bool got_occupied_cell_cost = ros::param::get("occupied_cell_cost", occupied_cell_cost);

            bool got_order = ros::param::get("order", m_order);
            bool got_seraching_dim = ros::param::get("searching_dim", m_searching_dim);
            bool got_seraching_dim_old = ros::param::get("searching_dim", m_searching_dim_old); // assign for the first time
            bool got_expend_seraching_dim = ros::param::get("expend_searching_dim_amount", m_expend_searching_dim_amount);
            bool got_expend_seraching_dim_bound = ros::param::get("expend_searching_dim_bound", m_expend_searching_dim_amount_bound);
            bool got_reached_goal_threshold = ros::param::get("reached_goal_threshold", m_reached_goal_threshold);
            bool got_num_waypoints = ros::param::get("num_waypoints", m_num_waypoints);
            bool got_skip_distance = ros::param::get("waypoint_distance", waypoint_distance);
            bool got_skip_index = ros::param::get("waypoint_index", m_waypoint_index);
            bool got_replannning_timeout = ros::param::get("replannning_timeout", replannning_timeout);
            bool got_update_time = ros::param::get("update_time", update_time);
            bool got_planning_horizon = ros::param::get("blind_horizon", m_blind_horizon);
            



			// if (!utils::checkParameters(3, got_method, got_method, got_cassie_frame)){
			//     ROS_INFO("using hard-coded parameters");
			//     // TODO: check completeness 
			//     sleep(3);
			// }
        }
}; // class
} // namespace
#endif
