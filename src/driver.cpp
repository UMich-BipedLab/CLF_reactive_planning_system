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
#include <pcl_conversions/pcl_conversions.h>


#define BOOST_BIND_NO_PLACEHOLDERS
#include <boost/bind/bind.hpp>


#include <unistd.h>

#include "driver.h"
#include "lie_group.h"
#include "utils/plotting.h"
#include "utils/utils.h"
#include "utils/ros_utils.h"

#include "point.h"


#include <Eigen/Geometry>



namespace bipedlab
{
    
Driver::Driver(ros::NodeHandle& nh):
    data_received_(false), goal_received_(false),
    global_map_updated_(false), pose_updated_(false), goal_updated_(false), 
    pose_look_up_counter_(0),
    found_path_(true)
{
    nh_ = nh;
    // ros::Rate r(10); 
    if (!Driver::getParameters_())
    {
        debugger::debugTitleTextOutput("[Driver]", "NOTICE!!!!", 10, BR, BOLD);
        debugger::debugColorOutput("[Driver] Not enough parameters: ", 
                                   "Using default values", 10, BR, BOLD);
        debugger::debugTitleTextOutput("[Driver]", "", 10, BR, BOLD);
        utils::pressEnterToContinue();
    }
    else
    {
        debugger::debugColorOutput("[Driver] Received all parameters", "", 10, BC);

    }

    debugger::debugColorOutput("[Driver] Using EIGEN_WORLD_VERSION: ", EIGEN_WORLD_VERSION, 10, BC);
    debugger::debugColorOutput("[Driver] Using EIGEN_MAJOR_VERSION: ", EIGEN_MAJOR_VERSION, 10, BC);
    debugger::debugColorOutput("[Driver] Using EIGEN_MINOR_VERSION: ", EIGEN_MINOR_VERSION, 10, BC);

    if (log_commands_ == 1)
    {
        try
        {
            std::string date = utils::getTimeNDate();
            std::string path = utils::getCurrentDirectory();

            command_csv_.open(std::string(path) + "/" + date + "_command_history.csv"); // throws exceptions!

            // Header
            command_csv_ << "behavior" << "velocity_x" << "velocity_y" << "omega" 
                << "velocity_r" << "velocity_theta"
                << "roll" << "pitch"  << "yaw" << endrow;
        }
        catch (const std::exception& ex)
        {
            std::cout << "Exception was thrown: " << ex.what() << std::endl;
        }
    }

    if (map_number_ == 0)
    {
        is_exploration_ = 0;
    }
    else if (map_number_ > 0)
    {
        is_exploration_ = 0;
        Driver::getGoalPoseForMap_(map_number_);
    }
    else if (map_number_ == -1)
    {
        if (goal_searching_.goal_search_mode != 2)
        {
            std::string error_msg = "ERROR: Choosing exploration (map_number = "  
                +std::to_string(map_number_) +") "
                "but goal_search_mode is not 2: "
                + std::to_string(goal_searching_.goal_search_mode) +
                "!! \nConsider changing: \n1) map_number >= 0, or \n"
                "2) goal_search_mode = 2";
            debugger::debugExitColor(error_msg, __LINE__, __FILE__);
        }

        debugger::debugTitleTextOutput("[Driver]", "NOTICE!!!!", 10, BR, BOLD);
        debugger::debugColorOutput("[Driver] Turn on Exploration Mode", 
                "", 10, BR, BOLD);
        debugger::debugTitleTextOutput("[Driver]", "ARE YOU SURE?", 10, BR, BOLD);
        utils::pressEnterToContinue();
        is_exploration_ = 1;
    }
    else
    {
        std::string error_msg = "ERROR: No such map_number_ supported: "
            + std::to_string(map_number_);
        debugger::debugExitColor(error_msg, __LINE__, __FILE__);
    }




    // subscribers
    map_sub_ = nh_.subscribe(std::string(multi_layer_map_topic_), 1, 
                               &Driver::getMultiLayerCallBack_, this);
    inekf_sub_ = nh_.subscribe(std::string(inekf_topic_), 1, 
                               &Driver::getInEKFCallBack_, this);
    click_point_sub_ = nh_.subscribe("/clicked_point", 1, 
                                     &Driver::getClickPointCallBack_, this);
    debugger::debugOutput("[Driver] subscribed to: ", multi_layer_map_topic_, 10);
    debugger::debugOutput("[Driver] subscribed to: ", inekf_topic_, 10);


    // publishers
    global_map_pub_ = nh_.advertise<grid_map_msgs::GridMap>("world_map", 1, true);
    local_map_pub_ = nh_.advertise<grid_map_msgs::GridMap>("local_map", 1, true);
    rrt_path_pub_ = 
        nh_.advertise<pcl::PointCloud<pcl::PointXYZI>> ("rrt_path_points", 1);
    planner_path_pub_ = 
        nh_.advertise<pcl::PointCloud<pcl::PointXYZI>> ("planner_path_points", 1);
    planner_command_pub_ = nh_.advertise<planner_msgs::State> ("planner_commands", 1);

    // rviz tool
    visual_tools_.reset(new rviz_visual_tools::RvizVisualTools(map_frame_,"/rviz_visual_markers"));
    visual_tools_->loadMarkerPub();  // create publisher before waiting



    // planner_terrain_pub_ = nh_.advertise<planner_msgs::TerrainArray> ("planner_terrain", 1);

    marker_pub_ = nh_.advertise<visualization_msgs::Marker>(
            "visualization_marker", 10);
    marker_array_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(
            "MarkerArray", 10);
    trajectory_pub_ = nh_.advertise<nav_msgs::Path>("robot_trajectory", 1, true);


    // frontier_publisher_ = m_nh.advertise<sensor_msgs::PointCloud>("frontiers", 1);
    // side_walk_publisher_ = m_nh.advertise<sensor_msgs::PointCloud>("side_walk", 1);
    boost::thread ros_spin(&Driver::spin_, this);

    if (driver_method_ == 0 || driver_method_ == 2 || driver_method_ == 3)
    {
        ROS_INFO("waiting for data...");
    }
    else if (driver_method_ == 1)
    {
        ROS_INFO("waiting for data and a goal...");
    }
    else
    {
        debugger::debugColorOutput("[Driver]/[waitForData] "
                "unknown driver_method_: ", driver_method_, 10);
        exit(-1);
    }



    // create UDP in case of driver_method_ == 3
    udp_ = new Communication(udp_info_);
    boost::thread listen_to_root(&Communication::createListener_, &*udp_);

    // wait for data arriving
    Driver::waitForData_();

    Driver::updateRobotPose_();
    Driver::updateGlobalMap_();
    // for (auto name : multi_layer_map_.getLayers())
    //     std::cout << "layer: " << name << std::endl;
    // std::cout << "[In driver.cpp before Planner()] &multi_layer_map_ = " << &multi_layer_map_ << std::endl;
    planner_ = new CLFRRTStarPlanner(multi_layer_map_, local_map_params_,
            robot_state_.pose, robot_state_.pose, robot_state_, 
            pose_sampler_params_, rrt_params_, cost_map_params_, lyap_dist_params_,
            terrain_plane_params_);
    // std::cout << "[In driver.cpp after Planner()] planner_->printGlobalMapCLFAddress():" << std::endl;
    // planner_->printGlobalMapCLFAddress();
    // std::cout << std::endl;


    // keep publishing where to go given a map
    boost::thread publish_to_robot(&Driver::publishInfoToRobot_, this);

    // std::cout << "===== [In driver.cpp before driveRobot_()]\n";
    Driver::driveRobot_();
}

int Driver::driveRobot_()
{
    ros::Rate replanning_rate(replanning_rate_); 
    while(ros::ok())
    {
        debugger::debugTitleTextOutput("[Driver]", "driveRobot", 10, G);
        
        // update the robot pose in the planner
        debugger::debugTextOutput("[Driver]/[driveRobot] updateRobotPose()", 6);
        if (!Driver::updateRobotPose_())
        {
            // TODO: publish standing still commmand
             
            ros::Duration(1.0).sleep();
            continue;
        }

        // update the global map in the planner
        debugger::debugTextOutput("[Driver]/[driveRobot] updateGlobalMap()", 6);
        bool if_global_map_updated = Driver::updateGlobalMap_();


        debugger::debugTextOutput("[Driver]/[driveRobot] Update the start pose " 
                                  "in CLFRRTStarPlanner", 6);
        // debugger::debugOutput("[Driver]/[driveRobot] Current pose: ",
        //                       robot_state_.pose, 5);
        planner_->setStartPose(robot_state_.pose);



        // using the current pose and the map to update the locap map
        debugger::debugTextOutput("[Driver]/[driveRobot] updateLocalMap()", 6);
        planner_->updateLocalMap();

        // check fork




        // search a goal
        // debugger::debugTextOutput("[Driver]/[driveRobot] " 
        //                           "decideGoalForPlanner()", 5);
        Driver::decideSubGoalForPlanner_();
        // sub_goal_pose_ = pose_t(-5, -5, 0);



        // run rrt
        debugger::debugOutput("[Driver]/[driveRobot] Running CLFRRT...", "", 6);
        planner_->setGoalPose(sub_goal_pose_);

        bool found_path = false; // changed in planPathFromRRT
        bool improved_path = false; // changed in planPathFromRRT
        bool if_timeout = false; // changed in planPathFromRRT
        Driver::planPathFromRRT(if_global_map_updated, 
                                found_path, improved_path, if_timeout);


        // print results
        Driver::printResultsFromRRT(improved_path);


        // update planner results
        Driver::updatePlannerResults_(found_path, improved_path, if_timeout);

        // preparing path for publishing
        // planner_info_to_controller_ = Driver::prepareInfoWithMPCForRobot();


        // publish to robot
        // debugger::debugTextOutput("[Driver]/[driveRobot] "
        //                           "Sending data to robot...", 5);
        // Driver::publishInfoToRobot(planner_info_to_controller_);

        Driver::publishToRviz_();
        debugger::debugTextOutput("[Driver]/[driveRobot] Published all", 10);
        replanning_rate.sleep();
        // utils::pressEnterToContinue();
        // usleep(5e6);
        // debugger::debugTextOutput("[Driver]/[driveRobot] Done for the loop", 5);
    }
}

void Driver::planPathFromRRT(const bool& if_global_map_updated, 
        bool& found_path, bool& improved_path, bool& if_timeout)
{
    planner_results_lock_.lock();
    double remaining_time = old_path_timeout_ - 
        timing::spendElapsedTime(planner_results_.status.latest_time);
    planner_results_lock_.unlock();
    if_timeout =  remaining_time < 0;

    // debugger::debugColorOutput("[Driver]/[planPathFromRRT] "
    //         "if_global_map_updated: ", if_global_map_updated, 5, W);
    // debugger::debugColorOutput("[Driver]/[planPathFromRRT] "
    //         "if_timeout: ", if_timeout, 5, W);

    if (if_global_map_updated || if_timeout)
    // if (if_timeout)
    {
        debugger::debugColorTextOutput("[Driver]/[planPathFromRRT] "
                                       "Plannning a new path!", 6, BC);
        // plan a new path
        found_path = planner_->findNewPath(SIZE_MAX, allow_planning_time_, 1);

        // improve the path if have extra time
        improved_path = planner_->addMoreTimeNSamples(SIZE_MAX, 
                allow_planning_time_ - planner_->total_seconds_spent_, 0);
    }
    else
    {
        debugger::debugColorOutput("[Driver]/[planPathFromRRT] "
                "Improving existing path! Timeout in: ", remaining_time, 6, BC);
        improved_path = planner_->addMoreTimeNSamples(SIZE_MAX, 
                allow_planning_time_, 0);
        found_path = planner_->getPathStatus();
    }
}

void Driver::printResultsFromRRT(const bool& improved_path)
{
    planner_->printStatus(6, "Original Results");

    if (improved_path)
        planner_->printStatus(6, "Improved Results");
}


void Driver::decideSubGoalForPlanner_()
{
    sub_goal_pose_ = Driver::searchGoal_(); // the same as goal_searching_.goal_pose
    // if (driver_method_ == 0 || driver_method_ == 2)
    // {
    //     sub_goal_pose_ = Driver::searchGoal_(); // the same as goal_searching_.goal_pose
    //     debugger::debugOutput("[Driver]/[decideGoalForPlanner] Current robot pose: ",
    //             robot_state_.pose, 4);
    //     debugger::debugOutput("[Driver]/[decideGoalForPlanner] Picked the goal as: ",
    //             sub_goal_pose_, 4);
    // }
    // else if (driver_method_ == 1)
    // {
    //     clicked_lock_.lock();
    //     final_goal_pose_ = clicked_goal_pose_;
    //     clicked_lock_.unlock();
    // }
}

void Driver::updatePlannerResults_(
        const bool& found_path, const bool& improved_path, const bool& if_timeout)
{
    planner_results_lock_.lock();
    if (!found_path)
    {
        planner_results_.new_path_points_ptr = nullptr; 
        planner_results_.new_pose_to_follow_ptr = nullptr; 
        planner_results_.found_path = false;
        planner_results_.improved_path = false;
    }
    else 
    {
        // planner_results_.cost = planner_results_.new_cost;
        // planner_results_.new_path_points_ptr = planner_->getPlannedPathPtr();
        // planner_results_.new_pose_to_follow_ptr = planner_->getPlannedWaypointsPtr();
        // planner_results_.status.latest_time = timing::getCurrentTime();


        planner_results_.new_cost = planner_->getMinimumCost();
        // debugger::debugOutput("[Driver]/[updatedPlannerresults] new cost:", planner_results_.new_cost.getTotalCost(), 5);
        // debugger::debugOutput("[Driver]/[updatedPlannerresults] old cost:", planner_results_.cost.getTotalCost(), 5);
        cost_t diff = (planner_results_.new_cost - planner_results_.cost);
        // debugger::debugOutput("[Driver]/[updatedPlannerresults] diff:", diff.getTotalCost(), 5);
        debugger::debugOutput("[Driver]/[updatedPlannerresults] old cost:", 
                planner_results_.cost.getTotalCost(), 6);
        debugger::debugOutput("[Driver]/[updatedPlannerresults] new cost:", 
                planner_results_.new_cost.getTotalCost(), 6);
        debugger::debugOutput("[Driver]/[updatedPlannerresults] diff:", diff.getTotalCost(), 6);

        // accept the path if 
        // 1) no path to goal
        // OR
        // 2) has found a path before (within timeout horizon) and 
        // the new path is better
        if ((!planner_results_.status.has_path_to_use) ||
            (diff.getTotalCost() < cost_diff_threshold_within_timeout_ && 
             !if_timeout))
        {
            debugger::debugColorTextOutput("[Driver]/[updatedPlannerResults] "
                    "New path accepted!", 6, BC);
            planner_results_.cost = planner_results_.new_cost;

            planner_results_.found_path = found_path;
            planner_results_.improved_path = improved_path;

            planner_results_.new_path_points_ptr = planner_->getPlannedPathPtr();
            planner_results_.new_pose_to_follow_ptr = planner_->getPlannedWaypointsPtr();

            planner_results_.path_points = planner_->getPlannedPath(); 
            planner_results_.pose_to_follow = planner_->getPlannedWaypoints(); 

            planner_results_.status.has_path_to_use = true;
            planner_results_.status.latest_time = timing::getCurrentTime();
        }
        else 
        {
            debugger::debugColorTextOutput("[Driver]/[updatedPlannerResults] New path too bad!", 6, Y);
            planner_results_.new_path_points_ptr = nullptr; 
            planner_results_.new_pose_to_follow_ptr = nullptr; 
            planner_results_.found_path = false;
            planner_results_.improved_path = false;
        }
    }
    // planner_results_.found_path = found_path;
    // planner_results_.improved_path = improved_path;



    planner_results_.status.updated = true;
    planner_results_lock_.unlock();
}


void Driver::publishInfoToRobot_()
{
    ros::Rate publishing_rate(publishing_rate_); 
    control_variables_t control_variables;
    while(ros::ok())
    {
        // planner_info_to_controller_ updated here
        Driver::prepareInfoWithMPCForRobot_(control_variables);


        // debugger::debugOutput("[prepareInfo] control command: ", 
        //     control_commands::returnPublishData(planner_info_to_controller_), 5);
        udp_->publishToRobotGivenInfoToRobotType(planner_info_to_controller_);

        if (DEBUG_LEVEL <= 4)
        {
            ROS_INFO_STREAM_THROTTLE(1, 
                    control_commands::returnPublishData(
                        planner_info_to_controller_));
        }

        if (local_map_params_.mode <= 5)
        {
            planner_command_pub_.publish(
                    control_commands::convertToPlannerMsg(
                        planner_info_to_controller_, ros::Time::now(), map_frame_));
        }
        else if (local_map_params_.mode == 6 || local_map_params_.mode == 7)
        {

            // debugger::debugOutput("[prepareInfo] control command: ", "before", 5);
            planner_->local_map_->lockTerrainThread(true);
            planner_command_pub_.publish(
                    control_commands::convertToPlannerMsgWithPlane(
                        planner_info_to_controller_, 
                        planner_->local_map_->terrain_info, 
                        ros::Time::now(), map_frame_));
            planner_->local_map_->lockTerrainThread(false);
        }
        else
        {
            std::string error_msg = "No such mode: " +
                std::to_string(local_map_params_.mode);
            debugger::debugExitColor(error_msg, __LINE__, __FILE__);
        }

        if (log_commands_)
        {
            Driver::logCommands(planner_info_to_controller_, control_variables);
        }
        publishing_rate.sleep();
    }
}

void
Driver::prepareInfoWithMPCForRobot_(control_variables_t& control_variables)
{
    double throttle_time = 1;
    goal_selection_lock_.lock();
    planner_results_lock_.lock();


    // debugger::debugTitleTextOutput("[Driver]/[ExecutionThread]", 
    //         "has_chosen_goal status", 10, BG, BOLD);
    // debugger::debugColorOutput("has_chosen_goal: ", 
    //         goal_selection_.has_chosen_goal, 10, G);
    // control_variables, goal_selection_, planner_results_ are updated here
    if (!Driver::examinePathStatusForExecutionThread_(throttle_time))
    {
        control_variables.heading = robot_state_.pose.theta;
        planner_info_to_controller_  = 
            control_commands::assignWalkInPlaceWithInEKFMsg(
                    robot_state_.inekf_state);
        // debugger::debugColorOutput("[After]/[examinePathStatusForExecutionThread]"
        //     "has_chosen_goal: ", goal_selection_.has_chosen_goal, 10, G);
        goal_selection_lock_.unlock();
        planner_results_lock_.unlock();
        return;
    }
    // debugger::debugOutput("[Driver]/[prepareInfowithMPC] planner updated: ",
    //         planner_results_.status.updated, 5);
    // debugger::debugOutput("[Driver]/[prepareInfowithMPC] planner found path: ",
    //         planner_results_.found_path, 5);
    
    // ROS_INFO_STREAM_THROTTLE(throttle_time, 
    //         "[Driver]/[prepareInfowithMPC] planner updated: " <<
    //         planner_results_.status.updated);
    // ROS_INFO_STREAM_THROTTLE(throttle_time, 
    //         "[Driver]/[prepareInfowithMPC] planner found path: "<<
    //         planner_results_.found_path);
    // ROS_INFO_STREAM_THROTTLE(throttle_time, 
    //         "[Driver]/[prepareInfowithMPC] planner has path to use: "<<
    //         planner_results_.status.has_path_to_use);


    // robot current pose
    robot_state_t current_robot_state;
    bool status = Driver::getRobotCurrentState_(current_robot_state);
    
    // if no new state, use the same one for planning
    if (!status)
        current_robot_state = robot_state_;


    // if a final goal exists, check if reached the final goal
    // <has_reach_goal, has_goal_left>
    std::pair<bool, bool> goal_status = 
        Driver::checkWhetherAnyGoalLeftForExecutionThread_(throttle_time, control_variables, 
                                        current_robot_state);
    // debugger::debugColorOutput("[After]/[checkWhetherAnyGoalLeftForExecutionThread]"
    //         "has_chosen_goal: ", goal_selection_.has_chosen_goal, 4, G);

    if (goal_status.first)
    {
        goal_selection_lock_.unlock();
        planner_results_lock_.unlock();
        return;
    }


    // find a goal to go
    Driver::findNextGoalForExecutionThread_(throttle_time, current_robot_state);
    // debugger::debugColorOutput("[After]/[findNextGoalForExecutionThread]"
    //         "has_chosen_goal: ", goal_selection_.has_chosen_goal, 10, G);

    // execute to the goal
    Driver::executeReactivePlanner_(throttle_time, control_variables, 
                                   current_robot_state);
    //debugger::debugColorOutput("[After]/[executeReactivePlanner]"
    //        "has_chosen_goal: ", goal_selection_.has_chosen_goal, 10, G);
    goal_selection_lock_.unlock();
    planner_results_lock_.unlock();
}


// control_variables, goal_selection_, planner_results_ are updated here
int Driver::examinePathStatusForExecutionThread_(const double& throttle_time)
{
    int walk_foward = 1;
    bool flag_found_new_path = planner_results_.status.updated && 
                               planner_results_.found_path;
    bool flag_use_old_path = (planner_results_.status.updated && 
                              !planner_results_.found_path    &&
                              planner_results_.status.has_path_to_use) ||
                             (!planner_results_.status.updated && 
                              planner_results_.found_path &&
                              planner_results_.status.has_path_to_use);
    bool flag_no_path_to_go = 
        (!planner_results_.status.updated && !planner_results_.found_path) ||
         !planner_results_.status.has_path_to_use;
    if (flag_found_new_path)
    {
        walk_foward = 1;
        debugger::debugColorTextOutput("[Driver]/"
                "[examinePathStatusForExecutionThread] Got a new path!", 5, W);
        planner_results_.status.updated = false;
        goal_selection_.goal_num = -1;
        // ROS_WARN_STREAM_THROTTLE(throttle_time, "[Driver]/[prepareInfowithMPC] "
        //         "Got a new path!");

        // ROS_INFO_STREAM_THROTTLE(0.5, "======================= updated =======================");

        // path pionts and way poses
        // ROS_INFO_STREAM_THROTTLE(0.5, "(old, new): " << planner_results_.path_points_ptr->size() << "," << planner_results_.new_path_points_ptr->size());
        // planner_results_.path_points_ptr = planner_results_.new_path_points_ptr; 
        // ROS_INFO_STREAM_THROTTLE(0.5, "Updated path_points_ptr");
        // ROS_INFO_STREAM_THROTTLE(0.5, "(old, new): " << planner_results_.pose_to_follow_ptr->size() << "," << planner_results_.new_pose_to_follow_ptr->size());
        // planner_results_.pose_to_follow_ptr = planner_results_.new_pose_to_follow_ptr; 
        // ROS_INFO_STREAM_THROTTLE(0.5, "Updated pose_to_follow_ptr");
    }
    // using old path
    else if (flag_use_old_path)
    //else if (!planner_results_.found_path)
    {
        walk_foward = 1;
        // debugger::debugColorTextOutput(
        //         "[Driver]/[examinePathStatusForExecutionThread] "
        //         "No new path yet, using old path!", 10, G);
        ROS_WARN_STREAM_THROTTLE(throttle_time, 
                "[Driver]/[examinePathStatusForExecutionThread] "
                "No new path yet, using old path!");
        double time_diff = 
            timing::spendElapsedTime(planner_results_.status.latest_time);

        // debugger::debugOutput("[Driver]/[prepareInfoWithMPC] time elapse: ", time_diff, 5);
        if ( time_diff > old_path_timeout_)
        {
            walk_foward = 0;
            // debugger::debugColorTextOutput(
            //         "[Driver]/[examinePathStatusForExecutionThread] "
            //         "Walking in-place: Old path timeout!", 10, G);
            // usleep(1e6); // wait for one second
            ROS_WARN_STREAM_THROTTLE(throttle_time, 
                    "[Driver]/[examinePathStatusForExecutionThread] "
                    "Walking in-place: Old path timeout!");
            goal_selection_.has_chosen_goal = false;
            planner_results_.status.has_path_to_use = false;
            return walk_foward;
        }

    }
    else if (flag_no_path_to_go)
    {
        // debugger::debugColorTextOutput(
        //         "[Driver]/[examinePathStatusForExecutionThread] "
        //         "Walking in-place: No path to go at time " + 
        //         std::to_string(timing::getCurrentCPUTime()), 10, G);
        ROS_WARN_STREAM_THROTTLE(throttle_time, 
                "[Driver]/[examinePathStatusForExecutionThread] "
                "Walking in-place: No path to go!");
        // usleep(1e6); // wait for one second
        walk_foward = 0;
        goal_selection_.has_chosen_goal = false;
        planner_results_.status.updated = false;
        planner_results_.status.has_path_to_use = false;

        // debugger::debugOutput("before control_command: ", robot_state_.pose, 5);
        // debugger::debugOutput("before control_command: ", robot_state_.inekf_state, 5);
    }
    else
    {
        debugger::debugColorOutput("[Driver]/[examinePathStatusForExecutionThread] "
                "has_chosen_goal: ", 
                goal_selection_.has_chosen_goal, 10);
        debugger::debugColorOutput("[Driver]/[examinePathStatusForExecutionThread] "
                "planner_results-found_path: ", 
                planner_results_.found_path, 10);
        debugger::debugColorOutput("[Driver]/[examinePathStatusForExecutionThread] "
                "planner_results-updated: ", 
                planner_results_.status.updated, 10);
        debugger::debugColorOutput("[Driver]/[examinePathStatusForExecutionThread] "
                "planner_resultsl-has_path_to_use: ", 
                planner_results_.status.has_path_to_use, 10);

        std::string error_msg = "[Driver]/[examinePathStatusForExecutionThread]"
            "Why are you here? check above flags ";
        debugger::debugExitColor(error_msg, __LINE__, __FILE__);
    }

    return walk_foward;
}

std::pair<bool, bool> // <has_reached_goal, has_goal_left>
Driver::checkWhetherAnyGoalLeftForExecutionThread_(double throttle_time,
        control_variables_t& control_variables,
        const robot_state_t& current_robot_state)
{
    bool has_reached_goal = false;
    bool has_goal_left = true;

    if (is_exploration_ == 0)
    {
        goal_selection_.robot_pose = current_robot_state.pose;
        double dis_robot_to_final_goal = squared_point_distance(
                current_robot_state.pose.to2DPosition(), 
                final_goal_pose_.to2DPosition());
        if (dis_robot_to_final_goal < is_subgoal_threshold_)
        {
            has_reached_goal = true;
            // ROS_INFO_STREAM_THROTTLE(throttle_time, "[Driver]/[prepareInfowithMPC] "
            //         "------------ FINAL GOAL REACHED: Walking in-place!! ------------");

            control_variables.heading = robot_state_.pose.theta;
            planner_info_to_controller_  =
                control_commands::assignWalkInPlaceWithInEKFMsg(
                        robot_state_.inekf_state);

            // go to next one if there exists
            if (final_pose_x_list_.size() != 0)
            {
                debugger::debugColorOutput(
                        "[Driver]/[checkWhetherAnyGoalLeftForExecutionThread] ",
                        "------------ Move onto next goal!! ------------", 
                        10, BM, BOLD);
                debugger::debugColorOutput(
                        "[Driver]/[checkWhetherAnyGoalLeftForExecutionThread] ",
                        "------------ " + std::to_string(final_pose_x_list_.size()-1) + 
                        " left!! ------------", 
                        10, BM, BOLD);
                final_goal_pose_ = pose_t(final_pose_x_list_[0], final_pose_y_list_[0], 
                        deg_to_rad(final_pose_yaw_list_[0]));
                final_pose_x_list_.erase(final_pose_x_list_.begin());
                final_pose_y_list_.erase(final_pose_y_list_.begin());
                final_pose_yaw_list_.erase(final_pose_yaw_list_.begin());
                has_goal_left = true;
            }
            else
            {
                debugger::debugColorOutput(
                        "[Driver]/[checkWhetherAnyGoalLeftForExecutionThread] ",
                        "------------ FINAL GOAL REACHED: Walking in-place (" + 
                        std::to_string(timing::getCurrentCPUTime()) + ")!! ------------", 
                        10, BM, BOLD);
                has_goal_left = false;
            }
        }
    }
    return {has_reached_goal, has_goal_left};
}

// return next goal for the robot, 
// if no goal left, return current robot pose, i.e., walking in-place
void
Driver::findNextGoalForExecutionThread_(double throttle_time,
const robot_state_t& current_robot_state)
{
    if (goal_selection_.goal_num != -1)
    {
        // double dis_robot_to_pose = squared_point_distance(
        //         current_robot_state.pose.to2DPosition(), 
        //         goal_selection_.chosen_goal.first.to2DPosition());
        // if (dis_robot_to_pose < is_subgoal_threshold_)
        //     goal_selection_.goal_num ++;
        double dis_robot_to_pose = squared_point_distance(
                current_robot_state.pose.to2DPosition(), 
                goal_selection_.chosen_goal.first.to2DPosition());
        double angle_goal_to_robot_position_x_axis = 
            angle_between_two_points_and_x_axis_2pi(
                current_robot_state.pose.x,
                current_robot_state.pose.y,
                goal_selection_.chosen_goal.first.x,
                goal_selection_.chosen_goal.first.y);
        double angle_goal_to_robot_position = 
            wrap_to_2pi(angle_goal_to_robot_position_x_axis - 
                    current_robot_state.pose.theta);
        bool goal_is_right_behind = 
            (angle_goal_to_robot_position > deg_to_rad(150) &&
             angle_goal_to_robot_position < deg_to_rad(210));
        // debugger::debugColorOutput("[Driver]/[findNextGoalForExecutionThread] "
        //         "angle: ", rad_to_deg(angle_goal_to_robot_position), 4, BM);
        // debugger::debugColorOutput("[Driver]/[findNextGoalForExecutionThread] "
        //         "right_behind: ", goal_is_right_behind, 4, BM);
        // if (dis_robot_to_pose < is_subgoal_threshold_ || 
        //     (angle_goal_to_robot_position > deg_to_rad(160) ||
        //      angle_goal_to_robot_position < deg_to_rad(200))
        //    )
        if (dis_robot_to_pose < is_subgoal_threshold_ || goal_is_right_behind)
        {
            goal_selection_.goal_num ++;
            // debugger::debugColorOutput("[Driver]/[findNextGoalForExecutionThread] "
            //         "move onto next goal num: ", goal_selection_.goal_num,
            //         4, BM);
        }
        else
        {
            // debugger::debugColorOutput("[Driver]/[findNextGoalForExecutionThread] "
            //         "continue current goal num: ",
            //         goal_selection_.goal_num,
            //         4, BB, BOLD);
        }
    }
    else if (goal_selection_.goal_num == -1)
    {
        // debugger::debugColorOutput("[Driver]/[findNextGoalForExecutionThread] "
        //         "goal_num initialized", "",
        //         10, BM, BOLD);
        goal_selection_.goal_num = 1; // not start from itself
    }
    else
    {
        std::string error_msg = "[Driver]/[findNextGoalForExecutionThread]"
            "Why are you here? goal_num: " + 
            std::to_string(goal_selection_.goal_num);
        debugger::debugExitColor(error_msg, __LINE__, __FILE__);
    }


    if (DEBUG_LEVEL <= 3)
    {
        ROS_INFO_STREAM_THROTTLE(throttle_time, "[Driver]/[prepareInfowithMPC] "
                "goal_num: " << goal_selection_.goal_num);
        ROS_INFO_STREAM_THROTTLE(throttle_time, "[Driver]/[prepareInfowithMPC] "
                "path number: " << planner_results_.pose_to_follow.size());
    }
}

void Driver::executeReactivePlanner_(double throttle_time,
        control_variables_t& control_variables,
        const robot_state_t& current_robot_state)
{
    if (goal_selection_.goal_num > planner_results_.pose_to_follow.size() - 1)
    { 
        ROS_WARN_STREAM_THROTTLE(throttle_time, "[Driver]/[executeReactivePlanner] "
                "Walking in-place: No goal left!");
        goal_selection_.has_chosen_goal = false;
        goal_selection_lock_.unlock();
        planner_results_lock_.unlock();

        control_variables.heading = robot_state_.pose.theta;
        planner_info_to_controller_  =
            control_commands::assignWalkInPlaceWithInEKFMsg(
                    robot_state_.inekf_state);
    }
    else
    {
        goal_selection_.chosen_goal = 
            planner_results_.pose_to_follow[goal_selection_.goal_num];
        goal_selection_.has_chosen_goal = true;
        // XXX: testing only
        // double rand_x = utils::genInclusiveRandomNumber(-5, 5);
        // double rand_y = utils::genInclusiveRandomNumber(-5, 5);
        // current_robot_state.pose.x += rand_x;
        // current_robot_state.pose.y += rand_y;
        // current_robot_state.updateInEKFWithPose();
        // goal_selection_.robot_pose = current_robot_state.pose; 
        
        // TODO: include robot dynamic (MPC) here
        double behavior = 1; // walking
        double roll = 0;
        double pitch = 0;

        // rotate in place if goal is outside of FOV
        if (clf_model_ == 1)
        { 
            reduced_omni_ego_polar_coords_t rd = omni_local_chart::convertPose2rd(
                        current_robot_state.pose, 
                        goal_selection_.chosen_goal.first.to2DPosition());
            std::pair<int, double> res =
                checkOutsideFovReturnModifiedDelta(rd.delta, 
                        lyap_dist_params_.beta);

            // if the goal is not within the FOV, rotate in palce
            if (res.first == 1)
            { 
                if (robot_params_.heading_angle_preintegration)
                {
                    double new_yaw = current_robot_state.pose.theta +
                        robot_params_.default_omega * 
                        robot_params_.step_time_interval;
                    planner_info_to_controller_ = 
                        control_commands::assignInfoReducedWithInEKFMsg(
                                0, 
                                {0, 0, -robot_params_.default_omega}, 
                                roll, pitch, 
                                new_yaw, current_robot_state.inekf_state);

                    control_variables.omega = -robot_params_.default_omega;
                    control_variables.heading = new_yaw;
                    // debugger::debugColorOutput("new yaw (1): ", rad_to_deg(new_yaw), 5);
                }
                else
                {
                    planner_info_to_controller_ = 
                        control_commands::assignRotateInPlaceWithInEKFMsg(
                                current_robot_state.inekf_state,
                                -robot_params_.default_omega);
                    control_variables.omega = -robot_params_.default_omega;
                }

                return;
            }
            else if (res.first == -1)
            { 
                if (robot_params_.heading_angle_preintegration)
                {
                    double new_yaw = current_robot_state.pose.theta - 
                        robot_params_.default_omega * 
                        robot_params_.step_time_interval;
                    planner_info_to_controller_ = 
                        control_commands::assignInfoReducedWithInEKFMsg(
                                0, 
                                {0, 0, robot_params_.default_omega}, 
                                roll, pitch, 
                                new_yaw, current_robot_state.inekf_state);
                    control_variables.omega = robot_params_.default_omega;
                    control_variables.heading = new_yaw;
                    // debugger::debugColorOutput("new yaw (-1): ", rad_to_deg(new_yaw), 5);
                }
                else
                {
                    planner_info_to_controller_ = 
                        control_commands::assignRotateInPlaceWithInEKFMsg(
                                current_robot_state.inekf_state,
                                robot_params_.default_omega);
                    control_variables.omega = robot_params_.default_omega;
                }

                return; 
            }

            // used to test rotate in-place
            // goal_selection_.has_chosen_goal = true;
            // goal_selection_lock_.unlock();
            // planner_results_lock_.unlock();

            // return planner_info_to_controller_; 
        }

        // double delta_yaw = planner_->lyap_dist_->stabilizeDeltaStarWithTargetPose(
        //         current_robot_state.pose.to2DPosition(), 
        //         goal_selection_.chosen_goal.first);
        // double yaw = delta_yaw + current_robot_state.pose.theta;
        // debugger::debugColorOutput("yaw: ", yaw, 5);

        // control_variables update here
        Driver::computeOptimalControlWithBoundsAndThresholds(
                control_variables, current_robot_state);

        // // yaw = M_PI;
        // debugger::debugColorOutput("point: ", current_robot_state.pose.to2DPosition(), 5);
        // debugger::debugColorOutput("pose: ", goal_selection_.chosen_goal.first, 5);
        // debugger::debugColorOutput("yaw: ", yaw, 5);

        // smooth commands
        Driver::smoothCommandFromHistory_(
                control_variables.vx, control_variables.vy, 
                control_variables.vr, control_variables.vd, 
                control_variables.omega, 
                roll, pitch, control_variables.heading);
        // Driver::smoothCommandFromHistory_(speed, roll, pitch, delta_yaw);
        // debugger::debugColorOutput("yaw: ", yaw, 5);

        // preintegrated heading angle

        // int preintegrated_heading_angle = 1;
        // if (clf_model_ == 1 && preintegrated_heading_angle)
        if (clf_model_ == 1 && robot_params_.heading_angle_preintegration)
        {
            double yaw = current_robot_state.pose.theta - 
                control_variables.omega * robot_params_.step_time_interval;
            control_variables.heading = yaw;
        }

        // x-axis is yaw = 0
        // std::vector<double> velocity{speed * std::cos(yaw), speed * std::sin(yaw)}; 

        planner_info_to_controller_ = 
            control_commands::assignInfoReducedWithInEKFMsg(
                    behavior, 
                    {control_variables.vx, control_variables.vy, control_variables.omega}, 
                    roll, pitch, 
                    control_variables.heading, current_robot_state.inekf_state);

        return ; 
    }
}

// control_variables update here
void Driver::computeOptimalControlWithBoundsAndThresholds(
        control_variables_t& control_variables, 
        const robot_state_t& current_robot_state)
{
        control_variables = 
            planner_->lyap_dist_->stabilizeControlWithTargetPose(
                current_robot_state.pose, 
                goal_selection_.chosen_goal.first);


        if (robot_params_.enlarge_mode == 1)
        {
            double speed = std::sqrt(control_variables.vx * control_variables.vx + 
                                     control_variables.vy * control_variables.vy);
            double top_speed = robot_params_.normalized_speed;
            double enlarge_velocity_factor = top_speed / speed;
            control_variables.vx *= enlarge_velocity_factor;
            control_variables.vy *= enlarge_velocity_factor;
            control_variables.omega *= enlarge_velocity_factor;
        }
        else if (robot_params_.enlarge_mode == 2)
        {
            control_variables.vx *= robot_params_.velocity_factor;
            control_variables.vy *= robot_params_.velocity_factor;
            control_variables.omega *= robot_params_.velocity_factor;
        }
        else if (robot_params_.enlarge_mode == 3)
        {
            // debugger::debugColorOutput("===================", "", 5);
            // debugger::debugColorOutput("Before adjusting vx: ", control_variables.vx, 5);
            // debugger::debugColorOutput("Before adjusting vy: ", control_variables.vy, 5);
            double m = std::abs(robot_params_.vy_upper_bound / 
                                control_variables.vy);
            double n = std::abs(robot_params_.vx_upper_bound / 
                                control_variables.vx);
            double k = std::min(m, n);
            control_variables.vy *= k;
            control_variables.vx *= k;
            control_variables.omega *= k;
            // debugger::debugColorOutput("k: ", k, 5);
            // debugger::debugColorOutput("After adjusting vx: ", control_variables.vx, 5);
            // debugger::debugColorOutput("After adjusting vy: ", control_variables.vy, 5);
        }
        else if (robot_params_.enlarge_mode == 0)
        {
        }
        else
        {
            std::string error_msg = "ERROR: No such enlarge_mode supported: " + 
                std::to_string(robot_params_.enlarge_mode);
            debugger::debugExitColor(error_msg, __LINE__, __FILE__);
        }


        if (robot_params_.enlarge_mode >= 0)
        {
            // velocity bound
            control_variables.vx = std::copysign(
                    std::min(std::abs(control_variables.vx), 
                             robot_params_.vx_upper_bound), 
                    control_variables.vx);

            control_variables.vy = std::copysign(
                    std::min(std::abs(control_variables.vy), 
                             robot_params_.vy_upper_bound), 
                    control_variables.vy);
            // debugger::debugColorOutput("After bounds vx: ", control_variables.vx, 5);
            // debugger::debugColorOutput("After bounds vy: ", control_variables.vy, 5);
        }
}


void Driver::smoothCommandFromHistory_(double& vx, double& vy, 
        double& vr, double& vd,
        double& omega,
        double& roll, double& pitch, double& yaw)
{
    if (command_history_.getLength() >= command_history_length_)
    {
        command_history_.popFront();
    }
    command_history_.pushBack(vx, vy, vr, vd, omega, roll, pitch, yaw);
    vx = utils::getAverage(command_history_.vx_list);
    vy = utils::getAverage(command_history_.vy_list);
    vr = utils::getAverage(command_history_.vr_list);
    vd = utils::getAverage(command_history_.vd_list);
    omega = utils::getAverage(command_history_.omega_list);
    roll = utils::getAverage(command_history_.roll_list);
    pitch = utils::getAverage(command_history_.pitch_list);
    yaw = utils::getAverage(command_history_.yaw_list);


    // debugger::debugColorOutput("roll1: ", roll, 5);
    // debugger::debugColorOutput("pitch1: ", pitch, 5);
    // debugger::debugColorOutput("yaw1: ", yaw, 5);

    // Eigen::Matrix3d m;
    // m = Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()) * 
    //     Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) * 
    //     Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());
    // Eigen::Vector3d v = lie_group::computeLogSO3(m);


    // command_history_.pushBack(speed, v(0), v(1), v(2));
    // speed = utils::getAverage(command_history_.speed_list);
    // double ave_w1 = utils::getAverage(command_history_.w1_list);
    // double ave_w2 = utils::getAverage(command_history_.w2_list);
    // double ave_w3 = utils::getAverage(command_history_.w3_list);

    // Eigen::Vector3d ave_v;
    // ave_v << ave_w1, ave_w2, ave_w3;
    // Eigen::Matrix3d ave_m = lie_group::computeExpSO3(ave_v);
    // Eigen::Vector3d rpy = ave_m.eulerAngles(0, 1, 2);
    // 
    // roll = rpy(0);
    // pitch = rpy(1);
    // yaw = rpy(2);
    // debugger::debugColorOutput("roll2: ", roll, 5);
    // debugger::debugColorOutput("pitch2: ", pitch, 5);
    // debugger::debugColorOutput("yaw2: ", yaw, 5);
}


pose_t
Driver::searchGoal_()
{

    // if a final goal exist, use it to find the direction of the searching arc
    double theta = robot_state_.pose.theta - M_PI/2;
    if (is_exploration_ == 0)
    {
        // if too close to final goal, return the final goal
        if (Driver::computeDirectionOfSearchingArc(theta))
        {
            goal_searching_.goal_pose = final_goal_pose_;
            return goal_searching_.goal_pose;
        }
    }

    double buffer_increment = 0.5;
    size_t num_of_buffer_cost = 2 * goal_searching_.buffer / buffer_increment;

    // find search angle
    double view_angle = goal_searching_.search_view.first + 
        goal_searching_.search_view.second; // deg

    // delta_theta is in deg
    goal_searching_.num_poses = std::round(view_angle/goal_searching_.delta_theta);


    // get map cost 
    map_cost_ = planner_->getMapCostClass();


    // generate angles
    goal_searching_.list_of_angles = utils::genListOfNumbers(
            (theta - deg_to_rad(goal_searching_.search_view.second)), 
            goal_searching_.num_poses, 
            deg_to_rad(goal_searching_.delta_theta));

    goal_searching_.list_of_poses.resize(goal_searching_.num_poses);

    goal_searching_.goal_cost = cost_t(DBL_MAX / 2 - 1, DBL_MAX / 2 - 1);

    // generate poses and find a proper goal with lowest cost
    for (int i = 0; i < goal_searching_.num_poses; ++i)
    {
        // generate a pose on a circle
        double x = robot_state_.pose.x + 
                   goal_searching_.search_radius * 
                   std::cos(goal_searching_.list_of_angles[i]);
        double y = robot_state_.pose.y + 
                   goal_searching_.search_radius * 
                   std::sin(goal_searching_.list_of_angles[i]);
        pose_t goal_pose(x, y, goal_searching_.list_of_angles[i]);
        goal_searching_.list_of_poses[i].first = goal_pose;


        // if a final goal exists, compute cost_to_final_goal
        // compute the distance from the current position to the final goal
        // and then compute cost_to_goal
        cost_t cost_to_goal;
        if (is_exploration_ == 0)
        {
            double dis_subgoal_to_final_goal = squared_point_distance(
                    point2d_t<float>(x, y), final_goal_pose_.to2DPosition());
            cost_to_goal.distance += goal_searching_.weight_subgoal_and_final_goal * 
                dis_subgoal_to_final_goal;
        }

        cost_t cost;
        bool is_free = map_cost_->isNeighborObstacleFree(goal_pose, 
                pose_sampler_params_.distance_threshold);
        if (goal_searching_.goal_search_mode == 0)
        {
            cost = cost_to_goal;
            // debugger::debugColorOutput("weight_subgoal_and_final_goal: ", 
            //         goal_searching_.weight_subgoal_and_final_goal, 5);
            // debugger::debugColorOutput("dis_subgoal_to_final_goal: ", 
            //         dis_subgoal_to_final_goal, 5);
            // debugger::debugColorOutput("min_cost: ", 
            //         goal_searching_.goal_cost.getTotalCost(), 5);
            // debugger::debugColorOutput("cost_to_goal: ", cost_to_goal.getTotalCost(), 5);
            // debugger::debugColorOutput("cost: ", cost.getTotalCost(), 5);
            // debugger::debugColorOutput("theta: ", 
            //         goal_searching_.list_of_angles[i], 5);
        }
        else if (goal_searching_.goal_search_mode == 1 || 
                 goal_searching_.goal_search_mode == 2)
        {
            // find the cost of the goal pose
            cost = map_cost_->computePositionMapCost(goal_pose);

            // compute the cost of a sliding window
            std::vector<double> angle_for_accumulated_cost = utils::genListOfNumbers(
                    goal_searching_.list_of_angles[i] - goal_searching_.buffer,
                    num_of_buffer_cost, buffer_increment);
            for (const auto& buffer_angle : angle_for_accumulated_cost)
            {
                pose_t buffer_goal_pose(x, y, buffer_angle);
                cost = cost + map_cost_->computePositionMapCost(buffer_goal_pose);
            }

            cost = cost + cost_to_goal;

            // XXX: weight of orientation
            double weight_of_orientation = 
                goal_searching_.percentage_weight_of_orientation * 
                cost.getTotalCost();
            double cost_of_orientation = weight_of_orientation * 
                                         std::abs(theta - robot_state_.yaw);

            cost.distance += cost_of_orientation;
            // debugger::debugColorOutput("cost_of_orientation: ", cost_of_orientation, 5);
            // debugger::debugColorOutput("height cost: ", cost.getTotalCost(), 5);
        }
        else
        {
            std::string error_msg = "No such goal_search_mode: " + 
                std::to_string(goal_searching_.goal_search_mode);
            debugger::debugExitColor(error_msg, __LINE__, __FILE__);
        }

        // goal_searching_.list_of_poses[i].second = cost;
        goal_searching_.list_of_poses[i].second = cost;



        if (cost < goal_searching_.goal_cost && is_free)
        {
            goal_searching_.goal_cost = cost;
            goal_searching_.goal_pose = goal_pose;
        }
        // if cost is the same, compare angle with the goal
        else if (lazy_equal::absolute_fuzzy_equal(
                    cost.getTotalCost(), goal_searching_.goal_cost.getTotalCost()) &&
                is_free)
        {
            if (lazy_equal::absolute_fuzzy_equal(goal_pose.theta, 
                                                 theta) < 
                lazy_equal::absolute_fuzzy_equal(goal_searching_.goal_pose.theta, 
                                                 theta))
            {
                goal_searching_.goal_cost = cost;
                goal_searching_.goal_pose = goal_pose;
            }
        }
    }

    if (goal_searching_.goal_search_mode == 2)
    {
        Driver::detectIntersection(goal_searching_);
    }

    return goal_searching_.goal_pose;
}


// hierarchical unsupervised clustering
void Driver::detectIntersection(goal_searching_for_planner_t& goal_searching)
{
    // path_segments updated here
    std::list<std::list<std::pair<pose_t, cost_t>>> path_segments;
    std::list<std::pair<size_t, double>> segment_info; // size and angle of each segemnt 
    Driver::cluterViaHierarchicalAlgorithm(goal_searching, path_segments);
    debugger::debugColorOutput("[detectIntersection] [BeforeRemove] num of segment: ", 
                     path_segments.size(), 5, C);

    // remove segments and collect information
    Driver::pruneClustersAndCollectInformation(goal_searching, 
            path_segments, segment_info);

    size_t path_segment_size = path_segments.size();
    debugger::debugColorOutput("[detectIntersection] [AfterRemove] num of segment: ", 
                     path_segment_size, 5, C);

    // cluster segments
    auto it = path_segments.begin();
    goal_searching_.goal_cost = cost_t(DBL_MAX/2, DBL_MAX/2);
    double distance_from_seg_to_goal = DBL_MAX;
    double angle_seg_to_robot = DBL_MAX;
    bool flag_picked = false;
    while (it != path_segments.end())
    {
        size_t segment_size = it->size();
        // sort by heading angles
        auto heading_compare_func = [](std::pair<pose_t, cost_t> a, 
                std::pair<pose_t, cost_t> b)
        { 
            return a.first.theta > b.first.theta; 
        };

        it->sort(heading_compare_func);

        // pick the middle of the segment 
        int middle = std::floor(segment_size / 2);
        auto pose_it = (*it).begin();
        std::advance (pose_it, middle);
        Driver::pickGoalSegment(pose_it, distance_from_seg_to_goal, 
                                angle_seg_to_robot, path_segment_size, flag_picked);
        ++it;
    }
    goal_searching.path_segments = path_segments;
}


void Driver::pruneClustersAndCollectInformation(
        goal_searching_for_planner_t& goal_searching,
        std::list<std::list<std::pair<pose_t, cost_t>>>& path_segments,
        std::list<std::pair<size_t, double>>& segment_info)
{
    // sort by length
    // auto length_compare_func = [](std::list<std::pair<pose_t, cost_t>> a, 
    //                               std::list<std::pair<pose_t, cost_t>> b)
    // { 
    //     return a.size() > b.size(); 
    // };
    // path_segments.sort(length_compare_func);

    // remove segments
    auto it = path_segments.begin();
    while (it != path_segments.end())
    {
        size_t segment_size = it->size();
        debugger::debugColorOutput("[detectIntersection] size of this segment: ", 
                     segment_size, 5, C);

        // skip if the segment is too short
        if (segment_size < goal_searching.goal_segment_minimum_len)
        {
            it = path_segments.erase(it);
            continue;
        }

        // skip if the cost of a segment is too high
        auto pose_it = (*it).begin();
        double sum_cost = 0;
        while (pose_it != (*it).end())
        {
            sum_cost += pose_it->second.getTotalCost();
            pose_it ++;
        }

        double ave_cost = sum_cost / segment_size;
        debugger::debugColorOutput("[detectIntersection] ave_cost: ", 
                ave_cost, 5, C);
        if (ave_cost > goal_searching.goal_segment_max_cost)
        {
            it = path_segments.erase(it);
            continue;
        }
        ++it;
    }


}

void Driver::cluterViaHierarchicalAlgorithm(
        goal_searching_for_planner_t& goal_searching,
        std::list<std::list<std::pair<pose_t, cost_t>>>& path_segments)
{
    double buffer_increment = 0.5;
    size_t num_of_buffer_cost = 2 * goal_searching_.buffer / buffer_increment;

    double cost_weight = 1;
    double linkage_dis_threshold = goal_searching.goal_segment_linkage_dis_weight * 
                                   goal_searching.search_radius *
                                   deg_to_rad(goal_searching.delta_theta);
    debugger::debugColorOutput("[detectIntersection] linkage_dis_threshold: ", 
                     linkage_dis_threshold, 5, C);
    debugger::debugColorOutput("[detectIntersection] num of poses on the arc: ", 
                     goal_searching.list_of_poses.size(), 5, C);

    double linkage_obs_dis_threshold = 
        std::min(pose_sampler_params_.distance_threshold, (float) 0.2);
    for (auto&t_goal : goal_searching.list_of_poses)
    {
        bool is_free = map_cost_->isNeighborObstacleFree(t_goal.first,
                linkage_obs_dis_threshold);
        if (!is_free)
            continue;

        if (path_segments.size() == 0)
        {
            path_segments.push_back({t_goal});
            continue;
        }

        // go through all the cluster to find a match
        // find the first segment linked and then continue
        // if there is no match, create a new cluster
        bool flag_linkage = false;
        for (auto& path_seg : path_segments)
        {
            std::pair<pose_t, cost_t> first = path_seg.front();
            double dis_to_first = squared_point_distance(
                    t_goal.first.to2DPosition(), first.first.to2DPosition());
            double cost_diff_first = 
                first.second.takeAbsoluteDifference(t_goal.second).getTotalCost() / 
                num_of_buffer_cost;

            std::pair<pose_t, cost_t> last = path_seg.back();
            double dis_to_last = squared_point_distance(
                    t_goal.first.to2DPosition(), last.first.to2DPosition());
            double cost_diff_last = 
                last.second.takeAbsoluteDifference(t_goal.second).getTotalCost() / 
                num_of_buffer_cost;


            flag_linkage = (dis_to_first <= linkage_dis_threshold && 
                            cost_diff_first <= 
                            goal_searching.goal_segment_linkage_cost_threshold) || 
                           (dis_to_last <= linkage_dis_threshold && 
                            cost_diff_last <= 
                            goal_searching.goal_segment_linkage_cost_threshold);
            if (flag_linkage)
            {
                if (dis_to_last < dis_to_first)
                {
                    path_seg.push_back(t_goal);
                }
                else
                {
                    path_seg.push_front(t_goal);
                }
                break;
            }
            // t_goal.first.print();
            // first.first.print();
            // last.first.print();
            // debugger::debugColorOutput("[detectIntersection] dis_to_first: ", dis_to_first, 7);
            // debugger::debugColorOutput("[detectIntersection] dis_to_last: ", dis_to_last, 7);
            // debugger::debugColorOutput("[detectIntersection] linking_dis_to_first: ", linking_dis_to_first, 7);
            // debugger::debugColorOutput("[detectIntersection] linking_dis_to_last: ", linking_dis_to_last, 7);
            // // debugger::debugColorOutput("[detectIntersection] median: ", median, 7);
            // debugger::debugColorOutput("[detectIntersection] threshold: ", linkage_threshold, 7);
            // debugger::debugColorOutput("[detectIntersection] flag: ", flag_linkage, 7);
            // utils::pressEnterToContinue();
        }
        if (!flag_linkage)
        {
            path_segments.push_back({t_goal});
        }
    }
}

// // return 1 if too close to final goal
// bool Driver::computeDirectionOfSearchingArc(double& theta)
// {
//     bool flag_too_close = 0;
//     double robot_to_goal_x = final_goal_pose_.x - robot_state_.pose.x;
//     double robot_to_goal_y = final_goal_pose_.y - robot_state_.pose.y;
//     double robot_to_goal_dis = std::sqrt(robot_to_goal_x * robot_to_goal_x + 
//                                          robot_to_goal_y * robot_to_goal_y);
//     // double robot_to_goal_dis = squared_point_distance(
//     //             final_goal_pose_.to2DPosition(), robot_state_.to2DPosition());
// 
// 
//     // if the final gose is close enough, return it as a subgoal
//     if (robot_to_goal_dis <= goal_searching_.search_radius && 
//             robot_to_goal_dis <= local_map_params_.length)
//     {
//         flag_too_close = 1;
//         return flag_too_close;
//     }
// 
// 
// 
//     theta = std::asin((robot_to_goal_y) / robot_to_goal_dis);
//     if (robot_to_goal_x > 0)
//         theta = theta;
//     else if (robot_to_goal_x < 0)
//         theta = M_PI - theta;
//     else if (robot_to_goal_x == 0 && robot_to_goal_y > 0)
//         theta = M_PI/2;
//     else if (robot_to_goal_x == 0 && robot_to_goal_y < 0)
//         theta = -M_PI/2;
//     else if (robot_to_goal_x == 0 && robot_to_goal_y == 0)
//         theta = 0;
// 
// 
//     return flag_too_close;
// }

// return 1 if too close to final goal
bool Driver::computeDirectionOfSearchingArc(double& theta)
{
    bool flag_too_close = 0;
    double robot_to_goal_x = final_goal_pose_.x - robot_state_.pose.x;
    double robot_to_goal_y = final_goal_pose_.y - robot_state_.pose.y;
    double robot_to_goal_dis = std::sqrt(robot_to_goal_x * robot_to_goal_x + 
                                         robot_to_goal_y * robot_to_goal_y);
    // double robot_to_goal_dis = squared_point_distance(
    //             final_goal_pose_.to2DPosition(), robot_state_.to2DPosition());


    // if the final gose is close enough, return it as a subgoal
    if (robot_to_goal_dis <= goal_searching_.search_radius && 
            robot_to_goal_dis <= local_map_params_.length)
    {
        flag_too_close = 1;
        return flag_too_close;
    }

    theta = angle_between_two_points_and_x_axis_pi(
            robot_state_.pose.x, robot_state_.pose.y, 
            final_goal_pose_.x, final_goal_pose_.y);
    debugger::debugColorOutput("[computeDirectionOfSearchingArc] "
            "directional theta: ", theta, 5, W);




    return flag_too_close;
}


void Driver::pickGoalSegment(
        const std::list<std::pair<pose_t, cost_t>>::iterator& pose_it, 
        double& distance_from_seg_to_goal, double& angle_seg_to_robot, 
        const size_t path_segment_size, bool& flag_picked)
{
    double front_angle = 20;
    double left_right_angle_max = 110;
    double left_right_angle_min = 10;

    // if a goal exist, pick the segment closer to the goal,
    // or pick left/right compared to robot current heading
    if (is_exploration_ == 0)
    {
        double cur_distance_from_seg_to_goal = squared_point_distance(
                pose_it->first.to2DPosition(), 
                final_goal_pose_.to2DPosition());
        if (cur_distance_from_seg_to_goal < distance_from_seg_to_goal)
        {
            goal_searching_.goal_cost = pose_it->second;
            goal_searching_.goal_pose = pose_it->first;
            distance_from_seg_to_goal = cur_distance_from_seg_to_goal;
        }
    }
    else
    {
        double seg_to_robot_x = pose_it->first.to2DPosition().x - 
            robot_state_.pose.x;
        double seg_to_robot_y = pose_it->first.to2DPosition().y - 
            robot_state_.pose.y;
        double seg_to_robot_dis = std::sqrt(seg_to_robot_x * seg_to_robot_x + 
                seg_to_robot_y * seg_to_robot_y);

        double seg_angle = std::asin((seg_to_robot_y) / seg_to_robot_dis);

        if (seg_to_robot_x > 0)
            seg_angle = seg_angle;
        else if (seg_to_robot_x < 0)
            seg_angle = M_PI - seg_angle;
        else if (seg_to_robot_x == 0 && seg_to_robot_y > 0)
            seg_angle = M_PI/2;
        else if (seg_to_robot_x == 0 && seg_to_robot_y < 0)
            seg_angle = -M_PI/2;
        else if (seg_to_robot_x == 0 && seg_to_robot_y == 0)
            seg_angle = 0;

        // double seg_angle = std::atan2(seg_to_robot_y, seg_to_robot_y);
        double diff_heading = wrap_to_pi(seg_angle - robot_state_.pose.theta); 


        debugger::debugColorOutput("[detectIntersection] seg position: ", 
                pose_it->first, 4, Y);
        debugger::debugColorOutput("[detectIntersection] robot pose: ", 
                robot_state_.pose, 4, Y);
        debugger::debugColorOutput("[detectIntersection] seg_angle: ", 
                seg_angle, 4, Y);
        debugger::debugColorOutput("[detectIntersection] diff_heading: ", 
                diff_heading, 4, Y);

        if (goal_searching_.goal_behavior == 0)
        {
            if (std::abs(diff_heading) < std::abs(angle_seg_to_robot))
            {
                goal_searching_.goal_cost = pose_it->second;
                goal_searching_.goal_pose = pose_it->first;
                angle_seg_to_robot = diff_heading;
            }
        }
        else if (goal_searching_.goal_behavior == 1)
        {
            // std::abs(diff_heading) < std::abs(angle_seg_to_robot) && 
            if (diff_heading >= deg_to_rad(-front_angle) && 
                diff_heading < deg_to_rad(left_right_angle_max))
            {
                if (!flag_picked)
                {
                    goal_searching_.goal_cost = pose_it->second;
                    goal_searching_.goal_pose = pose_it->first;
                    angle_seg_to_robot = diff_heading;
                    flag_picked = true;
                }

                // if (flag_picked && diff_heading >= deg_to_rad(left_right_angle_min))
                if (path_segment_size > 2 && 
                    diff_heading >= deg_to_rad(left_right_angle_min))
                {
                    goal_searching_.goal_cost = pose_it->second;
                    goal_searching_.goal_pose = pose_it->first;
                    angle_seg_to_robot = diff_heading;
                }
            }
        }
        else if (goal_searching_.goal_behavior == -1)
        {
            // std::abs(diff_heading) < std::abs(angle_seg_to_robot) && 
            if (diff_heading <= deg_to_rad(front_angle) && 
                diff_heading > deg_to_rad(-left_right_angle_max))
            {
                if (!flag_picked)
                {
                    goal_searching_.goal_cost = pose_it->second;
                    goal_searching_.goal_pose = pose_it->first;
                    angle_seg_to_robot = diff_heading;
                    flag_picked  = true;
                }

                // if (flag_picked && diff_heading <= deg_to_rad(-left_right_angle_min))
                if (path_segment_size > 2 && 
                    diff_heading <= deg_to_rad(-left_right_angle_min))
                {
                    goal_searching_.goal_cost = pose_it->second;
                    goal_searching_.goal_pose = pose_it->first;
                    angle_seg_to_robot = diff_heading;
                }
            }
        }
        else
        {
            std::string error_msg = "ERROR: No such goal_behavior: "  
                + std::to_string(goal_searching_.goal_behavior);
            debugger::debugExitColor(error_msg, __LINE__, __FILE__);
        }
    }
}

void Driver::publishToRviz_()
{
    // clear previous markers
    goal_selection_marker_array_.markers.clear();
    robot_marker_array_.markers.clear();
    planner_marker_array_.markers.clear();

    rrt_path_points = 
        pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);
    rrt_path_points->header.frame_id = map_frame_;

    planner_path_points = 
        pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);
    planner_path_points->header.frame_id = map_frame_;


    debugger::debugTextOutput("[Driver]/publishToRviz] Plotting...", 3);

    // global map
    grid_map_msgs::GridMap global_map_msg;
    grid_map::GridMapRosConverter::toMessage(multi_layer_map_, global_map_msg);
    global_map_pub_.publish(global_map_msg);

    // local map
    // planner_->printGlobalMapCLFAddress();
    // planner_->printGlobalMapLocalMapAddress();
    // planner_->printLocalMapAddress();
    grid_map::GridMap local_map = planner_->getLocalMap();
    grid_map_msgs::GridMap local_map_msg;
    grid_map::GridMapRosConverter::toMessage(local_map, local_map_msg);
    local_map_pub_.publish(local_map_msg);

    // robot pose
    plotting::addMarkerWithPose(marker_, visualization_msgs::Marker::ARROW,
            "robot_pose",
            0, 1, 0, 1, // color
            robot_state_.pose, // pose
            0, marker_lifetime_ // count, time
            );
    robot_marker_array_.markers.push_back(marker_);
    marker_array_pub_.publish(robot_marker_array_);

    if (is_exploration_ == 0)
    {
        plotting::addMarkerWithPose(marker_, visualization_msgs::Marker::ARROW,
                "final_pose",
                1, 0, 0, 1, // color
                final_goal_pose_, // pose
                0, 0 // count, time
                );
        robot_marker_array_.markers.push_back(marker_);


        // arrow from subgoal to final goal
        visualization_msgs::Marker line_marker;
        plotting::addMarkerWithTwoPoints(line_marker, 
                visualization_msgs::Marker::LINE_STRIP,
                "subgoal_to_goal",
                1, 1, 1, 1, // color
                goal_searching_.goal_pose.x, // x1
                goal_searching_.goal_pose.y, // y1
                goal_searching_.goal_pose.z, // z1
                final_goal_pose_.x, // x2
                final_goal_pose_.y, // y2
                final_goal_pose_.z, // z2
                0, 0, "",// count, time
                0.1, 0.01, 0.01
                );
        robot_marker_array_.markers.push_back(line_marker);
        marker_array_pub_.publish(robot_marker_array_);
    }




    // robot trajectory
    geometry_msgs::PoseStamped rob_traj;
    rob_traj.pose.position = robot_state_.inekf_state.position;
    rob_traj.pose.orientation = robot_state_.inekf_state.orientation;
    rob_traj.header = robot_state_.inekf_state.header;
    robot_trajectory_.poses.push_back(rob_traj);
    robot_trajectory_.header.frame_id = (map_frame_);
    robot_trajectory_.header.stamp = ros::Time::now();


    // goal searching
    size_t count = 0;
    double goal_search_height = 1;
    for (auto pose : goal_searching_.list_of_poses)
    {
        pose.first.z = goal_search_height;
        plotting::addMarkerWithPose(marker_, visualization_msgs::Marker::ARROW,
                "goal_searching",
                0, 0, 1, 1, // color
                pose.first, // pose
                count++, marker_lifetime_ // count, time
                );
        goal_selection_marker_array_.markers.push_back(marker_);
    }

    int seg_num = 0;
    for (auto seg : goal_searching_.path_segments)
    {
        std::vector<double> color = 
            utils::genListOfInclusiveRandomNumbers<double>(3, 0.0, 1.0);
        for (auto pose : seg)
        {
            pose.first.z = goal_search_height + 0.1;
            plotting::addMarkerWithPose(marker_, visualization_msgs::Marker::ARROW,
                    "goal_searching_segments-" + std::to_string(seg_num),
                    color[0], color[1], color[2], 1, // color
                    pose.first, // pose
                    count++, marker_lifetime_ // count, time
                    );
            goal_selection_marker_array_.markers.push_back(marker_);
        }
        seg_num ++;
    }
    pose_t selected_goal = goal_searching_.goal_pose;
    selected_goal.z = goal_search_height;
    plotting::addMarkerWithPose(marker_, visualization_msgs::Marker::ARROW,
            "selected_searching_goal",
            0, 1, 0, 1, // color
            selected_goal, // pose
            0, marker_lifetime_, // count, time
            "", 0.8, 0.2, 0.2); // text, size_x, size_y, size_z
    goal_selection_marker_array_.markers.push_back(marker_);
    marker_array_pub_.publish(goal_selection_marker_array_);
    

    // rrt markers
    plotting::addMarkerWithPose(marker_, visualization_msgs::Marker::ARROW,
            "rrt_start",
            0, 1, 0, 1, // color
            planner_->getStartPose(), // pose
            0, 0, // count, time
            "", 0.8, 0.3, 0.3); // text, size_x, size_y, size_z
    planner_marker_array_.markers.push_back(marker_);

    plotting::addMarkerWithPose(marker_, visualization_msgs::Marker::ARROW,
            "rrt_goal",
            0, 1, 0, 1, // color
            planner_->getGoalPose(), // pose
            0, 0, // count, time
            "", 0.8, 0.3, 0.3); // text, size_x, size_y, size_z
    planner_marker_array_.markers.push_back(marker_);

    std::vector<pose_t> samples = planner_->getSamples();
    count = 0;
    for (const auto& it : samples)
    {
        plotting::addMarkerWithPose(marker_, visualization_msgs::Marker::ARROW,
                "rrt_samples",
                1, 1, 1, 1, // color
                it,
                count++, marker_lifetime_ // count, time
                );
        planner_marker_array_.markers.push_back(marker_);
    }


    if (planner_->getPathStatus())
    {
        // path
        std::vector<point2d_t<double>>* path = planner_->getPlannedPathPtr();
        for (const auto& it : *path)
        {
            // plotting::addMarker(marker, visualization_msgs::Marker::SPHERE,
            //         "planned path",
            //         0, 0, 0, 0, // color
            //         it.x, it.y, 0, // location
            //         0, 0, 0, 1, // quaternion
            //         count, 0.05 // count, time
            //         );
            // count ++;
            // marker_array.markers.push_back(marker);
            rrt_path_points->points.push_back(pcl::PointXYZI(
                        (float) it.x, (float) it.y, 0.2, 0));
            // debugger::debugOutput("[Main] point: ", it.first, 5);
        }


        // way poses
        std::vector<std::pair<pose_t, bool>> *waypoints =
            planner_->getPlannedWaypointsPtr();
        count = 0;
        for (auto it : *waypoints)
        {
            it.first.z += 0.3;
            plotting::addMarker(marker_,
              visualization_msgs::Marker::SPHERE, 
              "rrt_waypoints",
              1, 1, 0, 1, // color
              it.first.x, it.first.y, it.first.z,
              0, 0, 0, 1, // orientation
              count++, marker_lifetime_, "",
              0.2, 0.2, 0.2 // marker size
              );

            // plot arrow as path 
            // plotting::addMarkerWithPose(marker_, visualization_msgs::Marker::SPHERE,
            //         "rrt_waypoints",
            //         1, 1, 0, 1, // color
            //         pose_t(it.first),
            //         count++, marker_lifetime_ // count, time
            //         );
            planner_marker_array_.markers.push_back(marker_);
            // debugger::debugOutput("[Main] waypoints: ", it.first, 5);
        }
    }


    planner_results_lock_.lock();
    if(planner_results_.status.has_path_to_use)
    {
        
        for (const auto& it : planner_results_.path_points)
        {
            planner_path_points->points.push_back(pcl::PointXYZI(
                        (float) it.x, (float) it.y, 0.1, 0));
        }
    }
    planner_results_lock_.unlock();


    // goal selection from findNextGoalForRobot_()
    goal_selection_lock_.lock();
    if (goal_selection_.has_chosen_goal)
    {
        plotting::addMarker(marker_, visualization_msgs::Marker::SPHERE,
                "goal_for_robot (c)",
                0, 1, 1, 1, // color
                goal_selection_.chosen_goal.first.x, 
                goal_selection_.chosen_goal.first.y, 
                goal_selection_.chosen_goal.first.z, 
                0, 0, 0, 1, // orientation
                0, 0, // count, time
                "", 0.5, 0.5, 0.5); // text, size_x, size_y, size_z
        // goal for robot (arrow)
        // plotting::addMarkerWithPose(marker_, visualization_msgs::Marker::ARROW,
        //         "goal_for_robot (c)",
        //         0, 1, 1, 1, // color
        //         goal_selection_.chosen_goal.first, // pose
        //         0, 0, // count, time
        //         "", 0.9, 0.4, 0.4); // text, size_x, size_y, size_z
        planner_marker_array_.markers.push_back(marker_);


        // direction for robot
        plotting::addMarkerWithPose(marker_, visualization_msgs::Marker::ARROW,
                "dir_for_robot (b)",
                0, 0, 1, 1, // color
                pose_t(planner_info_to_controller_.pose[0],
                       planner_info_to_controller_.pose[1],
                       planner_info_to_controller_.pose[2],
                       0.0, 0.0, // roll, pitch
                       planner_info_to_controller_.torso.yaw),
                0, marker_lifetime_, // count, time
                "", 0.8, 0.3, 0.3); // text, size_x, size_y, size_z
        planner_marker_array_.markers.push_back(marker_);
        // debugger::debugOutput("[Driver]/publishToRviz] robot at the command time: ", 
        //         goal_selection_.robot_pose, 5);
        // debugger::debugOutput("[Driver]/publishToRviz] goal_for_robot: ", 
        //         goal_selection_.chosen_goal.first, 5);
        // debugger::debugOutput("[Driver]/publishToRviz] dir_for_robot: ", 
        //         pose_t(planner_info_to_controller_.pose[0],
        //                planner_info_to_controller_.pose[1],
        //                planner_info_to_controller_.pose[2],
        //                0.0, 0.0, // roll, pitch
        //                planner_info_to_controller_.torso.yaw), 5);
    }
    else
    {
        debugger::debugColorOutput("[Driver]/publishToRviz] no goal has chosen: ", 
                goal_selection_.has_chosen_goal, 5, BC);
    }
    goal_selection_lock_.unlock();


    marker_array_pub_.publish(planner_marker_array_);
    pcl_conversions::toPCL(robot_state_.time, rrt_path_points->header.stamp);
    rrt_path_pub_.publish(rrt_path_points);
    planner_path_pub_.publish(planner_path_points);
    trajectory_pub_.publish(robot_trajectory_);



    // plane information 
    // Clear messages
    visual_tools_->resetMarkerCounts();
    visual_tools_->deleteAllMarkers();
    visual_tools_->enableBatchPublishing();

    planner_->local_map_->lockTerrainThread(true);
    std::vector<plane::terrain_info_t> terrain_info = planner_->local_map_->terrain_info;
    planner_->local_map_->lockTerrainThread(false);

    // publish plane information
    for (int seg = 0; seg < terrain_plane_params_.num_planes; ++seg)
    {
        // skip if status is -1
        if (terrain_info[seg].status == 0)
            continue;
        size_t num_seg_points = terrain_info.at(seg).points.cols();

        // geometry_msgs::Vector3 scale = 
        //     visual_tools_->getScale(rviz_visual_tools::XXXXLARGE);
        geometry_msgs::Vector3 scale = 
            visual_tools_->getScale(rviz_visual_tools::XLARGE);
        std_msgs::ColorRGBA color = 
            visual_tools_->getColorScale(
                    (float) seg / terrain_plane_params_.num_planes);
        // debugger::debugColorOutput("[Driver]/publishToRviz] # points: ", 
        //         terrain_info.at(seg).points.cols(), 6, BC);
        for (int i = 0; i < num_seg_points; ++i)
        {
            visual_tools_->publishSphere(
                    terrain_info.at(seg).points.col(i).cast<double>(),
                    color, scale, "plane points - " + std::to_string(seg), i);
        }
        std::vector<float> plane_coefficients =
            terrain_info.at(seg).getPlaneCoefficients();
        Eigen::Vector3d center = terrain_info.at(seg).points.rowwise().mean().cast<double>();
        visual_tools_->publishABCDPlaneWithCenter(plane_coefficients[0],
                plane_coefficients[1],
                plane_coefficients[2],
                plane_coefficients[3], center, color, 
                terrain_plane_params_.delta_x, 2 * terrain_plane_params_.delta_y);
    }
    visual_tools_->trigger();




    // std::shared_ptr<std::vector<plane::terrain_info_t>>
    //     terrain_ptr = local_map_->getTerrainInfo();
    // for (int seg = 0; seg < terrain_plane_params_.num_planes; ++seg)
    // {
    //     geometry_msgs::Vector3 scale = 
    //         visual_tools_->getScale(rviz_visual_tools::MEDIUM);
    //     std_msgs::ColorRGBA color = 
    //         visual_tools_->getColorScale(
    //                 (float) seg / terrain_plane_params_.num_planes);

    //     for (int i = 0; i < terrain_ptr->at(seg).points.cols(); ++i)
    //     {
    //         visual_tools_->publishSphere(
    //                 terrain_ptr->at(seg).points.col(i).cast<double>(),
    //                 color, scale, "plane points");
    //     }
    //     std::vector<float> plane_coefficients =
    //         terrain_ptr->at(seg).getPlaneCoefficients();
    //     Eigen::Vector3d center = terrain_ptr->at(seg).points.rowwise().mean().cast<double>();
    //     visual_tools_->publishABCDPlaneWithCenter(plane_coefficients[0],
    //             plane_coefficients[1],
    //             plane_coefficients[2],
    //             plane_coefficients[3], center, color, 
    //             terrain_plane_params_.delta_x, 2 * terrain_plane_params_.delta_y);
    // }
    // visual_tools_->trigger();


}


bool Driver::updateGlobalMap_()
{
    // boost::mutex::scoped_lock lock(map_lock_);
    bool status = true;
    map_lock_.lock();
    if (global_map_updated_)
    {
        // update the global map in the planner
        // debugger::debugOutput("[before address]: ", &multi_layer_map_, 5);
        multi_layer_map_ = new_multi_layer_map_;
        // debugger::debugOutput("[after address]: ", &multi_layer_map_, 5);
    }
    else
    {
        status = false;
        debugger::debugWarningOutput("[Driver]/[updateGlobalMap] "
                "No new map: using the existing map", "", 10);
    }
    global_map_updated_ = false;
    map_lock_.unlock();

    // debugger::debugColorTextOutput("In updateGlobalMap",3);
    // bool map_flag = true;
    // if (!multi_layer_map_.exists("elevation_map"))
    // {
    //     debugger::debugColorTextOutput("[LocalMap]/[updateLocalMap] "
    //             "no elevation_map", 10, BR);
    //     map_flag = false;
    // }
    // if (!multi_layer_map_.exists("occupancy_map"))
    // {
    //     debugger::debugColorTextOutput("[LocalMap]/[updateLocalMap] "
    //             "no occupancy_map", 10, BR);
    //     map_flag = false;
    // }
    // if (!map_flag)
    // {
    //     debugger::debugColorTextOutput("[LocalMap]/[updateLocalMap] "
    //             "Exiting...", 10, BR);

    //     exit(-1);
    // }
    return status;
}

bool Driver::getRobotCurrentState_(robot_state_t& current_state)
{
    pose_lock_.lock();
    bool update = false;
    if (driver_method_ == 0 || driver_method_ == 1 || driver_method_ == 3)
    {
        update = robot_state_updated_;
        if (update)
        {
            current_state.setFullPose(new_inekf_state_);
            current_state.time = new_inekf_state_.header.stamp;
            // debugger::debugOutput("received pose: ", current_state.pose, 5);
            // robot_state_.printPose();
        }
    }
    else if (driver_method_ == 2)
    {
        received_udp_pose_ = udp_->getReceivedMessage();
        update = received_udp_pose_.first;
        if (update)
        {
            current_state.setPoseStandaloneFromQuaternion(
                    received_udp_pose_.second.pose[0], // x
                    received_udp_pose_.second.pose[1], // y
                    received_udp_pose_.second.pose[2], // z
                    received_udp_pose_.second.pose[3], // qw
                    received_udp_pose_.second.pose[4], // qx
                    received_udp_pose_.second.pose[5], // qy
                    received_udp_pose_.second.pose[6] // qz
                    );
            // debugger::debugOutput("[Driver]/[getRobotCurrentState] ", 
            //         control_commands::returnReceivedData(
            //         received_udp_pose_.second), 5);
        }
    }
    else
    {
        debugger::debugColorOutput("[Driver]/[waitForData] "
                "unknown driver_method_: ", driver_method_, 10);
        exit(-1);
    }
    pose_lock_.unlock();

    return update;
}


bool Driver::updateRobotPose_()
{
    //boost::mutex::scoped_lock lock(pose_lock_);
    bool status = true;
    bool pose_updated = Driver::getRobotCurrentState_(robot_state_);
    // debugger::debugOutput("updated: ", robot_state_.pose, 5);



    // bool pose_updated = robot_state_updated_;
    // if (driver_method_ == 0 || driver_method_ == 1)
    // {
    //     if (robot_state_updated_)
    //     {
    //         robot_state_.setFullPose(*new_inekf_state_);
    //         robot_state_.time = new_inekf_state_->header.stamp;
    //         // robot_state_.printPose();
    //     }
    // }
    // else if (driver_method_ == 2)
    // {
    //     received_udp_pose_ = udp_->getReceivedMessage();
    //     robot_state_updated_ = received_udp_pose_.first;
    //     if (robot_state_updated_)
    //     {
    //         robot_state_.setPoseStandaloneFromQuaternion(
    //                 received_udp_pose_.second.pose[0], // x
    //                 received_udp_pose_.second.pose[1], // y
    //                 received_udp_pose_.second.pose[2], // z
    //                 received_udp_pose_.second.pose[3], // qw
    //                 received_udp_pose_.second.pose[4], // qx
    //                 received_udp_pose_.second.pose[5], // qy
    //                 received_udp_pose_.second.pose[6] // qz
    //                 );
    //     }
    // }


    if (pose_updated)
    {
        pose_lock_.lock();
        robot_state_updated_ = false;
        pose_lock_.unlock();

        pose_updated_ = true;
        pose_look_up_counter_ ++;


        // wait until stable
        if (pose_look_up_counter_ < 5)
            status = false;
    }
    else
    {
        debugger::debugWarningOutput("[Driver]/[robot_state_updated_] "
                "No robot pose", "", 10);
        status = false;
    }
    // debugger::debugOutput("[Driver]/[UpdateRobotPose] updated pose: ", 
    //         robot_state_.pose, 5);
    // robot_state_.printPose();



    return status;

    // bool status = true;
    // pose_lock_.lock();
    // if (robot_state_updated_)
    // {
    //     robot_state_.setFullPose(*new_inekf_state_);
    //     robot_state_.time = new_inekf_state_->header.stamp; 

    //     robot_state_updated_ = false;
    //     pose_updated_ = true;
    //     pose_look_up_counter_ ++;


    //     // wait until stable
    //     // if (pose_look_up_counter_ < 5) 
    //     //     status = false;
    // }
    // else
    // {
    //     debugger::debugColorOutput("[Driver]/[update] No robot pose", "", 10, Y);
    //     status = false;

    // }
    // pose_lock_.unlock();

    // return status;
}


void Driver::getClickPointCallBack_(
        const geometry_msgs::PointStamped::ConstPtr& msg)
{
    this->goal_received_ = true;
    this->goal_updated_ = true;
    debugger::debugColorTextOutput("[Driver] New clicked goal received!", 6, BC);
    clicked_lock_.lock();
    final_goal_pose_ = pose_t(msg->point.x, msg->point.y, 0);
    clicked_lock_.unlock();




    // m_lock.lock();
    // m_goal_queue.push(msg);
    // cout << "queue size: " << m_map_queue.size() << endl;
    // m_lock.unlock();
}


void Driver::getMultiLayerCallBack_(const grid_map_msgs::GridMap& grid_map_msg)
{
    // debugger::debugColorTextOutput("In MultiLayerCallback()",3);
    map_lock_.lock();
    grid_map::GridMapRosConverter::fromMessage(grid_map_msg, new_multi_layer_map_);
    global_map_updated_ = true;


    // debugger::debugColorTextOutput("===== In map callback function =====", 5, 
    //         BM, BOLD);
    // bool map_flag = true;
    // if (!new_multi_layer_map_.exists("elevation_map"))
    // {
    //     debugger::debugColorTextOutput("[LocalMap]/[updateLocalMap] "
    //             "no elevation_map", 10, BR);
    //     map_flag = false;
    // }
    // if (!new_multi_layer_map_.exists("occupancy_map"))
    // {
    //     debugger::debugColorTextOutput("[LocalMap]/[updateLocalMap] "
    //             "no occupancy_map", 10, BR);
    //     map_flag = false;
    // }
    // if (!map_flag)
    // {
    //     debugger::debugColorTextOutput("[LocalMap]/[updateLocalMap] "
    //             "Exiting...", 10, BR);

    //     exit(-1);
    // }

    map_lock_.unlock();

    // update global map in planer 
    // if (data_received_)
    //     clf_planner->updateGlobalMap(msg);
}

void Driver::getUDPCallBack_()
{
    while(1)
    {
        received_udp_pose_ = udp_->getReceivedMessage();
        // debugger::debugOutput("[UDPCallback]: ", received_udp_pose_.first, 5);
        if (received_udp_pose_.first)
        {
            robot_state_updated_ = true;
            break;
        }
        usleep(0.5e6);
    }
}


void Driver::getInEKFCallBack_(const inekf_msgs::State& msg)
{
    // debugger::debugColorTextOutput("In InEKFCallback()", 3);
    pose_lock_.lock();
    new_inekf_state_ = msg;
    robot_state_updated_ = true;
    pose_lock_.unlock();
}

bool Driver::lookUpRobotPose()
{
    // try{
    //     listener_.lookupTransform(map_frame_, robot_frame_, ros::Time::now(), 
    //                               robot_state_.robot_tf_pose);
    //     ROS_INFO("[LookUp] Looked up robot pose");
    //     robot_state_.time = ros::Time::now();
    //     pose_updated_ = true;
    //     pose_look_up_counter_ ++;


    //     // wait until stable
    //     if (pose_look_up_counter_ < 5) 
    //         return false;
    // }
    // catch (tf::TransformException ex){
    //     ROS_WARN("[LookUp] No robot pose");
    //     ROS_ERROR("%s", ex.what());
    //     /// robot_state = robot_state_old;
    //     ros::Duration(1.0).sleep();
    //     pose_updated_ = false;

    //     if (pose_look_up_counter_ == 0) 
    //         return false;
    // }
    // // cout << "---------" << endl;
    // // cout << "res: " << m_current_map->info.resolution << endl;
    // // ROS_DEBUG_STREAM("x in grid:" << robot_state.robot_pose.getOrigin().x()/m_current_map->info.resolution); // the grid map
    // // ROS_DEBUG_STREAM("y in grid:" << robot_state.robot_pose.getOrigin().y()/m_current_map->info.resolution); // the grid map
    // Driver::computeRobotVelocityFromTF();


    // // look up robot's pose in the grid map
    // // occupancy_grid_utils::Cell robot_pose((robot_state.robot_pose.getOrigin().x()
    // //             - m_current_map->info.origin.position.x)/m_current_map->info.resolution,
    // //         (robot_state.robot_pose.getOrigin().y()
    // //          - m_current_map->info.origin.position.y)/m_current_map->info.resolution);
    // // robot_state.grid_index = occupancy_grid_utils::cellIndex(m_current_map->info, robot_pose);
    // // robot_state.pose_in_cell.x = robot_pose.x; // pose in cell
    // // robot_state.pose_in_cell.y = robot_pose.y; // pose in cell

    // // robot_state_old = robot_state;

    // return true;
    // cout << "index: " << robot_state.grid_index << endl;
    // Mapper::updateBoundries();
    // Mapper::frontiers();

    // Mapper::checkFork();
    // Mapper::goalSelection();
    // cout << "---------" << endl;
    // cout << "Looked up" << endl;
}

void Driver::computeRobotVelocityFromTF()
{
    // tf::Quaternion q(robot_state_.robot_tf_pose.getRotation().x(),
    //                  robot_state_.robot_tf_pose.getRotation().y(),
    //                  robot_state_.robot_tf_pose.getRotation().z(),
    //                  robot_state_.robot_tf_pose.getRotation().w());

    // tf::Matrix3x3 m(q);
    // m.getRPY(robot_state_.roll, robot_state_.pitch, robot_state_.yaw); // in radian
    // robot_state_.setFullPose(robot_state_.robot_tf_pose, 
    //                      robot_state_.pitch, robot_state_.roll, robot_state_.yaw);
    // // debugger::debugOutput()
    // // ROS_DEBUG_STREAM(
    // //         "[Robot] (r,p,y) = "<< "("<< robot_state_.roll * 180/M_PI << ", " <<
    // //         robot_state_.pitch * 180/M_PI << ", " << robot_state_.yaw * 180/M_PI << ")");

    // // direction velocity
    // robot_state_.dir_velocity = velocity_t(std::cos(robot_state_.yaw), std::sin(robot_state_.yaw));


    // // real velocity
    // // double duration = robot_state.time.toSec() - robot_state_old.time.toSec();
    // // robot_state.velocity.v_x = (robot_state.robot_pose.getOrigin().x() -
    // //         robot_state_old.robot_pose.getOrigin().x())/duration;
    // // robot_state.velocity.v_y = (robot_state.robot_pose.getOrigin().y() -
    // //         robot_state_old.robot_pose.getOrigin().y())/duration;
    // // ROS_DEBUG_STREAM("[Robot] Dir_vx: " << robot_state.dir_velocity.v_x << ", dir_vy: " << robot_state.dir_velocity.v_y);
    // // robot_state_old.time = robot_state.time;
}


void Driver::waitForData_() {
    if (driver_method_ == 2)
    {
        boost::thread listen_to_udp_pose(&Driver::getUDPCallBack_, this);
    }

    if (driver_method_ == 0 || driver_method_ == 2 || driver_method_ == 3)
    {
        while (ros::ok() && (!global_map_updated_ || !robot_state_updated_))
        {
            ROS_WARN_THROTTLE(1, "Received map: %i", global_map_updated_);
            ROS_WARN_THROTTLE(1, "Received pose: %i", robot_state_updated_);
            sleep(0.5);
        }

        if (driver_method_ == 3)
        {
            pose_lock_.lock();
            tf2::Quaternion q(
                    new_inekf_state_.orientation.x, 
                    new_inekf_state_.orientation.y, 
                    new_inekf_state_.orientation.z, 
                    new_inekf_state_.orientation.w);
            tf2::Matrix3x3 m(q);
            double roll;
            double pitch;
            double yaw;
            m.getRPY(roll, pitch, yaw); // in radian

            final_goal_pose_ = pose_t(
                    new_inekf_state_.position.x + 
                        std::cos(yaw) * final_goal_pose_.x, 
                    new_inekf_state_.position.y + 
                        std::sin(yaw) * final_goal_pose_.y, 
                    yaw + final_goal_pose_.theta);
            pose_lock_.unlock();
        }
    }
    else if (driver_method_ == 1) // need to check if goal clicked
    { 
        while (ros::ok() && 
               (!global_map_updated_ || !robot_state_updated_ || !goal_received_))
        {
            // cout << "-------" << endl;
            // cout << "1:" << m_data_received << endl;
            // cout << "2:" << m_goal_received << endl;
            ROS_WARN_THROTTLE(1, "Received map: %i", global_map_updated_);
            ROS_WARN_THROTTLE(1, "Received pose: %i", robot_state_updated_);
            ROS_WARN_THROTTLE(1, "Received goal: %i", goal_received_);
            sleep(0.5);
        }
    }
    else
    {
        debugger::debugColorOutput("[Driver]/[waitForData] "
                "unknown driver_method_: ", driver_method_, 10);
        exit(-1);
    }
    ROS_INFO("Received map: %i", global_map_updated_);
    ROS_INFO("Received pose: %i", robot_state_updated_);
}


void Driver::spin_()
{
    while (ros::ok()){
        ros::spinOnce();
    }
}

void Driver::logCommands(
        const planner_info_to_controller_t& data, 
        const control_variables_t& control_variables)
{
        size_t precision = 2;
        command_csv_ << utils::toStringWithPrecision(data.behavior, precision)
         << utils::toStringWithPrecision(data.velocity[0], precision) 
         << utils::toStringWithPrecision(data.velocity[1], precision)
         << utils::toStringWithPrecision(data.velocity[2], precision)
         << utils::toStringWithPrecision(control_variables.vr, precision)
         << utils::toStringWithPrecision(control_variables.vd, precision)
         << utils::toStringWithPrecision(data.torso.roll, precision)
         << utils::toStringWithPrecision(data.torso.pitch, precision)
         << utils::toStringWithPrecision(data.torso.yaw, precision) << endrow;
         // debugger::debugOutput("[logCommands] control command: ", 
         //        control_commands::returnPublishData(data), 5);
}

// bool Driver::checkROSParam(const ros::NodeHandle& nh, 
//                    const std::string& ros_param_name,
//                    std::string& member_name)
// {
//     if (!nh.getParam(ros_param_name, member_name))
//     {
//         debugger::debugColorOutput("[Driver]/[getParameters] ", 
//                 "missing " + member_name, 10, R);
//         return false;
//     }
//     return true;
// }

void Driver::getGoalPoseForMap_(int map_number)
{
    switch (map_number)
    {
        case 1:
            final_pose_x_list_ = {13};
            final_pose_y_list_ = {-13};
            final_pose_yaw_list_ = {0};
            break;
        case 3:
            final_pose_x_list_ = {13};
            final_pose_y_list_ = {-13};
            final_pose_yaw_list_ = {0};
            break;
        case 100:
            final_pose_x_list_ = {0};
            final_pose_y_list_ = {8};
            final_pose_yaw_list_ = {90};
            break;
        case 101:
            final_pose_x_list_ = {0};
            final_pose_y_list_ = {8};
            final_pose_yaw_list_ = {90};
            break;
        case 102:
            final_pose_x_list_ = {0,    0, -14, -14, -10};
            final_pose_y_list_ = {0,    9,   9,   0,   0};
            final_pose_yaw_list_ = {0, 90, -90,  90,   0};
            break;
        case 110:
            final_pose_x_list_ = {22};
            final_pose_y_list_ = {12};
            final_pose_yaw_list_ = {0};
            break;
        case 111:
            final_pose_x_list_ = {22};
            final_pose_y_list_ = {12};
            final_pose_yaw_list_ = {0};
            break;
        case 112:
            final_pose_x_list_ = {21};
            final_pose_y_list_ = {-12};
            final_pose_yaw_list_ = {0};
            break;
        case 113:
            final_pose_x_list_ = {21};
            final_pose_y_list_ = {-12};
            final_pose_yaw_list_ = {0};
            break;
        case 63:
            final_pose_x_list_ = {23};
            final_pose_y_list_ = {13};
            final_pose_yaw_list_ = {0};
            break;
        default:
            std::string error_msg = "ERROR: no goal pose "
                "supported for this map: " + std::to_string(map_number);
            debugger::debugExitColor(error_msg, __LINE__, __FILE__);
    }

    final_goal_pose_ = pose_t(final_pose_x_list_[0], final_pose_y_list_[0], 
                              deg_to_rad(final_pose_yaw_list_[0]));
        final_pose_x_list_.erase(final_pose_x_list_.begin());
        final_pose_y_list_.erase(final_pose_y_list_.begin());
        final_pose_yaw_list_.erase(final_pose_yaw_list_.begin());
}

bool Driver::getParameters_()
{
    std::string title_name("[Driver]/[getParameters] ");
    bool received_all = true;

    // log command
    ros_utils::checkROSParam(nh_, "log_commands", log_commands_, 
            getNameOf(log_commands_), title_name, received_all);

     // driver parameters
    ros_utils::checkROSParam(nh_, "driver_mode", driver_method_, 
            getNameOf(driver_method_), title_name, received_all);
    ros_utils::checkROSParam(nh_, "clf_model", clf_model_, 
            getNameOf(clf_model_), title_name, received_all);


    ros_utils::checkROSParam(nh_, "replanning_rate", replanning_rate_, 
            getNameOf(replanning_rate_), title_name, received_all);
    ros_utils::checkROSParam(nh_, "publishing_rate", publishing_rate_, 
            getNameOf(publishing_rate_), title_name, received_all);

    ros_utils::checkROSParam(nh_, "old_path_timeout", old_path_timeout_, 
            getNameOf(old_path_timeout_), title_name, received_all);
    ros_utils::checkROSParam(nh_, "walking_in_place_timeout", 
            walking_in_place_timeout_, 
            getNameOf(walking_in_place_timeout_), title_name, received_all);
    ros_utils::checkROSParam(nh_, "cost_diff_threshold_within_timeout", 
            cost_diff_threshold_within_timeout_, 
            getNameOf(cost_diff_threshold_within_timeout_), 
            title_name, received_all);
    ros_utils::checkROSParam(nh_, "command_history_length", 
            command_history_length_, 
            getNameOf(command_history_length_), 
            title_name, received_all);


    ros_utils::checkROSParam(nh_, "map_number", 
            map_number_, getNameOf(map_number_), title_name, received_all);


    ros_utils::checkROSParam(nh_, "final_pose_x_list", final_pose_x_list_, 
            getNameOf(final_pose_x_list_), title_name, received_all);
    ros_utils::checkROSParam(nh_, "final_pose_y_list", final_pose_y_list_, 
            getNameOf(final_pose_y_list_), title_name, received_all);
    ros_utils::checkROSParam(nh_, "final_pose_yaw_list", final_pose_yaw_list_, 
            getNameOf(final_pose_yaw_list_), title_name, received_all);


     // communication 
    ros_utils::checkROSParam(nh_, "communication/port_to_send", 
            udp_info_.port_to_send, 
             getNameOf(udp_info_) + "." + getNameOf(udp_info_.port_to_send), 
             title_name, received_all);
    ros_utils::checkROSParam(nh_, "communication/port_to_receive", 
            udp_info_.port_to_receive, 
             getNameOf(udp_info_) + "." + getNameOf(udp_info_.port_to_receive), 
             title_name, received_all);
    ros_utils::checkROSParam(nh_, "communication/ip_to_send", udp_info_.ip_to_send, 
                getNameOf(udp_info_) + "." + getNameOf(udp_info_.ip_to_send), 
                title_name, received_all);


    // cost map params
    ros_utils::checkROSParam(nh_, "cost_map/obstacle_cost", cost_map_params_.obstacle_cost, 
            getNameOf(cost_map_params_) + "." + getNameOf(cost_map_params_.obstacle_cost), 
            title_name, received_all);
    ros_utils::checkROSParam(nh_, "cost_map/unknown_cost", cost_map_params_.unknown_cost, 
            getNameOf(cost_map_params_) + "." + getNameOf(cost_map_params_.unknown_cost), 
            title_name, received_all);
    ros_utils::checkROSParam(nh_, "cost_map/weight_of_height", cost_map_params_.weight_of_height, 
            getNameOf(cost_map_params_) + "." + getNameOf(cost_map_params_.weight_of_height), 
            title_name, received_all);
    ros_utils::checkROSParam(nh_, "cost_map/outside_map_cost", cost_map_params_.outside_map_cost, 
            getNameOf(cost_map_params_) + "." + getNameOf(cost_map_params_.outside_map_cost), 
            title_name, received_all);



    // rrt parameters
    ros_utils::checkROSParam(nh_, "rrt_params/mode", rrt_params_.mode, 
            getNameOf(rrt_params_) + "." + getNameOf(rrt_params_.mode), 
            title_name, received_all);
    ros_utils::checkROSParam(nh_, "rrt_params/max_extension_length",
            rrt_params_.max_extension_length,
            getNameOf(rrt_params_) + "." +
            getNameOf(rrt_params_.max_extension_length),
            title_name, received_all);
    ros_utils::checkROSParam(nh_, "rrt_params/goal_threshold",
            rrt_params_.goal_threshold,
            getNameOf(rrt_params_) + "." +
            getNameOf(rrt_params_.goal_threshold),
            title_name, received_all);
    ros_utils::checkROSParam(nh_, "rrt_params/path_obstacle_threshold",
            rrt_params_.path_obstacle_threshold,
            getNameOf(rrt_params_) + "." +
            getNameOf(rrt_params_.path_obstacle_threshold),
            title_name, received_all);
    ros_utils::checkROSParam(nh_, "rrt_params/backward_motion_sampling_rate",
            rrt_params_.backward_motion_sampling_rate,
            getNameOf(rrt_params_) + "." +
            getNameOf(rrt_params_.backward_motion_sampling_rate),
            title_name, received_all);

    // ros_utils::checkROSParam(nh_, "rrt_params/num_samples", 
    //         rrt_params_.num_samples, 
    //         getNameOf(rrt_params_) + "." + getNameOf(rrt_params_.num_samples), 
    //         title_name, received_all);
    // ros_utils::checkROSParam(nh_, "rrt_params/allowed_computation_time", 
    //         rrt_params_.allowed_computation_time, 
    //         getNameOf(rrt_params_) + "." + 
    //         getNameOf(rrt_params_.allowed_computation_time), title_name, received_all);
    // ros_utils::checkROSParam(nh_, "rrt_params/terminate_if_path", 
    //         rrt_params_.terminate_if_path, 
    //         getNameOf(rrt_params_) + "." + 
    //         getNameOf(rrt_params_.terminate_if_path), title_name, received_all);
    // ros_utils::checkROSParam(nh_, "rrt_params/length_of_local_map", 
    //         length_of_local_map_, 
    //         getNameOf(length_of_local_map_), title_name, received_all);
    


    // lyapunov distance params 
    lyap_dist_params_.clf_model = clf_model_;

    // dirrerential driven robot
    ros_utils::checkROSParam(nh_, "lyap_dist/k_phi", 
            lyap_dist_params_.k_phi, 
            getNameOf(lyap_dist_params_) + "." + 
            getNameOf(lyap_dist_params_.k_phi), title_name, received_all);
    ros_utils::checkROSParam(nh_, "lyap_dist/k_delta", 
            lyap_dist_params_.k_delta, 
            getNameOf(lyap_dist_params_) + "." + 
            getNameOf(lyap_dist_params_.k_delta), title_name, received_all);
    ros_utils::checkROSParam(nh_, "lyap_dist/small_radius", 
            lyap_dist_params_.small_radius, 
            getNameOf(lyap_dist_params_) + "." + 
            getNameOf(lyap_dist_params_.small_radius), title_name, received_all);
    int vector_field_type;
    ros_utils::checkROSParam(nh_, "lyap_dist/vector_field_type", 
            vector_field_type, 
            getNameOf(vector_field_type),
            title_name, received_all);
    decideStabilizingVectorFieldType(
            lyap_dist_params_.vector_field_type, vector_field_type);

    // omni-directional robot 
    ros_utils::checkROSParam(nh_, "lyap_dist/gamma", 
            lyap_dist_params_.gamma, 
            getNameOf(lyap_dist_params_) + "." + 
            getNameOf(lyap_dist_params_.gamma), title_name, received_all);
    ros_utils::checkROSParam(nh_, "lyap_dist/alpha", 
            lyap_dist_params_.alpha, 
            getNameOf(lyap_dist_params_) + "." + 
            getNameOf(lyap_dist_params_.alpha), title_name, received_all);
    ros_utils::checkROSParam(nh_, "lyap_dist/beta", 
            lyap_dist_params_.beta, 
            getNameOf(lyap_dist_params_) + "." + 
            getNameOf(lyap_dist_params_.beta), title_name, received_all);
    ros_utils::checkROSParam(nh_, "lyap_dist/k_r1", 
            lyap_dist_params_.k_r1, 
            getNameOf(lyap_dist_params_) + "." + 
            getNameOf(lyap_dist_params_.k_r1), title_name, received_all);
    ros_utils::checkROSParam(nh_, "lyap_dist/k_r2", 
            lyap_dist_params_.k_r2, 
            getNameOf(lyap_dist_params_) + "." + 
            getNameOf(lyap_dist_params_.k_r2), title_name, received_all);
    ros_utils::checkROSParam(nh_, "lyap_dist/k_delta1", 
            lyap_dist_params_.k_delta1, 
            getNameOf(lyap_dist_params_) + "." + 
            getNameOf(lyap_dist_params_.k_delta1), title_name, received_all);
    ros_utils::checkROSParam(nh_, "lyap_dist/k_delta2", 
            lyap_dist_params_.k_delta2, 
            getNameOf(lyap_dist_params_) + "." + 
            getNameOf(lyap_dist_params_.k_delta2), title_name, received_all);
    ros_utils::checkROSParam(nh_, "lyap_dist/k_delta_to_manifold", 
            lyap_dist_params_.k_delta_to_manifold, 
            getNameOf(lyap_dist_params_) + "." + 
            getNameOf(lyap_dist_params_.k_delta_to_manifold), 
            title_name, received_all);

    int omni_CLF_model;
    ros_utils::checkROSParam(nh_, "lyap_dist/omni_CLF_model", 
            omni_CLF_model, 
            getNameOf(omni_CLF_model),
            title_name, received_all);
    decideOmniCLFModel(lyap_dist_params_.omni_CLF_model, omni_CLF_model);

    int omni_CLF_solution;
    ros_utils::checkROSParam(nh_, "lyap_dist/omni_CLF_solution", 
            omni_CLF_solution, 
            getNameOf(omni_CLF_solution),
            title_name, received_all);
    decideOmniCLFSolution(lyap_dist_params_.omni_CLF_solution, omni_CLF_solution);

    //debugger::debugColorOutput("[Driver]/[getParam] vector_field_type: ", 
    //                           lyap_dist_params_.vector_field_type, 10, BR, BOLD);
    //debugger::debugColorOutput("[Driver]/[getParam] Omni_CLF_model: ", 
    //                           lyap_dist_params_.omni_CLF_model, 10, BR, BOLD);
    //debugger::debugColorOutput("[Driver]/[getParam] Omni_CLF_solution: ", 
    //                           lyap_dist_params_.omni_CLF_solution, 10, BR, BOLD);


    // local map 
    ros_utils::checkROSParam(nh_, "local_map/mode", 
            local_map_params_.mode, 
            getNameOf(local_map_params_) + "." + 
            getNameOf(local_map_params_.mode), title_name, received_all);
    ros_utils::checkROSParam(nh_, "local_map/obstacle_threshold", 
            local_map_params_.obstacle_threshold, 
            getNameOf(local_map_params_) + "." + 
            getNameOf(local_map_params_.obstacle_threshold), title_name, received_all);
    ros_utils::checkROSParam(nh_, "local_map/length", 
            local_map_params_.length, 
            getNameOf(local_map_params_) + "." + 
            getNameOf(local_map_params_.length), title_name, received_all);
    ros_utils::checkROSParam(nh_, "local_map/nan_percentage_in_radius", 
            local_map_params_.nan_percentage_in_radius, 
            getNameOf(local_map_params_) + "." + 
            getNameOf(local_map_params_.nan_percentage_in_radius), 
            title_name, received_all);
    ros_utils::checkROSParam(nh_, "local_map/smooth_radius", 
            local_map_params_.smooth_radius, 
            getNameOf(local_map_params_) + "." + 
            getNameOf(local_map_params_.smooth_radius), title_name, received_all);
    local_map_params_.cost_params = cost_map_params_;
    ros_utils::checkROSParam(nh_, "local_map/SDF_radius", 
            local_map_params_.SDF_radius, 
            getNameOf(local_map_params_) + "." + 
            getNameOf(local_map_params_.SDF_radius), 
            title_name, received_all);



    // pose sampler
    ros_utils::checkROSParam(nh_, "pose_sampler_params/sampling_mode", 
            pose_sampler_params_.sampling_mode, 
            getNameOf(pose_sampler_params_) + "." + 
            getNameOf(pose_sampler_params_.sampling_mode), title_name, received_all);
    ros_utils::checkROSParam(nh_, "pose_sampler_params/goal_bias", 
            pose_sampler_params_.goal_bias, 
            getNameOf(pose_sampler_params_) + "." + 
            getNameOf(pose_sampler_params_.goal_bias), title_name, received_all);
    ros_utils::checkROSParam(nh_, "pose_sampler_params/front_bias", 
            pose_sampler_params_.front_bias, 
            getNameOf(pose_sampler_params_) + "." + 
            getNameOf(pose_sampler_params_.front_bias), title_name, received_all);
    ros_utils::checkROSParam(nh_, "pose_sampler_params/front_angle", 
            pose_sampler_params_.front_angle, 
            getNameOf(pose_sampler_params_) + "." + 
            getNameOf(pose_sampler_params_.front_angle), title_name, received_all);
    ros_utils::checkROSParam(nh_, "pose_sampler_params/distance_threshold", 
            pose_sampler_params_.distance_threshold, 
            getNameOf(pose_sampler_params_) + "." + 
            getNameOf(pose_sampler_params_.distance_threshold), title_name, received_all);


    // goal searching parameters
    ros_utils::checkROSParam(nh_, "goal_search_mode", 
                          goal_searching_.goal_search_mode, 
                          getNameOf(goal_searching_) + "." + 
                          getNameOf(goal_searching_.goal_search_mode), 
                          title_name, received_all);
    ros_utils::checkROSParam(nh_, "goal_search_radius", 
                          goal_searching_.search_radius, 
                          getNameOf(goal_searching_) + "." + 
                          getNameOf(goal_searching_.search_radius), 
                          title_name, received_all);
    ros_utils::checkROSParam(nh_, "goal_search_delta_theta", 
                          goal_searching_.delta_theta, 
                          getNameOf(goal_searching_) + "." + 
                          getNameOf(goal_searching_.delta_theta), 
                          title_name, received_all);
    ros_utils::checkROSParam(nh_, "goal_search_view_left", 
                          goal_searching_.search_view.first, 
                          getNameOf(goal_searching_) + "." + 
                          getNameOf(goal_searching_.search_view) + "." +
                          getNameOf(goal_searching_.search_view.first), title_name,
                          received_all);
    ros_utils::checkROSParam(nh_, "goal_search_view_right", 
                          goal_searching_.search_view.second, 
                          getNameOf(goal_searching_) + "." + 
                          getNameOf(goal_searching_.search_view) + "." +
                          getNameOf(goal_searching_.search_view.second), title_name,
                          received_all);
    ros_utils::checkROSParam(nh_, "goal_search_buffer", 
            goal_searching_.buffer, 
            getNameOf(goal_searching_) + "." + 
            getNameOf(goal_searching_.buffer), title_name, received_all);
    ros_utils::checkROSParam(nh_, "percentage_weight_of_orientation", 
            goal_searching_.percentage_weight_of_orientation, 
            getNameOf(goal_searching_) + "." + 
            getNameOf(goal_searching_.percentage_weight_of_orientation), 
            title_name, received_all);
    ros_utils::checkROSParam(nh_, "weight_subgoal_and_final_goal", 
            goal_searching_.weight_subgoal_and_final_goal, 
            getNameOf(goal_searching_) + "." + 
            getNameOf(goal_searching_.weight_subgoal_and_final_goal), 
            title_name, received_all);
    ros_utils::checkROSParam(nh_, "goal_segment_min_len", 
            goal_searching_.goal_segment_minimum_len, 
            getNameOf(goal_searching_) + "." + 
            getNameOf(goal_searching_.goal_segment_minimum_len), title_name, received_all);
    ros_utils::checkROSParam(nh_, "goal_segment_max_cost", 
            goal_searching_.goal_segment_max_cost, 
            getNameOf(goal_searching_) + "." + 
            getNameOf(goal_searching_.goal_segment_max_cost), title_name, received_all);
    ros_utils::checkROSParam(nh_, "goal_segment_linkage_dis_weight", 
            goal_searching_.goal_segment_linkage_dis_weight, 
            getNameOf(goal_searching_) + "." + 
            getNameOf(goal_searching_.goal_segment_linkage_dis_weight), title_name, received_all);
    ros_utils::checkROSParam(nh_, "goal_segment_linkage_cost_threshold", 
            goal_searching_.goal_segment_linkage_cost_threshold, 
            getNameOf(goal_searching_) + "." + 
            getNameOf(goal_searching_.goal_segment_linkage_cost_threshold), title_name, received_all);
    ros_utils::checkROSParam(nh_, "goal_behavior", 
            goal_searching_.goal_behavior, 
            getNameOf(goal_searching_) + "." + 
            getNameOf(goal_searching_.goal_behavior), title_name, received_all);


    // robot params
    ros_utils::checkROSParam(nh_, "robot_params/heading_angle_preintegration", 
            robot_params_.heading_angle_preintegration, 
            getNameOf(robot_params_) + "." + 
            getNameOf(robot_params_.heading_angle_preintegration), 
            title_name, received_all);
    ros_utils::checkROSParam(nh_, "robot_params/step_time_interval", 
            robot_params_.step_time_interval, 
            getNameOf(robot_params_) + "." + 
            getNameOf(robot_params_.step_time_interval), 
            title_name, received_all);
    double default_omega;
    ros_utils::checkROSParam(nh_, "robot_params/default_omega", 
            default_omega, 
            getNameOf(default_omega), 
            title_name, received_all);
    robot_params_.default_omega = deg_to_rad(default_omega);

    ros_utils::checkROSParam(nh_, "robot_params/enlarge_mode", 
            robot_params_.enlarge_mode, 
            getNameOf(robot_params_) + "." + 
            getNameOf(robot_params_.enlarge_mode), 
            title_name, received_all);
    ros_utils::checkROSParam(nh_, "robot_params/normalized_speed", 
            robot_params_.normalized_speed, 
            getNameOf(robot_params_) + "." + 
            getNameOf(robot_params_.normalized_speed), 
            title_name, received_all);
    ros_utils::checkROSParam(nh_, "robot_params/velocity_factor", 
            robot_params_.velocity_factor, 
            getNameOf(robot_params_) + "." + 
            getNameOf(robot_params_.velocity_factor), 
            title_name, received_all);
    ros_utils::checkROSParam(nh_, "robot_params/vx_upper_bound", 
            robot_params_.vx_upper_bound, 
            getNameOf(robot_params_) + "." + 
            getNameOf(robot_params_.vx_upper_bound), 
            title_name, received_all);
    ros_utils::checkROSParam(nh_, "robot_params/vy_upper_bound", 
            robot_params_.vy_upper_bound, 
            getNameOf(robot_params_) + "." + 
            getNameOf(robot_params_.vy_upper_bound), 
            title_name, received_all);


    // debugger::debugColorOutput("[Driver]/[getParam] heading_angle_preintegration: ", 
    //                            robot_params_.heading_angle_preintegration, 10, BR, BOLD);
    // debugger::debugColorOutput("[Driver]/[getParam] step_time_interval: ", 
    //                            robot_params_.step_time_interval, 10, BR, BOLD);
    // debugger::debugColorOutput("[Driver]/[getParam] default_omega: ", 
    //                            robot_params_.default_omega, 10, BR, BOLD);



    // general parameters
    ros_utils::checkROSParam(nh_, "map_frame", map_frame_, 
                          getNameOf(map_frame_), title_name, received_all);
    ros_utils::checkROSParam(nh_, "robot_frame", robot_frame_, 
                          getNameOf(robot_frame_), title_name, received_all);
    ros_utils::checkROSParam(nh_, "multi_layer_map_topic", multi_layer_map_topic_, 
                          getNameOf(multi_layer_map_topic_), title_name, received_all);
    ros_utils::checkROSParam(nh_, "inekf_topic", inekf_topic_, 
                          getNameOf(inekf_topic_), title_name, received_all);
    ros_utils::checkROSParam(nh_, "is_subgoal_threshold", is_subgoal_threshold_, 
                          getNameOf(is_subgoal_threshold_), title_name, received_all);
    ros_utils::checkROSParam(nh_, "marker_lifetime", marker_lifetime_, 
                          getNameOf(marker_lifetime_), title_name, received_all);

    // terrain info parameters
    ros_utils::checkROSParam(nh_, "plane_params/delta_x", 
            terrain_plane_params_.delta_x, 
            getNameOf(terrain_plane_params_) + "." + 
            getNameOf(terrain_plane_params_.delta_x), 
            title_name, received_all);
    ros_utils::checkROSParam(nh_, "plane_params/delta_y", 
            terrain_plane_params_.delta_y, 
            getNameOf(terrain_plane_params_) + "." + 
            getNameOf(terrain_plane_params_.delta_y), 
            title_name, received_all);
    ros_utils::checkROSParam(nh_, "plane_params/num_planes", 
            terrain_plane_params_.num_planes, 
            getNameOf(terrain_plane_params_) + "." + 
            getNameOf(terrain_plane_params_.num_planes), 
            title_name, received_all);
    ros_utils::checkROSParam(nh_, "plane_params/min_num_points_for_fitting", 
            terrain_plane_params_.min_num_points_for_fitting, 
            getNameOf(terrain_plane_params_) + "." + 
            getNameOf(terrain_plane_params_.min_num_points_for_fitting), 
            title_name, received_all);



    if (!received_all)
    {
        driver_method_ = 0;
        clf_model_ = 0;
        replanning_rate_ = 5;
        publishing_rate_ = 300;
        old_path_timeout_ = 5; // seconds
        is_subgoal_threshold_ = 0.2; // meter
        cost_diff_threshold_within_timeout_ = 5;

        pose_sampler_params_.goal_bias = 0.2;
        pose_sampler_params_.distance_threshold = 0;

        // goal searching parameters
        goal_searching_.search_radius = 2;
        goal_searching_.delta_theta = 3;
        goal_searching_.search_view = {-10, 10};


        udp_info_.port_to_send = "26000"; 
        udp_info_.port_to_receive = "28000"; 
        udp_info_.ip_to_send = "10.10.10.101";


        map_frame_ = "map";
        robot_frame_ = "cassie";
        multi_layer_map_topic_ = "/fake_map_publisher/multi_layer_map";
        inekf_topic_ = "/fake_robot_publisher/inekf";
        marker_lifetime_ = 0.2;
    }

    allow_planning_time_ = 1.0/replanning_rate_;
    final_goal_pose_ = pose_t(final_pose_x_list_[0], final_pose_y_list_[0], 
            deg_to_rad(final_pose_yaw_list_[0]));
    final_pose_x_list_.erase(final_pose_x_list_.begin());
    final_pose_y_list_.erase(final_pose_y_list_.begin());
    final_pose_yaw_list_.erase(final_pose_yaw_list_.begin());
    // if (map_number_ < 0)
    // {
    //     is_exploration_ = 1;
    // }
    // else
    // {
    //     is_exploration_ = 0;
    // }


    return received_all;
}



Driver::~Driver() { }


} /* bipedlab */ 
