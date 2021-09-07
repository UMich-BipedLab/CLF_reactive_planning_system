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


#include <unistd.h>
#include "driver.h"
#include "utils/plotting.h"
#include "utils/utils.h"
#include "utils/ros_utils.h"

#include "point.h"


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



    // subscribers
    map_sub_ = nh_.subscribe(std::string(multi_layer_map_topic_), 10, 
                               &Driver::getMultiLayerCallBack_, this);
    inekf_sub_ = nh_.subscribe(std::string(inekf_topic_), 10, 
                               &Driver::getInEKFCallBack_, this);
    click_point_sub_ = nh_.subscribe("/clicked_point", 10, 
                                     &Driver::getClickPointCallBack_, this);
    debugger::debugOutput("[Driver] subscribed to: ", multi_layer_map_topic_, 10);
    debugger::debugOutput("[Driver] subscribed to: ", inekf_topic_, 10);


    // publishers
    global_map_pub_ = nh_.advertise<grid_map_msgs::GridMap>("world_map", 1, true);
    local_map_pub_ = nh_.advertise<grid_map_msgs::GridMap>("local_map", 1, true);
    path_pub_ = nh_.advertise<pcl::PointCloud<pcl::PointXYZI>> ("path_points", 1);

    marker_pub_ = nh_.advertise<visualization_msgs::Marker>(
            "visualization_marker", 10);
    marker_array_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(
            "MarkerArray", 10);

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
            robot_state_.pose, robot_state_.pose,
            robot_state_, pose_sampler_params_, rrt_params_);
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
    double allow_time = 1.0/replanning_rate_;
    while(ros::ok())
    {
        debugger::debugTitleTextOutput("[Driver]", "driveRobot", 10, G);
        
        // update the robot pose in the planner
        debugger::debugTextOutput("[Driver]/[driveRobot] updateRobotPose", 5);
        if (!Driver::updateRobotPose_())
        {
            // TODO: publish standing still commmand
             
            ros::Duration(1.0).sleep();
            continue;
        }

        // update the global map in the planner
        debugger::debugTextOutput("[Driver]/[driveRobot] updateGlobalMap", 5);
        Driver::updateGlobalMap_();


        // using the current pose and the map to update the locap map
        debugger::debugTextOutput("[Driver]/[driveRobot] updateLocalMap", 5);
        planner_->updateLocalMap();

        // check fork

        // search a goal
        debugger::debugTextOutput("[Driver]/[driveRobot] decideGoalForPlanner", 5);
        Driver::decideGoalForPlanner();


        // run rrt
        debugger::debugOutput("[Driver]/[driveRobot] Running CLFRRT...", "", 5);
        planner_->setStartPose(robot_state_.pose);
        planner_->setGoalPose(goal_pose_);
        bool found_path = planner_->findNewPath(SIZE_MAX, allow_time, 1);

        // improve the path if have extra time
        planner_->printStatus(5, "Original Results");
        bool improved_path = planner_->addMoreTimeNSamples(SIZE_MAX, 
                allow_time - planner_->total_seconds_spent_, 0);


        if (improved_path)
            planner_->printStatus(5, "Improved Results");

        // update planner results
        Driver::updatePlannerResults_(found_path, improved_path);

        // preparing path for publishing
        // planner_info_to_controller_ = Driver::prepareInfoWithMPCForRobot();


        // publish to robot
        // debugger::debugTextOutput("[Driver]/[driveRobot] "
        //                           "Sending data to robot...", 5);
        // Driver::publishInfoToRobot(planner_info_to_controller_);

        Driver::publishToRviz_();
        debugger::debugTextOutput("[Driver]/[driveRobot] Published all", 5);
        // replanning_rate.sleep();
        utils::pressEnterToContinue();
        // usleep(5e6);
        debugger::debugTextOutput("[Driver]/[driveRobot] Done for the loop", 5);
    }
}


void Driver::decideGoalForPlanner()
{
    if (driver_method_ == 0 || driver_method_ == 2)
    {
        goal_pose_ = Driver::searchGoal_(); // the same as goal_searching_.goal_pose
        debugger::debugOutput("[Driver]/[driveRobot] Current robot pose: ", 
                robot_state_.pose, 4);
        debugger::debugOutput("[Driver]/[driveRobot] Picked the goal as: ", 
                goal_pose_, 5);
    }
    else if (driver_method_ == 1)
    {
        clicked_lock_.lock();
        goal_pose_ = clicked_goal_pose_;
        clicked_lock_.unlock();
    }
}

void Driver::updatePlannerResults_(
        const bool& found_path, const bool& improved_path)
{
    planner_lock_.lock();
    planner_results_.found_path = found_path;
    planner_results_.improved_path = improved_path;
    planner_results_.status.updated = true;

    if (!found_path)
    {
        planner_results_.new_path_points_ptr = nullptr; 
        planner_results_.new_pose_to_follow_ptr = nullptr; 
    }
    else 
    {
        planner_results_.new_path_points_ptr = planner_->getPlannedPathPtr();
        planner_results_.new_pose_to_follow_ptr = planner_->getPlannedWaypointsPtr();
        planner_results_.status.latest_time = timing::getCurrentTime();
    }
    planner_lock_.unlock();
}


void Driver::publishInfoToRobot_()
{
    ros::Rate publishing_rate(publishing_rate_); 
    while(ros::ok())
    {
        planner_info_to_controller_ = Driver::prepareInfoWithMPCForRobot_();
        udp_->publishToRobotGivenInfoToRobotType(planner_info_to_controller_);
        publishing_rate.sleep();
    }
}

// void Driver::publishInfoToRobot_(const planner_info_to_controller_t& planner_info_to_controller)
// {
//     // publish
// }


planner_info_to_controller_t
Driver::prepareInfoWithMPCForRobot_()
{
    goal_selection_lock_.lock();
    planner_lock_.lock();
    debugger::debugOutput("[Driver]/[prepareInfowithMPC] planner found path: ", 
            planner_results_.found_path, 5);
    debugger::debugOutput("[Driver]/[prepareInfowithMPC] planner updated: ", 
            planner_results_.status.updated, 5);
    if (planner_results_.status.updated && planner_results_.found_path)
    {
        planner_results_.status.updated = false;

        ROS_INFO_STREAM_THROTTLE(0.5, "======================= updated =======================");

        // path pionts and way poses
        ROS_INFO_STREAM_THROTTLE(0.5, "(old, new): " << planner_results_.path_points_ptr->size() << "," << planner_results_.new_path_points_ptr->size());
        planner_results_.path_points_ptr = planner_results_.new_path_points_ptr; 

       ROS_INFO_STREAM_THROTTLE(0.5, "(old, new): " << planner_results_.pose_to_follow_ptr->size() << "," << planner_results_.new_pose_to_follow_ptr->size());
        planner_results_.pose_to_follow_ptr = planner_results_.new_pose_to_follow_ptr; 
    }
    else if (planner_results_.status.updated && !planner_results_.found_path)
    {
        if (timing::spendElapsedTime(
                    planner_results_.status.latest_time,
                    timing::getCurrentTime()) > old_path_timeout_)
        {
            debugger::debugColorTextOutput("[Driver]/[prepareInfowithMPC] "
                "Walking in-place: Old path timeout at time " + 
                std::to_string(timing::getCurrentCPUTime()), 5, G);
            goal_selection_.has_chosen_goal = false;

            goal_selection_lock_.unlock();
            planner_lock_.unlock();
            usleep(1e6); // wait for one second

            return control_commands::assignWalkInPlace();
        }

    }
    else if (!planner_results_.status.updated && !planner_results_.found_path)
    {
            debugger::debugColorTextOutput("[Driver]/[prepareInfowithMPC] "
                "Walking in-place: No Path to go at time " + 
                std::to_string(timing::getCurrentCPUTime()), 5, G);
            goal_selection_.has_chosen_goal = false;

            goal_selection_lock_.unlock();
            planner_lock_.unlock();
            usleep(1e6); // wait for one second


            return control_commands::assignWalkInPlace();
    }

    // robot current pose
    robot_state_t current_robot_state;
    pose_lock_.lock();
    current_robot_state.setPose(*new_inekf_state_);
    // inekf_msgs::State current_robot_state = *new_inekf_state_;
    pose_lock_.unlock();

    // find a goal to go
    std::pair<int, pose_t> goal_for_robot = Driver::findNextGoalForRobot_(
            current_robot_state, planner_results_.pose_to_follow_ptr);
    goal_selection_.chosen_goal = goal_for_robot;

    debugger::debugOutput("chosen goal: ", goal_selection_.chosen_goal.second, 5);
    planner_lock_.unlock();

    if (goal_for_robot.first == 0)
    {
        debugger::debugColorTextOutput("[Driver]/[prepareInfowithMPC] "
                "Waling in-place: No goal left", 5, G, BOLD);
        planner_info_to_controller_ = control_commands::assignWalkInPlaceWithInEKFMsg(
                current_robot_state.inekf_state);
        goal_selection_.has_chosen_goal = false;
        goal_selection_lock_.unlock();


        return planner_info_to_controller_; 

    }
    else if (goal_for_robot.first == 1)
    {
        // TODO: include robot dynamic (MPC) here

        //
        // debugger::debugColorTextOutput("[Driver]/[prepareInfo] "
        //         "A goal found!", 5, G, BOLD);
        ROS_INFO_STREAM_THROTTLE(0.5, "chosen goal: " << goal_selection_.chosen_goal.second);
        
        

        // testing only
        // current_robot_state.inekf_state.pose.position.x += 
        //     utils::genInclusiveRandomNumber(-5, 5);
        // current_robot_state.inekf_state.pose.position.y += 
        //     utils::genInclusiveRandomNumber(-5, 5);

        double delta_yaw = planner_->lyap_dist_->stabilizeHeadingWithTargetPose(
                current_robot_state.pose.to2DPosition(), goal_for_robot.second);

        double yaw = delta_yaw + current_robot_state.pose.theta;
        double speed = 0.2; // TODO: FIXIT



        // x-axis is yaw = 0
        std::vector<double> velocity{speed * std::cos(yaw), speed * std::sin(yaw)}; 
        double behavior = 1;
        double roll = 0;
        double pitch = 0;

        planner_info_to_controller_ = 
            control_commands::assignInfoReducedWithInEKFMsg(behavior, velocity, 
                    roll, pitch, yaw, current_robot_state.inekf_state);

        goal_selection_.has_chosen_goal = true;
        goal_selection_lock_.unlock();

        return planner_info_to_controller_; 
    }
    goal_selection_lock_.unlock();


    debugger::debugOutput("[prepareInfo] out: ", planner_results_.status.updated, 5);

    return control_commands::assignWalkInPlace();
}

// return next goal for the robot, 
// if no goal left, return current robot pose, i.e., walking in-place
std::pair<int, pose_t>
Driver::findNextGoalForRobot_(const robot_state_t& robot_state, 
        std::vector<std::pair<pose_t, bool>>* planned_poses)
{
    if (planned_poses != nullptr)
    {
        for (const auto& pose : *planned_poses)
        {
            // double dis_robot_to_pose = 
            //     planner_->lyap_dist->computeDistanceFromPoseWithTargetPose(
            //     robot_pose, pose);
            double dis_robot_to_pose = squared_point_distance(
                    robot_state.pose.to2DPosition(), pose.first.to2DPosition());
            if (dis_robot_to_pose > is_goal_threshold_)
                return {1, pose.first};
        }
    }

    return {0, robot_state.pose};
}

pose_t
Driver::searchGoal_()
{
    // get map cost 
    map_cost_ = planner_->getMapCostClass();

    
    // find search angle
    double view_angle = goal_searching_.search_view.first + 
                        goal_searching_.search_view.second; // deg

    // delta_theta is in deg
    goal_searching_.num_poses = std::round(view_angle/goal_searching_.delta_theta);


    // generate angles
    goal_searching_.list_of_angles = utils::genListOfNumbers(
            (robot_state_.yaw - deg_to_rad(goal_searching_.search_view.second)), 
            goal_searching_.num_poses, 
            deg_to_rad(goal_searching_.delta_theta));

    goal_searching_.list_of_poses.resize(goal_searching_.num_poses);

    goal_searching_.goal_cost = cost_t(DBL_MAX, DBL_MAX);

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

        // find the cost of the pose
        cost_t cost = map_cost_->computePositionMapCost(goal_pose);
        goal_searching_.list_of_poses[i].second = cost;


        if (cost < goal_searching_.goal_cost)
        {
            goal_searching_.goal_cost = cost;
            goal_searching_.goal_pose = goal_pose;
        }
        // if cost is the same, compare angle
        else if (lazy_equal::absolute_fuzzy_equal(
                    cost.getTotalCost(), goal_searching_.goal_cost.getTotalCost()))
        {
            if (lazy_equal::absolute_fuzzy_equal(goal_pose.theta, 
                                                 robot_state_.yaw) < 
                lazy_equal::absolute_fuzzy_equal(goal_searching_.goal_pose.theta, 
                                                 robot_state_.yaw))
            {
                goal_searching_.goal_cost = cost;
                goal_searching_.goal_pose = goal_pose;
            }
        }

    }
    return goal_searching_.goal_pose;
}


void Driver::publishToRviz_()
{
    // clear previous markers
    goal_selection_marker_array_.markers.clear();
    robot_marker_array_.markers.clear();
    planner_marker_array_.markers.clear();
    path_points  = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);
    path_points->header.frame_id = map_frame_;




    debugger::debugTextOutput("[publish] Plotting...", 3);

    // global map
    grid_map_msgs::GridMap global_map_msg;
    grid_map::GridMapRosConverter::toMessage(multi_layer_map_, global_map_msg);
    global_map_pub_.publish(global_map_msg);

    // local map
    //
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


    // goal searching
    size_t count = 0;
    for (const auto& pose : goal_searching_.list_of_poses)
    {
        plotting::addMarkerWithPose(marker_, visualization_msgs::Marker::ARROW,
                "goal_searching",
                0, 0, 1, 1, // color
                pose.first, // pose
                count++, marker_lifetime_ // count, time
                );
        goal_selection_marker_array_.markers.push_back(marker_);
    }
    plotting::addMarkerWithPose(marker_, visualization_msgs::Marker::ARROW,
            "selected_searching_goal",
            0, 1, 0, 1, // color
            goal_searching_.goal_pose, // pose
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
        std::vector<point2d_t<double>> path = planner_->getPlannedPath();
        for (const auto& it : path)
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
            path_points->points.push_back(pcl::PointXYZI(
                        (float) it.x, (float) it.y, 0.1, 0));
            // debugger::debugOutput("[Main] point: ", it.first, 5);
        }


        // way poses
        std::vector<std::pair<pose_t, bool>> waypoints =
            planner_->getPlannedWaypoints();
        count = 0;
        for (const auto& it : waypoints)
        {
            plotting::addMarkerWithPose(marker_, visualization_msgs::Marker::ARROW,
                    "rrt_waypoints",
                    1, 1, 0, 1, // color
                    pose_t(it.first),
                    count++, marker_lifetime_ // count, time
                    );
            planner_marker_array_.markers.push_back(marker_);
            // debugger::debugOutput("[Main] waypoints: ", it.first, 5);
        }
    }


    // goal selection from findNextGoalForRobot_()
    goal_selection_lock_.lock();
    if (goal_selection_.has_chosen_goal)
    {
        // goal for robot
        plotting::addMarkerWithPose(marker_, visualization_msgs::Marker::ARROW,
                "goal_for_robot",
                0, 1, 1, 1, // color
                goal_selection_.chosen_goal.second, // pose
                0, marker_lifetime_, // count, time
                "", 0.8, 0.3, 0.3); // text, size_x, size_y, size_z
        planner_marker_array_.markers.push_back(marker_);

        // direction for robot
        plotting::addMarkerWithPose(marker_, visualization_msgs::Marker::ARROW,
                "dir_for_robot",
                0, 0, 1, 1, // color
                pose_t(planner_info_to_controller_.pose[0],
                       planner_info_to_controller_.pose[1],
                       planner_info_to_controller_.pose[2],
                       0.0, 0.0, // roll, pitch
                       planner_info_to_controller_.torso.yaw),
                0, marker_lifetime_, // count, time
                "", 0.8, 0.3, 0.3); // text, size_x, size_y, size_z
        planner_marker_array_.markers.push_back(marker_);
    }
    goal_selection_lock_.unlock();


    marker_array_pub_.publish(planner_marker_array_);
    pcl_conversions::toPCL(robot_state_.time, 
            path_points->header.stamp);
    path_pub_.publish(path_points);
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
        global_map_updated_ = false;
    }
    else
    {
        status = false;
        debugger::debugColorOutput("[Driver]/[update] No new map: using last map", 
                "", 10, Y);
    }
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


bool Driver::updateRobotPose_()
{
    //boost::mutex::scoped_lock lock(pose_lock_);
    bool status = true;

    pose_lock_.lock();
    if (driver_method_ == 0 || driver_method_ == 1)
    {
        if (robot_state_updated_)
        {
            robot_state_.setFullPose(*new_inekf_state_);
            robot_state_.time = new_inekf_state_->header.stamp; 
            // robot_state_.printPose();
        }
    }
    else if (driver_method_ == 2)
    {
        received_udp_pose_ = udp_->getReceivedMessage();
        robot_state_updated_ = received_udp_pose_.first;
        if (robot_state_updated_)
        {
            robot_state_.setPoseStandalone(
                    received_udp_pose_.second.pose[0], // x
                    received_udp_pose_.second.pose[1], // y
                    received_udp_pose_.second.pose[2], // z
                    received_udp_pose_.second.pose[3], // qw
                    received_udp_pose_.second.pose[4], // qx
                    received_udp_pose_.second.pose[5], // qy
                    received_udp_pose_.second.pose[6] // qz
                    );
        }
    }


    if (robot_state_updated_)
    {
        robot_state_updated_ = false;
        pose_updated_ = true;
        pose_look_up_counter_ ++;


        // wait until stable
        if (pose_look_up_counter_ < 5) 
            status = false;
    }
    else
    {
        debugger::debugColorOutput("[Driver]/[robot_state_updated_] No robot pose", 
                                   "", 10, Y);
        status = false;
    }
    pose_lock_.unlock();


    return status;


    // pose_lock_.lock();
    // if (driver_method_ == 2)
    // {
    //     received_udp_pose_ = udp_->getReceivedMessage();
    //     robot_state_updated_ = received_udp_pose_.first;
    // }

    // if (robot_state_updated_)
    // {
    //     if (driver_method_ == 0 || driver_method_ == 1)
    //     {
    //         robot_state_.setFullPose(*new_inekf_state_);
    //         robot_state_.time = new_inekf_state_->header.stamp; 
    //     }
    //     else if (driver_method_ == 2)
    //     {
    //         robot_state_.setPoseStandalone(
    //                 received_udp_pose_.pose[0], // x
    //                 received_udp_pose_.pose[1], // y
    //                 received_udp_pose_.pose[2], // z
    //                 received_udp_pose_.pose[3], // qw
    //                 received_udp_pose_.pose[4], // qx
    //                 received_udp_pose_.pose[5], // qy
    //                 received_udp_pose_.pose[6] // qz
    //                 )
    //     }

    //     robot_state_updated_ = false;
    //     pose_updated_ = true;
    //     pose_look_up_counter_ ++;


    //     // wait until stable
    //     if (pose_look_up_counter_ < 5) 
    //         status = false;
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
    debugger::debugColorTextOutput("[Driver] New clicked goal received!", 5, BC);
    clicked_lock_.lock();
    clicked_goal_pose_ = pose_t(msg->point.x, msg->point.y, 0);
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


    // debugger::debugColorTextOutput("In callback function",3);
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
    new_inekf_state_ = &msg;
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
    // receive data from UDP
    if (driver_method_ == 2)
    {
        boost::thread listen_to_udp_pose(&Driver::getUDPCallBack_, this);
    }


    if (driver_method_ == 0 || driver_method_ == 2)
    {
        while (ros::ok() && (!global_map_updated_ || !robot_state_updated_))
        {
            ROS_WARN_THROTTLE(1, "Received map: %i", global_map_updated_);
            ROS_WARN_THROTTLE(1, "Received pose: %i", robot_state_updated_);
            sleep(0.5);
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
            sleep(0.5);
        }
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

bool Driver::getParameters_()
{
    std::string title_name("[Driver]/[getParameters] ");
    bool received_all = true;
     // driver parameters
    ros_utils::checkROSParam(nh_, "driver_mode", driver_method_, 
            getNameOf(driver_method_), title_name, received_all);

    ros_utils::checkROSParam(nh_, "replanning_rate", replanning_rate_, 
            getNameOf(replanning_rate_), title_name, received_all);
    ros_utils::checkROSParam(nh_, "publishing_rate", publishing_rate_, 
            getNameOf(publishing_rate_), title_name, received_all);

    ros_utils::checkROSParam(nh_, "old_path_timeout", old_path_timeout_, 
            getNameOf(old_path_timeout_), title_name, received_all);

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

    // debugger::debugOutput("ip_to_send: ", udp_info_.ip_to_send, 5);




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



    // pose sampler
    ros_utils::checkROSParam(nh_, "pose_sampler_params/goal_bias", 
            pose_sampler_params_.goal_bias, 
            getNameOf(pose_sampler_params_) + "." + 
            getNameOf(pose_sampler_params_.goal_bias), title_name, received_all);
    ros_utils::checkROSParam(nh_, "pose_sampler_params/distance_threshold", 
            pose_sampler_params_.distance_threshold, 
            getNameOf(pose_sampler_params_) + "." + 
            getNameOf(pose_sampler_params_.distance_threshold), title_name, received_all);


    // goal searching parameters
    ros_utils::checkROSParam(nh_, "goal_search_radius", 
                          goal_searching_.search_radius, 
                          getNameOf(goal_searching_) + "." + 
                          getNameOf(goal_searching_.search_radius), title_name, received_all);
    ros_utils::checkROSParam(nh_, "goal_search_delta_theta", 
                          goal_searching_.delta_theta, 
                          getNameOf(goal_searching_) + "." + 
                          getNameOf(goal_searching_.delta_theta), title_name, received_all);
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



    // general parameters
    ros_utils::checkROSParam(nh_, "map_frame", map_frame_, 
                          getNameOf(map_frame_), title_name, received_all);
    ros_utils::checkROSParam(nh_, "robot_frame", robot_frame_, 
                          getNameOf(robot_frame_), title_name, received_all);
    ros_utils::checkROSParam(nh_, "multi_layer_map_topic", multi_layer_map_topic_, 
                          getNameOf(multi_layer_map_topic_), title_name, received_all);
    ros_utils::checkROSParam(nh_, "inekf_topic", inekf_topic_, 
                          getNameOf(inekf_topic_), title_name, received_all);
    ros_utils::checkROSParam(nh_, "is_goal_threshold", is_goal_threshold_, 
                          getNameOf(is_goal_threshold_), title_name, received_all);
    ros_utils::checkROSParam(nh_, "marker_lifetime", marker_lifetime_, 
                          getNameOf(marker_lifetime_), title_name, received_all);


    if (!received_all)
    {
        driver_method_ = 2;
        replanning_rate_ = 5;
        publishing_rate_ = 300;
        old_path_timeout_ = 5; // seconds
        is_goal_threshold_ = 0.2; // meter


        pose_sampler_params_.goal_bias = 0.2;
        pose_sampler_params_.distance_threshold = 0;

        // goal searching parameters
        goal_searching_.search_radius = 2;
        goal_searching_.delta_theta = 3;
        goal_searching_.search_view = {-10, 10};




        map_frame_ = "map";
        robot_frame_ = "cassie";
        multi_layer_map_topic_ = "/fake_map_publisher/multi_layer_map";
        inekf_topic_ = "/fake_robot_publisher/inekf";
        marker_lifetime_ = 0.2;
    }

    return received_all;
}



Driver::~Driver() { }


} /* bipedlab */ 
