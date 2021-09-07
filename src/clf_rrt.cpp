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

#include "clf_rrt.h"
#include "utils/utils.h"
#include "utils/debugger.h"
#include "utils/timing.h"
#include "utils/plotting.h"
#include <float.h> // for DBL_MAX

// #ifndef DEBUG_LEVEL
// #define DEBUG_LEVEL 3
// #endif /* ifndef DEBUG_LEVEL */

namespace bipedlab 
{
    CLFRRTStarPlanner::CLFRRTStarPlanner(
            const grid_map::GridMap& global_map, // no need to keep local copy (LC)
            const local_map_params_t& local_map_params,
            const pose_t& start_pose, const pose_t& goal_pose, // need to keep LC 
            robot_state_t& robot_state, // no need to keep LC
            pose_sampler_params_t& pose_sampler_params,
            rrt_params_t& rrt_params,
            cost_params_t& map_cost_params,
            lyapunov_distance_params_t& lyap_dist_params) : 
        global_map_(&global_map), 
        start_pose_(start_pose), 
        goal_pose_(goal_pose), 
        robot_state_(robot_state),
        rrt_params_(rrt_params),
        lyap_dist_params_(lyap_dist_params),
        total_seconds_spent_(0),
        is_debugging_(2) // will save more data for visualization
                        // 2: to show sampled poses
                        // 3: to show nearby poses
    { 
        // std::cout << "[In CLFRRTStarPlanner()] &global_map = " << &global_map << std::endl;
        // std::cout << "[In CLFRRTStarPlanner()] global_map_ = " << global_map_ << std::endl;

        debugger::debugTextOutput("[CLFRRT] CLFRRTStarPlanner initializing...", 4);


        // for (auto name : global_map.getLayers())
        //     std::cout << "layer: " << name << std::endl;

        local_map_ = new LocalMap(&start_pose_, global_map_, local_map_params);
        // std::cout << "[In CLFRRTStarPlanner()] global_map_ in Plannar == map_ of LocalMap: ";
        // local_map_->printMapAddress();
        // std::cout << std::endl;

        // new_local_map_ = new LocalMap(start_pose_, global_map, length_of_local_map);
        // new_local_map_ = *local_map_; 
        // debug_local_map = local_map_->local_map;


        sampling_ = new SamplePose(pose_sampler_params, 
                                   local_map_, 
                                   &goal_pose_,
                                   &robot_state_);

        map_cost_ = new MapCost(map_cost_params, local_map_, rrt_params_.mode);

        // LyapunovDistance() is declared here for speed
        // whichever class (lyapunovPath, CassieRRTTree) 
        // wants to change the target pose of the local chart inside of 
        // the LyapunovDistance(), CHANGE ON ITS OWN!
        lyap_dist_ = new LyapunovDistance(lyap_dist_params_); 
        lyap_path_ = new LyapunovPath(*lyap_dist_, *local_map_, *map_cost_, rrt_params_.mode);
        rrt_tree_ = new CassieRRTTree(start_pose_, *lyap_dist_, *lyap_path_);


        if (is_debugging_)
        {
            path_pub_ = 
                nh_.advertise<pcl::PointCloud<pcl::PointXYZI>> ("debug_path_points", 1);

            marker_pub_ =
                nh_.advertise<visualization_msgs::MarkerArray>("debug_results", 10);

            debug_path_points_ = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);
            debug_path_points_->header.frame_id = "map";
        }

        debugger::debugTextOutput("[CLFRRT] Classes in CLFRRTStarPlanner are "
                                  "all initialized!", 5);

        // debugger::debugTextOutput("[CLFRRT Constructor] testing sampling function... ", 0);
        // bool dummy;
        // sample_pose_testing = 
        //     sampling_->sampleRandomPoseWithGoalBiasedWithinLocalMap(dummy);

        // debugger::debugTextOutput("[CLFRRT] CLFRRT is running... ", 5);
        // is_path_ = CLFRRTStarPlanner::findNewPath(
        //         rrt_params_.num_samples, 
        //         rrt_params_.allowed_computation_time, 
        //         rrt_params_.terminate_if_path);
        // debugger::debugOutput("[CLFRRT] has found a path? ", is_path_, 5);
        // debugger::debugOutput("[CLFRRT] Cost:", minimum_cost_to_goal_, 5);
        // debugger::debugOutput("[CLFRRT] points of path ", 
        //         planned_path_.size(), 5);
        // debugger::debugOutput("[CLFRRT] number of waypoints ", 
        //         planned_waypoints_.size(), 5);
    }


    CLFRRTStarPlanner::~CLFRRTStarPlanner() { }


    bool CLFRRTStarPlanner::addMoreTimeNSamples(
            size_t max_additional_samples, 
            double allow_more_time,
            bool terminate_if_path)
    {
        if (num_samples_ == 0) {
            return CLFRRTStarPlanner::findNewPath(
                    max_additional_samples, allow_more_time, terminate_if_path);
        } else {
            return CLFRRTStarPlanner::runCLFRRTAlgorithm(
                    max_additional_samples, allow_more_time, terminate_if_path);
        }
    }


    bool CLFRRTStarPlanner::findNewPath(
            size_t max_num_samples, 
            double longest_duration,
            bool terminate_if_path)
    {
        debugger::debugTextOutput("[CLFRRT] Initializaing a new tree...", 1);


        rrt_tree_->initializeTree(start_pose_);
        total_seconds_spent_ = 0;
        num_samples_ = 0;


        // prevent from overflow
        minimum_cost_to_goal_ = cost_t(DBL_MAX/2 - 1.0, DBL_MAX/2 - 1.0); 
        minimum_cost_leaf_ = nullptr;

        // runCLFRRTAlgorithm() updates 
        // num_samples_, minimum_cost_to_goal_, minimum_cost_leaf_ internally
        is_path_ = runCLFRRTAlgorithm(
                max_num_samples, longest_duration, terminate_if_path);

        return is_path_;
    }

    bool CLFRRTStarPlanner::runCLFRRTAlgorithm(
            size_t max_additional_samples, 
            double allow_more_time,
            bool terminate_if_path)
    {
        size_t num_new_samples = 0;
        have_improved_path_ = false;
        size_t num_path_improvement = 0;

        // if (global_map_updated_)
        // {
        //     new_local_map_->updateLocalMap(robot_state_.pose);
        //     local_map_ = new_local_map_;
        //     global_map_updated_ = false;
        // }

        // if (robot_state_updated_)
        // {
        //     robot_state_ = new_robot_state_;
        //     robot_state_updated_ = false;
        // }


        // timing 
        auto start_time = timing::getCurrentTime();
        double time_of_spend = timing::spendElapsedTime(start_time);


        if (is_debugging_)
            sampling_poses_.clear();
        while (time_of_spend < allow_more_time && 
               num_new_samples < max_additional_samples)
        {
            debugger::debugTitleTextOutput("[CLFRRTStarPlanner]",
                    "runCLFRRTAlgorithm", 4);
            debugger::debugOutput(
                    "[runCLFRRTAlgorithm] Current tree size: ", rrt_tree_->tree_.size(), 4);
            debugger::debugOutput(
                    "[runCLFRRTAlgorithm] start: ", start_pose_, 2);
            debugger::debugOutput(
                    "[runCLFRRTAlgorithm] goal: ", goal_pose_, 2);
            // always false for now, will be changed in samplePose() later 
            bool move_backward = false; 

            // will be changed in samplePose();
            bool is_goal = false; 

            // sample a pose
            debugger::debugTextOutput(
                    "[runCLFRRTAlgorithm] Sampling a pose...", 4);

            // (x, y, theta)
            pose_t sampled_pose = CLFRRTStarPlanner::samplePose_(is_goal);
            // debugger::debugOutput(
            //         "[RRTAlgorithm] Sampled pose:", sampled_pose, 3);

            debugger::debugTextOutput(
                    "[runCLFRRTAlgorithm] Finding the nearest node...", 4);
            cassie_rrt_node_t* nearest_node = 
                CLFRRTStarPlanner::findNearestNodeToPose_(sampled_pose, 
                                                          move_backward);

            debugger::debugTextOutput(
                    "[runCLFRRTAlgorithm] Extending from the nearest node...", 4);
            path_segment_t edge_path = 
                CLFRRTStarPlanner::extendFromTreeNodeToSampledPose_(
                    *nearest_node, sampled_pose, 
                    rrt_params_.max_extension_length, move_backward);
            pose_t new_sample = edge_path.steps.back();
            
            if (is_debugging_ >= 2)
            {
                marker_array_.markers.clear();
                plotting::addMarkerWithPose(marker_, 
                        visualization_msgs::Marker::ARROW,
                        "sampled pose",
                        0, 1, 0, 1, // color
                        sampled_pose, // pose
                        0, 0 // count, time
                        );
                marker_array_.markers.push_back(marker_);

                if (is_debugging_ == 3)
                {
                    plotting::addMarkerWithPose(marker_, 
                            visualization_msgs::Marker::ARROW,
                            "nearest",
                            0, 0, 1, 1, // color
                            nearest_node->pose, // pose
                            0, 0 // count, time
                            );
                    marker_array_.markers.push_back(marker_);


                    plotting::addMarkerWithPose(marker_, 
                            visualization_msgs::Marker::ARROW,
                            "new_sample",
                            0, 1, 1, 1, // color
                            new_sample, // pose
                            0, 0 // count, time
                            );
                    marker_array_.markers.push_back(marker_);

                    debugger::debugOutput("nearest [b]", nearest_node->pose, 3);
                    debugger::debugOutput("sampled_pose [g]", sampled_pose, 3);
                    debugger::debugOutput("new_sample [c]", new_sample, 3);


                    debug_path_points_->points.clear();
                    for (const auto& it : edge_path.path)
                    {
                        debugger::debugOutput("path_point: ", it, 1);
                        debug_path_points_->points.push_back(pcl::PointXYZI(
                                    (float) it.x, (float) it.y, 0.1, 0));
                    }
                }
                // CLFRRTStarPlanner::publishDebug_();
            }

            if (is_debugging_)
                sampling_poses_.push_back(new_sample);

            if (CLFRRTStarPlanner::isObstacleFree(edge_path.steps))
            {
                // debugger::debugOutput("[runCLFRRTAlgorithm] Path to the nearest node"
                //         "is collision free:\n" , nearest_node->pose, 4);
                debugger::debugOutput("[runCLFRRTAlgorithm] Path to the nearest node"
                        "is collision freen", "", 4);

                is_goal = lyap_dist_->computeDistanceFromPoseWithTargetPose(
                        new_sample, goal_pose_) < rrt_params_.goal_threshold; 
                debugger::debugOutput("[runCLFRRTAlgorithm] goal_threshold: ", 
                        rrt_params_.goal_threshold, 3);

                
                // find nearby nodes from the extended node
                debugger::debugTextOutput("[runCLFRRTAlgorithm] " 
                        "Searching nearby nodes...", 4);
                std::vector<cassie_rrt_node_t*> nearby_nodes = 
                    CLFRRTStarPlanner::findNearbyNodesToPose_(new_sample, 
                                                              move_backward);

                // choose the parent by choosing the minimum-cost route from the root
                debugger::debugTextOutput("[runCLFRRTAlgorithm] "  
                        "Finding parent node...", 4);
                cassie_rrt_node_t* parent_node = 
                    CLFRRTStarPlanner::chooseRRTParent_(
                            nearby_nodes, nearest_node, new_sample, move_backward);

                if (is_debugging_ >= 2)
                {
                    size_t count = 0;
                    for (const auto& it : sampling_poses_)
                    {
                        plotting::addMarkerWithPose(marker_, 
                                visualization_msgs::Marker::ARROW,
                                "samples", // black
                                0, 0, 0, 1, // color
                                it, // pose
                                count, 0 // count, time
                                );
                        marker_array_.markers.push_back(marker_);
                        count ++;
                    }
                    debugger::debugOutput("[runCLFRRTAlgorithm] num of samples: ", 
                                          count, 3);
                    count = 0;

                    if (is_debugging_ == 3)
                    {
                        for (const auto& it : nearby_nodes)
                        {
                            plotting::addMarkerWithPose(marker_, 
                                    visualization_msgs::Marker::ARROW,
                                    "nearby nodes [to]", // light green
                                    1, 1, 1, 1, // color
                                    it->pose, // pose
                                    count, 2, // count, time
                                    "", 1, 1); // text, size x, size y
                            marker_array_.markers.push_back(marker_);
                            count ++;
                        }
                        debugger::debugOutput("[runCLFRRTAlgorithm] num of nearby nodes [to]: ", 
                                count, 3);
                        plotting::addMarkerWithPose(marker_, 
                                visualization_msgs::Marker::ARROW,
                                "parent", // purple
                                0.4, 0, 1, 1, // color
                                parent_node->pose, // pose
                                count, 0 // count, time
                                );
                        marker_array_.markers.push_back(marker_);
                    }
                }


                if (!parent_node->is_goal) 
                {
                    // insert the node to the tree
                    debugger::debugTextOutput("[runCLFRRTAlgorithm] "  
                            "Node inserting...", 4);
                    cassie_rrt_node_t* new_node = CLFRRTStarPlanner::insertRRTNode_(
                            parent_node, new_sample, is_goal, move_backward);

                    // find nearby nodes from the sample 
                    // (this will include itself, which is the last sample in the tree)
                    debugger::debugTextOutput("[runCLFRRTAlgorithm] "  
                            "Finding nodes near from the inserted node...", 4);
                    nearby_nodes = CLFRRTStarPlanner::findNearbyNodesFromPose_(
                            new_sample, move_backward);

                    // rewire neargy nodes to the new node, 
                    // if it decreases overall cost.
                    debugger::debugTextOutput("[runCLFRRTAlgorithm] Rewiring...", 4);
                    CLFRRTStarPlanner::rewireRRTNodes(nearby_nodes, new_node);

                    if (is_debugging_ == 3)
                    {
                        size_t count = 0;
                        for (const auto& it : nearby_nodes)
                        {
                            plotting::addMarkerWithPose(marker_, 
                                    visualization_msgs::Marker::ARROW,
                                    "nearby nodes [from]", // light green
                                    0,0,0, 1, // color
                                    it->pose, // pose
                                    count, 5, // count, time
                                    "", 1, 1); // text, size x, size y
                            marker_array_.markers.push_back(marker_);
                            count ++;
                        }
                        debugger::debugOutput("[RRTAlgorithm] num of nearby nodes [from]: ", 
                                count, 4);
                        // utils::pressEnterToContinue();
                    }


                    if (is_goal && new_node->cost < minimum_cost_to_goal_) 
                    {
                        minimum_cost_to_goal_ = new_node->cost;
                        minimum_cost_leaf_ = new_node;

                        if (num_path_improvement == 0)
                        {
                            debugger::debugTextOutput("[CLFRRT] Found a path", 3);
                            debugger::debugOutput("[CLFRRT] Goal:", 
                                    goal_pose_, 3);
                            debugger::debugOutput("[CLFRRT] Cost:", 
                                    minimum_cost_to_goal_, 3);
                            debugger::debugOutput("[CLFRRT] Has Spent (s): ", 
                                    timing::spendElapsedTime(start_time), 3);
                        }
                        else
                        {
                            debugger::debugOutput("[CLFRRT] Improved the path: ", 
                                    num_path_improvement, 3);
                            debugger::debugOutput("[CLFRRT] Goal:", 
                                    goal_pose_, 3);
                            debugger::debugOutput("[CLFRRT] Cost:", minimum_cost_to_goal_, 3);
                        }
                        num_path_improvement ++;
                        have_improved_path_ = true;


                        if (is_debugging_ == 2)
                        {
                            debugger::debugTitleTextOutput("[CLFRRTAlgorithm]", 
                                    "goal reach!", 4, Y);
                            CLFRRTStarPlanner::retrievePathFromLeaf_(
                                    minimum_cost_leaf_);
                            std::vector<point2d_t<double>> path = this->getPlannedPath();
                            for (const auto& it : path)
                            {
                                debug_path_points_->points.push_back(pcl::PointXYZI(
                                            (float) it.x, (float) it.y, 0.1, 0));
                            }


                            // way poses
                            std::vector<std::pair<pose_t, bool>> waypoints =
                                this->getPlannedWaypoints();
                            size_t count = 0;
                            for (const auto& it : waypoints)
                            {
                                plotting::addMarkerWithPose(marker_, visualization_msgs::Marker::ARROW,
                                        "waypoints",
                                        1, 1, 0, 1, // color
                                        pose_t(it.first),
                                        count, 0 // count, time
                                        );
                                count ++;
                                marker_array_.markers.push_back(marker_);
                                debugger::debugOutput("[CLFRRTAlgorithm] waypoints: ", it.first, 3);
                            }
                            // utils::pressEnterToContinue();
                        }
                    }
                }
            }

            num_new_samples ++;
            debugger::debugOutput("[RRTALgorithm] num of new samples: ", 
                                num_new_samples, 1);
            time_of_spend = timing::spendElapsedTime(start_time);
            if (have_improved_path_ && terminate_if_path) {
                break;
            }
            if (is_debugging_)
                CLFRRTStarPlanner::publishDebug_();
        }

        // update statitistic
        num_samples_ += num_new_samples;
        total_seconds_spent_ += time_of_spend;

        if (num_samples_ != 0) {
            computation_analysis_.num_samples_per_second = 
                num_samples_ / total_seconds_spent_;
        }


        CLFRRTStarPlanner::printStatus(4, "");

        if (have_improved_path_) {
            // update path information
            CLFRRTStarPlanner::retrievePathFromLeaf_(minimum_cost_leaf_);
        }
        // utils::pressEnterToContinue();

        return have_improved_path_;
    }

    void CLFRRTStarPlanner::printStatus(int level, std::string text)
    {
        if (text.size() == 0)
            text = "Planning Results";
        debugger::debugColorTextOutput("[CLFRRT] ====== " + text + " ======", 
                level, BY, BOLD);
        debugger::debugOutput("[CLFRRT] Has found a path? ", is_path_, level);
        // debugger::debugOutput("[CLFRRT] Has improved the path? ", have_improved_path_, level);
        debugger::debugOutput("[CLFRRT] Cost: ", minimum_cost_to_goal_, level);
        debugger::debugOutput("[CLFRRT] Total cost: ", 
                              minimum_cost_to_goal_.getTotalCost(), level);
        debugger::debugOutput("[CLFRRT] samples / seconds: ", 
                              computation_analysis_.num_samples_per_second, level);
        debugger::debugOutput("[CLFRRT] Number of samples: ", num_samples_, level);
        debugger::debugOutput("[CLFRRT] Total Spent [s]: ", 
                              total_seconds_spent_, level);
    }


    pose_t CLFRRTStarPlanner::samplePose_(bool& is_goal)
    {
        return sampling_->sampleRandomPoseWithGoalBiasedWithinLocalMap(is_goal);
    }


    cassie_rrt_node_t* CLFRRTStarPlanner::findNearestNodeToPose_(
            const pose_t& pose,
            const bool move_backward) 
    {
        if (rrt_params_.mode == 0)
            return rrt_tree_->findNearestNodeToPose(pose);
        else if (rrt_params_.mode == 1)
            return rrt_tree_->findLowestCostNodeToPoseWithMapInfo(pose);
    }


    path_segment_t CLFRRTStarPlanner::extendFromTreeNodeToSampledPose_(
            const cassie_rrt_node_t& rrt_node,
            const pose_t& sampled_pose,
            const double& max_extension,
            const bool move_backward) const
    {
        // debugger::debugTextOutput( "[CLFRRTStarPlanner]/"
        // "[extendFromTreeNodeToSampledPose]" , 5);
        // pose_t poseFrom = isMovingBackward ? rrtNode.pose.flip() : rrtNode.pose;
        // pose_t poseTo = isMovingBackward ? sampledPose.flip() : sampledPose;

        // UnicycleLyapunovDistance lyap(poseTo, params_.distanceParams);
        // cost along the path is computed inside according to the mode
        return lyap_path_->extend(rrt_node.pose,  sampled_pose, max_extension);
    }

    
    // TODO: change to splitting the path into 100 segment (?)
    bool CLFRRTStarPlanner::isObstacleFree(const std::vector<pose_t>& steps) const
    {
        // debugger::debugTextOutput(
        //             "[CLFRRTStarPlanner]/[isObstacleFree] "
        //             "Check if neighbor of path points are free ", 3);
        bool is_collision_free = true;
        for (auto step_it = steps.begin(); step_it != steps.end(); step_it++) {
            // if (grid.getObstacleDistance((*stepIt).toPoint()) < kDistanceThreshlod) {
            position_t position = (*step_it).to2DPosition();

            bool is_locally_free = local_map_->isNeighborObstacleFree(
                        position.x, position.y, 
                        rrt_params_.path_obstacle_threshold);
            if (!is_locally_free) {
                is_collision_free = false;
                return is_collision_free;
            }
        }

        return is_collision_free;
    }

        
    std::vector<cassie_rrt_node_t*> 
        CLFRRTStarPlanner::findNearbyNodesToPose_(
                const pose_t& sampled_pose, const bool move_backward)
    {
        if (rrt_params_.mode == 0)
            return rrt_tree_->findNearbyNodesToPose(sampled_pose, move_backward);
        else if (rrt_params_.mode == 1)
            return rrt_tree_->findNearbyNodesToPoseWithMapInfo(
                    sampled_pose, move_backward);
    }

    std::vector<cassie_rrt_node_t*> 
        CLFRRTStarPlanner::findNearbyNodesFromPose_(const pose_t& sampled_pose, 
                                                    const bool move_backward)
    {
        if (rrt_params_.mode == 0)
            return rrt_tree_->findNearbyNodesFromPose(sampled_pose, move_backward);
        else if (rrt_params_.mode == 1)
            return rrt_tree_->findNearbyNodesFromPoseWithMapInfo(
                    sampled_pose, move_backward);
    }


    // Cost definitions
    cost_t CLFRRTStarPlanner::computeRRTCostFromNodeToPose(
            const cassie_rrt_node_t& from_node,
            const pose_t& to_pose,
            const bool move_backward)
    {
        // LyapunovDistance lyap(isMovingBackward ? toPose.flip() : toPose, params_.distanceParams);
        //return lyap.distanceFromPose(isMovingBackward ? fromNode.pose : fromNode.pose);

        if (rrt_params_.mode == 0)
        {
            lyap_dist_->assignNewTargetPose(to_pose);
            return cost_t(lyap_dist_->computeDistanceFromPose(from_node.pose), 0);
        }
        else if (rrt_params_.mode == 1)
        {
            path_segment_t path_segment =  
                lyap_path_->steer(from_node.pose, to_pose);

            return path_segment.cost_along_path;
        }
    }


    cost_t CLFRRTStarPlanner::computeRRTCostFromPoseToNode(
            const pose_t& from_pose, const cassie_rrt_node_t& to_node)
    {
        // LyapunovDistance lyap(toNode.shouldBeApproachedBackward ? toNode.pose.flip() : toNode.pose,
        //                               params_.distanceParams);
        // return lyap.distanceFromPose(toNode.shouldBeApproachedBackward ? fromPose.flip() : fromPose);

        lyap_dist_->assignNewTargetPose(to_node.pose);
        if (rrt_params_.mode == 0)
            return cost_t(lyap_dist_->computeDistanceFromPose(from_pose), 0);
        else if (rrt_params_.mode == 1){
            path_segment_t path_segment = 
                lyap_path_->steer(from_pose, to_node.pose);
            return path_segment.cost_along_path;
        }
    }




    // RRT-Star methods
    cassie_rrt_node_t* 
        CLFRRTStarPlanner::chooseRRTParent_(
                const std::vector<cassie_rrt_node_t*>& nearby_nodes,
                cassie_rrt_node_t* nearest_node,
                const pose_t& sampled_pose,
                const bool moving_backward)
    {

        //if (1)
        if (rrt_params_.mode == 0)
            return CLFRRTStarPlanner::chooseRRTParentVanilla_(nearby_nodes,
                    nearest_node, sampled_pose, moving_backward);
        else if (rrt_params_.mode == 1)
            return CLFRRTStarPlanner::chooseRRTParentWithMapInfo_(nearby_nodes,
                    nearest_node, sampled_pose, moving_backward);
    }

     
    cassie_rrt_node_t* 
        CLFRRTStarPlanner::insertRRTNode_(cassie_rrt_node_t* parent_node,
                                          const pose_t& child_pose,
                                          bool is_goal,
                                          bool move_backward)
    {
        cost_t cost_from_parent = CLFRRTStarPlanner::computeRRTCostFromNodeToPose(
            *parent_node, child_pose, move_backward);
        cassie_rrt_node_t* inserted_node =
            rrt_tree_->insertNode(child_pose, cost_from_parent, is_goal, 
                                  move_backward, parent_node);

        return inserted_node;
    }


    // find path from sampled pose to a given node. used for rrt-star-rewire
    path_segment_t CLFRRTStarPlanner::findPathFromSampledNodeToPose(
            const pose_t& sampled_pose, const cassie_rrt_node_t& rrt_node) const
    {
        bool move_backward = rrt_node.should_be_approached_backward;
        // pose_t poseFrom = isMovingBackward ? sampledPose.flip() : sampledPose;
        // pose_t poseTo = isMovingBackward ? rrtNode.pose.flip() : rrtNode.pose;
        // UnicycleLyapunovDistance lyap(poseTo, params_.distanceParams);
        // return steering_.steer(poseFrom, lyap);
        // lyap_dist_->assignNewTargetPose(rrt_node.pose);
        return lyap_path_->steer(sampled_pose, rrt_node.pose);
    }


    void CLFRRTStarPlanner::rewireRRTNodes(
            const std::vector<cassie_rrt_node_t*>& nearby_nodes, 
            cassie_rrt_node_t* rewire_node)
    {
        for (auto node_it = nearby_nodes.begin(); 
             node_it != nearby_nodes.end(); node_it++) {
            // if a node nearby is not the parent or itself
            if ((*node_it) != rewire_node->parent && (*node_it) != rewire_node) {
                // find path to the node
                // TODO: this could be avoid, since this has been computed in
                // fidingNearbyNodeFromPose
                path_segment_t path_to_node = 
                    CLFRRTStarPlanner::findPathFromSampledNodeToPose(
                            rewire_node->pose, **node_it);

                // check collision
                bool pose_can_be_reached_from_node = 
                    CLFRRTStarPlanner::isObstacleFree(path_to_node.steps);

                if (pose_can_be_reached_from_node) {
                    // compute cost via rewire node
                    cost_t rewired_edge_cost = path_to_node.cost_along_path;
                    cost_t rewired_cost_to_root = 
                        rewire_node->cost + rewired_edge_cost;

                    // if the cost can be improved
                    if (rewired_cost_to_root < (*node_it)->cost) {
                        // reconnect edges and propagate cost to leaves
                        CLFRRTStarPlanner::reconnectRRTTree(
                                rewire_node, *node_it, rewired_edge_cost);
                        debugger::debugTextOutput("[CLFRRT] Graph rewired", 2);
                    }
                }
            }
        }

        // propagate cost down the line
        CLFRRTStarPlanner::propagateRRTCostToChildren(rewire_node);
    }

    void CLFRRTStarPlanner::reconnectRRTTree(
            cassie_rrt_node_t* new_parent,
            cassie_rrt_node_t* child_to_rewire,
            cost_t rewired_edge_cost)
    {
        // delete connection to old parent
        rrt_tree_->deleteChildFromParent(child_to_rewire, child_to_rewire->parent);

        // rewire to the new node
        new_parent->children.push_back(child_to_rewire);   // set child to parent
        child_to_rewire->parent = new_parent;              // set parent to child
        child_to_rewire->cost = new_parent->cost + rewired_edge_cost;

    }

    void CLFRRTStarPlanner::propagateRRTCostToChildren(
            cassie_rrt_node_t* parent_node)
    {
        for (auto child_it = parent_node->children.begin(); 
             child_it != parent_node->children.end(); child_it++) {
            // compute cost between edge
            cost_t cost_to_child =
                CLFRRTStarPlanner::computeRRTCostFromNodeToPose(
                        *parent_node, (*child_it)->pose, 
                        (*child_it)->should_be_approached_backward);
            (*child_it)->cost = parent_node->cost + cost_to_child;

            // propagate until reached leaves, where there are no more edges to propagate
            CLFRRTStarPlanner::propagateRRTCostToChildren(*child_it);
        }
    }


    bool CLFRRTStarPlanner::retrievePathFromLeaf_(cassie_rrt_node_t* leaf_node)
    {
        if (minimum_cost_leaf_ == nullptr) {
            return false;
        } else {
            // clearn data
            path_nodes_.clear();
            planned_path_.clear();
            planned_waypoints_.clear();

            // retrieve path data
            cassie_rrt_node_t* node_ptr = leaf_node;
            debugger::debugTextOutput("[CLFRRT] Retrieving path...", 4);
            while (node_ptr != nullptr) {
                path_nodes_.push_back(cassie_rrt_node_data_t(*node_ptr));
                node_ptr = node_ptr->parent;
            }

            debugger::debugTextOutput("[CLFRRT] Constructing waypoints...", 4);
            // construct other path representations
            if (!path_nodes_.empty()) {
                std::reverse(path_nodes_.begin(), path_nodes_.end());

                for (auto path_node_it = path_nodes_.begin(); 
                        path_node_it != path_nodes_.end(); path_node_it++) {
                    // path info
                    if (path_node_it != path_nodes_.begin()) {
                        pose_t robot_pose = path_node_it->parent_pose.front();
                        pose_t target_pose = path_node_it->pose;

                        if (path_node_it->should_be_approached_backward) {
                            // robotPose = robotPose.flip();
                            // targetPose = targetPose.flip();
                        }

                        // lyap_dis_->assignNewTargetPose(target_pose);
                        path_segment_t path_segment = 
                            lyap_path_->steer(robot_pose, target_pose);

                        // TODO: Is steps sufficient?
                        planned_path_.insert(planned_path_.begin(),
                                path_segment.path.begin(),
                                path_segment.path.end());   
                    }

                    // waypoint info
                    planned_waypoints_.push_back(std::make_pair(path_node_it->pose, 
                                path_node_it->should_be_approached_backward));
                    // planned_waypoints_.push_back(std::make_pair(path_node_it->pose, 
                    //                              false));
                }
            }

            return true;
        }
    }


    // find parents nodes from the sampled/extended pose
    cassie_rrt_node_t* 
        CLFRRTStarPlanner::chooseRRTParentWithMapInfo_(
                const std::vector<cassie_rrt_node_t*>& nearby_nodes,
                cassie_rrt_node_t* nearest_node,
                const pose_t& sampled_pose,
                const bool moving_backward)
    {
        // control-laypunov distance function to target pose
        //LyapunovDistance lyap(sampled_pose, params_.distanceParams);
        lyap_dist_->assignNewTargetPose(sampled_pose);

        // initialize with the nearest node
        cassie_rrt_node_t* parent_node = nearest_node;
        cost_t min_cost_from_root = 
            parent_node->cost + CLFRRTStarPlanner::computeRRTCostFromNodeToPose(
                                    *parent_node, sampled_pose, moving_backward);

        // iterate among nearby nodes to find the best node
        for (auto node_it = nearby_nodes.begin(); 
             node_it != nearby_nodes.end(); node_it++) {

            // generate path to the node
            path_segment_t path_to_pose = lyap_path_->steer((*node_it)->pose, 
                                                            sampled_pose);

            // collision check
            bool pose_can_be_reached_from_node = 
                CLFRRTStarPlanner::isObstacleFree(path_to_pose.steps);

            // update, if collision-free
            if (pose_can_be_reached_from_node) {
                // check cost
                cost_t cost_from_root = (*node_it)->cost + 
                                        path_to_pose.cost_along_path;
                    
                // choose the node with the minimum cost from the root as the parent
                if (cost_from_root < min_cost_from_root) {
                    parent_node = *node_it;
                    min_cost_from_root = cost_from_root;
                }
            }
        }

        return parent_node;
    }

    cassie_rrt_node_t* 
        CLFRRTStarPlanner::chooseRRTParentVanilla_(
                const std::vector<cassie_rrt_node_t*>& nearby_nodes,
                cassie_rrt_node_t* nearest_node,
                const pose_t& sampled_pose,
                const bool moving_backward)
    {
        // control-laypunov distance function to target pose
        //LyapunovDistance lyap(sampled_pose, params_.distanceParams);
        lyap_dist_->assignNewTargetPose(sampled_pose);

        // initialize with the nearest node
        cassie_rrt_node_t* parent_node = nearest_node;
        cost_t min_cost_from_root = 
            parent_node->cost + CLFRRTStarPlanner::computeRRTCostFromNodeToPose(
                                    *parent_node, 
                                    sampled_pose, 
                                    moving_backward);

        // iterate among nearby nodes to find the best node
        for (auto node_it = nearby_nodes.begin(); 
             node_it != nearby_nodes.end(); node_it++) {

            // generate path to the node
            path_segment_t path_to_pose = lyap_path_->steer((*node_it)->pose, 
                                                            sampled_pose);

            // collision check
            bool pose_can_be_reached_from_node = 
                CLFRRTStarPlanner::isObstacleFree(path_to_pose.steps);

            // update, if collision-free
            if (pose_can_be_reached_from_node) {
                // check cost
                cost_t cost_from_root = (*node_it)->cost + 
                    CLFRRTStarPlanner::computeRRTCostFromNodeToPose(
                    **node_it, sampled_pose, moving_backward);

                // choose the node with the minimum cost from the root as the parent
                if (cost_from_root < min_cost_from_root) {
                    parent_node = *node_it;
                    min_cost_from_root = cost_from_root;
                }
            }
        }

        return parent_node;
    }


    void CLFRRTStarPlanner::updateGlobalMap(const grid_map::GridMap* new_global_map)
    {
        this->global_map_ = new_global_map;
        // this->global_map_updated_ = true;
    }


    void CLFRRTStarPlanner::updateRobotState(const robot_state_t* new_robot_state)
    {
        this->robot_state_ = *new_robot_state;
        // this->robot_state_updated_ = true;
    }

    void CLFRRTStarPlanner::updateLocalMapAtCurrentRobotState(
            const robot_state_t* new_robot_state)
    {
        CLFRRTStarPlanner::updateRobotState(new_robot_state);
        local_map_->updateLocalMap(robot_state_.pose);
    }


    void CLFRRTStarPlanner::updateLocalMap()
    {
        // debugger::debugOutput("[CLFRRT]/[updatreLocalMap] Current pose: ", 
        //                       robot_state_.pose, 5);
        local_map_->updateLocalMap(robot_state_.pose);
        // debugger::debugTextOutput("hey",3);
    }


    void CLFRRTStarPlanner::setStartPose(const pose_t& new_start_pose)
    {
        start_pose_ = new_start_pose;
        robot_state_.pose = new_start_pose;
    }

    void CLFRRTStarPlanner::setGoalPose(const pose_t& new_goal_pose)
    {
        goal_pose_ = new_goal_pose;
    }

















    void CLFRRTStarPlanner::publishDebug_()
    {
        marker_pub_.publish(marker_array_);

        pcl_conversions::toPCL(ros::Time::now(),
                debug_path_points_->header.stamp);
        path_pub_.publish(debug_path_points_);
        debugger::debugTextOutput("published debug info", 3);
    }
}
