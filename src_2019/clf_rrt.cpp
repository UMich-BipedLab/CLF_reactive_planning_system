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
// #include "utils/utils.h"
#include "utils/debugger.h"
#include "utils/timing.h"
#include <float.h> // for DBL_MAX

// #ifndef DEBUG_LEVEL
// #define DEBUG_LEVEL 3
// #endif /* ifndef DEBUG_LEVEL */

namespace bipedlab 
{
    CLFRRTStarPlanner::CLFRRTStarPlanner(
            const grid_map::GridMap& map, double length_of_local_map,
            const pose_6dof_t& start_pose, const pose_6dof_t& goal_pose,
            robot_state_t& robot_state, 
            pose_sampler_params_t& pose_sampler_params,
            rrt_params_t& rrt_params) : 
        map_(map), 
        start_pose_(start_pose), 
        goal_pose_(goal_pose), 
        robot_state_(robot_state),
        rrt_params_(rrt_params),
        total_seconds_spent_(0)
    { 

        debugger::debugTextOutput("[CLFRRT] CLFRRTStarPlanner initializing...", 5);
        local_map_ = new LocalMap(start_pose, map, length_of_local_map);
        // debug_local_map = local_map_->local_map;


        sampling_ = new SamplePose(pose_sampler_params, 
                                   *local_map_, 
                                   goal_pose,
                                   robot_state);

        // LyapunovDistance() is declared here for speed
        // whichever class (lyapunovPath, CassieRRTTree) 
        // wants to change the target pose of the local chart inside of 
        // the LyapunovDistance(), CHANGE ON ITS OWN!
        lyap_dist_ = new LyapunovDistance(*local_map_); 
        lyap_path_ = new LyapunovPath(*lyap_dist_, *local_map_);
        rrt_tree_ = new CassieRRTTree(start_pose, *lyap_dist_, *local_map_);


        debugger::debugTextOutput("[CLFRRT Constructor] \
                testing sampling function... ", 0);
        bool dummy;
        sample_pose_testing = 
            sampling_->sampleRandomPoseWithGoalBiasedWithinLocalMap(dummy);

        debugger::debugTextOutput("[CLFRRT] CLFRRT is running... ", 5);
        is_path_ = CLFRRTStarPlanner::findNewPath(
                rrt_params_.num_samples, 
                rrt_params_.allowed_computation_time, 
                rrt_params_.terminate_if_path);
        debugger::debugOutput("[CLFRRT] has found a path? ", is_path_, 5);
        debugger::debugOutput("[CLFRRT] points of path ", 
                planned_path_.size(), 5);
        debugger::debugOutput("[CLFRRT] number of waypoints ", 
                planned_waypoints_.size(), 5);

    }
    CLFRRTStarPlanner::~CLFRRTStarPlanner() { }



    bool CLFRRTStarPlanner::addMoreTimeNSamples(
            size_t max_additional_samples, 
            double allow_more_time)
    {
        if (num_samples_ == 0) {
            return CLFRRTStarPlanner::findNewPath(
                    max_additional_samples, allow_more_time, true);
        } else {
            return CLFRRTStarPlanner::runCLFRRTAlgorithm(
                    max_additional_samples, allow_more_time, false);
        }
    }


    bool CLFRRTStarPlanner::findNewPath(
            size_t max_num_samples, 
            double longest_duration,
            bool terminate_if_path)
    {
        debugger::debugTextOutput("[CLFRRT] Initializaing a new tree...", 1);
        num_samples_ = 0;
        minimum_cost_to_goal_ = DBL_MAX;
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
        bool have_improved_path = false;
        size_t num_path_improvement = 0;


        // timing 
        auto start_time = timing::getCurrentTime();
        double time_of_spend = timing::spendElapsedTime(start_time);


        while (time_of_spend < allow_more_time && 
               num_new_samples < max_additional_samples)
        {
            // always false for now, will be changed in samplePose() later 
            bool move_backward = false; 

            // will be changed in samplePose();
            bool is_goal = false; 

            // sample a pose
            debugger::debugTextOutput(
                    "[RRTAlgorithm] Sampling a pose...", 1);
            pose_t sampled_pose = CLFRRTStarPlanner::samplePose_(is_goal);

            debugger::debugTextOutput(
                    "[RRTALgorithm] Finding the nearest node...", 1);
            cassie_rrt_node_t* nearest_node = 
                CLFRRTStarPlanner::findNearestNodeToPose_(sampled_pose, 
                                                         move_backward);

            debugger::debugTextOutput(
                    "[RRTALgorithm] Extending from the nearest node...", 1);
            path_segment_t edge_path = 
                CLFRRTStarPlanner::extendFromTreeNodeToSampledPose_(
                    *nearest_node, sampled_pose, 
                    rrt_params_.max_extension_length, move_backward);
            
            if (CLFRRTStarPlanner::isObstacleFree(edge_path.steps))
            {
                debugger::debugOutput("[RRTALgorithm] Path to the nearest node \
                        is collision free:\n" , nearest_node->pose, 1);
                
                // find nearby nodes from the extended node
                debugger::debugTextOutput("[RRTALgorithm] \
                        Searching nearby nodes...", 1);
                pose_t new_sample = edge_path.steps.back();
                std::vector<cassie_rrt_node_t*> nearby_nodes = 
                    CLFRRTStarPlanner::findNearbyNodesToPose_(sampled_pose, 
                                                              move_backward);

                // choose a parent by choosing the minimum-cost route from the root
                debugger::debugTextOutput("[RRTALgorithm] \
                        Finding parent node...", 1);
                cassie_rrt_node_t* parent_node = 
                    CLFRRTStarPlanner::chooseRRTParent_(
                            nearby_nodes, nearest_node, new_sample, move_backward);

                // insert the node to the tree
                debugger::debugTextOutput("[RRTALgorithm] \
                        Node inserting...", 1);
                cassie_rrt_node_t* new_node = CLFRRTStarPlanner::insertRRTNode_(
                        parent_node, new_sample, is_goal, move_backward);

                // find nearby nodes from the sample 
                // (this will include itself, which is the last sample in the tree)
                debugger::debugTextOutput("[RRTALgorithm] \
                        Finding nodes near from the inserted node...", 1);
                nearby_nodes = CLFRRTStarPlanner::findNearbyNodesFromPose_(
                        new_sample, move_backward);



                // rewire neargy nodes to the new node, 
                // if it decreases overall cost.
                debugger::debugTextOutput("[RRTALgorithm] \
                        Rewiring...", 1);
                CLFRRTStarPlanner::rewireRRTNodes(nearby_nodes, new_node);

                if (is_goal) 
                {
                    if (new_node->cost < minimum_cost_to_goal_) 
                    {
                        minimum_cost_to_goal_ = new_node->cost;
                        minimum_cost_leaf_ = new_node;
                    }
                    if (num_path_improvement == 0)
                    {
                        debugger::debugTextOutput("[CLFRRT] Found a path", 5);
                    }
                    else
                    {
                        debugger::debugOutput("[RRTALgorithm] Improved the path: ", 
                                num_path_improvement, 2);
                        debugger::debugOutput("[RRTALgorithm] Has Spent (s): ", 
                                timing::spendElapsedTime(start_time), 2);
                    }
                    num_path_improvement ++;
                    have_improved_path = true;
                }
            }

            num_new_samples ++;
            debugger::debugOutput("[RRTALgorithm] num of new samples: ", 
                                num_new_samples, 2);
            time_of_spend = timing::spendElapsedTime(start_time);
            if (have_improved_path && terminate_if_path) {
                break;
            }
        }

        // update statitistic
        num_samples_ += num_new_samples;
        total_seconds_spent_ += time_of_spend;

        if (num_samples_ != 0) {
            computation_analysis_.num_samples_per_second = 
                num_samples_ / total_seconds_spent_;
        }
        debugger::debugTextOutput("[CLFRRT] ------------ Results ------------", 5);
        debugger::debugOutput("[CLFRRT] samples / seconds: ", 
                computation_analysis_.num_samples_per_second, 5);
        debugger::debugOutput("[CLFRRT] Number of samples: ", num_samples_, 5);
        debugger::debugOutput("[CLFRRT] Total Spent [s]: ", total_seconds_spent_, 5);

        if (have_improved_path) {
            // update path information
            CLFRRTStarPlanner::retrievePathFromLeaf_(minimum_cost_leaf_);
        }

        return have_improved_path;
    }


    pose_t CLFRRTStarPlanner::samplePose_(bool& is_goal)
    {
        return sampling_->sampleRandomPoseWithGoalBiasedWithinLocalMap(is_goal);
    }


    cassie_rrt_node_t* CLFRRTStarPlanner::findNearestNodeToPose_(
            const pose_t& pose,
            const bool move_backward) 
    {
        return rrt_tree_->findNearestNodeToPoseWithMap(pose);
    }


    path_segment_t CLFRRTStarPlanner::extendFromTreeNodeToSampledPose_(
            const cassie_rrt_node_t& rrt_node,
            const pose_t& sampled_pose,
            const double& max_extension,
            const bool move_backward) const
    {
        // pose_t poseFrom = isMovingBackward ? rrtNode.pose.flip() : rrtNode.pose;
        // pose_t poseTo = isMovingBackward ? sampledPose.flip() : sampledPose;

        // UnicycleLyapunovDistance lyap(poseTo, params_.distanceParams);
        return lyap_path_->extend(rrt_node.pose,  sampled_pose, max_extension);
    }

    
    // TODO: change to splitting the path into 100 segment (?)
    bool CLFRRTStarPlanner::isObstacleFree(const std::vector<pose_t>& steps) const
    {

        bool is_collision_free = true;
        for (auto step_it = steps.begin(); step_it != steps.end(); step_it++) {
            // if (grid.getObstacleDistance((*stepIt).toPoint()) < kDistanceThreshlod) {
            position_t position = (*step_it).to2DPosition();

            bool is_locally_free = local_map_->isNeighborObstacleFree(
                        position.x, position.y, rrt_params_.distance_threshold);
            if (!is_locally_free) {
                is_collision_free = false;
                break;
            }
        }

        return is_collision_free;
    }

        
    std::vector<cassie_rrt_node_t*> 
        CLFRRTStarPlanner::findNearbyNodesToPose_(
                const pose_t& sampled_pose, const bool move_backward)
    {
        return rrt_tree_->findNearbyNodesToPose(sampled_pose, move_backward);
    }


    // Cost definitions
    double CLFRRTStarPlanner::computeRRTCostFromNodeToPose(
            const cassie_rrt_node_t& from_node,
            const pose_t& to_pose,
            const bool move_backward)
    {
        // LyapunovDistance lyap(isMovingBackward ? toPose.flip() : toPose, params_.distanceParams);
        //return lyap.distanceFromPose(isMovingBackward ? fromNode.pose : fromNode.pose);

        lyap_dist_->assignNewTargetPose(to_pose);
        return lyap_dist_->computeDistanceFromPose(from_node.pose);
    }

    double CLFRRTStarPlanner::computeRRTCostFromPoseToNode(
            const pose_t& from_pose, const cassie_rrt_node_t& to_node)
    {
        // LyapunovDistance lyap(toNode.shouldBeApproachedBackward ? toNode.pose.flip() : toNode.pose,
        //                               params_.distanceParams);
        // return lyap.distanceFromPose(toNode.shouldBeApproachedBackward ? fromPose.flip() : fromPose);

        lyap_dist_->assignNewTargetPose(to_node.pose);
        return lyap_dist_->computeDistanceFromPose(from_pose);
    }




    // RRT-Star methods
    cassie_rrt_node_t* 
        CLFRRTStarPlanner::chooseRRTParent_(
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
        double min_cost_from_root = 
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
                double cost_from_root = (*node_it)->cost + 
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

     
    cassie_rrt_node_t* 
        CLFRRTStarPlanner::insertRRTNode_(cassie_rrt_node_t* parent_node,
                                          const pose_t& child_pose,
                                          bool is_goal,
                                          bool move_backward)
    {
        double cost_from_parent = CLFRRTStarPlanner::computeRRTCostFromNodeToPose(
            *parent_node, child_pose, move_backward);
        cassie_rrt_node_t* inserted_node =
            rrt_tree_->insertNode(child_pose, cost_from_parent, is_goal, 
                                  move_backward, parent_node);

        return inserted_node;
    }

    std::vector<cassie_rrt_node_t*> 
        CLFRRTStarPlanner::findNearbyNodesFromPose_(const pose_t& sampled_pose, 
                                                    const bool move_backward)
    {
        return rrt_tree_->findNearbyNodesFromPose(sampled_pose, move_backward);
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
                path_segment_t path_to_node = 
                    CLFRRTStarPlanner::findPathFromSampledNodeToPose(
                            rewire_node->pose, **node_it);

                // check collision
                bool pose_can_be_reached_from_node = 
                    CLFRRTStarPlanner::isObstacleFree(path_to_node.steps);

                if (pose_can_be_reached_from_node) {
                    // compute cost via rewire node
                    double rewired_edge_cost = 
                        CLFRRTStarPlanner::computeRRTCostFromPoseToNode(
                            rewire_node->pose, **node_it);
                    double rewired_cost_to_root = 
                        rewire_node->cost + rewired_edge_cost;

                    // if the cost can be improved
                    if (rewired_cost_to_root < (*node_it)->cost) {
                        // reconnect edges and propagate cost to leaves
                        CLFRRTStarPlanner::reconnectRRTTree(
                                rewire_node, *node_it, rewired_edge_cost);
                    }
                }
            }
        }
    }

    void CLFRRTStarPlanner::reconnectRRTTree(
            cassie_rrt_node_t* new_parent,
            cassie_rrt_node_t* child_to_rewire,
            double rewired_edge_cost)
    {
        // delete connection to old parent
        rrt_tree_->deleteChildFromParent(child_to_rewire, child_to_rewire->parent);

        // rewire to the new node
        new_parent->children.push_back(child_to_rewire);   // set child to parent
        child_to_rewire->parent = new_parent;              // set parent to child
        child_to_rewire->cost = new_parent->cost + rewired_edge_cost;

        // propagate cost down the line
        CLFRRTStarPlanner::propagateRRTCostToChildren(child_to_rewire);
    }

    void CLFRRTStarPlanner::propagateRRTCostToChildren(
            cassie_rrt_node_t* parent_node)
    {
        for (auto child_it = parent_node->children.begin(); 
             child_it != parent_node->children.end(); child_it++) {
            // compute cost between edge
            double cost_to_child =
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
        // retrieve path data
        cassie_rrt_node_t* node_ptr = leaf_node;
        while (node_ptr != nullptr) {
            path_nodes_.push_back(cassie_rrt_node_data_t(*node_ptr));
            node_ptr = node_ptr->parent;
        }

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
            }
        }

        return true;
    }
}


    
}
