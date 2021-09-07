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
 * \file     cassie_rrt_tree.cpp
 * \author   Bruce JK Huang, Jong Jin Park
 *
 * Definition of CassieRRTTree class, which encapsulates the rapidly exploring
 * tree of poses.
 */

#include "cassie_rrt_tree.h"
#include <float.h>

namespace bipedlab
{

CassieRRTTree::CassieRRTTree(const pose_t& start_pose, 
                             LyapunovDistance& lyap_dist,
                             const LyapunovPath& lyap_path)
{
    lyap_dist_ = &lyap_dist;
    lyap_path_ = &lyap_path;
    // initializeTree(start_pose);
}


void CassieRRTTree::initializeTree(const pose_t& start_pose)
{
    // create root node (id, pose, cost, is_goal, backward)
    cassie_rrt_node_t root_node(0, start_pose, cost_t(0.0), 0, 0);

    // initialize the tree and add a node
    tree_.clear();
    tree_.push_back(root_node);
}


cassie_rrt_node_t* CassieRRTTree::insertNode(const pose_t& new_sample,
                                             cost_t cost_from_parent,
                                             bool is_goal,
                                             bool move_backward,
                                             cassie_rrt_node_t* parent_node_ptr)
{
    assert(parent_node_ptr != nullptr);

    // create a new node
    // this depends on the monotonically increasing IDs in the tree, 
    // which is guaranteed by construction.
    int new_node_id = tree_.back().node_id + 1;   
    cassie_rrt_node_t new_node(new_node_id, new_sample, 
                               parent_node_ptr->cost + cost_from_parent, 
                               is_goal, move_backward);
    new_node.parent = parent_node_ptr;   // set parent

    // add to tree
    tree_.push_back(new_node);

    // add a child to the parent node
    cassie_rrt_node_t* inserted_node = &(tree_.back());
    parent_node_ptr->children.push_back(inserted_node);

    return inserted_node;
}


bool CassieRRTTree::deleteChildFromParent(
        const cassie_rrt_node_t* child_to_delete, 
        cassie_rrt_node_t* parent_node)
{
    bool is_deleted = false;

    for (auto child_it = parent_node->children.begin(); 
         child_it != parent_node->children.end(); child_it++) {
        if (*child_it == child_to_delete) {
            is_deleted = true;
            // TODO: deletion maybe slow! perhaps change to list(?)
            // if (*child_it == nullptr) 
            //     debugger::debugTextOutput("[CLFRRT] erased0", 5);
            // debugger::debugTextOutput("[CLFRRT] erased1", 5);
            // free(*child_it);
            // // *child_it = nullptr;
            // debugger::debugTextOutput("[CLFRRT] erased2", 5);
            // // parent_node->children.erase(child_it);   // this invalidates the iterator! has to exit now.
            debugger::debugTextOutput("[CLFRRT] Erased!", 0);
            parent_node->children.erase(child_it);   // this invalidates the iterator! has to exit now.
            break;                                 // assume no duplicate child
        }
    }

    return is_deleted;   // report successful deletion
}


// Neighbor searches
double CassieRRTTree::computeNeighborhoodRadius(const bool move_backward)
{
    double num_nodes = static_cast<double>(tree_.size());
    double distance_multiplier = pow(log(num_nodes) / num_nodes, 
                                    1.0 / search_params_.k_search_space_dimension);

    if (move_backward) {
        distance_multiplier /= search_params_.backward_distance_weight;
    }

    // debugger::debugOutput("distance radious: ", distance_multiplier * search_params_.nearness_param_gamma, 3);
    return distance_multiplier * search_params_.nearness_param_gamma;
}



std::vector<cassie_rrt_node_t*>
  CassieRRTTree::findNearbyNodesFromPose(const pose_t& pose, 
                                         const bool move_backward)
{
    std::vector<cassie_rrt_node_t*> neighbors;
    double distance_threshold = 
        CassieRRTTree::computeNeighborhoodRadius(move_backward);

    // compute distance *from* a pose ...
    // pose_t fromPose = move_backward ? pose.flip() : pose;

    for (auto node_it = tree_.begin(); node_it != tree_.end(); node_it++) {
        // *to* a node
        // pose_t toPose = move_backward ? node_it->pose.flip() : pose;
        lyap_dist_->assignNewTargetPose(node_it->pose);
        double distance_to_node = lyap_dist_->computeDistanceFromPose(pose);

        // add to neighbors if the distance is less than threshold
        if (distance_to_node < distance_threshold) {
            neighbors.push_back(&(*node_it));
        }
    }

    return neighbors;
}

std::vector<cassie_rrt_node_t*>
  CassieRRTTree::findNearbyNodesFromPoseWithMapInfo(const pose_t& pose, 
                                                    const bool move_backward)
{
    std::vector<cassie_rrt_node_t*> neighbors;
    double distance_threshold = 
        CassieRRTTree::computeNeighborhoodRadius(move_backward);

    // compute distance *from* a pose ...
    // pose_t fromPose = move_backward ? pose.flip() : pose;

    for (auto node_it = tree_.begin(); node_it != tree_.end(); node_it++) {
        // *to* a node
        // pose_t toPose = move_backward ? node_it->pose.flip() : pose;
        // lyap_dist_->assignNewTargetPose(node_it->pose);
        path_segment_t path_segment = lyap_path_->steer(pose, node_it->pose);

        // add to neighbors if the distance is less than threshold
        if (path_segment.cost_along_path.distance < distance_threshold &&
            path_segment.cost_along_path.height < 
            search_params_.height_search_threshold) {
            neighbors.push_back(&(*node_it));
        }
    }

    return neighbors;
}

std::vector<cassie_rrt_node_t*>
  CassieRRTTree::findNearbyNodesToPoseWithMapInfo(const pose_t& pose, 
                                       const bool moving_backward)
{
    std::vector<cassie_rrt_node_t*> neighbors;
    double distance_threshold = 
        CassieRRTTree::computeNeighborhoodRadius(moving_backward);

    // compute distance *to* a pose ...
    // pose_t toPose = move_backward ? pose.flip() : pose;
    // LyapunovDistance lyap(toPose, distanceParams_);   // Lyapunov distance function around the pose
    // lyap_dist_->assignNewTargetPose(pose);

    // TODO: if too slow, check this logic again 
    // debugger::debugOutput("[RRTTree]/[nearbyNodesToPose] distance threshold: ", 
    //         distance_threshold, 3);
    // debugger::debugOutput("[RRTTree]/[nearbyNodesToPose] height threshold: ", 
    //         search_params_.height_search_threshold, 3);
    // debugger::debugOutput("[RRTTree]/[nearbyNodesToPose] target pose: ", 
    //         pose, 4);
    for (auto node_it = tree_.begin(); node_it != tree_.end(); node_it++) {
        // *from* a node
        // pose_t fromPose = move_backward ? node_it->pose.flip() : node_it->pose;
        path_segment_t path_segment = lyap_path_->steer(node_it->pose, pose);
        // double distance_to_node = lyap_dist_->computeDistanceFromPose(node_it->pose);
        // debugger::debugOutput("[RRTTree]/[nearbyNodesToPose] distance: ", 
        //         distance_to_node, 4);
        // debugger::debugOutput("[RRTTree]/[nearbyNodesToPose] current pose: ", 
        //         node_it->pose, 3);
        // debugger::debugOutput("[RRTTree]/[nearbyNodesToPose] current cost: ", 
        //         path_segment.cost_along_path, 3);

        // add to neighbors if the distance is less than threshold
        if (path_segment.cost_along_path.distance < distance_threshold &&
            path_segment.cost_along_path.height < 
            search_params_.height_search_threshold) {
            neighbors.push_back(&(*node_it));
        }
    }

    return neighbors;
}


std::vector<cassie_rrt_node_t*>
  CassieRRTTree::findNearbyNodesToPose(const pose_t& pose, 
                                       const bool moving_backward)
{
    std::vector<cassie_rrt_node_t*> neighbors;
    double distance_threshold = 
        CassieRRTTree::computeNeighborhoodRadius(moving_backward);

    // compute distance *to* a pose ...
    // pose_t toPose = move_backward ? pose.flip() : pose;
    // LyapunovDistance lyap(toPose, distanceParams_);   // Lyapunov distance function around the pose
    lyap_dist_->assignNewTargetPose(pose);

    for (auto node_it = tree_.begin(); node_it != tree_.end(); node_it++) {
        // *from* a node
        // pose_t fromPose = move_backward ? node_it->pose.flip() : node_it->pose;
        double distance_to_node = lyap_dist_->computeDistanceFromPose(node_it->pose);

        // add to neighbors if the distance is less than threshold
        if (distance_to_node < distance_threshold) {
            neighbors.push_back(&(*node_it));
        }
    }

    return neighbors;
}


// cassie_rrt_node_t*
//   CassieRRTTree::findNearestNodeFromPose(const pose_t& pose, double* nonHolonomicDistance, bool move_backward)
// {
//     cassie_rrt_node_t* nearestNode = nullptr;
//     double minDistance = 1000000.0;   // some large number
// 
//     /* need to fix 6dof pose issue
//     // compute distance *from* a pose
//     pose_t fromPose = move_backward ? pose.flip() : pose;
// 
//     for (auto node_it = tree_.begin(), nodeEnd = tree_.end(); node_it != nodeEnd; node_it++) {
//         // *to* a node
//         pose_t toPose = move_backward ? node_it->pose.flip() : node_it->pose;
//         LyapunovDistance lyap(toPose, distanceParams_);
//         double distanceToNode = lyap.computeDistanceFromPose(fromPose);
// 
//         if (minDistance > distanceToNode) {
//             nearestNode = &(*node_it);
//             minDistance = distanceToNode;
//         }
//     }
// 
//     if (nonHolonomicDistance) {
//         *nonHolonomicDistance = minDistance;
//     }
//     */
// 
//     return nearestNode;
// }

cassie_rrt_node_t*
CassieRRTTree::findNearestNodeToPose(const pose_t& pose, 
                                     bool move_backward)
{
    cassie_rrt_node_t* nearest_node = nullptr;
    double min_distance = DBL_MAX;   // some large number;

    // pose_t toPose = move_backward ? pose.flip() : pose;
    lyap_dist_->assignNewTargetPose(pose);

    // compute distance *to* a pose
    for (cassie_rrt_node_t& node_it : tree_) {
        if (!node_it.is_goal)   // need to exclude the goal pose here!
        {
            // pose_t fromPose = move_backward ? node_it->pose.flip() : node_it->pose;
            double distance_from_node = 
                lyap_dist_->computeDistanceFromPose(node_it.pose);

            if (min_distance > distance_from_node) {
                nearest_node = &node_it;
                min_distance = distance_from_node;
            }
        }
    }

    return nearest_node;
}

cassie_rrt_node_t*
CassieRRTTree::findLowestCostNodeToPoseWithMapInfo(const pose_t& pose, 
                                            bool move_backward)
{
    cassie_rrt_node_t* nearest_node = nullptr; // node with lowest cost
    cost_t min_cost = cost_t(DBL_MAX/2 - 1.0, DBL_MAX/2 - 1.0);

    // pose_t toPose = move_backward ? pose.flip() : pose;
    lyap_dist_->assignNewTargetPose(pose);

    // compute distance *to* a pose
    for (cassie_rrt_node_t& node_it : tree_) {
        if (!node_it.is_goal)   // need to exclude the goal pose here!
        {
            // pose_t fromPose = move_backward ? node_it->pose.flip() : node_it->pose;
            path_segment_t path_segment = lyap_path_->steer(node_it.pose, pose);


            if (path_segment.cost_along_path < min_cost) {
                nearest_node = &node_it;
                min_cost = path_segment.cost_along_path;
            }
        }
    }

    return nearest_node;
}

}   // namespace bipedlab

