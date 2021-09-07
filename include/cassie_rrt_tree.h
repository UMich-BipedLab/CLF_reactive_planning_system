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
#ifndef CASSIE_RRT_TREE_H
#define CASSIE_RRT_TREE_H

#include "lyapunov_distance.h"
#include "lyapunov_path.h"
#include "cassie_rrt_node.h"
#include <cassert>
#include <list>

namespace bipedlab
{

typedef struct rrt_tree_search
{
    double backward_distance_weight;       // > 1.0
    double nearness_param_gamma;
    size_t k_search_space_dimension;

    double height_search_threshold;

    rrt_tree_search(void) : 
        backward_distance_weight(3.0), 
        nearness_param_gamma(40.0), 
        k_search_space_dimension(3),
        height_search_threshold(25) { }
} rrt_tree_search_t; 


class CassieRRTTree
{
public:
    using nodePtr = cassie_rrt_node_t*;

    /**
     * Constructor for CassieRRTTree
     *
     * \param    startPose       start pose for the path. Forms the root node.
     * \param    distanceParams  parameters for UnicycleLyapunovDistance
     */
    CassieRRTTree(const pose_t& startPose, 
                  LyapunovDistance& lyapunov,
                  const LyapunovPath& lyap_path);

    CassieRRTTree(void){};

    // mutators
    /**
     * initializeTree initializes the rrt tree with the goal node
     *
     * \param    initialPose     pose of the goal
     */
    void initializeTree(const pose_t& startPose);

    /**
     * insertNode creates a node, insert it to the tree, and connect it to a parent node
     *
     * \param    newSample               sampled pose to be added
     * \param    costFromParent          cost from the parent
     * \param    isGoal                  sample is the goal
     * \param    isMovingBackward        sample should be approached with backward motion
     * \param    parentNodePtr           parent node to be connected (will be modified)
     * \return   pointer to the inserted node
     */
    nodePtr insertNode(const pose_t& newSample,
                       cost_t costFromParent,
                       bool isGoal,
                       bool isMovingBackward,
                       nodePtr parentNodePtr);

    /**
     * deleteChildFromParent attetmpts to delete a child node from a parent node.
     *
     * \param    childNodeToDelete       pointer to the child node to be deleted
     * \param    parentNode              pointer to the parent node
     *
     * \return   success/failure
     */
    bool deleteChildFromParent(const cassie_rrt_node_t* childToDelete, 
                               nodePtr parentNode);


    double computeNeighborhoodRadius(const bool move_backward);

    // find operations

    /**
     * findNearbyNodesFromPose finds a set of nodes whose directed distance *from*
     * a pose is less than specified threshold.
     *
     * \param    pose                    a pose from which the distance is measured.
     * \param    distanceThreshold       threshold for the non-holonomic distance
     * \param    isMovingBackward        approach direction indicator (optional. default = forward)
     *
     * \return   a vector of pointers to nearby nodes
     */
    std::vector<nodePtr>
      findNearbyNodesFromPose(const pose_t& pose, 
                              const bool isMovingBackward = false);
    std::vector<nodePtr>
      findNearbyNodesFromPoseWithMapInfo(const pose_t& pose, 
                              const bool isMovingBackward = false);

    /**
     * findNearbyNodesToPose finds a set of nodes whose directed distance *to*
     * a pose is less than specified threshold.
     *
     * \param    pose                    a pose to which the distance is measured.
     * \param    distanceThreshold       threshold for the non-holonomic distance
     * \param    isMovingBackward        approach direction indicator (optional. default = forward)
     *
     * \return   a vector of pointers to nearby nodes
     */
    std::vector<nodePtr>
      findNearbyNodesToPose(const pose_t& pose, 
                            const bool isMovingBackward = false);
    std::vector<nodePtr>
      findNearbyNodesToPoseWithMapInfo(const pose_t& pose, 
                            const bool isMovingBackward = false);

    /**
     * findNeaestNodeFromPose finds the nearest node *from* a pose.
     *
     * \param    pose                    a pose from which the distance is measured.
     * \param    isMovingBackward        approach direction indicator (optional. default = forward)
     *
     * \return   pointer to the nearest node
     */
    nodePtr findNearestNodeFromPose(const pose_t& pose, double* nonHolonomicDistance, bool isMovingBackward = false);

    /**
     * findNearestNodesToPose finds the nearest node *to* a pose, *excluding the goal.*
     *
     * \param    pose                    a pose to which the distance is measured.
     * \param    isMovingBackward        approach direction indicator (optional. default = forward)
     *
     * \return   pointer to the nearest node
     */
    nodePtr findNearestNodeToPose(const pose_t& pose, 
                                  bool move_backward = false);
    nodePtr findLowestCostNodeToPoseWithMapInfo(const pose_t& pose, 
                                         bool move_backward = false);



private:
    friend class CLFRRTStarPlanner; // to get the number of tree nodes
    rrt_tree_search_t search_params_;


    //lyapunov_distance_params_t lyapunov_params_;
    LyapunovDistance* lyap_dist_;
    const LyapunovPath* lyap_path_;
    std::list<cassie_rrt_node_t> tree_;   // WARNING: using vector here can invalidate pointers.
};

}   // namespace bipedlab

#endif

