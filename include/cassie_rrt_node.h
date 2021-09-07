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
#ifndef CASSIE_RRT_NODE_H
#define CASSIE_RRT_NODE_H

#include "pose.h"
#include "map_cost.h"
#include <iosfwd>
#include <list>
#include <iostream>

#include "cereal/access.hpp"
#include "cereal/types/vector.hpp"

namespace bipedlab
{


// typedef struct cost
// {
//     // no total cost here on purpose
//     double distance;
//     double height;
// 
// 
//     explicit cost(const double& distance_cost, const double& height_cost) :
//         distance(distance_cost), 
//         height(height_cost) { }
// 
//     explicit cost(const double& distance_cost) :
//          distance(distance_cost), height(0) { }
// 
//     cost(void) : distance(0), height(0) { };
// 
//     cost(const cost& t_cost) :
//         distance(t_cost.distance), height(t_cost.height) { }
// 
//     inline cost operator+ (const cost& t_cost) 
//     {
//         return cost(t_cost.distance + distance, t_cost.height + height);
//     }
// 
//     inline bool operator< (const cost& rhs) const
//     {
//         return distance + height < rhs.distance + rhs.height;
//     }
// 
//     // std::ostream& operator<<(std::ostream& out, const cost& t_cost) 
//     // {
//     //     out << '(' << t_cost.distance << ',' << t_cost.height << ')';
//     //     return out;
//     // }
// 
// 
//     double getTotalCost(void) {return distance + height; };
// 
// } cost_t;



/*
 * cassie_rrt_node_t stores data for a node in a tree (pose graph).
 */
struct cassie_rrt_node_t
{
    // node ID
    size_t node_id;                                   

    // pose of the waypoint associated with the node
    pose_6dof_t pose;                              

    // pointer to parent. (This is nullptr for the root node)
    cassie_rrt_node_t* parent;                  

    // pointers to children nodes
    std::list<cassie_rrt_node_t*> children;   

    // cost from the root node
    cost_t cost;                                  

    // indicator for reaching the goal
    bool is_goal;                                  

    // approach direction indicator
    bool should_be_approached_backward;

    cassie_rrt_node_t(void){};
    cassie_rrt_node_t(int node_id, const pose_t& pose, 
            cost_t cost, bool is_goal, 
            bool should_be_approached_backward)
    : node_id(node_id), 
      pose(pose), 
      cost(cost), 
      parent(nullptr), 
      is_goal(is_goal), 
      should_be_approached_backward(should_be_approached_backward){
          children.clear();
      };
};


/*
 * cassie_rrt_node_t stores read/writable data associated with node (without pointers)
 * NOTE: edge data (e.g. points on a path) is not stored as they can be
 *       reconstructed easily via steering. Edge data may get added if needed.
 */
struct cassie_rrt_node_data_t
{
    size_t node_id;
    pose_6dof_t pose;
    std::vector<size_t> parent_id;        // can be empty
    std::vector<pose_t> parent_pose;   // can be empty
                                      //     std::vector<int>           childIDs;   // can be empty
                                      //     std::vector<pose_t> childPoses; // can be empty
    cost_t cost;
    bool is_goal;
    bool should_be_approached_backward;

    cassie_rrt_node_data_t(void){};
    cassie_rrt_node_data_t(const cassie_rrt_node_t& node)
    : node_id(node.node_id), pose(node.pose), cost(node.cost), 
      is_goal(node.is_goal), 
      should_be_approached_backward(node.should_be_approached_backward)
    {
        if (node.parent) {
            parent_id.push_back(node.parent->node_id);
            parent_pose.push_back(node.parent->pose);
        };

        //         for(auto childNodeIt = node.children.begin(); childNodeIt != node.children.end(); childNodeIt++)
        //         {
        //             childIDs.push_back((*childNodeIt)->nodeID);
        //             childPoses.push_back((*childNodeIt)->pose);
        //         };
    };
};


// Serialization support
template <class Archive>
void serialize(Archive& ar, cassie_rrt_node_data_t& data)
{
    ar(data.node_id,
       data.pose,
       data.parent_id,
       data.parent_pose,
       data.cost,
       data.is_goal,
       data.should_be_approached_backward);
}
}   // namespace bipedlab

#endif   // cassie_RRT_NODE_H

