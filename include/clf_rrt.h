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
// #pragma once

#ifndef CLF_RRT_H
#define CLF_RRT_H


#include "cassie_rrt_tree.h"
#include "lyapunov_distance.h"
#include "lyapunov_path.h"
#include "fake_map.h"
#include "local_map.h"
#include "pose.h"
#include "robot_state.h"
#include "sample_pose.h"
#include "computation_analysis.h"
#include "map_cost.h"

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h> // Marker



namespace bipedlab 
{



typedef struct rrt_params
{
    int mode = 0; // 0: vanilla, 1: height // use int for ros parameters
    // bool terminate_if_path = true;
    // int num_samples = 1000; // use int for ros parameters
    // double allowed_computation_time = 1; // in seconds
    double max_extension_length = 4.0;
    double goal_threshold = 0.1; // under this, consider goal is reached


    // a radius to search if a waypoint is obstacle free
    // <should be the times of grid_resolution>
    double path_obstacle_threshold = 0.1; 


    double backward_motion_sampling_rate = 0.2;   // 0.0 ~ 1.0


    // phi, delta, radious, type
    lyapunov_distance_params_t lyap_dist_params;

    // goal bias, distance threshold  away from obstalces
    pose_sampler_params_t samplerParams;
} rrt_params_t;


typedef struct rrt_path_info
{
    bool have_found_path;
    bool num_poses_sampled;
    std::vector<point2d_t<float>> path;
    std::vector<std::pair<pose_t, bool>> waypoints;
    std::vector<std::pair<LyapunovDistance, bool>> manifold;
} rrt_path_info_t;

// typedef struct planner_status
// {
//     bool found_path;
//     bool robot_state_updated;
//     bool gloal_map_updated;
// } planner_status_t;


class CLFRRTStarPlanner
{
private:
    friend class Driver;
    pose_t samplePose_(bool& is_goal);


    cassie_rrt_node_t* findNearestNodeToPose_(const pose_t& pose, 
                                       const bool move_backward);

    path_segment_t extendFromTreeNodeToSampledPose_(
            const cassie_rrt_node_t& rrt_node,
            const pose_t& sampled_pose,
            const double& max_extension,
            const bool move_backward) const;

    bool isObstacleFree(const std::vector<pose_t>& steps) const;

    std::vector<cassie_rrt_node_t*> 
        findNearbyNodesToPose_(
                const pose_t& sampled_pose, const bool move_backward);


    cassie_rrt_node_t* 
        chooseRRTParent_(const std::vector<cassie_rrt_node_t*>& nearby_nodes,
                        cassie_rrt_node_t* nearest_node,
                        const pose_t& sampled_pose,
                        const bool moving_backward);
    cassie_rrt_node_t* 
        chooseRRTParentVanilla_(const std::vector<cassie_rrt_node_t*>& nearby_nodes,
                        cassie_rrt_node_t* nearest_node,
                        const pose_t& sampled_pose,
                        const bool moving_backward);
    cassie_rrt_node_t* 
        chooseRRTParentWithMapInfo_(
                const std::vector<cassie_rrt_node_t*>& nearby_nodes,
                cassie_rrt_node_t* nearest_node,
                const pose_t& sampled_pose,
                const bool moving_backward);



    cassie_rrt_node_t*
        insertRRTNode_(cassie_rrt_node_t* parent_node,
                       const pose_t& child_pose,
                       bool is_goal,
                       bool move_backward);

    // find path from sampled pose to a given node. used for rrt-star-rewire
    path_segment_t findPathFromSampledNodeToPose( 
            const pose_t& sampled_pose, const cassie_rrt_node_t& rrt_node) const;

    std::vector<cassie_rrt_node_t*>
        findNearbyNodesFromPose_(const pose_t& sampled_pose,
                                 const bool move_backward);

    void rewireRRTNodes(const std::vector<cassie_rrt_node_t*>& nearby_nodes, 
                        cassie_rrt_node_t* rewire_node);

    void reconnectRRTTree(cassie_rrt_node_t* new_parent,
                          cassie_rrt_node_t* child_to_rewire,
                          cost_t rewired_edge_cost);

    void propagateRRTCostToChildren(cassie_rrt_node_t* parent_node);


    // cost definition
    cost_t computeRRTCostFromNodeToPose(const cassie_rrt_node_t& from_node,
                                        const pose_t& to_pose,
                                        const bool move_backward);


    cost_t computeRRTCostFromPoseToNode(const pose_t& from_pose, 
                                        const cassie_rrt_node_t& to_node);








    MapCost* map_cost_;
    LocalMap* local_map_; // planner will keep using this until a new map arrives
    SamplePose* sampling_;
    CassieRRTTree* rrt_tree_;
    LyapunovDistance* lyap_dist_;
    LyapunovPath* lyap_path_;


    const grid_map::GridMap* global_map_;

    
    // poses and robot state
    pose_t start_pose_;
    pose_t goal_pose_;
    robot_state_t robot_state_;


    // pose_t sampleRandomPoseWithGoalBiased_(void);

    pose_sampler_params_t sampler_params_;
    lyapunov_distance_params_t lyap_dist_params_;
    rrt_params_t rrt_params_;
    rrt_path_info_t found_path_info_;


    // 
    size_t num_samples_;
    double total_seconds_spent_;


    cost_t minimum_cost_to_goal_;
    cassie_rrt_node_t* minimum_cost_leaf_;

    bool is_path_;
    bool have_improved_path_;


    analysis::computation_analysis_t computation_analysis_;






    // various representation of the planned path (i.e. the best path found)
    // a series of rrt nodes leading to the goal
    std::vector<cassie_rrt_node_data_t> path_nodes_;   

    // a series of 2D points which represents a path on a plane.
    std::vector<point2d_t<double>> planned_path_;            

    // a series of waypoints and approach direction. i.e. <pose, isBackward> from start to goal
    //     std::vector<std::pair<UnicycleLyapunovDistance, bool>> plannedManifold_;  // a series of lyapunov functions
    //     anchored around each waypoint on the path.
    std::vector<std::pair<pose_t, bool>> planned_waypoints_;   

    bool retrievePathFromLeaf_(cassie_rrt_node_t* leaf_node);

    int is_debugging_; // flag to save more data for visualization
    std::vector<pose_t> sampling_poses_;
    void publishDebug_(void);

    // ros
    ros::NodeHandle nh_;
    ros::Publisher path_pub_;
    ros::Publisher marker_pub_;
    pcl::PointCloud<pcl::PointXYZI>::Ptr debug_path_points_;
    visualization_msgs::Marker marker_;
    visualization_msgs::MarkerArray marker_array_;


public:
    CLFRRTStarPlanner(const grid_map::GridMap& map, 
                      const local_map_params_t& local_map_params,
                      const pose_t& start_pose, 
                      const pose_t& goal_pose,
                      robot_state_t& robot_state, 
                      pose_sampler_params_t& pose_sampler_params,
                      rrt_params_t& rrt_params, 
                      cost_params_t& cost_map_params,
                      lyapunov_distance_params_t& lyap_dist_params);
    virtual ~CLFRRTStarPlanner();



    // Path-building methods
    // -- Attempt to find a NEW path within:
    //      1) max allowed number of samples. 
    //      2) max allowed time in second
    // -- Terminates as soon as a path is found.
    bool findNewPath(size_t max_num_samples, 
                     double longest_duration, 
                     bool terminate_if_path);

    // core algorithm 
    bool runCLFRRTAlgorithm(size_t max_samples,
                            double allow_time,
                            bool terminate_if_path);

    // attempt to IMPROVE an existing path within 
    //      1) max allowed number of additional samples. 
    //      2) max allowed time in second
    //      3) decide if terminates as soon as a path is improved.
    bool addMoreTimeNSamples(size_t max_additional_samples, 
                             double allow_more_time,
                             bool terminate_if_path);


    void updateGlobalMap(const grid_map::GridMap* new_global_map);
    void updateRobotState(const robot_state_t* new_robot_state);
    void updateLocalMapAtCurrentRobotState(const robot_state_t* new_robot_state);
    void updateLocalMap();


    void setStartPose(const pose_t& new_start_pose);
    void setGoalPose(const pose_t& new_goal_pose);




    // Getters
    grid_map::GridMap getLocalMap(void) { return local_map_->local_map; };
    void printLocalMapAddress(void) { debugger::debugOutput("local address: ", &(local_map_->local_map), 5); };
    void printGlobalMapCLFAddress(void) { debugger::debugOutput("global address in clf: ", global_map_, 5); };
    void printGlobalMapLocalMapAddress(void) { local_map_->printMapAddress(), 5; };

    LocalMap* getLocalMapClass(void) const { return local_map_; };
    MapCost* getMapCostClass(void) const { return map_cost_; };

    pose_t getGoalPose(void) const { return goal_pose_; };
    pose_t getStartPose(void) const { return start_pose_; };
    cost_t getMinimumCost(void) const { return minimum_cost_to_goal_; };

    // getters for retrieving various description of the planned path. 
    // Will return empty vector if no path has been found.
    bool getPathStatus(void) const { return is_path_; };
    double getDuration(void) const { return total_seconds_spent_; };
    std::vector<pose_t> getSamples(void) const { return sampling_poses_; };

    std::vector<point2d_t<double>> 
        getPlannedPath(void) const { return planned_path_; };
    std::vector<std::pair<pose_t, bool>> 
        getPlannedWaypoints(void) const { return planned_waypoints_; };

    std::vector<point2d_t<double>>*
        getPlannedPathPtr(void) { return &planned_path_; };

    std::vector<std::pair<pose_t, bool>>*
        getPlannedWaypointsPtr(void) { return &planned_waypoints_; };


    // getters for planner info
    std::size_t getNumOfTreeNodes(void) { return rrt_tree_->tree_.size(); };
    // std::list<cassie_rrt_node_data_t> getGraphInfo(void);
    //

    void printStatus(int level, std::string text = "");


    pose_t sample_pose_testing;
    //grid_map::GridMap debug_local_map;
};
} // bipedlab

#endif /* CLF_RRT_H */
