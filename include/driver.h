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
#ifndef DRIVER_H
#define DRIVER_H
#include <utility> // std::pair, std::make_pair
#include <list>

#include <eigen3/Eigen/Dense> // SVD
#include <eigen3/Eigen/StdVector>
#include <eigen3/Eigen/Eigenvalues>


#include "boost/thread/mutex.hpp"
#include "boost/thread.hpp"

#include <ros/ros.h>

#include <nav_msgs/Path.h>
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

#include "clf_rrt.h"
#include "communication.h"
#include "grid_map_msgs/GridMap.h"
#include "control_commands.h"
#include "standalone_omni_local_chart.h"

#include "utils/utils.h"
#include "utils/timing.h"
#include "csv.h"

#include "inekf_msgs/State.h"
#include "planner_msgs/State.h"


// #include <stdio.h>
// #define getName(var) #var


namespace bipedlab
{
typedef struct robot_params
{
    int heading_angle_preintegration; // preintegrate omega for heading angle
    double step_time_interval; // sed for heading as well (heading = omega * time)
    double default_omega; // rad/s (when robot rotates in-place)

    // velocity 
    int enlarge_mode; 
    double normalized_speed; 
    double velocity_factor; 
    double vx_upper_bound;
    double vy_upper_bound;
} robot_params_t;


typedef struct goal_searching_for_planner
{
    int goal_search_mode;
    int goal_behavior;
    double search_radius;
    double delta_theta;
    size_t num_poses;


    // buffer of an angle: given an angle, the cost inside the buffer
    // will accumulated for the angle at 0.5 deg increment. In total,
    // (2 * (buffer / 0.5)) will be evaulated
    double buffer;



    // weight of the yaw angle between the subgoal and the robot orientation.
    // it is the percentage of the total_cost of the subgoal 
    // actual weight is percentage_weight_of_orientation * cost.getTotalCost()
    // new_cost  = 
    //     cost_t(old_cost.distance + weight * (subgoal_theta - robot_theta), 
    //            old_cost.height);
    double percentage_weight_of_orientation;


    //                   
    // weight between cost of a subgoal and distance from the subgoal to final goal
    //                ----- A -------     ------------------- B -------------------
    // to bias the goal selection so it is closer to final goal
    // You can also think it is used to compute the cost_to_goal
    // new_cost = subgoal_cost + w * distance_subgoal_to_final_goal
    double weight_subgoal_and_final_goal;


    std::vector<double> list_of_angles;

    // <pose, cost of the pose>
    std::vector<std::pair<pose_t, cost_t>> list_of_poses; 
    
    std::pair<double, double> search_view;


    std::list<std::list<std::pair<pose_t, cost_t>>> path_segments;
    int goal_segment_minimum_len;
    double goal_segment_max_cost;
    double goal_segment_linkage_dis_weight;
    double goal_segment_linkage_cost_threshold;



    pose_t goal_pose;
    cost_t goal_cost;
} goal_searching_for_planner_t;

typedef struct planner_status
{
    bool has_path_to_use;
    bool updated;
    std::chrono::steady_clock::time_point latest_time;


    planner_status(void) : updated(false), has_path_to_use(false),
                           latest_time(timing::getCurrentTime()) { }

    planner_status(const bool& updated) : updated(updated), has_path_to_use(false),
                           latest_time(timing::getCurrentTime()) { }
} planner_status_t;

typedef struct planner_results
{
    bool found_path;
    bool improved_path;

    cost_t new_cost;
    cost_t cost;

    planner_status_t status;


    std::vector<point2d_t<double>>* new_path_points_ptr;
    std::vector<std::pair<pose_t, bool>>* new_pose_to_follow_ptr;

    std::vector<std::pair<pose_t, bool>> pose_to_follow;
    std::vector<point2d_t<double>> path_points;
    

    planner_results(void) : status(false),
                            new_path_points_ptr(nullptr),
                            new_pose_to_follow_ptr(nullptr),
                            new_cost(0, 0), cost(1e5, 1e5){ }
} planner_results_t;

typedef struct goal_selection_for_robot
{
    pose_t robot_pose; // robot pose at the moment of command

    std::pair<pose_t, bool> chosen_goal; // goal chosen by findNextGoalForRobot_()

    // which goal number is chosen (start with 0)
    // This is used to determine if the current goal is reached and 
    // should assign the next one to go 
    size_t goal_num; 
    bool has_chosen_goal;
    goal_selection_for_robot(void) : chosen_goal({pose_t(), false}), 
                                     has_chosen_goal(false), goal_num(0) { }
} goal_selection_for_robot_t;
    

typedef struct command_history
{
    std::list<double> vx_list;
    std::list<double> vy_list;
    std::list<double> vr_list;
    std::list<double> vd_list;
    std::list<double> omega_list;
    std::list<double> roll_list;
    std::list<double> pitch_list;
    std::list<double> yaw_list;

    size_t getLength()
    {
        return vx_list.size();
    }

    void popFront()
    {
        vx_list.pop_front();
        vy_list.pop_front();
        vr_list.pop_front();
        vd_list.pop_front();
        omega_list.pop_front();
        roll_list.pop_front();
        pitch_list.pop_front();
        yaw_list.pop_front();
    }

    void pushBack(const double& vx, const double& vy,
            const double& vr, const double& vd,
            const double& omega,
            const double& roll, const double& pitch, const double& yaw)
    {
        vx_list.push_back(vx);
        vy_list.push_back(vy);
        vr_list.push_back(vr);
        vd_list.push_back(vd);
        omega_list.push_back(omega);
        roll_list.push_back(roll);
        pitch_list.push_back(pitch);
        yaw_list.push_back(yaw);
    }
    // std::list<double> w1_list;
    // std::list<double> w2_list;
    // std::list<double> w3_list;

    // size_t getLength()
    // {
    //     return speed_list.size();
    // }

    // void popFront()
    // {
    //     speed_list.pop_front();
    //     w1_list.pop_front();
    //     w2_list.pop_front();
    //     w3_list.pop_front();
    // }

    // void pushBack(const double& speed, 
    //         const double& roll, const double& pitch, const double& yaw)
    // {
    //     speed_list.push_back(speed);
    //     w1_list.push_back(roll);
    //     w2_list.push_back(pitch);
    //     w3_list.push_back(yaw);
    // }



} command_history_t;

class Driver
{
private:

    // initialization functions
    void waitForData_();
    bool getParameters_();
    void spin_();


    // callback functions
    void getClickPointCallBack_(const geometry_msgs::PointStamped::ConstPtr& msg);
    void getMultiLayerCallBack_(const grid_map_msgs::GridMap& msg);
    void getInEKFCallBack_(const inekf_msgs::State& msg);
    void getUDPCallBack_();


    // main loop function (running at slower freq.:5 Hz)
    int driveRobot_();

    bool getRobotCurrentState_(robot_state_t& current_state);
    bool updateRobotPose_();

    bool updateGlobalMap_();
    void decideSubGoalForPlanner_();

    // goal finder
    pose_t searchGoal_();
    void detectIntersection(goal_searching_for_planner_t& goal_searching);
    bool computeDirectionOfSearchingArc(double& theta); // return 1 if too close to fianl goal
    void cluterViaHierarchicalAlgorithm(
        goal_searching_for_planner_t& goal_searching,
        std::list<std::list<std::pair<pose_t, cost_t>>>& path_segments);
    void pruneClustersAndCollectInformation(
        goal_searching_for_planner_t& goal_searching,
        std::list<std::list<std::pair<pose_t, cost_t>>>& path_segments,
        std::list<std::pair<size_t, double>>& segment_info);


    void pickGoalSegment(
        const std::list<std::pair<pose_t, cost_t>>::iterator& pose_it,
        double& distance_from_seg_to_goal, double& angle_seg_to_robot,
        const size_t path_segment_size,
        bool& flag_picked);








    // update goal pose for map
    void getGoalPoseForMap_(int map_number);


    void planPathFromRRT(const bool& if_global_map_updated, bool& found_path, 
                         bool& improved_path, bool& if_timeout);
    void printResultsFromRRT(const bool& improved_path);
    void updatePlannerResults_(const bool& found_path, const bool& improved_path, 
                               const bool& if_timeout);


    void publishToRviz_();


    // publish loop function (running at faster freq.: 300 Hz)
    void publishInfoToRobot_();
    void prepareInfoWithMPCForRobot_(control_variables_t& control_variables);
    int examinePathStatusForExecutionThread_(const double& throttle_time);
    std::pair<bool, bool> // <has_reached_goal, has_goal_left>
        checkWhetherAnyGoalLeftForExecutionThread_(
                double throttle_time,
                control_variables_t& control_variables,
                const robot_state_t& current_robot_state); 
    void findNextGoalForExecutionThread_(double throttle_time, 
            const robot_state_t& current_robot_state);
    void computeOptimalControlWithBoundsAndThresholds(
            control_variables_t& control_variables,
            const robot_state_t& current_robot_state);
    void executeReactivePlanner_(double throttle_time, 
            control_variables_t& control_variables,
            const robot_state_t& current_robot_state);






    // first: if a goal is found to reach
    // second: the goal to reach. If no goal, return robot current pose
    void smoothCommandFromHistory_(double& vx, double& vy, 
            double& vr, double& vd, double& omega,
            double& roll, double& pitch, double& yaw);

    void logCommands(const planner_info_to_controller_t& data,
                     const control_variables_t& control_variables);




    // not used so far
    bool lookUpRobotPose();
    void computeRobotVelocityFromTF();


    // ROS 
    ros::NodeHandle nh_;


    // publisher
    ros::Publisher global_map_pub_;
    ros::Publisher local_map_pub_;
    ros::Publisher rrt_path_pub_; // publish path from the rrt 
    ros::Publisher planner_path_pub_; // publish path from the planner
    ros::Publisher planner_command_pub_; // publish path from the planner
    ros::Publisher trajectory_pub_; // publish robot trajectory 
    ros::Publisher marker_pub_;
    ros::Publisher marker_array_pub_;
    double marker_lifetime_;

    // ros::Publisher frontier_publisher_;
    // ros::Publisher side_walk_publisher_;
    
    //subscriber
    ros::Subscriber map_sub_;
    ros::Subscriber inekf_sub_;
    ros::Subscriber click_point_sub_;

    // markers
    nav_msgs::Path robot_trajectory_;
    visualization_msgs::Marker marker_;
    visualization_msgs::MarkerArray goal_selection_marker_array_;
    visualization_msgs::MarkerArray robot_marker_array_;
    visualization_msgs::MarkerArray planner_marker_array_;
    pcl::PointCloud<pcl::PointXYZI>::Ptr rrt_path_points; // path from rrt
    pcl::PointCloud<pcl::PointXYZI>::Ptr planner_path_points; // path from planner




    // subscribe channel
    std::string map_frame_;
    std::string robot_frame_;
    std::string costmap_topic_;
    std::string multi_layer_map_topic_;
    std::string inekf_topic_;

    // tf
    tf::TransformBroadcaster broadcaster_;
    tf::TransformListener listener_;

    // if data received
    bool data_received_=0;
    bool goal_received_=0;


    // mutex locks
    boost::mutex map_lock_;
    boost::mutex pose_lock_;
    boost::mutex planner_results_lock_;
    boost::mutex goal_selection_lock_;
    boost::mutex clicked_lock_;


    // if data updated
    bool global_map_updated_ =0;
    bool robot_state_updated_=0;
    bool pose_updated_=0;
    bool goal_updated_=0;
    bool found_path_=0;


    // planner class
    CLFRRTStarPlanner* planner_;
    Communication* udp_;




    // map and state
    grid_map::GridMap multi_layer_map_;
    inekf_msgs::State inekf_state_;
    std::vector<double> final_pose_x_list_; // a list of final poses <rosparm can only use std::vector>
    std::vector<double> final_pose_y_list_; // a list of final poses
    std::vector<double> final_pose_yaw_list_; // a list of final poses
    pose_t final_goal_pose_; // the same as goal_selection_.goal_pose
    pose_t sub_goal_pose_; // guided by the fianl goal and used for rrt in local map
    pose_t clicked_goal_pose_; 
    std::pair<int, controller_info_to_planner_t> received_udp_pose_;




    // robot state and map
    // grid_map::GridMap local_map_;
    const MapCost* map_cost_;
    const LocalMap* local_map_;
    robot_state_t robot_state_;

    // subsciber update to these (if new info arrives, assign it to here 
    // and will be used for the next iteration)
    grid_map::GridMap new_multi_layer_map_;
    inekf_msgs::State new_inekf_state_;



    // counter 
    size_t pose_look_up_counter_;



    // parameters
    int map_number_; // update for goal pose
    int log_commands_;
    int driver_method_;
    int clf_model_;
    double publishing_rate_;
    double replanning_rate_;
    double allow_planning_time_;


    robot_params_t robot_params_;
    rrt_params_t rrt_params_;
    local_map_params_t local_map_params_;
    cost_params_t cost_map_params_;
    lyapunov_distance_params_t lyap_dist_params_;
    pose_sampler_params_t pose_sampler_params_;
    goal_searching_for_planner_t goal_searching_;
    goal_selection_for_robot_t goal_selection_;
    communication_t udp_info_;


    int is_exploration_;
    double is_subgoal_threshold_;
    double old_path_timeout_;
    double walking_in_place_timeout_;
    double cost_diff_threshold_within_timeout_; // 
    int command_history_length_; // 


    // results
    planner_results_t planner_results_;
    command_history_t command_history_;
    planner_info_to_controller_t planner_info_to_controller_;

    // log
     CSVFile command_csv_;



public:
    Driver(ros::NodeHandle& nh);
    virtual ~Driver();
};

} /* bipedlab */ 
#endif /* DRIVER_H */
