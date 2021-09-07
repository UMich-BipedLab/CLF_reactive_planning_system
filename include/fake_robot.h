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
#ifndef FAKE_ROBOT_H
#define FAKE_ROBOT_H


#include <ros/ros.h>
#include <ros/console.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h> // Marker

#include "boost/thread/mutex.hpp"
#include "boost/thread.hpp"


#include <cmath>

#include "inekf_msgs/State.h"
#include "planner_msgs/State.h"
#include "pose.h"


namespace bipedlab
{
    
class FakeRobot
{
private:
    void getPlannerMsgsCallback_(const planner_msgs::State& msg);
    void publishToRviz_();
    void updatePlannerCommands();
    void estimateRobotMotion();
    void getRobotPoseForMap(int map_number);
    void assignRobotPoseXYZRPH(double x, double y, double z, 
                               double roll, double pitch, double heading);




    void spin_();
    void publishCurrentPose_();
    bool getParameters_();


    // ros stuff
    ros::NodeHandle nh_;
    ros::Publisher pose_pub_;
    ros::Publisher marker_pub_;
    ros::Subscriber planner_sub_;


    // markers
    visualization_msgs::Marker marker_;
    visualization_msgs::MarkerArray marker_array_;


    double receiving_rate_;
    double publishing_rate_;
    double rviz_rate_;
    double delta_t_;

    inekf_msgs::State fake_robot_;
    inekf_msgs::State old_fake_robot_;
    planner_msgs::State received_msg_;

    float roll_;
    float pitch_;
    float yaw_;
    int behavior_;

    bool command_updated_;

    // pose only for visualization
    pose_t start_; 
    pose_t current_; 


    boost::mutex new_command_lock_;
    boost::mutex fake_robot_lock_;

    int debug_;
    int robot_model_;
    int pose_mode_;
    int clf_mode_;
    int heading_angle_preintegration_;
    double step_time_interval_;

    

public:
    FakeRobot(ros::NodeHandle& nh);
    void preceedRobot();


    virtual ~FakeRobot();
};


} /*  bipedlab */ 
#endif /* FAKE_ROBOT_H */
