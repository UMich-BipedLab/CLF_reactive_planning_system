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
#include "fake_robot.h"
#include "angle_functions.h"
#include "robot_motion.h"


#include "utils/utils.h"
#include "utils/ros_utils.h"
#include "utils/nameof.h"
#include "utils/debugger.h"
#include "utils/plotting.h"




namespace bipedlab
{
FakeRobot::FakeRobot(ros::NodeHandle& nh):
    receiving_rate_(3), publishing_rate_(300), delta_t_(0.01), 
    roll_(0), pitch_(0), yaw_(0), 
    command_updated_(false), debug_(0)
{
    nh_ = nh;
    if (debug_)
        command_updated_ = true;

    // ros::Rate r(10);
    if (!FakeRobot::getParameters_())
    {
        debugger::debugTitleTextOutput("[FakeRobot]", "NOTICE!!!!", 10, BR, BOLD);
        debugger::debugColorOutput("[FakeRobot] Not enough parameters: ",
                                   "", 10, BR, BOLD);
        exit(-1);
    }
    else
    {
        debugger::debugColorOutput("[FakeRobot] Received all parameters", "", 10, BC);
    }


    if (pose_mode_)
    {
        getRobotPoseForMap(pose_mode_);
    }

    // process the pose
    fake_robot_.header.frame_id = "map";
    tf2::Quaternion q_tf;
    q_tf.setRPY((roll_), (pitch_), (yaw_));
    fake_robot_.orientation = tf2::toMsg(q_tf);
    start_ = pose_t(fake_robot_.position.x, fake_robot_.position.y, yaw_);
    // debugger::debugOutput("yaw:", yaw_, 5);


    pose_pub_ = nh_.advertise<inekf_msgs::State>("inekf", 1, true);
    marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("fake_robot", 10);




    // received messages
    planner_sub_ = nh_.subscribe("/planner/planner_commands", 10, 
            &FakeRobot::getPlannerMsgsCallback_, this);




    // ready to get data
    boost::thread ros_spin(&FakeRobot::spin_, this);
    boost::thread rviz_pub(&FakeRobot::publishToRviz_, this);
    boost::thread current_pose_pub(&FakeRobot::publishCurrentPose_, this);
    debugger::debugColorOutput("[FakeRobot] Initialized!", "", 5, Y);
}

// get called in fake_robot_publisher.cpp
void FakeRobot::preceedRobot()
{
    double time_diff = 0;
    ros::Time time = ros::Time::now();
    double receiving_duration = 1.0 / receiving_rate_;

    while(nh_.ok())
    {
        // debugger::debugTitleTextOutput("[FakeRobot]/[preceedRobot]", 
        //         "start preceedRobot()", 5, G);
        ROS_INFO_STREAM_THROTTLE(1, "[FakeRobot]/[preceedRobot] " << 
                "start preceedRobot()");
        new_command_lock_.lock();
        time_diff = ros::Time::now().toSec() - time.toSec();
        if (time_diff > receiving_duration)
        {
            if (!command_updated_)
            {
                // debugger::debugColorOutput("[FakeRobot]/[preceedRobot]" 
                //         "no command received!", "", 5, Y);
                ROS_WARN_STREAM_THROTTLE(1, "[FakeRobot]/[preceedRobot] " 
                        "no command received!");

            }
            else
            {
                // ROS_INFO_STREAM_THROTTLE(1, "[FakeRobot]/[preceedRobot] " 
                //         "planner commands updated!");
                // debugger::debugTextOutput("[FakeRobot]/[preceedRobot] " 
                //         "planner commands updated!", 5);
                time = ros::Time::now();
                FakeRobot::updatePlannerCommands();
                if (robot_model_ == 1)
                {
                    FakeRobot::estimateRobotMotion();
                }
            }
        }
        new_command_lock_.unlock();

        if (robot_model_ == 0)
        {
            FakeRobot::estimateRobotMotion();
        }

        usleep(delta_t_ * 1e6);
        // utils::pressEnterToContinue();
    }
}

void FakeRobot::updatePlannerCommands()
{
    // update planner commands
    if (!debug_)
    {
        command_updated_ = false;

        fake_robot_lock_.lock();
        old_fake_robot_ = fake_robot_;
        fake_robot_.velocity = received_msg_.velocity;
        roll_ = received_msg_.torso.roll;
        pitch_ = received_msg_.torso.pitch;
        yaw_ = received_msg_.torso.yaw;
        behavior_ = received_msg_.behavior;
        fake_robot_lock_.unlock();

        if (behavior_ == 0)
        {
            debugger::debugOutput("[FakeRobot]/[updatePlannerCommands] "
                    "Walking in-place", "", 5);
        }
        else if(behavior_ == 1)
        {
            debugger::debugOutput("[FakeRobot]/[updatePlannerCommands] "
                    "Walking", "", 5);
        }
        else if(behavior_ == -1)
        {
            debugger::debugOutput("[FakeRobot]/[updatePlannerCommands] "
                    "Stand", "", 5);
        }

        // debugger::debugOutput("[FakeRobot]/[updatePlannerCommands] Current vx: ", 
        //         fake_robot_.velocity.x, 5);
        // debugger::debugOutput("[FakeRobot]/[updatePlannerCommands] Current vy: ", 
        //         fake_robot_.velocity.y, 5);
        // debugger::debugOutput("[FakeRobot]/[updatePlannerCommands] Current roll: ", 
        //         rad_to_deg(roll_), 5);
        // debugger::debugOutput("[FakeRobot]/[updatePlannerCommands] Current pitch: ", 
        //         rad_to_deg(pitch_), 5);
        // debugger::debugOutput("[FakeRobot]/[updatePlannerCommands] Current yaw: ", 
        //         rad_to_deg(yaw_), 5);
    }
    else if (debug_ == 1)
    {
        command_updated_ = true;
        roll_ = utils::genInclusiveRandomNumber(0, M_PI);
        debugger::debugOutput("[FakeRobot]/[updatePlannerCommands] Current vx: ", 
                fake_robot_.velocity.x, 5);
        debugger::debugOutput("[FakeRobot]/[updatePlannerCommands] Current vy: ", 
                fake_robot_.velocity.y, 5);
        debugger::debugOutput("[FakeRobot]/[updatePlannerCommands] Current roll: ", 
                rad_to_deg(roll_), 5);
        debugger::debugOutput("[FakeRobot]/[updatePlannerCommands] Current pitch: ", 
                rad_to_deg(pitch_), 5);
        debugger::debugOutput("[FakeRobot]/[updatePlannerCommands] Current yaw: ", 
                rad_to_deg(yaw_), 5);
    }
    else if (debug_ == 2)
    {
        command_updated_ = true;
        fake_robot_.velocity.x = utils::genInclusiveRandomNumber(0, 2);
        fake_robot_.velocity.y = utils::genInclusiveRandomNumber(0, 2);
        fake_robot_.velocity.z = 0; 
        roll_ = utils::genInclusiveRandomNumber(0, M_PI);
        pitch_ = utils::genInclusiveRandomNumber(0, M_PI); 
        yaw_ = utils::genInclusiveRandomNumber(0, M_PI);
        debugger::debugOutput("[FakeRobot]/[updatePlannerCommands] Current vx: ", 
                fake_robot_.velocity.x, 5);
        debugger::debugOutput("[FakeRobot]/[updatePlannerCommands] Current vy: ", 
                fake_robot_.velocity.y, 5);
        debugger::debugOutput("[FakeRobot]/[updatePlannerCommands] Current roll: ", 
                rad_to_deg(roll_), 5);
        debugger::debugOutput("[FakeRobot]/[updatePlannerCommands] Current pitch: ", 
                rad_to_deg(pitch_), 5);
        debugger::debugOutput("[FakeRobot]/[updatePlannerCommands] Current yaw: ", 
                rad_to_deg(yaw_), 5);
    }
    else if (debug_ == 3)
    {
        command_updated_ = true;
        double speed = 1.0;
        roll_ = 0; 
        pitch_ = 0;
        yaw_ = M_PI + utils::genInclusiveRandomNumber(-0.1, 0.1);
        fake_robot_.velocity.x = std::cos(yaw_);
        fake_robot_.velocity.y = std::sin(yaw_);
        fake_robot_.velocity.z = 0; 


        debugger::debugOutput("[FakeRobot]/[updatePlannerCommands] Current vx: ", 
                fake_robot_.velocity.x, 5);
        debugger::debugOutput("[FakeRobot]/[updatePlannerCommands] Current vy: ", 
                fake_robot_.velocity.y, 5);
        debugger::debugOutput("[FakeRobot]/[updatePlannerCommands] Current roll: ", 
                rad_to_deg(roll_), 5);
        debugger::debugOutput("[FakeRobot]/[updatePlannerCommands] Current pitch: ", 
                rad_to_deg(pitch_), 5);
        debugger::debugOutput("[FakeRobot]/[updatePlannerCommands] Current yaw: ", 
                rad_to_deg(yaw_), 5);
    }

}

void FakeRobot::estimateRobotMotion()
{
    if (robot_model_ == 0 && clf_mode_ == 0)
    {
        // for differential driven robot model
        tf2::Quaternion q;
        fake_robot_lock_.lock();
        fake_robot_.position.x +=  fake_robot_.velocity.x * delta_t_;
        fake_robot_.position.y +=  fake_robot_.velocity.y * delta_t_;
        fake_robot_.position.z +=  0;

        q.setRPY(roll_,  pitch_, yaw_);
        // debugger::debugColorOutput("[FakeRobot]/[estimateRobotMotion] Yaw:", 
        //         rad_to_deg(yaw_), 5, BC);
        // ROS_INFO_STREAM_THROTTLE(1, "[FakeRobot]/[estimateRobotMotion] Yaw:" << 
        //                             rad_to_deg(yaw_));
        geometry_msgs::Quaternion q_msg = tf2::toMsg(q);
        fake_robot_.orientation = q_msg;
        fake_robot_lock_.unlock();
    }
    else if (robot_model_ == 0 && clf_mode_ == 1)
    {
        tf2::Quaternion q;
        double cur_roll;
        double cur_pitch;
        double cur_yaw;

        fake_robot_lock_.lock();
        tf2::fromMsg(fake_robot_.orientation, q);
        tf2::Matrix3x3(q).getRPY(cur_roll, cur_pitch, cur_yaw);

        pose_t translated_pose = moveRobotInWorldFrame(
                fake_robot_.velocity.x, fake_robot_.velocity.y, 
                fake_robot_.velocity.z, 
                fake_robot_.position.x, fake_robot_.position.y, cur_yaw,
                delta_t_);

        fake_robot_.position.x =  translated_pose.x;
        fake_robot_.position.y =  translated_pose.y;
        fake_robot_.position.z =  0;

        if (!heading_angle_preintegration_)
        {
            // WITHOUT preintegrated heading angle in planner
            q.setRPY(cur_roll, cur_pitch, translated_pose.theta);
        }
        else
        {
            // WITH preintegrated heading angle in planner
            q.setRPY(cur_roll, cur_pitch, yaw_);
        }


        geometry_msgs::Quaternion q_msg = tf2::toMsg(q);
        fake_robot_.orientation = q_msg;
        fake_robot_lock_.unlock();
    }
    else if (robot_model_ == 1 && clf_mode_ == 1)
    {
        tf2::Quaternion q;
        double cur_roll;
        double cur_pitch;
        double cur_yaw;

        fake_robot_lock_.lock();
        tf2::fromMsg(fake_robot_.orientation, q);
        tf2::Matrix3x3(q).getRPY(cur_roll, cur_pitch, cur_yaw);
        

        pose_t new_pose = moveRobotWithALIPModelInWorldFrame(
                fake_robot_.velocity.x, 
                fake_robot_.velocity.y,  // desire speed
                old_fake_robot_.velocity.x,  
                old_fake_robot_.velocity.y, 
                old_fake_robot_.velocity.z, // current speed
                pose_t(fake_robot_.position.x, fake_robot_.position.y, cur_yaw), 
                step_time_interval_);
        fake_robot_.position.x = new_pose.x;
        fake_robot_.position.y = new_pose.y;

        if (!heading_angle_preintegration_)
        {
            std::string error_msg = 
                "ERROR: Set heading_angle_preintegration_ for ALIP model: " +
                std::to_string(heading_angle_preintegration_);
            debugger::debugExitColor(error_msg, __LINE__, __FILE__);
        }
        else
        {
            q.setRPY(cur_roll, cur_pitch, new_pose.theta);
        }
        geometry_msgs::Quaternion q_msg = tf2::toMsg(q);
        fake_robot_.orientation = q_msg;

        fake_robot_lock_.unlock();
    }
    else
    {
        std::string error_msg = 
            "ERROR: No such robot model and CLF_mode supported: " +
            std::to_string(robot_model_) + ", " + std::to_string(clf_mode_);
        debugger::debugExitColor(error_msg, __LINE__, __FILE__);


    }
    // debugger::debugColorOutput("[FakeRobot]/[estimateRobotMotion] Yaw:", 
    //         rad_to_deg(yaw_), 5, BC);
    ROS_INFO_STREAM_THROTTLE(1, "[FakeRobot]/[estimateRobotMotion] Yaw:" << 
                                rad_to_deg(yaw_));


    // for differential driven robot model
    // tf2::Quaternion q;
    // fake_robot_lock_.lock();
    // fake_robot_.position.x +=  fake_robot_.velocity.x * delta_t_;
    // fake_robot_.position.y +=  fake_robot_.velocity.y * delta_t_;
    // fake_robot_.position.z +=  0;

    // q.setRPY(roll_,  pitch_, yaw_);
    // // debugger::debugColorOutput("[FakeRobot]/[estimateRobotMotion] Yaw:", 
    // //         rad_to_deg(yaw_), 5, BC);
    // ROS_INFO_STREAM_THROTTLE(1, "[FakeRobot]/[estimateRobotMotion] Yaw:" << 
    //                             rad_to_deg(yaw_));
    // geometry_msgs::Quaternion q_msg = tf2::toMsg(q);
    // fake_robot_.orientation = q_msg;

    // double d_yaw = fake_robot_.velocity.z * delta_t_; // vz is omega

    // double cur_roll
    // double cur_pitch;
    // double cur_yaw;
    // tf2::Matrix3x3(q).getRPY(cur_roll, cur_pitch, cur_yaw);
    // fake_robot_lock_.unlock();


    // tf2::Quaternion q;
    // fake_robot_lock_.lock();
    //         Eigen::Vector2d velocity_robot_frame;
    //     velocity_robot_frame << fake_robot_.velocity.x, fake_robot_.velocity.y;

    //     // compute translation and change of robot orientation in robot frame
    // Eigen::Vector2d translation_robot_frame =
    //         velocity_robot_frame * delta_t_;


    // // rotation matrix to transform to world frame
    // double cos_theta = std::cos(yaw_);
    // double sin_theta = std::sin(yaw_);
    // Eigen::Matrix2d R;
    // R << cos_theta, -sin_theta,
    //   sin_theta, cos_theta;

    // // transform to world frame
    // Eigen::Vector2d translation_world_frame = R * translation_robot_frame;

    // translated_pose =  pose_t(translated_pose.x + translation_world_frame(0),
    //         translated_pose.y + translation_world_frame(1),
    //         translated_pose.theta - delta_theta_robot_frame);

    // fake_robot_lock_.unlock();
}

void FakeRobot::assignRobotPoseXYZRPH(double x, double y, double z,
                               double roll, double pitch, double yaw)
{
    fake_robot_.position.x = x;
    fake_robot_.position.y = y;
    fake_robot_.position.z = z;
    yaw_ = yaw;
    roll_ = roll;
    pitch_ = pitch;
}



void FakeRobot::getRobotPoseForMap(int map_number)
{
    switch (map_number)
    {
        case 1:
            FakeRobot::assignRobotPoseXYZRPH(-12, 12, 0, 0, 0, -45);
            break;
        case 3:
            FakeRobot::assignRobotPoseXYZRPH(-5, 14, 0, 0, 0, -45);
            break;
        case 100:
            FakeRobot::assignRobotPoseXYZRPH(-10, 0, 0, 0, 0, 0);
            // FakeRobot::assignRobotPoseXYZRPH(-2.3, 0, 0, 0, 0, 0);
            break;
        case 101:
            FakeRobot::assignRobotPoseXYZRPH(-10, 0, 0, 0, 0, 0);
            break;
        case 102:
            FakeRobot::assignRobotPoseXYZRPH(-10, 0, 0, 0, 0, 0);
            break;
        case 103:
            FakeRobot::assignRobotPoseXYZRPH(-10, 0, 0, 0, 0, 0);
            break;
        case 110:
            FakeRobot::assignRobotPoseXYZRPH(-23, -13, 0, 0, 0, 0);
            break;
        case 111:
            FakeRobot::assignRobotPoseXYZRPH(-23, -13, 0, 0, 0, 0);
            break;
        case 112:
            FakeRobot::assignRobotPoseXYZRPH(-23, 14, 0, 0, 0, 0);
            break;
        case 113:
            FakeRobot::assignRobotPoseXYZRPH(-23, 14, 0, 0, 0, 0);
            break;
        case 120:
            FakeRobot::assignRobotPoseXYZRPH(-14, 14, 0, 0, 0, 0);
            break;
        case 121:
            FakeRobot::assignRobotPoseXYZRPH(-14, 14, 0, 0, 0, 0);
            break;
        case 63:
            FakeRobot::assignRobotPoseXYZRPH(14, 14, 0, 0 , 0, -135);
            break;
        default:
            std::string error_msg = "ERROR: no robot starting pose "
                "supported for this map: " + std::to_string(map_number);
            debugger::debugExitColor(error_msg, __LINE__, __FILE__);

            // fake_robot_.position.x = 
            // fake_robot_.position.y = 
            // yaw_ = 
            // fake_robot_.position.z = 0;
            // roll_ = 0;
            // pitch_ = 0;
    }
    roll_ = deg_to_rad(roll_);
    pitch_ = deg_to_rad(pitch_);
    yaw_ = deg_to_rad(yaw_);
}


void FakeRobot::getPlannerMsgsCallback_(const planner_msgs::State& msg)
{
    new_command_lock_.lock();
    received_msg_ = msg;
    command_updated_ = true;
    new_command_lock_.unlock();
    // debugger::debugColorOutput("[FakeRobot]/[updatePlannerCommands] Received!", "", 
    //         5, BC);
}



void FakeRobot::spin_()
{
    while (nh_.ok()) {
        ros::spinOnce();
    }
}

void FakeRobot::publishToRviz_()
{
    ros::Rate rviz_rate(rviz_rate_);
    size_t i = 0;
    plotting::addMarkerWithPose(marker_, visualization_msgs::Marker::ARROW,
            "start",
            0, 1, 0, 1, // color
            start_, // pose
            0, 0 // count, time
            );
    marker_array_.markers.push_back(marker_);
    while(nh_.ok())
    {
        fake_robot_lock_.lock();
        plotting::addMarkerWithPose(marker_, visualization_msgs::Marker::ARROW,
                "fake_robot",
                1, 1, 1, 1, // color
                pose_t(fake_robot_.position.x,
                       fake_robot_.position.y,
                       yaw_), // pose
                i++, 0 // count, time
                ,"", 0.5, 0.4, 0.4);

        fake_robot_lock_.unlock();
        marker_array_.markers.push_back(marker_);


        marker_pub_.publish(marker_array_);
        rviz_rate.sleep();
    }
}

void FakeRobot::publishCurrentPose_()
{
    ros::Rate loop_rate(publishing_rate_);

    while (nh_.ok()) {

        fake_robot_lock_.lock();
        fake_robot_.header.stamp = ros::Time::now();

        // Add data to grid map.
        pose_pub_.publish(fake_robot_);
        fake_robot_lock_.unlock();



        ROS_INFO_STREAM_THROTTLE(1.0, "Fake robot published.");

        // Wait for next cycle.
        loop_rate.sleep();
    }

}

bool FakeRobot::getParameters_()
{
    std::string title_name("[FakeRobot]/[getParameters] ");
    bool received_all = true;
    ros_utils::checkROSParam(nh_, "fake_robot/receiving_rate", receiving_rate_,
            getNameOf(receiving_rate_), title_name, received_all);
    ros_utils::checkROSParam(nh_, "fake_robot/publishing_rate", publishing_rate_,
            getNameOf(receiving_rate_), title_name, received_all);
    ros_utils::checkROSParam(nh_, "fake_robot/rviz_rate", rviz_rate_,
            getNameOf(rviz_rate_), title_name, received_all);
    ros_utils::checkROSParam(nh_, "fake_robot/delta_t", delta_t_,
            getNameOf(delta_t_), title_name, received_all);

    ros_utils::checkROSParam(nh_, "fake_robot/robot_model", robot_model_,
            getNameOf(robot_model_), title_name, received_all);
    ros_utils::checkROSParam(nh_, "fake_robot/clf_mode", clf_mode_,
            getNameOf(clf_mode_), title_name, received_all);
    ros_utils::checkROSParam(nh_, "fake_robot/pose_mode", pose_mode_,
            getNameOf(pose_mode_), title_name, received_all);


    ros_utils::checkROSParam(nh_, "fake_robot/x", fake_robot_.position.x,
            getNameOf(fake_robot_.position.x), title_name, received_all);

    ros_utils::checkROSParam(nh_, "fake_robot/y", fake_robot_.position.y,
            getNameOf(fake_robot_.position.y), title_name, received_all);

    ros_utils::checkROSParam(nh_, "fake_robot/z", fake_robot_.position.z,
            getNameOf(fake_robot_.position.z), title_name, received_all);

    ros_utils::checkROSParam(nh_, "fake_robot/roll", roll_,
            getNameOf(roll_), title_name, received_all);

    ros_utils::checkROSParam(nh_, "fake_robot/pitch", pitch_,
            getNameOf(pitch_), title_name, received_all);

    ros_utils::checkROSParam(nh_, "fake_robot/yaw", yaw_,
            getNameOf(yaw_), title_name, received_all);


    ros_utils::checkROSParam(nh_, "fake_robot/vx", fake_robot_.velocity.x,
            getNameOf(fake_robot_.velocity.x), title_name, received_all);

    ros_utils::checkROSParam(nh_, "fake_robot/vy", fake_robot_.velocity.y,
            getNameOf(fake_robot_.velocity.y), title_name, received_all);

    ros_utils::checkROSParam(nh_, "fake_robot/vz", fake_robot_.velocity.z,
            getNameOf(fake_robot_.velocity.z), title_name, received_all);

    ros_utils::checkROSParam(nh_, "robot_params/heading_angle_preintegration",
            heading_angle_preintegration_,
            getNameOf(heading_angle_preintegration_),
            title_name, received_all);

    ros_utils::checkROSParam(nh_, "robot_params/step_time_interval",
            step_time_interval_,
            getNameOf(step_time_interval_),
            title_name, received_all);



    roll_ = deg_to_rad(roll_);
    pitch_ = deg_to_rad(pitch_);
    yaw_ = deg_to_rad(yaw_);
    behavior_ = 0;

    return received_all;
}

FakeRobot::~FakeRobot() { }
    
} /* bipedlab */ 
