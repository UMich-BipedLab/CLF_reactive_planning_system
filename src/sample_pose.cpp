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
#include "sample_pose.h"
#include "angle_functions.h"
#include "utils/utils.h"
#include "utils/debugger.h"
#include "utils/line.h"



namespace bipedlab
{
SamplePose::SamplePose(
        const pose_sampler_params_t& params, 
        const LocalMap* local_map,
        const pose_t* goal_pose,
        const robot_state_t* robot_state) :
    params_(params), local_map_(local_map), 
    goal_pose_(goal_pose), robot_state_(robot_state){ }

SamplePose::~SamplePose() { }

pose_t SamplePose::sampleRandomPoseWithGoalBiasedWithinLocalMap(bool& is_goal)
{
    double rand_num_for_goal_bias = utils::genInclusiveRandomNumber(0, 1);

    pose_t sampled_pose;
    is_goal = false;
    if (rand_num_for_goal_bias < params_.goal_bias) 
    {
        // If the goal is inside the local map,
        // return the goal; otherwise return the intersection goal and the 
        // boundary of the grid map 
        // If the boundary point is within an obstacle, random sample a pose instead
        grid_map::Index index;
        if (local_map_->local_map.getIndex(goal_pose_->getPosition2d(), index))
        {
            debugger::debugOutput("[sampling] sampled pose is the goal: ", 
                               *goal_pose_, 3);
            is_goal = true;
            return *(goal_pose_);
        } 
        else
        {
            SamplePose::sampleGoalOnBoundary_(sampled_pose);
            
            bool is_occupied = 
                SamplePose::checkOccupiedWithRadius(sampled_pose, 
                        params_.distance_threshold);
            // bool occupied = local_map_->local_map.atPosition(
            //         "occupancy_map", sampled_pose.get2DPosition());
            if (!is_occupied) 
            {
                debugger::debugOutput("[sampling] sampled pose is at the boundary: ",
                               sampled_pose, 0);
                return sampled_pose;
            }
            else
            {
                sampled_pose = SamplePose::sampleRandomPoseWithinLocalMap_();
                debugger::debugOutput(
                        "[sampling] the bounday pose is within an obstacle, "
                        "random sampling instead: ", 
                        sampled_pose, 0);
                return sampled_pose;
            }
        }
    }
    else
    {
        // 0: normal (random sample if not sample the goal)
        // 1: If sample the goal, select the goal
        //    If not, use front_bias to sample in front (front_angle) of the current robot pose
        //    (use robot pose as front direction)
        // 2: If sample the goal, select the goal
        //    If not, use front_bias to sample in front (front_angle) of the current
        //    robot position and the goal position
        //    (use the conneting line of robot position and goal position to decide front
        //    direction)
        // note: the different between #1 and #2 is the way of deciding front direction
        if (params_.sampling_mode == 0)
        {
            sampled_pose = SamplePose::sampleRandomPoseWithinLocalMap_();
        }
        else if (params_.sampling_mode == 1)
        {
            sampled_pose = 
                SamplePose::sampleRandomPoseWithFrontBiasWithinLocalMap_(
                        robot_state_->pose);
        }
        else if (params_.sampling_mode == 2)
        {
            // use goal position and robot position to decide the front direction
            double x = robot_state_->pose.x;
            double y = robot_state_->pose.y;
            double angle = angle_between_two_points_and_x_axis_2pi<double>(
                    x, y, goal_pose_->x, goal_pose_->y);
            pose_t pose_to_decide_font_angle = pose_t(x, y, angle);
            sampled_pose = SamplePose::sampleRandomPoseWithFrontBiasWithinLocalMap_(
                    pose_to_decide_font_angle);
        }
        else
        {
            std::string error_msg = "ERROR: No such sampling_mode supported: " +
                std::to_string(params_.sampling_mode);
            debugger::debugExitColor(error_msg, __LINE__, __FILE__);
        }


        debugger::debugOutput("[sampling] sampled pose is within the local map: ", 
                           sampled_pose, 0);
        return sampled_pose; 
    }
}

pose_t SamplePose::sampleRandomPoseWithFrontBiasWithinLocalMap_(
        const pose_t& pose_to_decide_font_angle)
{
    int is_occupied = 1;
    bool is_sameple_front = false;

    double rand_num_for_front_bias = utils::genInclusiveRandomNumber(0, 1);

    if (rand_num_for_front_bias <= params_.front_bias) 
    {
        // is_sameple_front or not does not matter 
        is_sameple_front = true;
    }


    pose_t sampled_pose;

    // size of the map
    // double map_length = std::min(local_map_->local_map.getLength().x(),
    //                             local_map_->local_map.getLength().y());
    // float max_radius = map_length / 2;
    double dis_robot_to_goal = 
        distance_between_points(robot_state_->pose.to2DPosition(), 
                goal_pose_->to2DPosition());

    int num_per_draw = 1;
    float max_radius = dis_robot_to_goal;
    float directional_theta = pose_to_decide_font_angle.theta;
    float front_angle = deg_to_rad(params_.front_angle);
    // float theta_max = wrap_to_2pi(directional_theta + front_angle);
    // float theta_min = wrap_to_2pi(directional_theta - front_angle);
    debugger::debugColorOutput("[sampling] goal_pose: ", 
            (*goal_pose_), 5, B);
    debugger::debugColorOutput("[sampling] start pose: ", 
            robot_state_->pose, 5, B);
    debugger::debugColorOutput("[sampling] max_radius: ", 
            max_radius, 5, B);
    debugger::debugColorOutput("[sampling] directional_theta: ", 
            rad_to_deg(directional_theta), 5, B);
    // // debugger::debugOutput("[sampling] theta_max: ", rad_to_deg(theta_max), 4);
    // debugger::debugOutput("[sampling] theta_min: ", rad_to_deg(theta_min), 4);

    bool found_valid_sample = false;
    while (!found_valid_sample)
    {
        // use polar coord. to decide x, y
        std::vector<float> rand_radius = 
            utils::genListOfInclusiveRandomNumbers<float>(num_per_draw, 
                    0, max_radius);
        std::vector<float> rand_theta;
        if (is_sameple_front)
        {
            rand_theta = 
                utils::genListOfInclusiveRandomNumbers<float>(num_per_draw, 
                    -front_angle, front_angle);
        }
        else
        {
            rand_theta = 
                utils::genListOfInclusiveRandomNumbers<float>(num_per_draw, 
                        front_angle, 2 * M_PI - front_angle);
        }
        std::vector<float> rand_yaw = utils::genListOfInclusiveRandomNumbers<float>(
            num_per_draw, 0.0, 2 * M_PI);

        for (int i = 0; i < num_per_draw; ++i)
        {
            float sample_theta = wrap_to_2pi(rand_theta[i] + directional_theta); 
            float rand_x = rand_radius[i] * std::cos(sample_theta) + 
                pose_to_decide_font_angle.x;
            float rand_y = rand_radius[i] * std::sin(sample_theta) + 
                pose_to_decide_font_angle.y;

            sampled_pose = pose_t(rand_x, rand_y, rand_yaw[i]);
            // is_occupied = local_map_->local_map.atPosition(
            //         "occupancy_map", sampled_pose.get2DPosition());
            is_occupied = SamplePose::checkOccupiedWithRadius(sampled_pose, 
                        params_.distance_threshold);
            // debugger::debugOutput("[sampling] r_max: ", max_radius, 4);
            // debugger::debugOutput("[sampling] theta_bound: ", directional_theta, 4);
            // debugger::debugOutput("[sampling] r: ", rand_radius[0], 4);
            // debugger::debugOutput("[sampling] theta: ", rand_theta[0], 4);
            debugger::debugOutput("[sampling] pose: ", sampled_pose, 5);
            debugger::debugOutput("[sampling] is_occupied: ", is_occupied, 5);
            if (!is_occupied)
            {
                // debugger::debugOutput("====== \n", "", 5);
                found_valid_sample = true;
                break;
            }
        }
    }

    return sampled_pose; 
}

pose_t SamplePose::sampleRandomPoseWithinLocalMap_(void)
{
    int is_occupied = 1;
    pose_t sampled_pose;

    // size of the map
    grid_map::Size grid_size = local_map_->local_map.getSize();

    // the smallest coordinate of the map is at the bottom right of the map, 
    // which is the largest index (i, j)
    // -1 is due to 0-based index in C++
    grid_map::Position bottom_right;
    local_map_->local_map.getPosition(
            Eigen::Array2i(grid_size(0) - 1, grid_size(1) - 1),
            bottom_right);
    debugger::debugOutput("[sampling] bottom right: \n", bottom_right, 0);

    // the top left is the largest coordinate of the map
    grid_map::Position top_left;
    local_map_->local_map.getPosition(
            Eigen::Array2i(0, 0),
            top_left);
    debugger::debugOutput("[sampling] top left: \n", top_left, 0);

    while (is_occupied) 
    {
        float rand_x = utils::genInclusiveRandomNumber(bottom_right(0), top_left(0));
        float rand_y = utils::genInclusiveRandomNumber(bottom_right(1), top_left(1));
        float rand_theta = utils::genRandomNumber(-M_PI, M_PI);   // in [-pi, pi)

        sampled_pose = pose_t(rand_x, rand_y, rand_theta);
        // debugger::debugOutput("[sampling] pose: \n", sampled_pose, 3);
        // is_occupied = local_map_->local_map.atPosition(
        //         "occupancy_map", sampled_pose.get2DPosition());
        is_occupied = 
            SamplePose::checkOccupiedWithRadius(sampled_pose, 
                                                params_.distance_threshold);
    }

    return sampled_pose; 
}

// return true if occupied
bool SamplePose::checkOccupiedWithRadius(const pose_t& pose, const double radius) const
{
    bool is_free = local_map_->isNeighborObstacleFree(pose.x, pose.y, radius);

    return !is_free;
}

void SamplePose::sampleGoalOnBoundary_(pose_t& sampled_pose) 
{
    // pose_t sampled_goal;
    const Eigen::Matrix<double, 2, 4> four_corners = local_map_->getFourCorners();

    // p1 is the intersection between the line of top left and top right,
    // and the line of the robot center and the goal
    std::pair<double, double> p1 = 
        line::computeIntersectionOfTwoLineSegments<double> (
                // top left
                std::make_pair(four_corners(0, 0), four_corners(1, 0)), 
                // top right
                std::make_pair(four_corners(0, 1), four_corners(1, 1)), 
                // robot
                std::make_pair(robot_state_->pose.x, 
                    robot_state_->pose.y), 
                // goal
                std::make_pair(goal_pose_->x, goal_pose_->y));

    // p2 is the intersection between 
    // the line of top right and bottom right, and 
    // the line of the robot center and the goal
    std::pair<double, double> p2 = 
        line::computeIntersectionOfTwoLineSegments<double> (
                // top left
                std::make_pair(four_corners(0, 1), four_corners(1, 1)), 
                // top right
                std::make_pair(four_corners(0, 2), four_corners(1, 2)), 
                // robot
                std::make_pair(robot_state_->pose.x, 
                    robot_state_->pose.y), 
                // goal
                std::make_pair(goal_pose_->x, goal_pose_->y));

    // p3 is the intersection between 
    // the line of bottom right and bottom left, and 
    // the line of the robot center and the goal
    std::pair<double, double> p3 = 
        line::computeIntersectionOfTwoLineSegments<double> (
                // top left
                std::make_pair(four_corners(0, 2), four_corners(1, 2)), 
                // top right
                std::make_pair(four_corners(0, 3), four_corners(1, 3)), 
                // robot
                std::make_pair(robot_state_->pose.x, 
                    robot_state_->pose.y), 
                // goal
                std::make_pair(goal_pose_->x, goal_pose_->y));

    // p4 is the intersection between 
    // the line of bottom left and top left, and 
    // the line of the robot center and the goal
    std::pair<double, double> p4 = 
        line::computeIntersectionOfTwoLineSegments<double> (
                // top left
                std::make_pair(four_corners(0, 3), four_corners(1, 3)), 
                // top right
                std::make_pair(four_corners(0, 0), four_corners(1, 0)), 
                // robot
                std::make_pair(robot_state_->pose.x, 
                    robot_state_->pose.y), 
                // goal
                std::make_pair(goal_pose_->x, goal_pose_->y));

    pose2d_t pose1(p1, angle_to_point(
                robot_state_->pose.to2DPosition(),
                point2d_t(p1)));

    pose2d_t pose2(p2, angle_to_point(
                robot_state_->pose.to2DPosition(),
                point2d_t(p2)));

    pose2d_t pose3(p3, angle_to_point(
                robot_state_->pose.to2DPosition(),
                point2d_t(p3)));

    pose2d_t pose4(p4, angle_to_point(
                robot_state_->pose.to2DPosition(),
                point2d_t(p4)));

    float distance = FLT_MAX;
    float dis1 = squared_point_distance(
            pose1.toPoint(), robot_state_->pose.to2DPosition());
    if (dis1 < distance) 
    {
        distance = dis1;
        sampled_pose = pose1;
    }

    float dis2 = squared_point_distance(
            pose2.toPoint(), robot_state_->pose.to2DPosition());
    if (dis2 < distance) 
    {
        distance = dis2;
        sampled_pose = pose2;
    }

    float dis3 = squared_point_distance(
            pose3.toPoint(), robot_state_->pose.to2DPosition());
    if (dis3 < distance) 
    {
        distance = dis3;
        sampled_pose = pose3;
    }

    float dis4 = squared_point_distance(
            pose4.toPoint(), robot_state_->pose.to2DPosition());
    if (dis4 < distance) 
    {
        distance = dis4;
        sampled_pose = pose4;
    }

    // point2d_t point1(p1);

    // point2d_t point2(p2);
    // point2d_t point3(p3);
    // point2d_t point4(p4);




    // // l1 : top left and top right
    // line::line_t l1 = line::compute2DLineGivenTwoPoints(
    //         four_corners(0, 0), four_corners(1, 0),
    //         four_corners(0, 1), four_corners(1, 1));

    // // l2 : top right and bottom right
    // line::line_t l2 = line::compute2DLineGivenTwoPoints(
    //         four_corners(0, 1), four_corners(1, 1),
    //         four_corners(0, 2), four_corners(1, 2));

    // // l3 : bottom right and bottom left
    // line::line_t l3 = line::compute2DLineGivenTwoPoints(
    //         four_corners(0, 2), four_corners(1, 2),
    //         four_corners(0, 3), four_corners(1, 3));

    // // l3 : bottom left and top left
    // line::line_t l4 = line::compute2DLineGivenTwoPoints(
    //         four_corners(0, 3), four_corners(1, 3),
    //         four_corners(0, 0), four_corners(1, 0));
}

} /* bipedlab */ 
