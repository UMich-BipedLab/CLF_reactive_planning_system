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
    double rand_num = utils::genInclusiveRandomNumber(0, 1);
    pose_t sampled_pose;
    is_goal = false;
    if (rand_num < params_.goal_bias) 
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
            SamplePose::sampleGoalOnBoundary(sampled_pose);
            
            int occupied = local_map_->local_map.atPosition(
                    "occupancy_map", sampled_pose.get2DPosition());
            if (!occupied) 
            {
                debugger::debugOutput("[sampling] sampled pose is at the boundary: ",
                               sampled_pose, 0);
                return sampled_pose;
            }
            else
            {
                sampled_pose = SamplePose::sampleRandomPoseWithinLocalMap();
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
        sampled_pose = SamplePose::sampleRandomPoseWithinLocalMap();
        debugger::debugOutput("[sampling] sampled pose is within the local map: ", 
                           sampled_pose, 0);
        return sampled_pose; 
    }
}

pose_t SamplePose::sampleRandomPoseWithinLocalMap(void)
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
        is_occupied = local_map_->local_map.atPosition(
                "occupancy_map", sampled_pose.get2DPosition());
    }

    return sampled_pose; 

}


void SamplePose::sampleGoalOnBoundary(pose_t& sampled_pose) 
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
