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
#include <cmath>
#include "local_map.h"
#include "map_operation.h"


namespace bipedlab
{
LocalMap::LocalMap(const pose_t* current_pose, 
                   const grid_map::GridMap* map, 
                   const local_map_params_t& local_map_info):
    current_pose_(current_pose), 
    map_(map), 
    local_map_info_(local_map_info),
    is_successful(false)
    // local_map_ptr(nullptr)
    {
        // std::cout << "[In LocalMap()] map == " << map << std::endl;
        // std::cout << "[In LocalMap()] map_ == " << map_ << std::endl;
        // debugger::debugColorTextOutput("In LocalMap constructor",3);
        // local_map = new grid_map::GridMap({"local_map"});
        // local_map->setFrameId(world_map_->getFrameId());
        // local_map->setGeometry(Length(2 * search_radious, 2 * search_radious), 
        //                        world_map->getResolution(), 
        //                        grid_map::Position(current_pose.x, current_pose.y));
        // grid_map::GridMap dummy(
        //         map.getSubmap(grid_map::Position(current_pose.x, current_pose.y), grid_map::Length(length, length), is_successful, 10));
        // dummy.flag = 11;
    // bool map_flag = true;
    // if (!map.exists("elevation_map")) 
    // {
    //     debugger::debugColorTextOutput("[LocalMap]/[updateLocalMap] "
    //             "no elevation_map", 10, BR);
    //     map_flag = false;
    // }
    // if (!map.exists("occupancy_map"))
    // {
    //     debugger::debugColorTextOutput("[LocalMap]/[updateLocalMap] "
    //             "no occupancy_map", 10, BR);
    //     map_flag = false;
    // }
    // if (!map_flag)
    // {
    //     debugger::debugColorTextOutput("[LocalMap]/[updateLocalMap] "
    //             "Exiting...", 10, BR);

    //     exit(-1);
    // }
        // debugger::debugColorTextOutput("Excecuting updateLocalMap()",3);
        updateLocalMap(*current_pose);
        // std::cout << "LocalMap::LocalMap() updateLocalMap() done\n";
        // debugger::debugTextOutput("hey5",3);
        // exit(-1);

        // map.add("local_map", -1);
    }

LocalMap::~LocalMap() {
    std::cout << "~LocalMap()\n";
};

void LocalMap::updateLocalMap(const pose_t& current_pose)
{
    if (!map_->exists("elevation_map") && !map_->exists("elevation")) 
    {
        debugger::debugColorTextOutput("[LocalMap]/[updateLocalMap] "
                "no elevation_map, about to exit...", 10, BR);
        exit(-1);
    }


    grid_map::Position robot_center(current_pose.x, current_pose.y);
    // debugger::debugOutput("[LocalMap] current pose.x: ", current_pose.x, 5);
    // debugger::debugOutput("[LocalMap] current pose.y: ", current_pose.y, 5);
    // debugger::debugOutput("[LocalMap] length: ", local_map_info_.length, 5);
    debugger::debugTextOutput("[LocalMap] extracting local map", 4);

    if (this->local_map.exists("occupancy_map"))
        this->local_map.erase("occupancy_map");

    if (this->local_map.exists("filtered_elevation_map"))
        this->local_map.erase("filtered_elevation_map");

    if (this->local_map.exists("filtered_slope"))
        this->local_map.erase("filtered_slope");

    if (this->local_map.exists("elevation"))
    {
        if (this->local_map.exists("elevation_map"))
            this->local_map.erase("elevation_map");
    }


    this->local_map = (map_->getSubmap(robot_center, 
                grid_map::Length(local_map_info_.length, local_map_info_.length), 
                this->is_successful));


    if (!(this->is_successful))
    {
        debugger::debugColorTextOutput("[LocalMap]/[updateLocalMap] "
                "getSubmap() failed!", 10, BR);
        exit(-1);
    }

    // if there is no elevaion_map layer...
    if (this->local_map.exists("elevation"))
    {
        if (!this->local_map.exists("elevation_map"))
        {
            if (!this->local_map.exists("elevation"))
            {
                debugger::debugColorTextOutput("[LocalMap]/[updateLocalMap] "
                        "No elevation map! Exiting...", 10, BR);
            }
            else
            {
                this->local_map.add("elevation_map", 
                        this->local_map.get("elevation"));
                // this->local_map.clearBasic();
            }
        }
    }



    // remove nan
    if (local_map_info_.mode == 1)
    {
        map_operation::removeNAN(this->local_map, 
                local_map_info_.cost_params.unknown_cost,
                "elevation_map");
        this->local_map.setBasicLayers({"elevation_map"});
    }


    // smooth (assign unknow = unknow_cost)
    if (local_map_info_.mode == 2)
    {
        this->local_map.add("filtered_elevation_map", 0);
        map_operation::averageNANFiltering(this->local_map, 
                this->local_map.getSize(),
                this->local_map_info_.smooth_radius,
                "elevation_map", "", 
                local_map_info_.nan_percentage_in_radius, 
                local_map_info_.cost_params.unknown_cost);
        size_t num_NAN = map_operation::checkNAN(this->local_map, 
                "filtered_elevation_map");
        debugger::debugColorOutput("[LocalMap] num_NAN: ", num_NAN, 5, W, BOLD);
        this->local_map.setBasicLayers({"filtered_elevation_map"});
    }

    // smooth and slope (assign unknow = unknow_cost + robot.z)
    if (local_map_info_.mode == 3)
    {

        this->local_map.add("filtered_elevation_map", 0);
        map_operation::averageNANFiltering(this->local_map, 
                this->local_map.getSize(),
                this->local_map_info_.smooth_radius,
                "elevation_map", "", 
                local_map_info_.nan_percentage_in_radius, 
                current_pose.z + local_map_info_.cost_params.unknown_cost);
        // size_t num_NAN = map_operation::checkNAN(this->local_map, 
        //         "filtered_elevation_map");
        // debugger::debugColorOutput("[LocalMap] num_NAN: ", num_NAN, 5, W, BOLD);


        this->local_map.setBasicLayers({"filtered_elevation_map"});
        this->local_map.add("filtered_slope", 0);


        map_operation::computeSlopeWRTRobotPose(this->local_map, 
                this->local_map.getSize(),
                "filtered_elevation_map", "filtered_slope",
                current_pose);
        this->local_map.setBasicLayers({"filtered_elevation_map"});
    }

    // smooth and use signed distnace to the cloest obstacles 
    // This do the smoothing first, computing SDF after computing obstalces
    if (local_map_info_.mode == 4)
    {
        this->local_map.add("filtered_elevation_map", 0);
        map_operation::averageNANFiltering(this->local_map, 
                this->local_map.getSize(),
                this->local_map_info_.smooth_radius,
                "elevation_map", "", 
                local_map_info_.nan_percentage_in_radius, 
                local_map_info_.cost_params.unknown_cost);
        size_t num_NAN = map_operation::checkNAN(this->local_map, 
                "filtered_elevation_map");
        debugger::debugColorOutput("[LocalMap] num_NAN: ", num_NAN, 5, W, BOLD);
        this->local_map.setBasicLayers({"filtered_elevation_map"});
    }


    // smooth and assign unknown behind obstalces
    // This do the smoothing first and then the assignment will be done after
    // computing obstalces 
    if (local_map_info_.mode == 5)
    {
        this->local_map.add("filtered_elevation_map", 0);
        map_operation::averageNANFiltering(this->local_map, 
                this->local_map.getSize(),
                this->local_map_info_.smooth_radius,
                "elevation_map", "", 
                local_map_info_.nan_percentage_in_radius, 
                local_map_info_.cost_params.unknown_cost);
        size_t num_NAN = map_operation::checkNAN(this->local_map, 
                "filtered_elevation_map");
        debugger::debugColorOutput("[LocalMap] num_NAN: ", num_NAN, 5, W, BOLD);
        this->local_map.setBasicLayers({"filtered_elevation_map"});
    }


    // update occupancy map 
    std::string elevaion_map_name = "elevation_map";
    if (local_map_info_.mode == 2 || local_map_info_.mode == 3)
    {
        elevaion_map_name = "filtered_elevation_map";
    }
    debugger::debugTextOutput("[LocalMap] creating occupancy_map", 4);
    this->local_map.add("occupancy_map", 0);
    bool has_nan = false;
    map_operation::assignObstacles(this->local_map, 
            this->local_map.getSize(),
            elevaion_map_name, "occupancy_map",
            local_map_info_.obstacle_threshold, has_nan);

    if (has_nan && local_map_info_.mode == 0)
    {
        debugger::debugColorTextOutput("[LocalMap] "
                "has nan value in the local map!", 10, BR, BOLD);
        size_t num_NAN = map_operation::checkNAN(this->local_map, 
                "elevation_map");
        debugger::debugColorOutput("[LocalMap] num_NAN: ", num_NAN, 19, W, BOLD);
        debugger::debugColorTextOutput("[LocalMap] "
                "Use mode 1 to assign nan to other values, exiting...", 
                10, BR, BOLD);
        exit(-1);
    }

    if (local_map_info_.mode == 4)
    {
        map_operation::computeSignedDistnaceField(this->local_map, 
                this->local_map.getSize(),
                this->local_map_info_.SDF_radius,
                "occupancy_map", "filtered_elevation_map");
    }

    if (local_map_info_.mode == 5)
    {
        grid_map::Index index;
        this->local_map.getIndex(robot_center, index);
        map_operation::assignUnknownBehindObstacles(this->local_map, 
                index, local_map_info_.cost_params.unknown_cost,
                "occupancy_map", "filtered_elevation_map");
    }


}

// the order of vertices are (0, 0), (0, 1), (1, 1), (1, 0) in the grid map
Eigen::Matrix<double, 2, 4>
LocalMap::getFourCorners(void) const 
{
    Eigen::Matrix<double, 2, 4> vertices;
    grid_map::Size grid_size = local_map.getSize();

    grid_map::Position top_left; // (0, 0)
    local_map.getPosition(
            Eigen::Array2i(0, 0),
            top_left);

    grid_map::Position top_right; // (0, 1)
    local_map.getPosition(
            Eigen::Array2i(0, grid_size(1) - 1),
            top_right);

    grid_map::Position bottom_right; // (1, 1)
    local_map.getPosition(
            Eigen::Array2i(grid_size(0) - 1, grid_size(1) - 1),
            bottom_right);

    grid_map::Position bottom_left; // (1, 0)
    local_map.getPosition(
            Eigen::Array2i(grid_size(0) - 1, 0),
            bottom_left);

    vertices.col(0) = top_left;
    vertices.col(1) = top_right;
    vertices.col(2) = bottom_right;
    vertices.col(3) = bottom_left;

    return vertices;
}


// return false if the point is outside of the map
// return true if obstacle free
bool LocalMap::isNeighborObstacleFree(
        const double& x, const double& y, const double& radius) const
{
    // search center
    grid_map::Position center(x, y);
    // debugger::debugOutput("[local map] center: ", center, 0);
    
    grid_map::Index index;
    bool in_map = this->local_map.getIndex(center, index);

    if (!in_map)
    {
        return false;
    }
    else
    {
        bool is_free = true;
        const grid_map::Matrix& data = this->local_map["occupancy_map"];

        // check if the query point itself is occupied
        if (data(index(0), index(1)) == 1)
            return false;

        // check around the query point given the radius
        for (grid_map::CircleIterator iterator(this->local_map, center, radius);
                !iterator.isPastEnd(); ++iterator) 
        {
            // debugger::debugOutput("[local map] nearby point: ", *iterator, 0);
            const grid_map::Index t_index(*iterator);
            if ( data(t_index(0), t_index(1)) == 1)
            {
                is_free = false;
                return is_free;
            }

        }
        return is_free;
    }
}


// double LocalMap::getCost(const pose_t& pose) 
// {
//     grid_map::Position center(pose.x, pose.y);
//     grid_map::Index index;
//     bool in_map = this->local_map.getIndex(center, index);
// 
//     if (in_map)
//     {
//         // debugger::debugOutput("[local map] center: ", center, 0);
//         if (this->local_map.at("occupancy_map", index) == 1)
//             return false;
// 
//         for (grid_map::CircleIterator iterator(this->local_map, center, radius);
//                 !iterator.isPastEnd(); ++iterator) {
//             // debugger::debugOutput("[local map] nearby point: ", *iterator, 0);
//             if (this->local_map.at("occupancy_map", *iterator) == 1)
//                 return false;
//         }
//         return true;
//     }
//     return false;
// 
// }


} /* bipedlab */ 
