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
#include "local_map.h"

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
    bool map_flag = true;
    if (!map_->exists("elevation_map")) 
    {
        debugger::debugColorTextOutput("[LocalMap]/[updateLocalMap] "
                "no elevation_map", 10, BR);
        map_flag = false;
    }
    if (!map_->exists("traversability_map"))
    {
        debugger::debugColorTextOutput("[LocalMap]/[updateLocalMap] "
                "no traversability_map", 10, BR);
        map_flag = false;
    }
    if (!map_flag)
    {
        debugger::debugColorTextOutput("[LocalMap]/[updateLocalMap] "
                "Exiting...", 10, BR);

        exit(-1);
    }


     grid_map::Position robot_center(current_pose.x, current_pose.y);
    // debugger::debugOutput("[LocalMap] current pose.x: ", current_pose.x, 3);
    // debugger::debugOutput("[LocalMap] current pose.y: ", current_pose.y, 3);
    // debugger::debugOutput("[LocalMap] length: ", local_map_info_.length, 3);
    debugger::debugTextOutput("[LocalMap] extracting local map", 4);

    if (this->local_map.exists("occupancy_map"))
        this->local_map.erase("occupancy_map");

    this->local_map = (map_->getSubmap(robot_center, 
                grid_map::Length(local_map_info_.length, local_map_info_.length), 
                this->is_successful));



    // update occupancy map 
    debugger::debugTextOutput("[LocalMap] creating occupancy_map", 4);
    this->local_map.add("occupancy_map", 0);
    double height = 0;
    for (grid_map::GridMapIterator it(this->local_map); !it.isPastEnd(); ++it) {
        grid_map::Position position;
        this->local_map.getPosition(*it, position);

        if (local_map_info_.mode == 0)
        {
            // too high, assign occupied
            height = this->local_map.at("elevation_map", *it);
            if (height >= local_map_info_.obstacle_threshold)
                this->local_map.at("occupancy_map", *it) = 1;
            else
                this->local_map.at("occupancy_map", *it) = 0;

            // not traversable, assign occupied
            if (!this->local_map.at("traversability_map", *it))
                this->local_map.at("occupancy_map", *it) = 1;
        }

    }


    if (!(this->is_successful))
    {
        debugger::debugColorTextOutput("[LocalMap]/[updateLocalMap] "
                "getSubmap() failed!", 10, BR);
        exit(-1);
    }


    // debugger::debugColorTextOutput("======= simple test ==========",3);
    // auto dummy = map_.getSubmap(1);
    // std::cout << "===dummy before===\n";
    // dummy.printInfo();
    // dummy.flag = 2;
    // std::cout << "===dummy after===\n";
    // dummy.printInfo();


    // debugger::debugColorTextOutput("======= complex test ==========",3);
    // this->is_successful = false;
    // grid_map::Position robot_center(current_pose.x, current_pose.y);
    // std::cout << "robot_center: " << robot_center << std::endl;
    // std::cout << "length_: " << length_ << std::endl;
    // if (map_.isInside(robot_center))
    //     std::cout << "robot_center is inside\n";
    // else
    //     std::cout << "robot center is not inside\n";
    // grid_map::GridMap dummy2(map_.getSubmap(robot_center, grid_map::Length(length_, length_), this->is_successful, 10));
    // dummy2.flag = 11;

    /*
    grid_map::GridMap* dummyPtr = new grid_map::GridMap(dummy2); //map_.getSubmap(robot_center, grid_map::Length(length_, length_), this->is_successful, 10));
    std::cout << "===dummy before===\n";
    std::cout << "is successful: " << this->is_successful << std::endl;
    dummyPtr->printInfo();
    dummyPtr->flag = 11;
    std::cout << "===dummy after===\n";
    dummyPtr->printInfo();
    

    debugger::debugColorTextOutput("======= assigning to ptr ==========",3);
    this->local_map_ptr = new grid_map::GridMap(*dummyPtr);
    std::cout << "=== local_map_ptr info ===\n";
    this->local_map_ptr->printInfo();



    // if (!(this->is_successful))
    // {
    //     debugger::debugColorTextOutput("[LocalMap]/[updateLocalMap] "
    //             "getSubmap() failed!", 10, BR);
    //     exit(-1);
    // }
    debugger::debugColorTextOutput("about to delete",3);
    if (dummyPtr == nullptr)
        std::cout << "dummyPtr is NULL\n";
    else
        std::cout << "dummyPtr is not Null\n";
    // delete dummyPtr;
    */
        

    // grid_map::Position robot_center(current_pose.x, current_pose.y);
    // debugger::debugColorTextOutput("hey1",3);
    // // auto dummy = map_.getSubmap(robot_center, grid_map::Length(length_, length_), this->is_successful);
    // grid_map::GridMap* dummyPtr = new grid_map::GridMap(map_.getSubmap(robot_center, grid_map::Length(length_, length_), this->is_successful));
    // std::cout << "===dummy===\n";
    // std::cout << "is successful: " << this->is_successful << std::endl;
    // dummyPtr->printInfo();
    // 
    // debugger::debugTextOutput("hey2",3);
    // this->local_map_ptr = new grid_map::GridMap(*dummyPtr);
    // std::cout << "=== local_map_ptr info ===\n";
    // this->local_map_ptr->printInfo();

    // debugger::debugColorTextOutput("hey3",3);


    // // if (!(this->is_successful))
    // // {
    // //     debugger::debugColorTextOutput("[LocalMap]/[updateLocalMap] "
    // //             "getSubmap() failed!", 10, BR);
    // //     exit(-1);
    // // }
    // debugger::debugColorTextOutput("hey4",3);
    // delete dummyPtr;





    //exit(-1);
    // debugger::debugTextOutput("hey1",3);
    // grid_map::Position robot_center(current_pose.x, current_pose.y);


    // debugger::debugTextOutput("hey2",3);
    // this->local_map;

    // 
    // grid_map::GridMap dummy;
    // grid_map::GridMap dummy2;
    // bool test1;
    // debugger::debugTextOutput("hey3",3);
    // dummy = (map_.getSubmap(robot_center, 
    //                              grid_map::Length(length_, length_), 
    //                              test1));
    // this->local_map = dummy;

    // debugger::debugOutput("is successful: ", test1, 3);


    // bool test2;
    // debugger::debugTextOutput("hey4",3);
    // dummy2 = (map_.getSubmap(robot_center, 
    //                              grid_map::Length(length_, length_), 
    //                              test2));
    // this->local_map = dummy2;
    // // auto test3 = (map_.getSubmap(robot_center, 
    // //                              grid_map::Length(length_, length_), 
    // //                              test2));
    // debugger::debugOutput("is successful: ", test2, 3);
    // debugger::debugTextOutput("hey5",3);
    // exit(-1);
    // 


    // // this->local_map = (map_.getSubmap(robot_center, 
    // //                              grid_map::Length(length_, length_), 
    // //                              this->is_successful));

    // debugger::debugTextOutput("hey6",3);


    // this->local_map.add("occupancy_map", 1); // all occupied

    // grid_map::Matrix& data = this->local_map["elevation"];
    // for (grid_map::GridMapIterator iterator(this->local_map); 
    //      !iterator.isPastEnd(); ++iterator) {
    //     const int i = iterator.getLinearIndex();
    //     if data(i) > ;
    // }
    // for (grid_map::CircleIterator iterator(world_map, robot_center, search_radious_);
    //         !iterator.isPastEnd(); ++iterator) {
    //     local_map.at("local_map", *it) = 
    // }
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
bool LocalMap::isNeighborObstacleFree(
        const double& x, const double& y, const double& radius)
{
    // search center
    grid_map::Position center(x, y);
    grid_map::Index index;
    bool in_map = this->local_map.getIndex(center, index);

    if (in_map)
    {
        // debugger::debugOutput("[local map] center: ", center, 0);
        if (this->local_map.at("occupancy_map", index) == 1)
            return false;

        for (grid_map::CircleIterator iterator(this->local_map, center, radius);
                !iterator.isPastEnd(); ++iterator) {
            // debugger::debugOutput("[local map] nearby point: ", *iterator, 0);
            if (this->local_map.at("occupancy_map", *iterator) == 1)
                return false;
        }
        return true;
    }
    return false;
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
