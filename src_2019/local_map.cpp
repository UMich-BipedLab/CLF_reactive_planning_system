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
LocalMap::LocalMap(const pose_t& current_pose, 
                   const grid_map::GridMap& map, 
                   const double length) :
    current_pose_(current_pose), 
    map_(map), 
    length_(length)
    { 
        // local_map = new grid_map::GridMap({"local_map"});
        // local_map->setFrameId(world_map_->getFrameId());
        // local_map->setGeometry(Length(2 * search_radious, 2 * search_radious), 
        //                        world_map->getResolution(), 
        //                        grid_map::Position(current_pose.x, current_pose.y));
        updateLocalMap(current_pose);
        // map.add("local_map", -1);
    }

LocalMap::~LocalMap() {};

void LocalMap::updateLocalMap(pose_t current_pose)
{
    assert(map_.exists("elevation_map") && map_.exists("occupancy_map"));

    grid_map::Position robot_center(current_pose.x, current_pose.y);
    
    this->local_map = (map_.getSubmap(robot_center, 
                                 grid_map::Length(length_, length_), 
                                 this->is_successful));
    assert(this->is_successful);
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
LocalMap::getFourCorners(void) 
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


bool LocalMap::isNeighborObstacleFree(
        const double& x, const double& y, const double& radius)
{
    // search center
    grid_map::Position center(x, y);

    for (grid_map::CircleIterator iterator(this->local_map, center, radius);
         !iterator.isPastEnd(); ++iterator) {
        if (this->local_map.at("occupancy_map", *iterator) == 1)
            return false;
    }
    return true;

}
} /* bipedlab */ 
