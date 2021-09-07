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
#include "map_cost.h"
#include "utils/debugger.h"

namespace bipedlab
{
MapCost::MapCost(cost_params_t& cost_params, const LocalMap* local_map, 
        const size_t& mode) : 
    cost_params_(cost_params), mode_(mode)
{
    local_map_ = local_map;
}


MapCost::MapCost(const LocalMap* local_map, const size_t& mode) : mode_(mode)
{
    local_map_ = local_map;
}
MapCost::~MapCost(void) { };


cost_t MapCost::computePositionMapCost(const pose_t& pose) const 
{
    grid_map::Position center(pose.x, pose.y);
    grid_map::Index index;
    bool in_map = this->local_map_->local_map.getIndex(center, index);

    // debugger::debugOutput("[MapCost]/[computePositionMapCost] weight: ", 
    //         cost_params_.weight_of_height, 4);
    if (in_map)
    {
        // debugger::debugOutput("[MapCost] center: ", center.transpose(), 3);
        if (this->local_map_->local_map.at("occupancy_map", index) == 1)
            return cost_t(0, cost_params_.obstacle_cost);
        else
        {
            if (mode_ == 0) // assume flat ground
                return cost_t(0, 0);
            else if (mode_ == 1) // use height map
            {
                if (this->local_map_->local_map_info_.mode == 0 || 
                    this->local_map_->local_map_info_.mode == 1) // no filtering
                {
                    // debugger::debugOutput("[MapCost]/[computePositionMapCost] ", 
                    //         "Use no filtering map", 3);
                    return cost_t(0, 
                            cost_params_.weight_of_height * 
                            this->local_map_->local_map.at("elevation_map", index));
                }
                else if (this->local_map_->local_map_info_.mode == 2 || 
                         this->local_map_->local_map_info_.mode == 4 || 
                         this->local_map_->local_map_info_.mode == 5) // filtering
                {
                    // debugger::debugOutput("[MapCost]/[computePositionMapCost] ", 
                    //         "Use filtered map", 3);
                    return cost_t(0, 
                            cost_params_.weight_of_height * 
                            this->local_map_->local_map.at(
                                "filtered_elevation_map", index));
                }
                else if (this->local_map_->local_map_info_.mode == 3) // slope 
                {
                    return cost_t(0, 
                            cost_params_.weight_of_height * 
                            std::abs(this->local_map_->local_map.at(
                                "filtered_slope", index)));
                }
                else
                {

                    std::string error_msg = "[MapCost] Unrecognize cost type: "
                        + std::to_string(this->local_map_->local_map_info_.mode);
                    debugger::debugExitColor(error_msg, __LINE__, __FILE__);
                }
            }
        }

    }
    else 
    {
        return cost_t(0, cost_params_.outside_map_cost);
    }
}

std::ostream& operator<<(std::ostream& out, const cost_t& cost) 
{
    out << '(' << cost.distance << ',' << cost.height << ')';
    // out << "This is the test";
    return out;
}

// return true if free
bool MapCost::isNeighborObstacleFree(const pose_t& pose, const double& radius) const
{
    return local_map_->isNeighborObstacleFree(pose.x, pose.y, radius);
}

    
// Cost::Cost(const double& distance_cost, const double& height_cost) :
//     distance_(distance_cost),
//     height_(height_cost) { };
// 
// Cost::Cost(const double& distance_cost) :
//     distance_(distance_cost), height_(0) { };
// 
// Cost::Cost(void) : distance_(0), height_(0) { };
// 
// Cost::Cost(const Cost& t_cost) :
//     distance_(t_cost.distance_), height_(t_cost.height_) { }
// 
// Cost::~Cost() { };
// 
// inline Cost Cost::operator+ (const Cost& t_cost)
// {
//     return Cost(t_cost.distance_ + distance_, t_cost.height_ + height_);
// }
// 
// inline bool Cost::operator< (const Cost& rhs) const
// {
//     return distance_ + height_ < rhs.distance_ + rhs.height_;
// }





} /*  bipedlab */ 
