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
#ifndef LOCAL_MAP_H
#define LOCAL_MAP_H


#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>
#include <eigen3/Eigen/Dense> 
#include "boost/thread/mutex.hpp"
#include "boost/thread.hpp"


#include "pose.h"
#include "robot_state.h"
#include "utils/plane.h"
#include "cost_params_t.h"
#include "utils/debugger.h"



namespace bipedlab
{
typedef struct local_map_info
{
    // in order to use roslaunch, it has to be int and not size_t
    int mode; // 0: original, 1: fillin holes
    double obstacle_threshold;
    double length; // how many grid behind and front

    // if mode != 0
    double smooth_radius; // search radius
    double nan_percentage_in_radius;  // remove nan if the ratio of nan within 
                                      // the smooth radius is less than this value
    double SDF_radius; // signed distance field

    cost_params_t cost_params;

    local_map_info(void): mode(0), obstacle_threshold(0.5), length(18) { }

    local_map_info(const int& mode, const double& obstacle_threshold, 
                   const double& length):
        mode(mode), obstacle_threshold(obstacle_threshold), length(length) { }
} local_map_params_t;
    

typedef struct terrain_plane_params
{
    double delta_x;
    double delta_y;
    int num_planes;
    int min_num_points_for_fitting; // minimum number of point for plane fitting
    terrain_plane_params(void): delta_x(0.3), delta_y(0.3), 
        num_planes(5), min_num_points_for_fitting(3) { }
} terrain_plane_params_t;


class LocalMap
{
private:
    friend class MapCost;
    const grid_map::GridMap* map_;
    const pose_t* current_pose_;
    local_map_params_t local_map_info_;
    terrain_plane_params_t terrain_plane_params_;
    std::shared_ptr<std::vector<plane::terrain_info_t>> terrain_ptr_;
    boost::mutex terrain_lock_;


    

public:
    LocalMap(const pose_t* current_pose, 
             const grid_map::GridMap* map, 
             const local_map_params_t& local_map_info,
             const terrain_plane_params_t& terrain_plane_params);

    LocalMap(const pose_t* current_pose, 
             const grid_map::GridMap* map, 
             const local_map_params_t& local_map_info);

    bool isNeighborObstacleFree(
            const double& x, const double& y, const double& radius = 0.1) const;
    Eigen::Matrix<double, 2, 4> getFourCorners(void) const;

    void updateLocalMap(const pose_t& current_pose);
    void printMapAddress(void) { debugger::debugOutput("[In LocalMap()] &(global_map_ in Planner == map_): ", map_, 5);};


    void lockTerrainThread(bool status) // true is lock, false is unlock.
    {
        if (status)
            terrain_lock_.lock();
        else
            terrain_lock_.unlock();
    }
    // void lockTerrainThread(bool status) // true is lock, false is unlock.
    // {
    //     // boost::mutex::scoped_lock guard(terrain_lock);

    //        boost::lock_guard<boost::mutex> guard(terrain_lock);
    //         // boost::unique_lock<boost::mutex> ownlock(terrain_lock);
    //         // ownlock.unlock();
    //         //
    //         // terrain_lock.unlock();
    // }
    std::shared_ptr<std::vector<plane::terrain_info_t>> getTerrainInfo(void) const { return terrain_ptr_; };
    std::vector<plane::terrain_info_t> terrain_info;


    grid_map::GridMap local_map;
    // grid_map::GridMap* local_map_ptr;
    bool is_successful;
    virtual ~LocalMap();
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
    
    
} /* bipedlab */ 


#endif /* LOCAL_MAP_H */

