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
#include <ros/ros.h>
#include <ros/console.h>


#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>
#include <cmath>

#include "fake_map.h"
#include "utils/debugger.h"


using namespace bipedlab;
int DEBUG_LEVEL = 5;

int main(int argc, char** argv)
{
    // Initialize node and publisher.
    ros::init(argc, argv, "fake_map_publisher");
    ros::NodeHandle nh("~");
    ros::Publisher map_pub = nh.advertise<grid_map_msgs::GridMap>(
            "multi_layer_map", 1, true);
    float rate = 100;

    // Create grid map.
    int scene = 2; // which map to use

    nh.getParam("rate", rate);
    nh.getParam("fake_map/scene", scene);


    debugger::debugTitleTextOutput("[fake map]", "Map Publisher", 10);
    debugger::debugColorOutput("[fake map] Publishing map at [Hz] ", rate, 10, BB);
    debugger::debugColorOutput("[fake map] Scene: ", scene, 10, BB);


    // load map
    FakeMap* fake_map = new FakeMap(scene);

    // Work with grid map in a loop.
    ros::Rate loop_rate(rate);
    while (nh.ok()) {

        // Add data to grid map.
        ros::Time time = ros::Time::now();
        fake_map->map->setTimestamp(time.toNSec());
        grid_map_msgs::GridMap message;
        grid_map::GridMapRosConverter::toMessage(*(fake_map->map), message);
        map_pub.publish(message);

        ROS_INFO_THROTTLE(1.0, "Fake map (timestamp %f) published.", message.info.header.stamp.toSec());

        // Wait for next cycle.
        loop_rate.sleep();
    }

    return 0;
}
