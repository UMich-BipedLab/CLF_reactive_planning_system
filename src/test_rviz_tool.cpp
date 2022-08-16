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
// ROS
#include <ros/ros.h>

// For visualizing things in rviz
#include <rviz_visual_tools/rviz_visual_tools.h>

// C++
#include <string>
#include <vector>

#include <eigen3/Eigen/Dense> // SVD
#include <eigen3/Eigen/StdVector>
#include <eigen3/Eigen/Eigenvalues>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>



#include "fake_map.h"
#include "lie_group.h"
#include "map_cost.h"
#include "utils/plotting.h"
#include "utils/debugger.h"
#include "utils/utils.h"
#include "utils/line.h"
#include "utils/plane.h"
#include "clf_rrt.h"
#include "csv.h"
#include "point.h"


// 0-4 general debugging purposes
// 5-8: algorithm status/process
int DEBUG_LEVEL = 3;


namespace rvt = rviz_visual_tools;
using namespace bipedlab;

namespace rviz_visual_tools
{

class RvizVisualToolsDemo
{
    private:
        // A shared node handle
        ros::NodeHandle nh_;

        // For visualizing things in rviz
        rvt::RvizVisualToolsPtr visual_tools_;

        std::string name_;

    public:
        /**
         * \brief Constructor
         */
        RvizVisualToolsDemo() : name_("rviz_demo") 
    {
        visual_tools_.reset(new rvt::RvizVisualTools("world", "/rviz_visual_tools"));
        visual_tools_->loadMarkerPub();  // create publisher before waiting

        ROS_INFO("Sleeping 5 seconds before running demo");
        ros::Duration(5.0).sleep();

        // Clear messages
        visual_tools_->deleteAllMarkers();
        visual_tools_->enableBatchPublishing();
    }

        void publishLabelHelper(const Eigen::Isometry3d& pose, const std::string& label)
        {
            Eigen::Isometry3d pose_copy = pose;
            pose_copy.translation().x() -= 0.2;
            visual_tools_->publishText(pose_copy, label, rvt::WHITE, rvt::XXLARGE, false);
        }

        void testRows(double& x_location)
        {
            debugger::debugOutput("[Unit test] ", "Plane Fitting", 5);
            plane::plane_t test_plane;
            test_plane.print();

            Eigen::MatrixXf test_points(3, 6); // input points
            test_points << 0, 1, 3, 5, 7, 9,
                        0, 2, 4, 6, 8, 10,
                        0, 3, 6, 9, 12, 15;
            std::cout << "points: " << test_points << std::endl;
            auto plane_params = plane::fitPlaneViaLeastSquares(test_points);
            plane_params->print(); // from matlab: 1.7764e-15          1.5           -1  -1.1552e-15
            std::vector<float> plane_coefficients = plane_params->getPlaneCoefficients();

            // rviz_visual_tools::RvizVisualToolsPtr visual_tools_;
            // visual_tools_.reset(new rviz_visual_tools::RvizVisualTools("base_frame","/rviz_visual_markers"));
            // visual_tools_->loadMarkerPub();  // create publisher before waiting

            // Clear messages
            // visual_tools_->deleteAllMarkers();
            // visual_tools_->enableBatchPublishing();

            visual_tools_->publishABCDPlane(plane_coefficients[0],
                    plane_coefficients[1],
                    plane_coefficients[2],
                    plane_coefficients[3]);

            // Create pose
            // Eigen::Isometry3d pose1 = Eigen::Isometry3d::Identity();
            // Eigen::Isometry3d pose2 = Eigen::Isometry3d::Identity();

            // pose1.translation().x() = x_location;

            // double space_between_rows = 0.2;
            // double y = 0;
            // double step;


            // // --------------------------------------------------------------------
            // ROS_INFO_STREAM_NAMED(name_, "Displaying Planes");
            // pose1 = Eigen::Isometry3d::Identity();
            // y += space_between_rows;
            // pose1.translation().y() = y;
            // step = 0.2;
            // double max_plane_size = 0.075;
            // double min_plane_size = 0.01;
            // for (double i = 0; i <= 1.0; i += step)
            // {
            //     visual_tools_->publishXYPlane(pose1, rvt::RED, i * max_plane_size + min_plane_size);
            //     visual_tools_->publishXZPlane(pose1, rvt::GREEN, i * max_plane_size + min_plane_size);
            //     visual_tools_->publishYZPlane(pose1, rvt::BLUE, i * max_plane_size + min_plane_size);
            //     if (i == 0.0)
            //     {
            //         publishLabelHelper(pose1, "Planes");
            //     }

            //     pose1.translation().x() += step;
            // }
            visual_tools_->trigger();
        }

};  // end class

}  // namespace rviz_visual_tools

int main(int argc, char** argv)
{
    ros::init(argc, argv, "visual_tools_demo");
    ROS_INFO_STREAM("Visual Tools Demo");

    // Allow the action server to recieve and send ros messages
    ros::AsyncSpinner spinner(1);
    spinner.start();

    rviz_visual_tools::RvizVisualToolsDemo demo;

    double x_location = 0;
    demo.testRows(x_location);

    ROS_INFO_STREAM("Shutting down.");

    return 0;
}

