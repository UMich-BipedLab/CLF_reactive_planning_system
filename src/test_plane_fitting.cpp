#include <iostream>
#include <Eigen/Dense>
#include <ros/ros.h>


#include "utils/plane.h"
#include "utils/debugger.h"
#include "utils/utils.h"
#include "utils/line.h"
#include "utils/timing.h"


#include "utils/plane.h"
#include "utils/plotting.h"

#include "pose.h"


#include "rviz_visual_tools/rviz_visual_tools.h"


int DEBUG_LEVEL = 3;

using namespace bipedlab;


int main(int argc, char *argv[]) 
{
    ros::init(argc, argv, "test_plane_fitting");
    ros::NodeHandle nh;


   Eigen::MatrixXf A = Eigen::MatrixXf::Random(3, 2);
   std::cout << "A: " << A << std::endl;

   Eigen::JacobiSVD<Eigen::MatrixXf> svd(A, Eigen::ComputeFullU | Eigen::ComputeFullV);
   std::cout << "U: " << svd.matrixU() << std::endl;;
   std::cout << "V: " << svd.matrixV() << std::endl;;



   debugger::debugTitleTextOutput("[Unit test] ", "Plane Fitting", 5);
   int dataset = 2;
    
   std::clock_t time_s = timing::getCurrentCPUTime();
   size_t num_fitting = 10000;
   Eigen::MatrixXd lidar_points; // input points
   for (int k = 0; k < num_fitting; ++k)
   {
       // debugger::debugOutput("[Unit test] k: ", k, 5);
       double y_width = 2;
       if (dataset == 1)
       {
           Eigen::MatrixXd test_points(3, 8); // input points
           test_points <<
               0.014047, 0.53454, 0.5782, 0.06592, 0.00070, 0.42795, 0.73865, 0.84611,
               0.047974, 0.83191, 0.4617, 0.45222, 0.33814, 0.67403, 0.52019, 0.45547,
               0.677590, 0.27622, 0.7249, 0.80713, 0.34365, 0.47471, 0.29546, 0.20986;
           lidar_points = test_points;
       }
       else if (dataset == 2)
       {
           size_t num_test_points = 10000;
           auto xs = utils::genListOfInclusiveRandomNumbers<float>(num_test_points, -1, 3);
           auto ys = utils::genListOfInclusiveRandomNumbers<float>(num_test_points,
                   -y_width/2, y_width/2);
           auto z_noise = utils::genListOfInclusiveRandomNumbers<float>(num_test_points,
                   -0.05, 0.05);
           std::vector<float> zs(num_test_points, 0);
           double k1 = 1;
           double k2 = 0;
           for (int i = 0; i < num_test_points; ++i)
           {
               zs[i] = std::cos(k1 * xs[i]) * std::cos(k2 * ys[i]) + z_noise[i];
           }

           Eigen::MatrixXf test_points(3, num_test_points);
           float* xs_ptr = &xs[0];
           test_points.row(0) = Eigen::Map<Eigen::VectorXf, Eigen::Unaligned>(xs.data(), xs.size());

           float* ys_ptr = &ys[0];
           test_points.row(1) = Eigen::Map<Eigen::VectorXf, Eigen::Unaligned>(ys.data(), ys.size());

           float* zs_ptr = &zs[0];
           test_points.row(2) = Eigen::Map<Eigen::VectorXf, Eigen::Unaligned>(zs.data(), zs.size());
           lidar_points = test_points.cast<double>();
       }

       plane::terrain_info_t terrain_plane();
       auto plane_params = 
           plane::fitPlaneViaLeastSquares(lidar_points.cast<float>());
       // plane_params.print();
   }
   double cpu_time = timing::spendCPUTime(time_s);
   debugger::debugOutput("[Unit test] cpu_time_sum [s]: ", cpu_time, 5);
   debugger::debugOutput("[Unit test] cpu_time_ave [ms]: ", 1000*cpu_time/num_fitting, 5);
   debugger::debugOutput("[Unit test] cpu_time_ave [hz]: ", 1.0/(cpu_time/num_fitting), 5);





   // robot pose
   pose_t robot_pose(0, 0, deg_to_rad(0));
   Eigen::Vector2f robot_pose_vec = Eigen::Vector2f(std::cos(robot_pose.theta),
                                                    std::sin(robot_pose.theta));

   // plane fitting
   int max_step = 5;
   double increment = 0.3;
   double delta_y = 0.3;
   size_t num_points = lidar_points.cols();
   std::vector<plane::terrain_info_t> terrain_plane_vec(max_step);
   std::vector<std::vector<Eigen::Vector3f>> segment_points(max_step);

   for (int i = 0; i < num_points; ++i)
   {
       Eigen::Vector2f v = lidar_points.block(0, i, 2, 1).cast<float>() - Eigen::Vector2f(robot_pose.x, robot_pose.y);
       double distance = v.norm();
       double x_distance = robot_pose_vec.dot(v);

       double theta = std::acos(x_distance / std::max(distance, 1e-5));
       double y_distance = distance * std::sin(theta);
       if (y_distance > delta_y || x_distance < 0)
           continue;

       int k = std::floor(x_distance / increment);
       if (k < max_step)
       {
           segment_points[k].push_back(lidar_points.col(i).cast<float>());
       }
   }

   // plane fitting

   for (int seg = 0; seg < max_step; ++seg)
   {
       size_t num_seg = segment_points[seg].size();
       Eigen::MatrixXf seg_points = Eigen::MatrixXf(3, num_seg);
       for (int i = 0; i < num_seg; ++i)
       {
           seg_points.col(i) = segment_points[seg][i];
       }
       terrain_plane_vec[seg].points = seg_points;
       auto plane_params = plane::fitPlaneViaLeastSquares(terrain_plane_vec[seg].points);
       std::vector<float> plane_coefficients = plane_params.getPlaneCoefficients();
       terrain_plane_vec[seg].normal_vector = plane_params.normal_vector;
       terrain_plane_vec[seg].fixed_point = plane_params.fixed_point;

   }

   // viz
   rviz_visual_tools::RvizVisualToolsPtr visual_tools_;
   visual_tools_.reset(new rviz_visual_tools::RvizVisualTools("odom","/rviz_visual_markers"));
   visual_tools_->loadMarkerPub();  // create publisher before waiting

   // Clear messages

   int k = 0;
   while (1)
   {
       debugger::debugOutput("[Unit test] Publishing: ", k, 5);
       k++;
       visual_tools_->resetMarkerCounts();
       visual_tools_->deleteAllMarkers();
       // visual_tools_->enableBatchPublishing();

       // publish all points
       for (int i = 0; i < lidar_points.cols(); ++i) {
           geometry_msgs::Vector3 scale = visual_tools_->getScale(rviz_visual_tools::MEDIUM);
           std_msgs::ColorRGBA color = visual_tools_->getColor(rviz_visual_tools::WHITE);
           visual_tools_->publishSphere(lidar_points.col(i), color, scale, "all points", i);
       }

       // publish plane information
       for (int seg = 0; seg < max_step; ++seg)
       {
           geometry_msgs::Vector3 scale = visual_tools_->getScale(rviz_visual_tools::MEDIUM);
           std_msgs::ColorRGBA color = visual_tools_->getColorScale((float) seg / max_step);
           for (int i = 0; i < terrain_plane_vec[seg].points.cols(); ++i)
           {
               visual_tools_->publishSphere(
                       terrain_plane_vec[seg].points.col(i).cast<double>(),
                       color, scale, "seg points", i);
           }
           std::vector<float> plane_coefficients =
               terrain_plane_vec[seg].getPlaneCoefficients();
           Eigen::Vector3d center = terrain_plane_vec[seg].points.rowwise().mean().cast<double>();
           visual_tools_->publishABCDPlaneWithCenter(plane_coefficients[0],
                   plane_coefficients[1],
                   plane_coefficients[2],
                   plane_coefficients[3], center, color, increment, 2*delta_y);
       }
       visual_tools_->trigger();
       usleep(5e6);
   }


}
