// UDP
#include <sys/types.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <memory.h>
#include <ifaddrs.h>
#include <net/if.h>
#include <errno.h>
#include <stdlib.h>
#include <iostream>

#include <ros/ros.h>
#include <ros/console.h>
#include <tf/transform_listener.h>
#include <std_msgs/String.h>

#include <fstream>
#include <unistd.h> // sleep
#include <chrono>  // timing
#include <ctime>
#include <time.h>

// terminate replanning if has been too long
#include <future>
#include <thread>
#include <chrono>

#include "utilsDstar.h"
#include "mapper.h"
#include "cassie_rrt_node.hpp"


#define PORT 8080 
#define MAX_STEPS 30000 // D* replanning steps
#define COST_FOR_UNKNOWN 100

using namespace bipedlab;

struct Node {
    public:
        Node() {}

        Node(geometry_msgs::Point p) : point(p) {}

        template <class T,class U >
            Node(T t, U u) {point.x=t; point.y=u;};

        template <class T,class U >
            Node(T t, U u, int id, int pId) {
                this->point.x=t;
                this->point.y=u;
                this->id=id;
                this->parentId=pId;
            };

        int id;
        geometry_msgs::Point point;
        std::vector<Node> children;
        int parentId;
};


int resolvehelper(const char* hostname, int family, const char* service, sockaddr_storage* pAddr) {
    int result;
    addrinfo* result_list = NULL;
    addrinfo hints = {};
    hints.ai_family = family;
    hints.ai_socktype = SOCK_DGRAM; // without this flag, getaddrinfo will return 3x the number of addresses (one for each socket type).
    result = getaddrinfo(hostname, service, &hints, &result_list);
    if (result == 0)
    {
        //ASSERT(result_list->ai_addrlen <= sizeof(sockaddr_in));
        memcpy(pAddr, result_list->ai_addr, result_list->ai_addrlen);
        freeaddrinfo(result_list);
    }

    return result;
}


int main(int argc, char **argv) {
    // std::list<cassie_rrt_node_data_t> info;

	// UDP setup
    int result = 0;
    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    sockaddr_in addrListen = {}; // zero-int, sin_port is 0, which picks a random port for bind.
    addrListen.sin_family = AF_INET;
    result = bind(sock, (sockaddr*)&addrListen, sizeof(addrListen));

    if (result == -1) {
       int lasterror = errno;
       std::cout << "error: " << lasterror;
       exit(1);
    }
    sockaddr_storage addrDest = {};
    result = resolvehelper("10.10.10.3", AF_INET, "28000", &addrDest);

    if (result != 0){
       int lasterror = errno;
       std::cout << "error: " << lasterror;
       exit(1);
    }

    // const char* msg = "test";
    // size_t msg_length = strlen(msg);
    // result = sendto(sock, msg, msg_length, 0, (sockaddr*)&addrDest, sizeof(addrDest));
    // std::cout << result << " bytes sent" << std::endl;



	// ROS
    ros::init(argc, argv, "cassie_planner");
	Mapper mapper(argc, argv);
    if(ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, 
                                      ros::console::levels::Debug))
        ros::console::notifyLoggerLevelsChanged();

    // XXX: UDP TEST
    // state test;
    // test.x = 1;
    // test.y = 2;
    // std::vector<state> v_test = {test, test, test,test};
    // std::string s_test = mapper.publishPath(v_test, 1);
    // char *c_msgs = new char [s_test.length()+1];
    // strcpy (c_msgs, s_test.c_str());
    // // cout << "returned: " << c_msgs << endl;
    // size_t msg_length = strlen(c_msgs);
    // result = sendto(sock, c_msgs, msg_length, 0, (sockaddr*)&addrDest, sizeof(addrDest));
    // std::cout << result << " bytes sent" << std::endl;

    // v_test.clear();
    // std::string s_test2 = mapper.publishPath(v_test, 1);
    // cout << "size: " << strlen(s_test2.c_str()) << endl;
    // cout << "size: " << s_test2.size() << endl;
    // exit(0);

    // start, goal position
    static int boundary = 40;
    static int init_x = 0;
    static int init_y = 0;
    static int goal_x = 20;
    static int goal_y = 20;
    static int occupied = -1;


    vector<float> vec_goal_x{8, 10, 13, -18};
    vector<float> vec_goal_y{-22, -32, -37, -6};
    vector<float> vec_goal_z{0, 0, 0, 0};
    vector<geometry_msgs::Point> goal_points;

    for (int i=0; i<vec_goal_x.size(); ++i){
        geometry_msgs::Point goal_point;
        goal_point.x = vec_goal_x[i];
        goal_point.y = vec_goal_y[i];
        goal_point.z = vec_goal_z[i];
        goal_points.push_back(goal_point);
    }

    Node initial(init_x, init_y, -1, -2);
    Node goal(goal_x, goal_y, 10000, 0);
    list<state> listpath;

    // known map, pre-defined goal without surranding information
    /*
    if (mapper.method==0){ // known map 
        ROS_INFO("RUNNING D* with known map");
        Dstar *dstar = new Dstar(MAX_STEPS, COST_FOR_UNKNOWN);
        int goal_count = 0;
        int update_count = 0;
        ros::Rate r(100); 
        ROS_INFO("Waiting for transform...");

        while (ros::ok()){
            std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();
            // draw robot
            dstar_utils::assignMarker(mapper.marker_pub, 
                                    mapper.robot_state.robot_pose.getOrigin().x(),
                                    mapper.robot_state.robot_pose.getOrigin().y(),
                                    0.0, visualization_msgs::Marker::CUBE, 
                                    "robot", 0, 0, 1, 
                                    1, 0.5);

            // draw goal
            dstar_utils::assignMarker(mapper.marker_pub, goal_points[goal_count].x, 
                         goal_points[goal_count].y, goal_points[goal_count].z,
                         visualization_msgs::Marker::CUBE, 
                         // "goal" + to_string(goal_count), 
                         "goal", 1, 0, 0, 
                         1, 1);
            mapper.updateMap();

            if (update_count==0){
                mapper.lookUpPose();

                if (!mapper.pose_updated){
                    continue;
                }
                dstar->init(mapper.grid_to_costmap_ratio * mapper.robot_state.robot_pose.getOrigin().x(), 
                            mapper.grid_to_costmap_ratio * mapper.robot_state.robot_pose.getOrigin().y(), 
                            mapper.grid_to_costmap_ratio * goal_points[0].x, 
                            mapper.grid_to_costmap_ratio * goal_points[0].y);
                mapper.updateGoal(goal_points[0].x, goal_points[0].y); // in grid map

                //TODO: Real map loader
                // mapper.updateDstar(dstar, occupied, boundary, gridToCostMapRatio);
                dstar->replan();
                std::vector<state> vPath{std::begin(listpath), std::end(listpath)};
                dstar_utils::drawDstar(vPath, mapper.marker_array_pub, mapper.grid_to_costmap_ratio, 
                                       string("original D*"), 1, 0, 0);
                ROS_INFO_STREAM(vPath.size() << " waypoints");

                // publish path
                std::string s_path = (mapper.publishPath(vPath)).s_waypoints_udp;
                if (s_path.size()!=0){
                    ROS_INFO_STREAM(result << " bytes sent");
                    char *c_msgs = new char [s_path.length()+1];
                    strcpy (c_msgs, s_path.c_str());
                    // cout << "returned: " << c_msgs << endl;
                    size_t msg_length = strlen(c_msgs);
                    result = sendto(sock, c_msgs, msg_length, 0, (sockaddr*)&addrDest, sizeof(addrDest));
                    ROS_INFO_STREAM(result << " bytes sent");
                    ROS_INFO("D* Initialized");
                    update_count ++;
                }
                else {
                    ROS_WARN("NO PATH");
                }
                // exit(0);
            }

            mapper.lookUpPose();
            if (mapper.checkReachGoal()){
                goal_count ++;

                if (goal_count>=goal_points.size()) {
                    break;
                }
                // update in grid and costmap (costmap is handled in the class)
                mapper.newGoal(dstar,
                               goal_points[goal_count].x,
                               goal_points[goal_count].y);
            }
            dstar->updateStart(mapper.robot_state.robot_pose.getOrigin().x(), 
                               mapper.robot_state.robot_pose.getOrigin().y());

            dstar->replan();
            listpath = dstar->getPath();
            std::vector<state> vPath{std::begin(listpath), std::end(listpath)};
            ROS_INFO_STREAM(vPath.size() << " waypoints");

            // publish path
            std::string s_path = mapper.publishPath(vPath).s_waypoints_udp;
            if (s_path.size()!=0){
                char *c_msgs = new char [s_path.length()+1];
                strcpy (c_msgs, s_path.c_str());
                // cout << "returned: " << c_msgs << endl;
                size_t msg_length = strlen(c_msgs);
                result = sendto(sock, c_msgs, msg_length, 0, (sockaddr*)&addrDest, sizeof(addrDest));
                ROS_INFO_STREAM(result << " bytes sent");

                dstar_utils::drawDstar(vPath, mapper.marker_array_pub, mapper.grid_to_costmap_ratio, 
                                       string("original D*"), 1, 0, 0);
            }
            else {
                ROS_WARN("NO PATH");
            }
            std::chrono::high_resolution_clock::time_point t2 = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double> time_span = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
            ROS_INFO_STREAM("computation: " << 1/time_span.count() << " Hz");
            r.sleep();
        }
        ROS_INFO("FINISHED");
    }

    // known map, pre-defined goal with surranding information
    if (mapper.method==1){
        ROS_INFO("RUNNING D*");
        Dstar *dstar = new Dstar(MAX_STEPS, COST_FOR_UNKNOWN);
        clock_t StartO = clock();
        int update_count = 0;
        int goal_count = 0;
        ros::Rate r(100);
        ROS_INFO("Waiting for transform...");

        while (ros::ok()){
            std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();

            // draw robot
            dstar_utils::assignMarker(mapper.marker_pub, 
                                    mapper.robot_state.robot_pose.getOrigin().x(),
                                    mapper.robot_state.robot_pose.getOrigin().y(),
                                    0.0, visualization_msgs::Marker::SPHERE, 
                                    "robot", 0, 0, 1, 
                                    1, 0.1);

            // draw goal
            dstar_utils::assignMarker(mapper.marker_pub, goal_points[goal_count].x, 
                         goal_points[goal_count].y, goal_points[goal_count].z,
                         visualization_msgs::Marker::CUBE, 
                         // "goal" + to_string(goal_count), 
                         "goal", 1, 0, 0, 
                         1, 1);
            mapper.updateMap();

            if (mapper.map_updated){
                if (update_count==0){
                    mapper.lookUpPose();

                    if (!mapper.pose_updated || 
                        mapper.boundaries.free_cell_vec.size()==0){
                        continue;
                    }
					dstar->init(mapper.robot_state.robot_pose.getOrigin().x(), 
								mapper.robot_state.robot_pose.getOrigin().y(), 
								mapper.grid_to_costmap_ratio*goal_points[0].x, 
								mapper.grid_to_costmap_ratio*goal_points[0].y);
					mapper.updateGoal(goal_points[0].x, goal_points[0].y); // update goal
					mapper.updateDstarMap(dstar); // update costmap
					dstar->replan();
					std::vector<state> vPath{std::begin(listpath), std::end(listpath)};
					dstar_utils::drawDstar(vPath, mapper.marker_array_pub, mapper.grid_to_costmap_ratio, 
										   string("original D*"), 1, 0, 0);
					ROS_INFO_STREAM(mapper.boundaries.free_cell_vec.size() << " free cells");
					dstar_utils::drawCell(mapper.boundaries.free_cell_vec, mapper.marker_array_pub, 
                                          mapper.grid_to_costmap_ratio, 
										  string("free cells"), 0, 1, 0, 0.2);
					ROS_INFO_STREAM(vPath.size() << " waypoints");

					// publish path
					std::string s_path = mapper.publishPath(vPath).s_waypoints_udp;
					if (s_path.size()!=0){
						ROS_INFO_STREAM(result << " bytes sent");
						char *c_msgs = new char [s_path.length()+1];
						strcpy (c_msgs, s_path.c_str());
						// cout << "returned: " << c_msgs << endl;
						size_t msg_length = strlen(c_msgs);
						result = sendto(sock, c_msgs, msg_length, 0, (sockaddr*)&addrDest, sizeof(addrDest));
						ROS_INFO_STREAM(result << " bytes sent");
						ROS_INFO("D* Initialized");
						update_count ++;
					}
					else {
						ROS_WARN("NO PATH");
					}

                    // mapper.newGoal();
                    // dstar->init(mapper.robot_state.robot_pose.getOrigin().x(), 
                    //             mapper.robot_state.robot_pose.getOrigin().y(), 
                    //             gridToCostMapRatio*goal_x, gridToCostMapRatio*goal_y);

                    //TODO: Real map loader
                    // mapper.updateDstar(dstar, occupied, boundary, gridToCostMapRatio);
                    ROS_INFO("D* Initialized");
                    update_count ++;
                    // exit(0);
                }
                // mapper.newGoal();
                dstar->replan();

                if (update_count!=0){
                    update_count ++;
                }
            }
            else {
                if (update_count==0){
                    continue;
                }
            }
            mapper.lookUpPose();
            dstar->updateStart(mapper.robot_state.robot_pose.getOrigin().x(), 
                               mapper.robot_state.robot_pose.getOrigin().y());
            mapper.updateDstarMap(dstar); // update costmap

            // check if reached the goal
            if (mapper.checkReachGoal()){
                goal_count ++;

                if (goal_count>=goal_points.size()) {
                    break;
                }
                mapper.newGoal(dstar,
                               mapper.grid_to_costmap_ratio*goal_points[goal_count].x,
                               mapper.grid_to_costmap_ratio*goal_points[goal_count].y);
            }

            dstar->replan();
            listpath = dstar->getPath();
            std::vector<state> vPath{std::begin(listpath), std::end(listpath)};
            ROS_INFO_STREAM(mapper.boundaries.free_cell_vec.size() << " free cells");
            dstar_utils::drawCell(mapper.boundaries.free_cell_vec, mapper.marker_array_pub, 
                                  mapper.grid_to_costmap_ratio, string("free cells"), 0, 1, 0, 0.2);

            ROS_INFO_STREAM(vPath.size() << " waypoints");

            // publish path
            std::string s_path = mapper.publishPath(vPath).s_waypoints_udp;
            if (s_path.size()!=0){
                char *c_msgs = new char [s_path.length()+1];
                strcpy (c_msgs, s_path.c_str());
                // cout << "returned: " << c_msgs << endl;
                size_t msg_length = strlen(c_msgs);
                result = sendto(sock, c_msgs, msg_length, 0, (sockaddr*)&addrDest, sizeof(addrDest));
                ROS_INFO_STREAM(result << " bytes sent");

                dstar_utils::drawDstar(vPath, mapper.marker_array_pub, mapper.grid_to_costmap_ratio, 
                                       string("original D*"), 1, 0, 0);
            }
            else {
                ROS_WARN("NO PATH");
            }
            std::chrono::high_resolution_clock::time_point t2 = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double> time_span = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
            ROS_INFO_STREAM("computation: " << 1/time_span.count() << " Hz");
            r.sleep();
        }
        ROS_INFO("FINISHED");
    }

    // clicked goal
    if (mapper.method==2){
        ROS_INFO("RUNNING D*");
        Dstar *dstar = new Dstar(MAX_STEPS, COST_FOR_UNKNOWN);
        clock_t StartO = clock();
        int update_count = 0;
        ros::Rate r(100);
        ROS_INFO("Waiting for transform...");

        while (ros::ok()){
            std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();

            // draw robot
            dstar_utils::assignMarker(mapper.marker_pub, 
                                    mapper.robot_state.robot_pose.getOrigin().x(),
                                    mapper.robot_state.robot_pose.getOrigin().y(),
                                    0.0, visualization_msgs::Marker::SPHERE, 
                                    "robot", 1, 1, 1, 
                                    1, 0.3);

            // draw goal
            dstar_utils::assignMarker(mapper.marker_pub, mapper.goal_point.x,
                                      mapper.goal_point.y, 0.5,
                                      visualization_msgs::Marker::CUBE, 
                                      "goal", 1, 0, 0, 
                                      1, 1);

            mapper.updateMap();
            if (mapper.goal_updated){
                mapper.goal_updated = false;
                mapper.newGoal(dstar,
                               mapper.grid_to_costmap_ratio*mapper.goal_point.x,
                               mapper.grid_to_costmap_ratio*mapper.goal_point.y);
            }

            if (mapper.map_updated){
                if (update_count==0){
                    mapper.lookUpPose();

                    if (!mapper.pose_updated || 
                        mapper.boundaries.free_cell_vec.size()==0){
                        continue;
                    }
					dstar->init(mapper.robot_state.robot_pose.getOrigin().x(), 
								mapper.robot_state.robot_pose.getOrigin().y(), 
								mapper.grid_to_costmap_ratio*mapper.goal_point.x, 
								mapper.grid_to_costmap_ratio*mapper.goal_point.y);
					mapper.updateGoal(mapper.goal_point.x, mapper.goal_point.y); // update goal
					mapper.updateDstarMap(dstar); // update costmap
					dstar->replan();
					std::vector<state> vPath{std::begin(listpath), std::end(listpath)};
					dstar_utils::drawDstar(vPath, mapper.marker_array_pub, mapper.grid_to_costmap_ratio, 
										   string("original D*"), 1, 0, 0);
					ROS_INFO_STREAM(mapper.boundaries.free_cell_vec.size() << " free cells");
					dstar_utils::drawCell(mapper.boundaries.free_cell_vec, mapper.marker_array_pub, 
                                          mapper.grid_to_costmap_ratio, 
										  string("free cells"), 0, 1, 0, 0.2);
					ROS_INFO_STREAM(vPath.size() << " waypoints");

					// publish path
					std::string s_path = mapper.publishPath(vPath).s_waypoints_udp;
					if (s_path.size()!=0){
                        // s_path = std::string("1.0000,2.0,3.0,4.0,5.0,6.0,7.0,8.0,9.0,10.0,11.0,12.0,13.0,14.0,15.0,16.0,17.0,18.0,19.0,20.0,21.0,22.0,23.0,24.0,25.0,26.0,27.0,8.0,9.0,10.0,11.0,12.0,13.0,14.0,15.0,16.0,17.0,18.0,19.0,20.0,21.0,22.0,23.0,24.0,25.0,26.0,27.0");
                        // cout << "string: " << s_path << endl;
						ROS_INFO_STREAM(result << " bytes sent");
						char *c_msgs = new char [s_path.length()+1];
						strcpy (c_msgs, s_path.c_str());
						// cout << "returned: " << c_msgs << endl;
						size_t msg_length = strlen(c_msgs);
                        // cout << "msg_length: " << msg_length << endl;
						result = sendto(sock, c_msgs, msg_length, 0, (sockaddr*)&addrDest, sizeof(addrDest));
						ROS_INFO_STREAM(result << " bytes sent");
						ROS_INFO("D* Initialized");
						update_count ++;
					}
					else {
						ROS_WARN("NO PATH");
					}

                    // mapper.newGoal();
                    // dstar->init(mapper.robot_state.robot_pose.getOrigin().x(), 
                    //             mapper.robot_state.robot_pose.getOrigin().y(), 
                    //             gridToCostMapRatio*goal_x, gridToCostMapRatio*goal_y);

                    //TODO: Real map loader
                    // mapper.updateDstar(dstar, occupied, boundary, gridToCostMapRatio);
                    ROS_INFO("D* Initialized");
                    update_count ++;
                    // exit(0);
                }
                if (update_count!=0){
                    update_count ++;
                }
            }
            else {
                if (update_count==0){
                    continue;
                }
            }
            mapper.lookUpPose();
            dstar->updateStart(mapper.robot_state.robot_pose.getOrigin().x(), 
                               mapper.robot_state.robot_pose.getOrigin().y());
            mapper.updateDstarMap(dstar); // update costmap

            // wait for a new goal
            if (mapper.checkReachGoal()){
                mapper.waitForNewClick();
            }

            dstar->replan();
            listpath = dstar->getPath();
            std::vector<state> vPath{std::begin(listpath), std::end(listpath)};
            ROS_INFO_STREAM(mapper.boundaries.free_cell_vec.size() << " free cells");
            ROS_INFO_STREAM(mapper.boundaries.occupied_cell_vec.size() << " occupied cells");

            dstar_utils::drawCell(mapper.boundaries.free_cell_vec, mapper.marker_array_pub, 
                                  mapper.grid_to_costmap_ratio, string("free cells"), 0, 1, 0, 0);
            dstar_utils::drawCell(mapper.boundaries.occupied_cell_vec, mapper.marker_array_pub, 
                                  mapper.grid_to_costmap_ratio, string("occupied cells"), 1, 1, 0, 0.2);

            ROS_INFO_STREAM(vPath.size() << " waypoints");

            // publish path
            std::string s_path = mapper.publishPath(vPath).s_waypoints_udp;
            if (s_path.size()!=0){
                char *c_msgs = new char [s_path.length()+1];
                strcpy (c_msgs, s_path.c_str());
                // cout << "returned: " << c_msgs << endl;
                size_t msg_length = strlen(c_msgs);
                result = sendto(sock, c_msgs, msg_length, 0, (sockaddr*)&addrDest, sizeof(addrDest));
                delete c_msgs;
                ROS_INFO_STREAM(result << " bytes sent");

                dstar_utils::drawDstar(vPath, mapper.marker_array_pub, mapper.grid_to_costmap_ratio, 
                                       string("original D*"), 1, 0, 0);
            }
            else {
                ROS_WARN("NO PATH");
            }
            std::chrono::high_resolution_clock::time_point t2 = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double> time_span = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
            ROS_INFO_STREAM("computation: " << 1/time_span.count() << " Hz");
            r.sleep();
        }
        ROS_INFO("FINISHED");
    }
    */

    // extract goal with searching ordered.
    if (mapper.method==3){
        ROS_INFO("RUNNING D*");
        Dstar *dstar = new Dstar(mapper.dstar_max_step, mapper.dstar_unknow_cell_cost);
        clock_t StartO = clock();
        int update_count = 0;
        ros::Rate r(100);
        ROS_INFO("Waiting for transform...");

        std::chrono::high_resolution_clock::time_point clock = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> clock_time;
        bool update = true;
        bool new_path;
        std::vector<state> vPath;
        std::vector<state> vPath_old;
        while (ros::ok()){
            std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();

            if (update){
                if (update_count > 0) {
                    cout << "\033[1;31m====================================== \033[0m\n";
                }
                else {
                    // draw robot
                    dstar_utils::assignMarker(mapper.marker_pub, 
                                              mapper.robot_state.robot_pose.getOrigin().x(),
                                              mapper.robot_state.robot_pose.getOrigin().y(),
                                              0.5, visualization_msgs::Marker::ARROW, 
                                              "robot", 1, 1, 1, 
                                              1, 0.5,
                                              mapper.robot_state.robot_pose.getRotation().x(), 
                                              mapper.robot_state.robot_pose.getRotation().y(),
                                              mapper.robot_state.robot_pose.getRotation().z(),
                                              mapper.robot_state.robot_pose.getRotation().w());

                    // draw goal
                    dstar_utils::assignMarker(mapper.marker_pub, mapper.goal_point.x,
                                              mapper.goal_point.y, 0.5,
                                              visualization_msgs::Marker::CUBE, 
                                              "goal", 1, 1, 0, 
                                              1, 0.5);
                }

                // update current map
                if (update_count > 0) ROS_DEBUG("[Step] Updating current map...");
                mapper.updateMap();

                if (update_count > 0) ROS_DEBUG("[Step] Updated");

                if (update_count > 0) {
                    ROS_DEBUG("[Step] Looking up the robot pose...");
                    mapper.lookUpPose();
                    mapper.updateBoundries();
                    // mapper.frontiers();
                    mapper.checkFork();

                    // if (mapper.checkReachGoal()){
                      ROS_DEBUG("[Step] Searching a goal...");
                      mapper.goalSelection();
                    // }
                }

                if (update_count > 0 && mapper.boundaries.free_cell_vec.size()==0) {
                    ROS_WARN("[Boundary] NO FREE CELL");
                    mapper.no_free_cell = true;
                }

                if (mapper.goal_updated){
                    mapper.goal_updated = false;
                    if (update_count > 0) ROS_DEBUG("[Step] Updating the goal...");

                    // update in grid and costmap (costmap is handled in the class)
                    dstar->updateGoal(mapper.goal_point.x*mapper.grid_to_costmap_ratio, 
                                      mapper.goal_point.y*mapper.grid_to_costmap_ratio);
                    if (update_count > 0) ROS_DEBUG("[Step] Updated");
                }

                if (mapper.map_updated){
                    if (update_count==0){
                        if (!mapper.pose_updated || !mapper.frontier ||
                            mapper.boundaries.free_cell_vec.size()==0){
                            if(mapper.lookUpPose()){
                                mapper.updateBoundries();
                                mapper.checkFork();
                                mapper.goalSelection();
                            }

                            if (mapper.boundaries.free_cell_vec.size()==0){
                                ROS_WARN("NO FREE CELL");
                            }

                            if (!mapper.frontier){
                                ROS_WARN("NO frontier");
                            }

                            if (!mapper.pose_updated){
                                ROS_WARN("No pose");
                            }
                            mapper.no_free_cell = true;

                            dstar_utils::drawCellText(mapper.boundaries.current_cell_vec, 
                                                      mapper.marker_array_pub,
                                                      1, string("cost text"), 0, 1, 1, 1, 
                                                      0.1); 
                            dstar_utils::drawCellNoText(mapper.boundaries.boundaries_cell_vec, 
                                                        mapper.marker_array_pub,
                                                        1, string("boundaries cells"), 1, 0, 0, 1, 
                                                        0.5, visualization_msgs::Marker::SPHERE);
                            std::chrono::high_resolution_clock::time_point t2 = std::chrono::high_resolution_clock::now();
                            std::chrono::duration<double> time_spent = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
                            ROS_INFO_STREAM("computation: " << 1/time_spent.count() << " Hz");
                            continue;
                        }
                        mapper.lookUpPose();
                        mapper.updateBoundries();
                        mapper.checkFork();
                        mapper.goalSelection();
                        ROS_INFO_STREAM(mapper.boundaries.free_cell_vec.size() << " new free cells");

                        // init planner
                        dstar->init(mapper.robot_state.robot_pose.getOrigin().x() * mapper.grid_to_costmap_ratio,
                                    mapper.robot_state.robot_pose.getOrigin().y() * mapper.grid_to_costmap_ratio,
                                    mapper.goal_point.x*mapper.grid_to_costmap_ratio, 
                                    mapper.goal_point.y*mapper.grid_to_costmap_ratio);

                        // update goal
                        mapper.updateGoal(mapper.goal_point.x , mapper.goal_point.y); // update goal in costmap
                        update_count ++;
                    }
                    if (update_count > 1){
                        update_count ++;
                    }
                    if (mapper.pose_updated ||
                        mapper.boundaries.free_cell_vec.size()!=0 ||
                        mapper.boundaries.occupied_cell_vec.size()!=0 ||
                        mapper.boundaries.unknown_cell_vec.size()!=0) {
                    // if (mapper.pose_updated){
                        ROS_DEBUG("[Step] Updating starting point...");
                        dstar->updateStart(mapper.robot_state.robot_pose.getOrigin().x() * mapper.grid_to_costmap_ratio, 
                                           mapper.robot_state.robot_pose.getOrigin().y() * mapper.grid_to_costmap_ratio);
                        ROS_DEBUG("[Step] Updating the costmap for the planner...");
                        mapper.updateDstarMap(dstar); // update costmap
                        ROS_DEBUG("[Step] Updated");

                        // replanning
                        ROS_DEBUG("[Step] Replanning...");
                        std::atomic_bool cancellation_token(false);
                        std::future<bool> future = std::async(std::launch::async, 
                                                              &Dstar::replan, dstar, 
                                                              std::ref(cancellation_token));
                        std::future_status status = future.wait_for(std::chrono::duration<double>(mapper.replannning_timeout)); 

                        if ( status == std::future_status::deferred ) {
                            ROS_WARN("[Replan] Deferred");
                            // continue;
                        }
                        else if ( status == std::future_status::timeout ) {
                            ROS_WARN("[Replan] Time out");
                            cancellation_token = true;
                            // continue;
                        }
                        else if ( status == std::future_status::ready ) {
                            if (!future.get()){
                                ROS_WARN("[Replan] NO PATH TO GOAL");
                                mapper.no_path=true;
                                // continue;
                            }
                            else {
                                ROS_INFO("[Replan] Replaned");
                            }
                        }

                        // if (!dstar->replan()){
                        //     ROS_WARN("[Replan] NO PATH TO GOAL");
                        // }

                        // if (!dstar->replan()){
                        //     ROS_WARN("[Replan] NO PATH TO GOAL");
                        // }
                        ROS_DEBUG_STREAM("[Cell] Free cells: " << mapper.boundaries.free_cell_vec.size());
                        ROS_DEBUG_STREAM("[Cell] Occupied cells: " << mapper.boundaries.occupied_cell_vec.size());
                        ROS_DEBUG_STREAM("[Cell] Unknown cells: " << mapper.boundaries.unknown_cell_vec.size());
                        ROS_DEBUG_STREAM("[Cell] Total cells: " << mapper.total_cost_map.size());

                        // if (!cancellation_token && !mapper.no_path){
                        if (!cancellation_token){
                            ROS_DEBUG("[Step] Getting a path from the planner...");
                            listpath = dstar->getPath();
                            ROS_DEBUG("[Step] Converting the path...");
                            vPath = {std::begin(listpath), std::end(listpath)};
                            vPath_old = vPath;
                            ROS_INFO_STREAM("[Path] Waypoints: " << vPath.size());

                            ROS_DEBUG("[Step] Preparing for publishing...");
                            PublishTypes_t s_path = mapper.publishPath(vPath);
                            if (s_path.s_waypoints_udp.size()!=0){
                                unsigned char *c_msgs = new unsigned char [WAYPOINT_DATA_T_PACKED_LEN+1];
                                pack_waypoint_data_t(&s_path.waypoints_udp, c_msgs);
                                result = sendto(sock, c_msgs, WAYPOINT_DATA_T_PACKED_LEN+1, 0, (sockaddr*)&addrDest, sizeof(addrDest));
                                delete c_msgs;
                                ROS_INFO_STREAM("[UDP] transmitted " << result << " bytes");
                                new_path = true;
                            }
                            else {
                                ROS_WARN("[Replan] NO AVAILABLE PATH");
                                mapper.no_path=true;
                                new_path = false;
                            }
                        }
                        else{
                            new_path = false;
                            if (vPath_old.size() != 0){
                                ROS_WARN("[Step] Using old path");
                                PublishTypes_t s_path = mapper.publishPath(vPath_old);

                                if (s_path.s_waypoints_udp.size()!=0){
                                    unsigned char *c_msgs = new unsigned char [WAYPOINT_DATA_T_PACKED_LEN+1];
                                    pack_waypoint_data_t(&s_path.waypoints_udp, c_msgs);
                                    result = sendto(sock, c_msgs, WAYPOINT_DATA_T_PACKED_LEN+1, 0, (sockaddr*)&addrDest, sizeof(addrDest));
                                    delete c_msgs;
                                    ROS_INFO_STREAM("[UDP] transmitted " << result << " bytes");
                                }
                            }
                        }
                    }
                }
                else {
                    // for the first run, if a map is not updated
                    // do not preceed
                    if (update_count==0){
                        continue;
                    }
                }


                /*
                if (mapper.pose_updated ||
                    mapper.boundaries.free_cell_vec.size()!=0 ||
                    mapper.boundaries.occupied_cell_vec.size()!=0 ||
                    mapper.boundaries.unknown_cell_vec.size()!=0) {
                // if (mapper.pose_updated){
                    ROS_DEBUG("[Step] Updating starting point...");
                    dstar->updateStart(mapper.robot_state.robot_pose.getOrigin().x() * mapper.grid_to_costmap_ratio, 
                                       mapper.robot_state.robot_pose.getOrigin().y() * mapper.grid_to_costmap_ratio);
                    ROS_DEBUG("[Step] Updating the costmap for the planner...");
                    mapper.updateDstarMap(dstar); // update costmap
                    ROS_DEBUG("[Step] Updated");

                    // replanning
                    ROS_DEBUG("[Step] Replanning...");
                    std::atomic_bool cancellation_token(false);
                    std::future<bool> future = std::async(std::launch::async, 
                                                          &Dstar::replan, dstar, 
                                                          std::ref(cancellation_token));
                    std::future_status status = future.wait_for(std::chrono::duration<double>(mapper.replannning_timeout)); 

                    if ( status == std::future_status::deferred ) {
                        ROS_WARN("[Replan] Deferred");
                        // continue;
                    }
                    else if ( status == std::future_status::timeout ) {
                        ROS_WARN("[Replan] Time out");
                        cancellation_token = true;
                        // continue;
                    }
                    else if ( status == std::future_status::ready ) {
                        if (!future.get()){
                            ROS_WARN("[Replan] NO PATH TO GOAL");
                            mapper.no_path=true;
                            // continue;
                        }
                        else {
                            ROS_INFO("Replaned");
                        }
                    }

                    // if (!dstar->replan()){
                    //     ROS_WARN("[Replan] NO PATH TO GOAL");
                    // }

                    // if (!dstar->replan()){
                    //     ROS_WARN("[Replan] NO PATH TO GOAL");
                    // }
                    ROS_INFO_STREAM(mapper.boundaries.free_cell_vec.size() << " new free cells");
                    ROS_INFO_STREAM(mapper.boundaries.occupied_cell_vec.size() << " new occupied cells");
                    ROS_INFO_STREAM(mapper.boundaries.unknown_cell_vec.size() << " new unknown cells");
                    ROS_INFO_STREAM(mapper.total_cost_map.size() << " total cells");

                    // if (!cancellation_token && !mapper.no_path){
                    if (!cancellation_token){
                        ROS_DEBUG("[Step] Getting a path from the planner...");
                        listpath = dstar->getPath();
                        ROS_DEBUG("[Step] Converting the path...");
                        std::vector<state> vPath {std::begin(listpath), std::end(listpath)};
                        ROS_INFO_STREAM(vPath.size() << " waypoints");

                        ROS_DEBUG("[Step] Preparing for publishing...");
                        PublishTypes_t s_path = mapper.publishPath(vPath);
                        if (s_path.s_waypoints_udp.size()!=0){
                            unsigned char *c_msgs = new unsigned char [WAYPOINT_DATA_T_PACKED_LEN+1];
                            pack_waypoint_data_t(&s_path.waypoints_udp, c_msgs);
                            result = sendto(sock, c_msgs, WAYPOINT_DATA_T_PACKED_LEN+1, 0, (sockaddr*)&addrDest, sizeof(addrDest));
                            delete c_msgs;
                            ROS_INFO_STREAM(result << " bytes sent");
                            dstar_utils::drawDstar(vPath, mapper.marker_array_pub, 
                                                   mapper.grid_to_costmap_ratio, 
                                                   string("original D*"), 1, 0, 0,
                                                   mapper.waypoint_distance);
                        }
                        else {
                            ROS_WARN("NO AVAILABLE PATH");
                            mapper.no_path=true;
                        }
                    }

                    // exit(0);


                    // publish path

                    // std::string s_path = mapper.publishPath(vPath);
                    // if (s_path.size()!=0){
                    //     char *c_msgs = new char [s_path.length()+1];
                    //     strcpy (c_msgs, s_path.c_str());
                    //     // cout << "returned: " << c_msgs << endl;
                    //     size_t msg_length = strlen(c_msgs);
                    //     result = sendto(sock, c_msgs, msg_length, 0, (sockaddr*)&addrDest, sizeof(addrDest));
                    //     delete c_msgs;
                    //     ROS_INFO_STREAM(result << " bytes sent");

                    //     dstar_utils::drawDstar(vPath, mapper.marker_array_pub, mapper.grid_to_costmap_ratio, 
                    //                            string("original D*"), 1, 0, 0);
                    // }
                    // else {
                    //     ROS_WARN("NO PATH");
                    //     mapper.no_path=true;
                    // }
                }
                */
                // dstar_utils::assignMarker(mapper.marker_pub, mapper.goals.left_goal1->goal.x,
                //                           mapper.goals.left_goal1->goal.y, 0.5,
                //                           visualization_msgs::Marker::SPHERE, 
                //                           "left goal 1",255/255.0,20/255.0,147/255.0, 
                //                           1, 0.5);

                // dstar_utils::assignMarker(mapper.marker_pub, mapper.goals.right_goal1->goal.x,
                //                           mapper.goals.right_goal1->goal.y, 0.5,
                //                           visualization_msgs::Marker::SPHERE, 
                //                           "right goal 1", 0/255.0,255/255.0,127/255.0, 
                //                           1, 0.5);

                // dstar_utils::assignMarker(mapper.marker_pub, mapper.goals.left_goal2.goal.x,
                //                           mapper.goals.left_goal2.goal.y, 0.5,
                //                           visualization_msgs::Marker::SPHERE, 
                //                           "left goal 2", 255/255.0,192/255.0,203/255.0, 
                //                           1, 0.5);

                // dstar_utils::assignMarker(mapper.marker_pub, mapper.goals.right_goal2.goal.x,
                //                           mapper.goals.right_goal2.goal.y, 0.5,
                //                           visualization_msgs::Marker::SPHERE, 
                //                           "right goal 2", 60/255.0,179/255.0,113/255.0, 
                //                           1, 0.5);

                dstar_utils::drawCellNoText(mapper.circle, 
                                            mapper.marker_array_pub,
                                            1, string("circle"), 
                                            123/255.0, 104/255.0, 238/255.0, 0.1, 
                                            0.5, visualization_msgs::Marker::SPHERE);

                dstar_utils::drawCellNoText(mapper.free_circle, 
                                            mapper.marker_array_pub,
                                            1, string("free_circle"), 
                                            0/255.0, 255/255.0, 127/255.0, 1, 
                                            0.1, visualization_msgs::Marker::SPHERE);

                dstar_utils::drawCellNoText(mapper.occupied_circle, 
                                            mapper.marker_array_pub,
                                            1, string("occupied_circle"), 
                                            123/255.0, 104/255.0, 238/255.0, 1, 
                                            0.1, visualization_msgs::Marker::SPHERE);

                dstar_utils::drawSegments(mapper.goal_segments, 
                                          mapper.marker_array_pub,
                                          1, string("goal_segments"), 
                                          255/255.0, 255/255.0, 0/255.0, 1, 
                                          0.2, visualization_msgs::Marker::SPHERE);

                // dstar_utils::assignMarker(mapper.marker_pub, mapper.goal_left.goal.x,
                //                           mapper.goal_left.goal.y, 0.5,
                //                           visualization_msgs::Marker::CUBE, 
                //                           "left goal", 0, 0.5, 0.5, 
                //                           1, 0.5);

                // dstar_utils::assignMarker(mapper.marker_pub, mapper.goal_right.goal.x,
                //                           mapper.goal_right.goal.y, 0.5,
                //                           visualization_msgs::Marker::CUBE, 
                //                           "right goal", 0.5, 0.5, 0, 
                //                           1, 0.5);

                dstar_utils::drawCellNoText(mapper.boundaries.boundaries_cell_vec, 
                                            mapper.marker_array_pub,
                                            1, string("boundaries cells"), 1, 0, 0, 1, 
                                            0.5, visualization_msgs::Marker::SPHERE);

                // draw robot
                dstar_utils::assignMarker(mapper.marker_pub, 
                                          mapper.robot_state.robot_pose.getOrigin().x(),
                                          mapper.robot_state.robot_pose.getOrigin().y(),
                                          0.5, visualization_msgs::Marker::ARROW, 
                                          "robot", 1, 1, 1, 
                                          1, 0.5,
                                          mapper.robot_state.robot_pose.getRotation().x(), 
                                          mapper.robot_state.robot_pose.getRotation().y(),
                                          mapper.robot_state.robot_pose.getRotation().z(),
                                          mapper.robot_state.robot_pose.getRotation().w());

                // draw goal
                dstar_utils::assignMarker(mapper.marker_pub, mapper.goal_point.x,
                                          mapper.goal_point.y, 0.5,
                                          visualization_msgs::Marker::CUBE, 
                                          "goal", 1, 1, 0, 
                                          1, 0.5);
                if (new_path){
                    dstar_utils::drawDstar(vPath, mapper.marker_array_pub, 
                                           mapper.grid_to_costmap_ratio, 
                                           string("original D*"), 1, 0, 0,
                                           mapper.waypoint_distance);
                }
                else {
                    dstar_utils::drawDstar(vPath_old, mapper.marker_array_pub, 
                                           mapper.grid_to_costmap_ratio, 
                                           string("original D*"), 0.5, 0, 0,
                                           mapper.waypoint_distance);
                }

                // dstar_utils::drawCellNoText(mapper.boundaries.current_cell_vec,
                //                             mapper.marker_array_pub,
                //                             1, string("cost cells"), 1, 1, 1, 1, 
                //                             1, visualization_msgs::Marker::SPHERE);

                dstar_utils::drawCellText(mapper.boundaries.current_cell_vec, 
                                          mapper.marker_array_pub,
                                          1, string("cost text"), 0, 1, 1, 1, 
                                          0.1); 
                // exit(0);

                if (mapper.pub_free){
                    dstar_utils::drawCell(mapper.boundaries.free_cell_vec, mapper.marker_array_pub, 
                                      mapper.grid_to_costmap_ratio, string("new free cells"), 0, 1, 0, 0, true);
                }

                if (mapper.pub_occupied){
                    dstar_utils::drawCell(mapper.boundaries.occupied_cell_vec, mapper.marker_array_pub, 
                                          mapper.grid_to_costmap_ratio, string("new occupied cells"), 1, 0, 0, 0.2, true);
                }

                if (mapper.pub_unknown){
                    dstar_utils::drawCell(mapper.boundaries.unknown_cell_vec, mapper.marker_array_pub, 
                                          mapper.grid_to_costmap_ratio, string("new unknown cells"), 1, 1, 0, 0.2, true);
                }

                if (mapper.pub_all_cells){
                    dstar_utils::drawUnorderedSet(mapper.total_cost_map, mapper.marker_array_pub, mapper.grid_to_costmap_ratio, 
                                                  mapper.occupied_cell, mapper.unknown_cell,
                                                  string("all cells"), true);
                }
            }
            std::chrono::high_resolution_clock::time_point t2 = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double> time_spent = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
            clock_time = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - clock);

            if (update){
                ROS_INFO_STREAM("[Time] Computation: " << 1/time_spent.count() << " Hz");
                update = false;
            }

            if (clock_time.count() > mapper.update_time){
                // ROS_INFO(" Will update");
                update = true;
                clock = std::chrono::high_resolution_clock::now();
            }
            r.sleep();
        }
        ROS_INFO("FINISHED");
    }
    // ROS_INFO("GOAL REACH!");
    return 0;
}
