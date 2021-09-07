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
#include <cstring>
#include <utility>

#include "communication.h"
#include "control_commands.h"
#include <cstring>
#include "controller_info_to_planner_t.h"
#include "utils/timing.h"

namespace bipedlab
{

Communication::Communication(const std::string& sender_ip, 
                             const std::string& sender_port, 
                             const std::string& receiver_port)
{
    receiver_ = udp_socket_t(receiver_port);

    sender_ = Communication::createUDPSender_(sender_ip, sender_port);
    // debugger::debugTextOutput("[communication] UDP socket successfully created", 4);

    // boost::thread listening(&Communication::createListener_, this);
}


Communication::Communication(const communication_t& udp_info)
{
    receiver_ = udp_socket_t(udp_info.port_to_receive);

    sender_ = Communication::createUDPSender_(udp_info.ip_to_send, 
                                              udp_info.port_to_send);

    // boost::thread listening(&Communication::createListener_, this);
    // debugger::debugTextOutput("[Communication] UDP socket successfully created", 5);
}

Communication::~Communication() { }

void Communication::createListener_(void)
{
    // int max_line = 1024;
    // receiver_.addr_sender.sin_port = htons(std::stoi(receiver_.port));
    // receiver_.addr_sender.sin_port = htons(std::stoi(receiver_.port));
    // receiver_.status = bind(receiver_.sock_fd, (sockaddr*)&(receiver_.addr_sender), 
    //                      sizeof(receiver_.addr_sender));

    // if (receiver_.status < 0) {
    //     int lasterror = errno;
    //     debugger::debugCerr("[Communication] socket binding failed: ", lasterror, 5);
    // }

    // unsigned int len = sizeof(receiver_.addr_receiver);
    // char buffer[max_line];
    // debugger::debugTextOutput("[communication] Listing...", 4);
    // while(1)
    // {
    //     int n = recvfrom(receiver_.sock_fd, (char *)buffer, max_line,
    //             MSG_WAITALL, ( struct sockaddr *) &receiver_.addr_receiver,
    //             &len);
    //     debugger::debugTextOutput("[communication] received...", 4);
    // }

    int max_line = CONTROLLER_INFO_TO_PLANNER_T_PACKED_LEN;
    receiver_.addr_sender.sin_port = htons(std::stoi(receiver_.port));
    int sockfd;
    unsigned char buffer[max_line];
    struct sockaddr_in servaddr, cliaddr;

    // Creating socket file descriptor
    if ( (sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0 ) {
        perror("[Communication] listening socket creation failed");
        exit(EXIT_FAILURE);
    }

    memset(&servaddr, 0, sizeof(servaddr));
    memset(&cliaddr, 0, sizeof(cliaddr));

    // Filling server information
    servaddr.sin_family    = AF_INET; // IPv4
    servaddr.sin_addr.s_addr = INADDR_ANY;
    servaddr.sin_port = receiver_.addr_sender.sin_port;

    // Bind the socket with the server address
    if ( bind(sockfd, (const struct sockaddr *)&servaddr,
                sizeof(servaddr)) < 0 )
    {
        perror("[Communication] Listening Bind Failed");
        exit(EXIT_FAILURE);
    }

    unsigned int len;
    int n;

    len = sizeof(cliaddr);  //len is value/resuslt
        ros::Time time = ros::Time::now();
    tf::Transform robot_pose;
    robot_pose.setIdentity();
    tf::StampedTransform tf(robot_pose, time, "map", "robot");

    debugger::debugTextOutput("[communication]/[Listener] UDP receiver created!", 5);

        // testing only
    controller_msg_.pose[0] = -5;
    controller_msg_.pose[1] = 1;
    controller_msg_.pose[2] = 0;
    controller_msg_.pose[3] = 1;
    controller_msg_.pose[4] = 0;
    controller_msg_.pose[5] = 0;
    controller_msg_.pose[6] = 0;

    controller_info_to_planner_t *received_msg = new controller_info_to_planner_t;
    while(1)
    {
        n = recvfrom(sockfd, (unsigned char *)buffer, max_line,
                MSG_WAITALL, ( struct sockaddr *) &cliaddr, &len);
        // debugger::debugOutput("[communication]/[Listener] Received UDP msg! At time: ",
        //         std::to_string(timing::getCurrentCPUTime()), 3);
        // debugger::debugOutput("[communication] n:", n, 4);
        debugger::debugTextOutput("[communication]/[Listener] Newed memory", 1);


        unpack_controller_info_to_planner_t(buffer, received_msg);
        receiver_lock_.lock();
        // controller_msg_ = *received_msg;
        debugger::debugTextOutput("[communication]/[Listener] assigned status", 1);
        receiver_.status = 1;
        receiver_lock_.unlock();
        

        // debugger::debugTextOutput("[communication]/[Listener] deleted pointer", 3);
        // debugger::debugOutput("[communication] CoM[0]: ",controller_msg->p_com[0], 4);
        // debugger::debugOutput("[communication] CoM[1]: ",controller_msg->p_com[1], 4);
        // debugger::debugOutput("[communication] CoM[2]: ",controller_msg->p_com[2], 4);
        // debugger::debugOutput("[communication] st_foot[0]: ",controller_msg->p_st_foot[0], 4);
        // debugger::debugOutput("[communication] st_foot[1]: ",controller_msg->p_st_foot[1], 4);
        // debugger::debugOutput("[communication] st_foot[2]: ",controller_msg->p_st_foot[2], 4);
        // Communication::publishToRobot(tf, controller_msg->p_com[2], 0.5);
        // debugger::debugTextOutput("[communication] published!", 4);

        // for (int i = 0; i < 6; i++) {
        //     float tmp;
        //     memcpy(&tmp, &buffer[i*4], 4 * sizeof(unsigned char));
        //     std::cout << "[" << i << "] = " << tmp << std::endl;
        // }
        // buffer[n] = '\0';
        // printf("Client : %s\n", buffer);
    }
    delete received_msg;
}

udp_socket_t 
Communication::createUDPSender_(const std::string& ip, const std::string& port)
{
    udp_socket_t sender = udp_socket_t(ip, port);
    sender.sock_fd = socket(AF_INET, SOCK_DGRAM, 0);


    // Creating socket file descriptor
    if (sender.sock_fd < 0 ) {
        debugger::debugTextCerr("[Communication] socket creation failed!", 10);
    }

    memset(&sender.addr_sender, 0, sizeof(sender.addr_sender));
    memset(&sender.addr_receiver, 0, sizeof(sender.addr_receiver));

    // Filling server information
    sender.addr_sender = {}; // zero-int, sin_port is 0, which picks a random port for bind.
    sender.addr_sender.sin_family = AF_INET;
    // sender.addr_sender.sin_port = htons(std::stoi(port));
    // servaddr.sin_addr.s_addr = INADDR_ANY;

    sender.status = bind(sender.sock_fd, (sockaddr*)&(sender.addr_sender), 
                         sizeof(sender.addr_sender));

    if (sender.status < 0) {
        int lasterror = errno;
        debugger::debugCerr("[Communication] socket binding failed: ", lasterror, 5);
    }

    sender.status = Communication::resolveUDPHelper_(sender);

    debugger::debugTextOutput("[Communication] UDP sender created", 5);
    return sender;
}


int Communication::resolveUDPHelper_(udp_socket_t& sender)
{
    addrinfo* status_list = NULL;
    addrinfo hints = {};
    hints.ai_family = sender.addr_sender.sin_family;
    hints.ai_socktype = SOCK_DGRAM; // without this flag, getaddrinfo will 
                                    // return 3x the number of addresses 
                                    // (one for each socket type).
    int status = getaddrinfo(sender.ip.c_str(), sender.port.c_str(), 
                             &hints, &status_list);

    if (status == 0)
    {
        //ASSERT(status_list->ai_addrlen <= sizeof(sockaddr_in));
        memcpy(&(sender.addr_dest), status_list->ai_addr, status_list->ai_addrlen);
        freeaddrinfo(status_list);
    }
    else 
    {
        int lasterror = errno;
        debugger::debugCerr("[Communication] Unable to get address info: ", 
                lasterror, 5);
    }

    return status;
}

// int Communication::createUDPSender_(const char* ip, const char* port)
// {
//     sender_.sock_ = socket(AF_INET, SOCK_DGRAM, 0);
//     if (sock_ < 0)
//     {
//         debugger::debugTextCerr("[Communication] socket creation failed!", 5);
//     }
//     sender_.addr_sender_ = {}; // zero-int, sin_port is 0, which picks a random port for bind.
//     sender_.addr_sender_.sin_family = AF_INET;
//     sender_.status_ = bind(sock_, (sockaddr*)&addr_sender_, sizeof(addr_sender_));
// 
//     if (sender_.status_ == -1) {
//         int lasterror = errno;
//         debugger::debugCerr("[Communication] socket binding failed: ", lasterror, 5);
//     }
// 
//     sender_.addr_dest_ = {};
//     sender_.status_ = Communication::resolveUDPHelper_(
//             ip, AF_INET, port, &sender_.addr_dest_);
// }
// 
// int Communication::resolveUDPHelper_(const char* hostname, int family, 
//                                      const char* service, sockaddr_storage* pAddr) 
// {
//     int status;
//     addrinfo* status_list = NULL;
//     addrinfo hints = {};
//     hints.ai_family = family;
//     hints.ai_socktype = SOCK_DGRAM; // without this flag, getaddrinfo will return 3x the number of addresses (one for each socket type).
//     status = getaddrinfo(hostname, service, &hints, &status_list);
//     if (status == 0)
//     {
//         //ASSERT(status_list->ai_addrlen <= sizeof(sockaddr_in));
//         memcpy(pAddr, status_list->ai_addr, status_list->ai_addrlen);
//         freeaddrinfo(status_list);
//     }
//     else 
//     {
//         int lasterror = errno;
//         debugger::debugCerr("[Communication] Unable to get address info: ", 
//                 lasterror, 5);
//     }
// 
//     return status;
// }
//


// planner_info_to_controller_t
// Communication::assignWalkInPlace(const inekf_msgs::State& inekf_state)
// {
//     return Communication::assignInfo(
//             0, // behavior
//             0, 0, 0, 0, // control 
//             nullptr, nullptr, //
//             inekf_state.pose.position.x,
//             inekf_state.pose.position.y,
//             inekf_state.pose.position.z,
//             inekf_state.pose.orientation.w,
//             inekf_state.pose.orientation.x,
//             inekf_state.pose.orientation.y,
//             inekf_state.pose.orientation.z);
// }
// 
// planner_info_to_controller_t
// Communication::assignInfo(
//         const double& behavior,
//         const double& speed, 
//         const double& roll, const double& pitch, const double& yaw,
//         const std::vector<point2d_t<double>>* path_points_ptr,
//         const std::vector<double>* foot_placement,
//         const double& x, const double& y, const double& z,
//         const double& qw, const double& qx, const double& qy, const double& qz)
// {
//     planner_info_to_controller_t planner_info_to_controller_udp;
//     memset(&planner_info_to_controller_udp, 0, sizeof(planner_info_to_controller_t));
// 
//     // assign behavior
//     planner_info_to_controller_udp.behavior[0] = behavior;
// 
//     // assign control commands
//     planner_info_to_controller_udp.speed[0] = speed;
//     planner_info_to_controller_udp.roll[0] = roll;
//     planner_info_to_controller_udp.pitch[0] = pitch;
//     planner_info_to_controller_udp.yaw[0] = yaw;
// 
//     // assign waypoint to follow (nothing for now)
//     if (path_points_ptr != nullptr)
//     {
//         for (const auto& point : *path_points_ptr)
//         {
// 
//         }
//     }
// 
// 
//     // assign foot placement (nothing for now)
//     if (foot_placement != nullptr)
//     {
//         for (const auto& foot_pos : *foot_placement)
//         {
// 
//         }
//     }
// 
// 
//     // assign pose
//     planner_info_to_controller_udp.pose[0] = x;
//     planner_info_to_controller_udp.pose[1] = y;
//     planner_info_to_controller_udp.pose[2] = z;
//     planner_info_to_controller_udp.pose[3] = qw;
//     planner_info_to_controller_udp.pose[4] = qx;
//     planner_info_to_controller_udp.pose[5] = qy;
//     planner_info_to_controller_udp.pose[6] = qz;
// 
// 
//     return planner_info_to_controller_udp;
// 
// 
// 
//     // size_t precision = 3;
//     // planner_info_to_controller_t planner_info_to_controller_udp;
//     // memset(&planner_info_to_controller_udp, 0, sizeof(planner_info_to_controller_t));
// 
//     // // assign behavior
//     // planner_info_to_controller_udp.behavior[0] = 
//     //     std::stod(utils::toStringWithPrecision(behavior, precision));
// 
//     // // assign control commands
//     // planner_info_to_controller_udp.speed[0] = 
//     //     std::stod(utils::toStringWithPrecision(speed, precision));
//     // planner_info_to_controller_udp.roll[0] = 
//     //     std::stod(utils::toStringWithPrecision(roll, precision));
//     // planner_info_to_controller_udp.pitch[0] = 
//     //     std::stod(utils::toStringWithPrecision(pitch, precision));
//     // planner_info_to_controller_udp.yaw[0] = 
//     //     std::stod(utils::toStringWithPrecision(yaw, precision));
// 
//     // // assign waypoint to follow (nothing for now)
//     // if (path_points_ptr != nullptr)
//     // {
//     //     for (const auto& : *path_points_ptr)
//     //     {
// 
//     //     }
//     // }
// 
// 
//     // // assign foot placement (nothing for now)
//     // if (foot_placement != nullptr)
//     // {
//     //     for (const auto& : *foot_placement)
//     //     {
// 
//     //     }
//     // }
// 
// 
//     // // assign pose
//     // planner_info_to_controller_udp.pose[0] = 
//     //     std::stod(utils::toStringWithPrecision(x, precision));
//     // planner_info_to_controller_udp.pose[1] = 
//     //     std::stod(utils::toStringWithPrecision(y, precision));
//     // planner_info_to_controller_udp.pose[2] = 
//     //     std::stod(utils::toStringWithPrecision(z, precision));
//     // planner_info_to_controller_udp.pose[3] = 
//     //     std::stod(utils::toStringWithPrecision(qw, precision));
//     // planner_info_to_controller_udp.pose[4] = 
//     //     std::stod(utils::toStringWithPrecision(qx, precision));
//     // planner_info_to_controller_udp.pose[5] = 
//     //     std::stod(utils::toStringWithPrecision(qy, precision));
//     // planner_info_to_controller_udp.pose[6] = 
//     //     std::stod(utils::toStringWithPrecision(qz, precision));
// }


// publish_types_t 
// Communication::convertToRobotCommunicationType_(
//         const double& behavior,
//         const double& speed, 
//         const double& roll, const double& pitch, const double& yaw,
//         const std::vector<point2d_t<double>>* path_points_ptr,
//         const std::vector<double>* foot_placement,
//         const double& x, const double& y, const double& z,
//         const double& qw, const double& qx, const double& qy, const double& qz)
// {
//     publish_data.planner_info_to_controller_udp = Communication::assignInfo(
//             behavior, speed, roll, pitch, yaw, path_points_ptr, foot_placement,
//             x, y, z, qw, qx, qy, qz);
//     publish_data.s_planner_info_to_controller_udp = "[Communication]: " + 
//         Communication::returnPublishData(publish_data.planner_info_to_controller_udp);
// 
// 
//     return publish_data;
// }



publish_types_t 
Communication::convertToRobotCommunicationTypeReduced_(
        const double& x, const double& y, const double& z,
        const double& qw, const double& qx, const double& qy, const double& qz,
        const double& speed, const double& heading)
{
    // if (speed == 0 && heading == 0){
    //     publish_types_t publish_string;
    //     publish_string.s_waypoints_udp = "";
    //     return publish_string;
    // }



    size_t precision = 3;
    planner_info_to_controller_t planner_info_to_controller_udp;
    memset(&planner_info_to_controller_udp, 0, sizeof(planner_info_to_controller_t));

    // if (!no_path && !no_free_cell && !no_goal){
    //     planner_info_to_controller_udp.controlsignal[0] = m_control_signals.normal;
    // }
    // else {
    //     planner_info_to_controller_udp.controlsignal[0] = 1;
    // }

    // planner_info_to_controller_udp.controlsignal[1] = 0;
    // planner_info_to_controller_udp.controlsignal[2] = 0;
    // planner_info_to_controller_udp.controlsignal[3] = 0;
    // planner_info_to_controller_udp.controlsignal[4] = 0;
    // planner_info_to_controller_udp.controlsignal[5] = 0;
    // planner_info_to_controller_udp.controlsignal[6] = 0;

    planner_info_to_controller_udp.pose[0] = std::stod(utils::toStringWithPrecision(x, precision));
    planner_info_to_controller_udp.pose[1] = std::stod(utils::toStringWithPrecision(y, precision));
    planner_info_to_controller_udp.pose[2] = std::stod(utils::toStringWithPrecision(z, precision));
    planner_info_to_controller_udp.pose[3] = std::stod(utils::toStringWithPrecision(qw, precision));
    planner_info_to_controller_udp.pose[4] = std::stod(utils::toStringWithPrecision(qx, precision));
    planner_info_to_controller_udp.pose[5] = std::stod(utils::toStringWithPrecision(qy, precision));
    planner_info_to_controller_udp.pose[6] = std::stod(utils::toStringWithPrecision(qz, precision));

    // planner_info_to_controller_udp.pose[1] =0;
    // planner_info_to_controller_udp.pose[2] =0;
    // planner_info_to_controller_udp.pose[3] =0;
    // planner_info_to_controller_udp.pose[4] =0;
    // planner_info_to_controller_udp.pose[5] =0;
    // planner_info_to_controller_udp.pose[6] =0;
    // ROS_DEBUG_STREAM ("[wp] robot x: " << std::stod(dstar_utils::toStringWithPrecision(robot_pose.getOrigin().x(), 3)));
    // ROS_DEBUG_STREAM ("[wp] robot y: " << std::stod(dstar_utils::toStringWithPrecision(robot_pose.getOrigin().y(), 3)));

    // if (!this->no_path && !this->no_free_cell && !this->no_goal){
    // if (this->no_goal){
    //     planner_info_to_controller_udp.pose[0] = m_control_signals.backward;
    //     ROS_DEBUG_STREAM("[UDP] Backward: " << planner_info_to_controller_udp.pose[0]);
    // }
    // else {
    //     planner_info_to_controller_udp.pose[0] = m_control_signals.normal;;
    //     ROS_DEBUG_STREAM("[UDP] Normal: " << planner_info_to_controller_udp.pose[0]);
    // }

    // speed
    planner_info_to_controller_udp.waypoints[0] = std::stod(utils::toStringWithPrecision(speed, precision));

    // heading
    planner_info_to_controller_udp.waypoints[1] = std::stod(utils::toStringWithPrecision(heading, precision));


    size_t string_precision = 3;
    std::string s_robot_pose = utils::toStringWithPrecision(x, string_precision) +
        std::string(", ") +
        utils::toStringWithPrecision(y, string_precision) +
        std::string(", ") +
        utils::toStringWithPrecision(z, string_precision) +
        std::string(", ") +
        utils::toStringWithPrecision(qw, string_precision) +
        std::string(", ") +
        utils::toStringWithPrecision(qx, string_precision) +
        std::string(", ") +
        utils::toStringWithPrecision(qy, string_precision) +
        std::string(", ") +
        utils::toStringWithPrecision(qz, string_precision) +
        std::string(", ");
    std::string s_velocity = utils::toStringWithPrecision(speed, string_precision) + 
                             std::string(", ") + 
                             utils::toStringWithPrecision(heading, string_precision);


    publish_types_t publish_string;
    std::string msgs = s_robot_pose + s_velocity;
    publish_string.planner_info_to_controller_udp = planner_info_to_controller_udp;
    publish_string.s_planner_info_to_controller_udp = msgs;

    debugger::debugOutput("[communication] robot_pose: ", s_robot_pose, 2);
    debugger::debugOutput("[communication] velocity: ", s_velocity, 2);
    debugger::debugOutput("[communication] msgs: ", msgs, 2);


    return publish_string;
}




size_t Communication::publishToRobot_(const publish_types_t& info)
{
    // std::string s_test = "Hey";
    // char *c_msgs = new char [s_test.length()+1];
    // strcpy (c_msgs, s_test.c_str());
    // size_t msg_length = strlen(c_msgs);
    // debugger::debugOutput("[Communication] c_msgs:", c_msgs, 4);
    // debugger::debugOutput("[Communication] length:", msg_length, 4);
    // status_ = sendto(sock_, c_msgs, msg_length, 0, 
    //                 (sockaddr*)&addr_sender_, sizeof(addr_sender_));


    unsigned char *c_msgs = new unsigned char [PLANNER_INFO_TO_CONTROLLER_T_PACKED_LEN + 1];
    // debugger::debugOutput("[Communication] about to pack", "", 5);
    pack_planner_info_to_controller_t(&info.planner_info_to_controller_udp, c_msgs);
    // debugger::debugOutput("[Communication] packed", "", 5);
    // for (int i = 0; i < 188; i++)
    // {
    //     std::cout << (int) c_msgs[i] << " ";
    // }
    // for (int i = 0; i < 47; i++) {
    //     float tmp;
    //     memcpy(&tmp, &c_msgs[i*4], 4 * sizeof(unsigned char));
    //     std::cout << "[" << i << "] = " << tmp << std::endl;
    // }
    
    sender_.status = sendto(sender_.sock_fd, c_msgs, PLANNER_INFO_TO_CONTROLLER_T_PACKED_LEN + 1, 
            0, (sockaddr*)&sender_.addr_dest, sizeof(sender_.addr_dest));
    // debugger::debugOutput("[Communication] type:", info, 2);
    // debugger::debugOutput("[communication] transmitted [bytes]: ", sender_.status, 2);
    delete c_msgs;


    return sender_.status;

    // PublishTypes_t s_path = mapper.publishPath(vPath);
    // if (s_path.s_planner_info_to_controller_udp.size()!=0){
    //     unsigned char *c_msgs = new unsigned char [WAYPOINT_DATA_T_PACKED_LEN+1];
    //     pack_waypoint_data_t(&s_path.planner_info_to_controller_udp, c_msgs);
    //     result = sendto(sock, c_msgs, WAYPOINT_DATA_T_PACKED_LEN+1, 0, (sockaddr*)&addrDest, sizeof(addrDest));
    //     delete c_msgs;
    //     debugger::debugOutput("[communication] transmitted [bytes]: ", result, 4);
    //     return true;
    // }
    // else {
    //     debugger::debugTextOutput("[communication] NO AVAILABLE PATH", 4);
    //     return false;
    // }
}

bool Communication::getUDPStatus_(size_t byte_sent)
{
    if (byte_sent)
    {
        // debugger::debugOutput("[Communication] transmitted [bytes]: ", 
        //                       byte_sent, 3);
        return true;
    }
    else
    {
        debugger::debugColorTextOutput("[Communication] Failed to send data!", 5, 
                BR, BOLD);
        return false;
    }
}


bool Communication::publishToRobotFromInEKFStateMsg(
        const inekf_msgs::State& inekf_state, 
        const double& speed,
        const double& heading)
{
    publish_types_t info = Communication::convertToRobotCommunicationTypeReduced_(
                                            inekf_state.pose.position.x,
                                            inekf_state.pose.position.y,
                                            inekf_state.pose.position.z,
                                            inekf_state.pose.orientation.w,
                                            inekf_state.pose.orientation.x,
                                            inekf_state.pose.orientation.y,
                                            inekf_state.pose.orientation.z,
                                            speed, heading);
    size_t byte_sent = Communication::publishToRobot_(info);


    return Communication::getUDPStatus_(byte_sent);
}

bool Communication::publishToRobotFromTFMsg(const tf::StampedTransform& robot_pose, 
                                    const double& speed,
                                    const double& heading)
{
    publish_types_t info =  Communication::convertToRobotCommunicationTypeReduced_(
                                            robot_pose.getOrigin().x(),
                                            robot_pose.getOrigin().y(),
                                            robot_pose.getOrigin().z(),
                                            robot_pose.getRotation().w(),
                                            robot_pose.getRotation().x(),
                                            robot_pose.getRotation().y(),
                                            robot_pose.getRotation().z(),
                                            speed, heading);
    size_t byte_sent = Communication::publishToRobot_(info);


    return Communication::getUDPStatus_(byte_sent);
}


bool Communication::publishToRobotGivenInfoToRobotType(const planner_info_to_controller_t& info)
{
    // debugger::debugOutput("[publishToRobot]", info, 5);
    publish_types_t publish_data;
    memcpy(&publish_data.planner_info_to_controller_udp, &info, sizeof(info));
    publish_data.s_planner_info_to_controller_udp = "[Communication]: " + 
                                       control_commands::returnPublishData(info);
    // debugger::debugOutput("[publishToRobot]", "", 5);
    size_t byte_sent = Communication::publishToRobot_(publish_data);

    return Communication::getUDPStatus_(byte_sent);
}

std::pair<int, controller_info_to_planner_t>
Communication::getReceivedMessage(void)
{
    receiver_lock_.lock();
    controller_info_to_planner_t tmp = controller_msg_;
    // debugger::debugTextOutput("[communication]/[getReceivedMessage] Assigned", 3);
    bool status = receiver_.status;
    receiver_.status = 0;
    receiver_lock_.unlock();
    // debugger::debugTextOutput("[communication]/[getReceivedMessage] about to return", 3);

    return {status, tmp};
}



std::ostream& operator<<(std::ostream& out, const planner_info_to_controller_t& info)
{
    out << control_commands::returnPublishData(info);
    // out << '(' 
    //     << "behavior: " << pub_type.planner_info_to_controller_udp.behavior[0] << ',' 
    //     << "speed: " << pub_type.planner_info_to_controller_udp.speed[0] << ',' 
    //     << "roll: " << pub_type.planner_info_to_controller_udp.roll[0] << ',' 
    //     << "pitch: " << pub_type.planner_info_to_controller_udp.pitch[0] << ',' 
    //     << "yaw: " << pub_type.planner_info_to_controller_udp.yaw[0] << ',' 
    //     << "x: " << pub_type.planner_info_to_controller_udp.pose[0] << ',' 
    //     << "y: " << pub_type.planner_info_to_controller_udp.pose[1] << ',' 
    //     << "z: " << pub_type.planner_info_to_controller_udp.pose[2] << ',' 
    //     << "qw: " << pub_type.planner_info_to_controller_udp.pose[3] << ',' 
    //     << "qx: " << pub_type.planner_info_to_controller_udp.pose[4] << ',' 
    //     << "qy: " << pub_type.planner_info_to_controller_udp.pose[5] << ',' 
    //     << "qz: " << pub_type.planner_info_to_controller_udp.pose[6] << ')' ;
    return out;
}


std::ostream& operator<<(std::ostream& out, const publish_types& pub_type)
{
    out << control_commands::returnPublishData(pub_type.planner_info_to_controller_udp);
    // out << '(' 
    //     << "behavior: " << pub_type.planner_info_to_controller_udp.behavior[0] << ',' 
    //     << "speed: " << pub_type.planner_info_to_controller_udp.speed[0] << ',' 
    //     << "roll: " << pub_type.planner_info_to_controller_udp.roll[0] << ',' 
    //     << "pitch: " << pub_type.planner_info_to_controller_udp.pitch[0] << ',' 
    //     << "yaw: " << pub_type.planner_info_to_controller_udp.yaw[0] << ',' 
    //     << "x: " << pub_type.planner_info_to_controller_udp.pose[0] << ',' 
    //     << "y: " << pub_type.planner_info_to_controller_udp.pose[1] << ',' 
    //     << "z: " << pub_type.planner_info_to_controller_udp.pose[2] << ',' 
    //     << "qw: " << pub_type.planner_info_to_controller_udp.pose[3] << ',' 
    //     << "qx: " << pub_type.planner_info_to_controller_udp.pose[4] << ',' 
    //     << "qy: " << pub_type.planner_info_to_controller_udp.pose[5] << ',' 
    //     << "qz: " << pub_type.planner_info_to_controller_udp.pose[6] << ')' ;
    return out;
}
} /* bipedlab */ 
