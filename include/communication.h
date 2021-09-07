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
#ifndef COMMUNICATION_H
#define COMMUNICATION_H
// #include "udp.h"

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

#include <vector>
#include <string>

#include <boost/thread/mutex.hpp>
#include <boost/thread.hpp>


#include <tf/transform_datatypes.h>


#include "utils/debugger.h"
#include "utils/utils.h"
#include "inekf_msgs/State.h"
#include "planner_info_to_controller_t.h"
#include "controller_info_to_planner_t.h"
#include "point.h"
#include "control_commands.h"



namespace bipedlab
{
typedef struct communication
{
    std::string ip_to_send;
    std::string port_to_send;
    std::string port_to_receive;

    communication(void): ip_to_send(""), port_to_send(""), port_to_receive("") { }
    communication(const std::string& ip_to_send, 
                  const std::string& port_to_send,
                  const std::string& port_to_receive):
        ip_to_send(ip_to_send),
        port_to_send(port_to_send),
        port_to_receive(port_to_receive) { }
} communication_t;

typedef struct publish_types
{
        planner_info_to_controller_t planner_info_to_controller_udp;
        std::string  s_planner_info_to_controller_udp;
} publish_types_t;


typedef struct udp_socket 
{
    std::string ip;
    std::string port;
    int status;

    int sock_fd; // socket file descriptor
    sockaddr_in addr_sender = {};
    sockaddr_in addr_receiver = {};
    sockaddr_storage addr_dest = {};

    udp_socket(const std::string& ip, const std::string& port) :
        ip(ip), port(port), status(0) { }

    udp_socket(const std::string& port) :
        port(port), status(0) { }
    udp_socket(void) : status(0) { }
} udp_socket_t;

    
class Communication
{
private:
    udp_socket_t createUDPSender_(const std::string& ip, const std::string& port);
    int resolveUDPHelper_(udp_socket_t& sender);

    size_t publishToRobot_(const publish_types_t& info);
    publish_types_t convertToRobotCommunicationTypeReduced_(
        const double& x, const double& y, const double& z,
        const double& qw, const double& qx, const double& qy, const double& qz,
        const double& vx, const double& vy,
        const double& heading);

    bool getUDPStatus_(size_t byte_sent);




    udp_socket_t sender_;
    udp_socket_t receiver_;
    boost::mutex receiver_lock_;


    // received message
    controller_info_to_planner_t controller_msg_;



public:
    friend class Driver; // to access Comunication::createListener_()
    Communication(const std::string& sender_ip, 
                  const std::string& sender_port,
                  const std::string& receiver_port);
    void createListener_();

    Communication(const communication_t& udp_info);




    // return string of data


    // // assign structure
    // planner_info_to_controller_t assignWalkInPlace(const inekf_msgs::State& robot_state);
    // planner_info_to_controller_t assignInfo(
    //     const double& behavior,
    //     const double& speed,
    //     const double& roll, const double& pitch, const double& yaw,
    //     const std::vector<point2d_t<double>>* path_points_ptr,
    //     const std::vector<double>* foot_placement,
    //     const double& x, const double& y, const double& z,
    //     const double& qw, const double& qx, const double& qy, const double& qz);


    // publish
    bool publishToRobotFromInEKFStateMsg(
        const inekf_msgs::State& inekf_state,
        const double& vx,const double& vy,
        const double& heading);


    bool publishToRobotFromTFMsg(
            const tf::StampedTransform& robot_pose, 
            const double& vx,const double& vy,
            const double& heading);

    bool publishToRobotGivenInfoToRobotType(const planner_info_to_controller_t& info);

    // getter
    std::pair<int, controller_info_to_planner_t> getReceivedMessage(void);





    virtual ~Communication();
};



std::ostream& operator<<(std::ostream& out, const publish_types_t& pub_type);
std::ostream& operator<<(std::ostream& out, const planner_info_to_controller_t& pub_type);


} /* bipedlab */ 
#endif /* COMMUNICATION_H */
