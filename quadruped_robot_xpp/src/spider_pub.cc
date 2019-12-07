/******************************************************************************
Copyright (c) 2017, Alexander W. Winkler. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

#include <algorithm>

#include <ros/ros.h>

#include <xpp_msgs/RobotStateCartesian.h>
#include <xpp_msgs/topic_names.h>

#include <xpp_states/convert.h>
#include <xpp_states/robot_state_cartesian.h>


using namespace xpp;

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "spider_publisher_node");

  ros::NodeHandle n;
  ros::Publisher state_pub = n.advertise<xpp_msgs::RobotStateCartesian>(xpp_msgs::robot_state_desired, 1);
  ROS_INFO_STREAM("Waiting for Subscriber...");
  while(ros::ok() && state_pub.getNumSubscribers() == 0) {
    ros::Rate(100).sleep();
  }
  ROS_INFO_STREAM("Subscriber to initial state connected");


  // visualize the state of a one-legged hopper
  RobotStateCartesian hopper(4);

  // publishes a sequence of states for a total duration of T spaced 0.01s apart.
  constexpr double T = 5.0;
  double t = 0.0;
  constexpr double dt = 0.01;
  constexpr double speed = 0.01;
  constexpr double step_length = 0.04;
  constexpr double total_dist = speed * T;
  while (t < T) {
    // base and foot follow half a sine motion up and down
    hopper.base_.lin.p_.z() = 0.06;
    //hopper.base_.lin.p_.x() = 0.5/T * t;
    hopper.base_.lin.p_.x() = speed * t;

//    hopper.ee_motion_.at(0).p_.z() = 0.1*sin(2*M_PI/(2*T)*t);
//    hopper.ee_forces_.at(0).z() = 100; // N
//    hopper.ee_contact_.at(0) = true;

    hopper.ee_motion_.at(0).p_.x() = 0.1;
    hopper.ee_motion_.at(0).p_.y() = 0.1;
    hopper.ee_motion_.at(0).p_.z() = 0;
    hopper.ee_forces_.at(0).z() = 1; // N
    hopper.ee_contact_.at(0) = true;

    hopper.ee_motion_.at(1).p_.x() = 0.1;
    hopper.ee_motion_.at(1).p_.y() = -0.1;
    hopper.ee_motion_.at(1).p_.z() = 0;
    hopper.ee_forces_.at(1).z() = 1; // N
    hopper.ee_contact_.at(1) = true;

    hopper.ee_motion_.at(2).p_.x() = -0.1;
    hopper.ee_motion_.at(2).p_.y() = 0.1;
    hopper.ee_motion_.at(2).p_.z() = 0;
    hopper.ee_forces_.at(2).z() = 1; // N
    hopper.ee_contact_.at(2) = true;

    hopper.ee_motion_.at(3).p_.x() = -0.1;
    hopper.ee_motion_.at(3).p_.y() = -0.1;
    hopper.ee_motion_.at(3).p_.z() = 0;
    hopper.ee_forces_.at(3).z() = 1; // N
    hopper.ee_contact_.at(3) = true;


    static_assert(T/(total_dist/step_length) == step_length/speed, "");


    const double step_time = step_length/2 / speed;
    if (t / step_time < 1.0) {
      hopper.ee_motion_.at(0).p_.z() = 0.04 * sin(M_PI * t/step_time);
      hopper.ee_motion_.at(0).p_.x() = 0.08 + 2*speed * t;
      hopper.ee_forces_.at(0).z() = 0.0;
      hopper.ee_contact_.at(0) = false;

      hopper.ee_motion_.at(3).p_.z() = 0.04 * sin(M_PI * t/step_time);
      hopper.ee_motion_.at(3).p_.x() = -0.08 + 2*speed * t;
      hopper.ee_forces_.at(3).z() = 0.0;
      hopper.ee_contact_.at(3) = false;
    }
    else {
      hopper.ee_motion_.at(0).p_.z() = 0.0;
      hopper.ee_motion_.at(0).p_.x() = 0.08 + step_length;
      hopper.ee_forces_.at(0).z() = 1; // N
      hopper.ee_contact_.at(0) = true;
      hopper.ee_motion_.at(3).p_.z() = 0.0;
      hopper.ee_motion_.at(3).p_.x() = -0.08 + step_length;
      hopper.ee_forces_.at(3).z() = 1; // N
      hopper.ee_contact_.at(3) = true;
    }


    //if ()
//    hopper.ee_motion_.at(0).p_.z() = 0.04 * std::max(0.0, sin(M_PI / ((step_length/2) / speed) * t));
//    if (std::max(0.0, sin(M_PI / ((step_length/2) / speed) * t)) != 0.0) {
//        hopper.ee_motion_.at(0).p_.x() = 0.08 + 2*speed * t;
//    }
//    else {
//        hopper.ee_motion_.at(0).p_.x() = 0.08 + step_length;
//    }

//    (step_length/2 * t/(T/(total_dist/step_length)));

//    total_dist / step_length ->
//    speed * t / step_length ->
//    speed * t % step_length ->

//    hopper.ee_motion_.at(i).p_.x() =

//    0.01*sin(M_PI/(2*T)*t

    for (int i = 0; i < 4; ++i) {
      //hopper.ee_motion_.at(i).p_.x() += hopper.base_.lin.p_.x();
      hopper.ee_forces_.at(i).x() = 0; // N
      hopper.ee_forces_.at(i).y() = 0; // N
    }

    state_pub.publish(Convert::ToRos(hopper));

    ros::spinOnce();
    ros::Duration(dt).sleep(); // pause loop so visualization has correct speed.
    t += dt;
  }


  return 0;
}

