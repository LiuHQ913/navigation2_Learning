/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, Locus Robotics
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef NAV_2D_UTILS__ODOM_SUBSCRIBER_HPP_
#define NAV_2D_UTILS__ODOM_SUBSCRIBER_HPP_

#include <cmath>
#include <memory>
#include <mutex>
#include <string>
#include "nav_2d_msgs/msg/twist2_d_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "nav2_util/node_utils.hpp"

namespace nav_2d_utils
{

/**
 * @class OdomSubscriber
 * Wrapper for some common odometry operations. Subscribes to the topic with a mutex.
 */
class OdomSubscriber
{
public:
  /**
   * @brief Constructor that subscribes to an Odometry topic
   *
   * @param nh NodeHandle for creating subscriber
   * @param default_topic Name of the topic that will be loaded of the odom_topic param is not set.
   */
  explicit OdomSubscriber(nav2_util::LifecycleNode::SharedPtr nh,
                          std::string default_topic = "odom")
  {
    /* code rclcpp::ParameterValue
        参数声明和设置时，接口需要统一的“参数值”类型，这样才能用同一个函数处理不同类型的参数。
        rclcpp::ParameterValue 就是这样一个“万能盒子”，它可以用来存储任意类型的参数值。
        这样声明参数时，不用关心参数是什么类型，直接用 ParameterValue 包装一下就行了。
    */
    // 如果参数服务器中还没有odom_topic这个参数，就用default_topic作为默认值声明它
    nav2_util::declare_parameter_if_not_declared(nh, "odom_topic", rclcpp::ParameterValue(default_topic));

    std::string odom_topic;
    // 参数服务器获取odom_topic的值，如果没有设置，则用default_topic赋值给odom_topic变量。
    nh->get_parameter_or("odom_topic", odom_topic, default_topic);
    odom_sub_ = nh->create_subscription<nav_msgs::msg::Odometry>(odom_topic,
                                                                 rclcpp::SystemDefaultsQoS(),
                                                                 std::bind(&OdomSubscriber::odomCallback, this, std::placeholders::_1));
  }

  inline nav_2d_msgs::msg::Twist2D getTwist() {return odom_vel_.velocity;}
  inline nav_2d_msgs::msg::Twist2DStamped getTwistStamped() {return odom_vel_;}

protected:
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    // ROS_INFO_ONCE("odom received!");
    std::lock_guard<std::mutex> lock(odom_mutex_);
    odom_vel_.header = msg->header;
    odom_vel_.velocity.x = msg->twist.twist.linear.x;
    odom_vel_.velocity.y = msg->twist.twist.linear.y;
    odom_vel_.velocity.theta = msg->twist.twist.angular.z;
  }

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  nav_2d_msgs::msg::Twist2DStamped odom_vel_;
  std::mutex odom_mutex_;
};

}  // namespace nav_2d_utils

#endif  // NAV_2D_UTILS__ODOM_SUBSCRIBER_HPP_
