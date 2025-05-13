// Copyright (c) 2019 Intel Corporation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <memory>

#include "nav2_controller/controller_server.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<nav2_controller::ControllerServer>();
  // 启动了ROS2的事件循环(自旋)（spin）。rclcpp::spin 会阻塞当前线程，并持续监听和处理与节点相关的事件（例如订阅的消息、服务请求等）。
  // 这里通过调用 node->get_node_base_interface() 获取节点的基础接口，确保事件循环能够正确运行。
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();

  return 0;
}
