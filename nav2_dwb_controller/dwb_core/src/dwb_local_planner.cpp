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

#include <algorithm>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "dwb_core/dwb_local_planner.hpp"
#include "dwb_core/exceptions.hpp"
#include "dwb_core/illegal_trajectory_tracker.hpp"
#include "dwb_msgs/msg/critic_score.hpp"
#include "nav_2d_msgs/msg/twist2_d.hpp"
#include "nav_2d_utils/conversions.hpp"
#include "nav_2d_utils/parameters.hpp"
#include "nav_2d_utils/tf_help.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"

using nav2_util::declare_parameter_if_not_declared;
using nav2_util::geometry_utils::euclidean_distance;

namespace dwb_core
{

DWBLocalPlanner::DWBLocalPlanner()
: traj_gen_loader_("dwb_core", "dwb_core::TrajectoryGenerator"),
  critic_loader_("dwb_core", "dwb_core::TrajectoryCritic")
{
}

void DWBLocalPlanner::configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
                                std::string name, 
                                std::shared_ptr<tf2_ros::Buffer> tf,
                                std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent;
  auto node = node_.lock();

  logger_ = node->get_logger();
  clock_ = node->get_clock();
  costmap_ros_ = costmap_ros;
  tf_ = tf;
  dwb_plugin_name_ = name;
  declare_parameter_if_not_declared(node, 
                                    dwb_plugin_name_ + ".critics",
                                    rclcpp::PARAMETER_STRING_ARRAY);
  declare_parameter_if_not_declared(node, 
                                    dwb_plugin_name_ + ".default_critic_namespaces",
                                    rclcpp::ParameterValue(std::vector<std::string>()));
  declare_parameter_if_not_declared(node, 
                                    dwb_plugin_name_ + ".prune_plan",
                                    rclcpp::ParameterValue(true));
  declare_parameter_if_not_declared(node, 
                                    dwb_plugin_name_ + ".prune_distance",
                                    rclcpp::ParameterValue(2.0));
  declare_parameter_if_not_declared(node, 
                                    dwb_plugin_name_ + ".forward_prune_distance",
                                    rclcpp::ParameterValue(2.0));
  declare_parameter_if_not_declared(node, 
                                    dwb_plugin_name_ + ".debug_trajectory_details",
                                    rclcpp::ParameterValue(false));
  declare_parameter_if_not_declared(node, 
                                    dwb_plugin_name_ + ".trajectory_generator_name",
                                    rclcpp::ParameterValue(std::string("dwb_plugins::StandardTrajectoryGenerator")));
  declare_parameter_if_not_declared(node, 
                                    dwb_plugin_name_ + ".transform_tolerance",
                                    rclcpp::ParameterValue(0.1));
  declare_parameter_if_not_declared(node, 
                                    dwb_plugin_name_ + ".shorten_transformed_plan",
                                    rclcpp::ParameterValue(true));
  declare_parameter_if_not_declared(node, 
                                    dwb_plugin_name_ + ".short_circuit_trajectory_evaluation",
                                    rclcpp::ParameterValue(true));

  std::string traj_generator_name;

  double transform_tolerance;
  node->get_parameter(dwb_plugin_name_ + ".transform_tolerance", transform_tolerance);
  transform_tolerance_ = rclcpp::Duration::from_seconds(transform_tolerance);
  RCLCPP_INFO(logger_, "Setting transform_tolerance to %f", transform_tolerance);

  node->get_parameter(dwb_plugin_name_ + ".prune_plan", prune_plan_);
  node->get_parameter(dwb_plugin_name_ + ".prune_distance", prune_distance_);
  node->get_parameter(dwb_plugin_name_ + ".forward_prune_distance", forward_prune_distance_);
  node->get_parameter(dwb_plugin_name_ + ".debug_trajectory_details", debug_trajectory_details_);
  node->get_parameter(dwb_plugin_name_ + ".trajectory_generator_name", traj_generator_name);
  node->get_parameter(dwb_plugin_name_ + ".short_circuit_trajectory_evaluation", short_circuit_trajectory_evaluation_);
  node->get_parameter(dwb_plugin_name_ + ".shorten_transformed_plan", shorten_transformed_plan_);

  pub_ = std::make_unique<DWBPublisher>(node, dwb_plugin_name_);
  pub_->on_configure();

  traj_generator_ = traj_gen_loader_.createUniqueInstance(traj_generator_name);

  traj_generator_->initialize(node, dwb_plugin_name_);

  try {
    loadCritics();
  } 
  catch (const std::exception & e) {
    RCLCPP_ERROR(logger_, "Couldn't load critics! Caught exception: %s", e.what());
    throw;
  }
}

void
DWBLocalPlanner::activate()
{
  pub_->on_activate();
}

void
DWBLocalPlanner::deactivate()
{
  pub_->on_deactivate();
}

void
DWBLocalPlanner::cleanup()
{
  pub_->on_cleanup();

  traj_generator_.reset();
}

std::string
DWBLocalPlanner::resolveCriticClassName(std::string base_name)
{
  if (base_name.find("Critic") == std::string::npos) {
    base_name = base_name + "Critic";
  }

  if (base_name.find("::") == std::string::npos) {
    for (unsigned int j = 0; j < default_critic_namespaces_.size(); j++) {
      std::string full_name = default_critic_namespaces_[j] + "::" + base_name;
      if (critic_loader_.isClassAvailable(full_name)) {
        return full_name;
      }
    }
  }
  return base_name;
}

void
DWBLocalPlanner::loadCritics()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  node->get_parameter(dwb_plugin_name_ + ".default_critic_namespaces", default_critic_namespaces_);
  if (default_critic_namespaces_.empty()) {
    default_critic_namespaces_.emplace_back("dwb_critics");
  }

  std::vector<std::string> critic_names;
  if (!node->get_parameter(dwb_plugin_name_ + ".critics", critic_names)) {
    throw std::runtime_error("No critics defined for " + dwb_plugin_name_);
  }

  node->get_parameter(dwb_plugin_name_ + ".critics", critic_names);
  for (unsigned int i = 0; i < critic_names.size(); i++) {
    std::string critic_plugin_name = critic_names[i];
    std::string plugin_class;

    declare_parameter_if_not_declared(
      node, dwb_plugin_name_ + "." + critic_plugin_name + ".class",
      rclcpp::ParameterValue(critic_plugin_name));
    node->get_parameter(dwb_plugin_name_ + "." + critic_plugin_name + ".class", plugin_class);

    plugin_class = resolveCriticClassName(plugin_class);

    TrajectoryCritic::Ptr plugin = critic_loader_.createUniqueInstance(plugin_class);
    RCLCPP_INFO(
      logger_,
      "Using critic \"%s\" (%s)", critic_plugin_name.c_str(), plugin_class.c_str());
    critics_.push_back(plugin);
    try {
      plugin->initialize(node, critic_plugin_name, dwb_plugin_name_, costmap_ros_);
    } catch (const std::exception & e) {
      RCLCPP_ERROR(logger_, "Couldn't initialize critic plugin!");
      throw;
    }
    RCLCPP_INFO(logger_, "Critic plugin initialized");
  }
}

void
DWBLocalPlanner::setPlan(const nav_msgs::msg::Path & path)
{
  auto path2d = nav_2d_utils::pathToPath2D(path); // lhq 设置全局路径
  for (TrajectoryCritic::Ptr & critic : critics_) {
    critic->reset();
  }

  traj_generator_->reset();

  pub_->publishGlobalPlan(path2d);
  global_plan_ = path2d;
}

/**
 * @brief nav2_core computeVelocityCommands - calculates the best command given the current pose and velocity
 *
 * It is presumed that the global plan is already set.
 *
 * This is mostly a wrapper for the protected computeVelocityCommands
 * function which has additional debugging info.
 *
 * @param pose Current robot pose
 * @param velocity Current robot velocity
 * @param goal_checker   Ptr to the goal checker for this task in case useful in computing commands
 * @return The best command for the robot to drive
 */
geometry_msgs::msg::TwistStamped
DWBLocalPlanner::computeVelocityCommands(const geometry_msgs::msg::PoseStamped& pose, // 机器人当前位姿
                                         const geometry_msgs::msg::Twist& velocity,   // 机器人当前速度
                                         nav2_core::GoalChecker * /*goal_checker*/)   // 目标检查器，用于目标检查
{
  std::shared_ptr<dwb_msgs::msg::LocalPlanEvaluation> results = nullptr; // 用于存储评估结果
  if (pub_->shouldRecordEvaluation()) // 查看是否需要保存评估结果
  {
    results = std::make_shared<dwb_msgs::msg::LocalPlanEvaluation>(); // 分配内存
  }

  try {
    nav_2d_msgs::msg::Twist2DStamped cmd_vel2d = computeVelocityCommands(nav_2d_utils::poseStampedToPose2D(pose), // 将机器人当前位姿转换为2D格式(四元数转为yaw)
                                                                         nav_2d_utils::twist3Dto2D(velocity),     // 将机器人当前速度转换为2D格式
                                                                         results);
    pub_->publishEvaluation(results); // 话题 "evaluation"
    geometry_msgs::msg::TwistStamped cmd_vel;
    cmd_vel.twist = nav_2d_utils::twist2Dto3D(cmd_vel2d.velocity);
    return cmd_vel;
  }
  catch (const nav2_core::PlannerException& e) {
    pub_->publishEvaluation(results);
    throw;
  }
}

/**
  @brief 辅助方法，主要用于对全局路径（global_plan）进行两项常见操作：
    1. 路径变换
    将全局路径（global_plan_）根据当前机器人的位姿（pose）进行坐标变换，得到相对于当前位姿的路径，并将结果保存到 transformed_plan 中。
    如果 publish_plan 为 true，还会发布变换后的路径，方便可视化或调试。
    2. 目标点变换
    取全局路径中的最后一个点（通常是目标点），并将其变换到局部代价地图（local costmap）所用的坐标系下，结果保存在 goal_pose 里。
  @param pose 当前机器人的位姿
  @param transformed_plan 用于存储变换后的路径
  @param goal_pose 用于存储变换后的目标点
  @param publish_plan 是否发布变换后的路径
*/
void
DWBLocalPlanner::prepareGlobalPlan(const nav_2d_msgs::msg::Pose2DStamped& pose, 
                                   nav_2d_msgs::msg::Path2D&        transformed_plan,
                                   nav_2d_msgs::msg::Pose2DStamped& goal_pose, 
                                   bool publish_plan)
{
  transformed_plan = transformGlobalPlan(pose);     // 出参 全局路径(global系) 转换到odom系下
  if (publish_plan) {
    pub_->publishTransformedPlan(transformed_plan); // 话题名称 transformed_global_plan
  }

  goal_pose.header.frame_id = global_plan_.header.frame_id;
  goal_pose.pose = global_plan_.poses.back();
  nav_2d_utils::transformPose(tf_, 
                              costmap_ros_->getGlobalFrameID(),  // lhq 变换到odom系下
                              goal_pose,                         // 入参 全局路径的最后一个点
                              goal_pose,                         // 出参 全局路径的最后一个点(odom系)
                              transform_tolerance_);
}

nav_2d_msgs::msg::Twist2DStamped
DWBLocalPlanner::computeVelocityCommands(const nav_2d_msgs::msg::Pose2DStamped& pose,                   // 机器人当前位姿
                                         const nav_2d_msgs::msg::Twist2D& velocity,                     // 机器人当前速度
                                         std::shared_ptr<dwb_msgs::msg::LocalPlanEvaluation>& results)  // 用于存储评估结果
{
  if (results) {
    results->header.frame_id = pose.header.frame_id;
    results->header.stamp = clock_->now();
  }

  nav_2d_msgs::msg::Path2D        transformed_plan;
  nav_2d_msgs::msg::Pose2DStamped goal_pose;

  prepareGlobalPlan(pose,              // 入参 机器人当前位姿
                    transformed_plan,  // 出参 全局路径(global系) 转换到odom系下
                    goal_pose);        // 出参 全局路径的最后一个点(odom系)

  nav2_costmap_2d::Costmap2D * costmap = costmap_ros_->getCostmap(); // 局部代价地图
  std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*(costmap->getMutex()));

  for (TrajectoryCritic::Ptr & critic : critics_)
  {
    if (!critic->prepare(pose.pose, velocity, goal_pose.pose, transformed_plan)) {
      RCLCPP_WARN(rclcpp::get_logger("DWBLocalPlanner"), "A scoring function failed to prepare");
    }
  }

  try {
    dwb_msgs::msg::TrajectoryScore best = coreScoringAlgorithm(pose.pose, velocity, results); // kernel 生成轨迹？

    // Return Value
    nav_2d_msgs::msg::Twist2DStamped cmd_vel;
    cmd_vel.header.stamp = clock_->now();
    cmd_vel.velocity = best.traj.velocity;

    // debrief stateful scoring functions
    for (TrajectoryCritic::Ptr & critic : critics_) {
      critic->debrief(cmd_vel.velocity); // kernel 评价速度？
    }

    lock.unlock();

    pub_->publishLocalPlan(pose.header, best.traj);  // 话题 "local_plan" // QA  best.traj odom轨迹？
    pub_->publishCostGrid(costmap_ros_, critics_);   // 话题 "cost_cloud" // TODO 发布的什么？

    return cmd_vel;
  } 
  catch (const dwb_core::NoLegalTrajectoriesException & e) {
    nav_2d_msgs::msg::Twist2D empty_cmd;
    dwb_msgs::msg::Trajectory2D empty_traj;
    // debrief stateful scoring functions
    for (TrajectoryCritic::Ptr & critic : critics_) {
      critic->debrief(empty_cmd);
    }

    lock.unlock();

    pub_->publishLocalPlan(pose.header, empty_traj);
    pub_->publishCostGrid(costmap_ros_, critics_);

    throw;
  }
}

/** 
  @brief 根据速度评价轨迹
    对所有可行的速度采样（Twist），生成对应的轨迹，并对每条轨迹进行打分，
    最终选出最佳和最差的轨迹分数，同时记录所有采样结果。
*/
dwb_msgs::msg::TrajectoryScore
DWBLocalPlanner::coreScoringAlgorithm(const geometry_msgs::msg::Pose2D & pose,
                                      const nav_2d_msgs::msg::Twist2D velocity,
                                      std::shared_ptr<dwb_msgs::msg::LocalPlanEvaluation> & results)
{
  nav_2d_msgs::msg::Twist2D      twist;
  dwb_msgs::msg::Trajectory2D    traj;
  dwb_msgs::msg::TrajectoryScore best, worst;
  best.total = -1;
  worst.total = -1;
  IllegalTrajectoryTracker tracker;

  traj_generator_->startNewIteration(velocity);
  while (traj_generator_->hasMoreTwists()) 
  {
    twist = traj_generator_->nextTwist();
    traj = traj_generator_->generateTrajectory(pose, velocity, twist);

    try {
      dwb_msgs::msg::TrajectoryScore score = scoreTrajectory(traj, best.total);
      tracker.addLegalTrajectory();
      if (results) {
        results->twists.push_back(score);
      }
      if (best.total < 0 || score.total < best.total) {
        best = score;
        if (results) {
          results->best_index = results->twists.size() - 1;
        }
      }
      if (worst.total < 0 || score.total > worst.total) {
        worst = score;
        if (results) {
          results->worst_index = results->twists.size() - 1;
        }
      }
    } 
    catch (const dwb_core::IllegalTrajectoryException & e) {
      if (results) 
      {
        dwb_msgs::msg::TrajectoryScore failed_score;
        failed_score.traj = traj;

        dwb_msgs::msg::CriticScore cs;
        cs.name = e.getCriticName();
        cs.raw_score = -1.0;
        failed_score.scores.push_back(cs);
        failed_score.total = -1.0;
        results->twists.push_back(failed_score);
      }
      tracker.addIllegalTrajectory(e);
    }
  }

  if (best.total < 0) 
  {
    if (debug_trajectory_details_) {
      RCLCPP_ERROR(rclcpp::get_logger("DWBLocalPlanner"), "%s", tracker.getMessage().c_str());
      for (auto const & x : tracker.getPercentages()) 
      {
        RCLCPP_ERROR(rclcpp::get_logger("DWBLocalPlanner"), "%.2f: %10s/%s", x.second, x.first.first.c_str(), x.first.second.c_str());
      }
    }
    throw NoLegalTrajectoriesException(tracker);
  }

  return best;
}

dwb_msgs::msg::TrajectoryScore
DWBLocalPlanner::scoreTrajectory(
  const dwb_msgs::msg::Trajectory2D & traj,
  double best_score)
{
  dwb_msgs::msg::TrajectoryScore score;
  score.traj = traj;

  for (TrajectoryCritic::Ptr & critic : critics_) {
    dwb_msgs::msg::CriticScore cs;
    cs.name = critic->getName();
    cs.scale = critic->getScale();

    if (cs.scale == 0.0) {
      score.scores.push_back(cs);
      continue;
    }

    double critic_score = critic->scoreTrajectory(traj);
    cs.raw_score = critic_score;
    score.scores.push_back(cs);
    score.total += critic_score * cs.scale;
    if (short_circuit_trajectory_evaluation_ && best_score > 0 && score.total > best_score) {
      // since we keep adding positives, once we are worse than the best, we will stay worse
      break;
    }
  }

  return score;
}


/**
 * @brief Transforms global plan into same frame as pose, clips far away poses and possibly prunes passed poses
 *
 * Three key operations
 * 1) Transforms global plan into frame of the given pose
 * 2) Only returns poses that are near the robot, i.e. whether they are likely on the local costmap
 * 3) If prune_plan_ is true, it will remove all points that we've already passed from both the transformed plan
 *     and the saved global_plan_. Technically, it iterates to a pose on the path that is within prune_distance_
 *     of the robot and erases all poses before that.
 *
 * Additionally, shorten_transformed_plan_ determines whether we will pass the full plan all
 * the way to the nav goal on to the critics or just a subset of the plan near the robot.
 * True means pass just a subset. This gives DWB less discretion to decide how it gets to the
 * nav goal. Instead it is encouraged to try to get on to the path generated by the global planner.
 * @param[in] 机器人当前位姿
 */
/*
将全局路径（global plan）处理成适合本地规划器（如 DWB）使用的局部路径。它完成了以下三项关键操作：
1. 坐标系转换
  将全局路径中的所有路径点，从全局坐标系转换到与当前机器人位姿（pose）相同的坐标系下。这样可以保证路径点和机器人位姿在同一参考系中，便于后续的路径跟踪和控制。
2. 裁剪远离机器人的路径点
  只保留靠近机器人、可能出现在本地代价地图（local costmap）上的路径点。这样可以减少不必要的计算量，只关注与机器人当前运动相关的路径片段。
3. 可选的路径修剪（prune）
  如果 prune_plan_ 标志为 true，则会将机器人已经经过的路径点从变换后的路径和保存的全局路径中删除。具体做法是：找到距离机器人小于 prune_distance_ 的路径点，并删除其之前的所有点。这样可以避免重复处理已经走过的路径。
*/
nav_2d_msgs::msg::Path2D
DWBLocalPlanner::transformGlobalPlan(const nav_2d_msgs::msg::Pose2DStamped& pose)
{
  if (global_plan_.poses.empty()) {
    throw nav2_core::PlannerException("Received plan with zero length");
  }

  // QA 这里应该是个冗余操作, 机器人位姿本身就在全局路径的坐标系下?
  // let's get the pose of the robot in the frame of the plan
  nav_2d_msgs::msg::Pose2DStamped robot_pose;
  if (!nav_2d_utils::transformPose(tf_,
                                   global_plan_.header.frame_id,  // lhq 转到这个坐标系下
                                   pose,                          // 入参 机器人当前位姿
                                   robot_pose,                    // 出参 机器人当前位姿(global系)
                                   transform_tolerance_))
  {
    throw dwb_core::PlannerTFException("Unable to transform robot pose into global plan's frame");
  }

  // we'll discard points on the plan that are outside the local costmap
  nav2_costmap_2d::Costmap2D * costmap = costmap_ros_->getCostmap();
  double dist_threshold = std::max(costmap->getSizeInCellsX(), costmap->getSizeInCellsY()) * costmap->getResolution() / 2.0; // 地图中心到边界的最大半径（即最大可能距离的一半）

  // If prune_plan is enabled (it is by default) then we want to restrict the
  // plan to distances within that range as well.
  double prune_dist = prune_distance_; // 设置路径裁剪距离 单位 : m

  // 设置在到达机器人当前位置（即路径起点）之前，最多包含的点的最大距离
  // Set the maximum distance we'll include points before getting to the part
  // of the path where the robot is located (the start of the plan). Basically,
  // these are the points the robot has already passed.
  double transform_start_threshold;
  if (prune_plan_) {
    transform_start_threshold = std::min(dist_threshold, prune_dist); // 取裁剪距离和地图最大半径的最小值
  }
  else {
    transform_start_threshold = dist_threshold;
  }

  // 设置在机器人当前位置（即路径终点）之后，最多包含的点的最大距离。这决定了要传递给评估模块（critics）的路径长度
  // 目的是防止将过长的路径传递给后续的评估模块（critics），提高局部规划的效率和实时性。
  // Set the maximum distance we'll include points after the part of the plan near the robot (the end of the plan).
  // This determines the amount of the plan passed on to the critics
  double transform_end_threshold;
  double forward_prune_dist = forward_prune_distance_;
  if (shorten_transformed_plan_) {
    transform_end_threshold = std::min(dist_threshold, forward_prune_dist);
  }
  else {
    transform_end_threshold = dist_threshold;
  }

  // 找到全局路径中，距离机器人当前位置超过裁剪距离的第一个点。(迭代器)
  // Find the first pose in the global plan that's further than prune distance from the robot using integrated distance
  auto prune_point = nav2_util::geometry_utils::first_after_integrated_distance(global_plan_.poses.begin(),
                                                                                global_plan_.poses.end(), 
                                                                                prune_dist);

  // Find the first pose in the plan (upto prune_point) that's less than transform_start_threshold from the robot.
  /* code std::find_if
      返回值：指向第一个满足条件的元素的迭代器
  */
  // QA 疑问：这里的结果基本就是global_plan_的起点吧？
  auto transformation_begin = std::find_if(begin(global_plan_.poses), 
                                           prune_point,
                                           [&](const auto& global_plan_pose) {
                                              return euclidean_distance(robot_pose.pose, global_plan_pose) < transform_start_threshold;});

  // Find the first pose in the end of the plan that's further than transform_end_threshold from the robot using integrated distance
  auto transformation_end = std::find_if(transformation_begin, 
                                         global_plan_.poses.end(),
                                         [&](const auto & pose) {
                                            return euclidean_distance(pose, robot_pose.pose) > transform_end_threshold;});
  // TODO 这里有个疑问: global_plan_做为全局路径, 起点会变动跟随机器人当前位姿而变动嘛？

  // Transform the near part of the global plan into the robot's frame of reference.
  nav_2d_msgs::msg::Path2D transformed_plan;
  transformed_plan.header.frame_id = costmap_ros_->getGlobalFrameID(); // lhq 这里是局部代价地图, 坐标系是odom系
  transformed_plan.header.stamp = pose.header.stamp;

  // Helper function for the transform below. Converts a pose2D from global frame to local
  // lhq 这是一个函数对象
  auto transformGlobalPoseToLocal = [&](const auto& global_plan_pose) {
                                              nav_2d_msgs::msg::Pose2DStamped stamped_pose, transformed_pose;
                                              stamped_pose.header.frame_id = global_plan_.header.frame_id;
                                              stamped_pose.pose = global_plan_pose;
                                              nav_2d_utils::transformPose(tf_, 
                                                                          transformed_plan.header.frame_id,  // lhq 变换到odom系下
                                                                          stamped_pose,     // 入参
                                                                          transformed_pose, // 出参
                                                                          transform_tolerance_);
                                              return transformed_pose.pose;
                                            };
  /* code std::transform
      将区间内的每个元素，经过 transformGlobalPoseToLocal 这个函数处理后，
      结果依次插入到 transformed_plan.poses 容器的末尾(std::back_inserter)。
  */
  std::transform(transformation_begin, // note 迭代器在谓词中会被解引用
                 transformation_end,
                 std::back_inserter(transformed_plan.poses), // 出参 结果放入到这里
                 transformGlobalPoseToLocal);

  // 移除已经经过的路径，避免重复处理
  // Remove the portion of the global plan that we've already passed so we don't process it on the next iteration.
  if (prune_plan_) {
    global_plan_.poses.erase(begin(global_plan_.poses), transformation_begin);
    pub_->publishGlobalPlan(global_plan_);
  }

  if (transformed_plan.poses.empty()) {
    throw nav2_core::PlannerException("Resulting plan has 0 poses in it.");
  }
  return transformed_plan;
}

}  // namespace dwb_core

// Register this controller as a nav2_core plugin
PLUGINLIB_EXPORT_CLASS(
  dwb_core::DWBLocalPlanner,
  nav2_core::Controller)
