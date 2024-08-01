/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
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
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Eitan Marder-Eppstein
 *********************************************************************/

#include <base_local_planner/local_planner_util.h>

#include <base_local_planner/goal_functions.h>

namespace base_local_planner {

void LocalPlannerUtil::initialize(
    tf2_ros::Buffer* tf,
    costmap_2d::Costmap2D* costmap,
    std::string global_frame) {
  if(!initialized_) {
    tf_ = tf;
    costmap_ = costmap;
    global_frame_ = global_frame;
    initialized_ = true;
  }
  else{
    ROS_WARN("Planner utils have already been initialized, doing nothing.");
  }
}

void LocalPlannerUtil::reconfigureCB(LocalPlannerLimits &config, bool restore_defaults)
{
  if(setup_ && restore_defaults) {
    config = default_limits_;
  }

  if(!setup_) {   
    default_limits_ = config;
    setup_ = true;
  }
  boost::mutex::scoped_lock l(limits_configuration_mutex_);
  limits_ = LocalPlannerLimits(config);
}

costmap_2d::Costmap2D* LocalPlannerUtil::getCostmap() {
  return costmap_;
}

LocalPlannerLimits LocalPlannerUtil::getCurrentLimits() {
  boost::mutex::scoped_lock l(limits_configuration_mutex_);
  return limits_;
}

void LocalPlannerUtil::enableReverse() {
  std::cout << "-----------------LocalPlannerUtil::enableReverse() 开启倒档" << std::endl;
  limits_.min_vel_x = -0.1;  
  limits_.max_vel_x = 0.2;  
}

void LocalPlannerUtil::disableReverse() {
    limits_.min_vel_x = default_limits_.min_vel_x;  
    limits_.max_vel_x = default_limits_.max_vel_x;  
    std::cout << "-----------------LocalPlannerUtil::enableReverse() 取消倒档 ------------------" << std::endl;
    std::cout << "-----------------max_vel_x: " << limits_.max_vel_x << ", min_vel_x: " << limits_.min_vel_x << std::endl;

}

bool LocalPlannerUtil::getGoal(geometry_msgs::PoseStamped& goal_pose) {
  //we assume the global goal is the last point in the global plan
  return base_local_planner::getGoalPose(*tf_,
        global_plan_,
        global_frame_,
        goal_pose);
}

bool LocalPlannerUtil::setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan) {
  if(!initialized_){
    ROS_ERROR("Planner utils have not been initialized, please call initialize() first");
    return false;
  }

  //reset the global plan
  global_plan_.clear();
  start_idx_ = 0; 
  global_plan_ = orig_global_plan;

  return true;
}

/**
 * @brief 
 * 
 * @param global_pose 机器人当前位姿，需要这个的目的是对全局路径进行裁剪  
 * @param[out] transformed_plan global_plan_裁剪后然后转换到global_frame_的结果 
 * @return true 
 * @return false 
 */
bool LocalPlannerUtil::getLocalPlan(const geometry_msgs::PoseStamped& global_pose, 
      std::vector<geometry_msgs::PoseStamped>& transformed_plan) {
  //get the global plan in our frame
  if(!base_local_planner::extractGlobalPlan(
      *tf_,
      global_plan_,   // 全局路径规划 
      global_pose,   // 机器人的全局位姿     在dwa中就是costmap的全局参考系odom下的位姿
      *costmap_,
      global_frame_,       // costmap_的全局参考系 一般是odom
      transformed_plan,
      start_idx_)) {
    ROS_WARN("Could not transform the global plan to the frame of the controller");
    return false;
  }
  //now we'll prune the plan based on the position of the robot
  if(limits_.prune_plan) {
    // 没执行
    // std::cout << "prunePlan" << std::endl;
    base_local_planner::prunePlan(global_pose, transformed_plan, global_plan_);
  }
  return true;
}

/**
   * @brief  将全局路径进行裁剪(只保留距离小于阈值的部分)，
   *                并且将路径转换到global_frame系(costmap的全局参考系，一般为odom)得到transformed_plan
   * 
   * @param tf 
   * @param global_plan 
   * @param global_pose  机器人在costmap全局参考系(一般为odom)下的姿态 
   * @param costmap 
   * @param global_frame 
   * @param transformed_plan[out] 
   * @return true 
   * @return false 
   */
  bool LocalPlannerUtil::transformGlobalPlan(
      const tf2_ros::Buffer& tf,
      const std::vector<geometry_msgs::PoseStamped>& global_plan,
      const geometry_msgs::PoseStamped& global_pose,
      const costmap_2d::Costmap2D& costmap,
      const std::string& global_frame,
      std::vector<geometry_msgs::PoseStamped>& transformed_plan) {
    // transformed_plan.clear();

    // if (global_plan.empty()) {
    //   ROS_ERROR("Received plan with zero length");
    //   return false;
    // }

    // const geometry_msgs::PoseStamped& plan_pose = global_plan[0];
    // try {
    //   // get plan_to_global_transform from plan frame to global_frame
    //   // 获取全局路径规划的参考系到global_frame(costmap的参考坐标系  odom)的变换  
    //   geometry_msgs::TransformStamped plan_to_global_transform = 
    //       // 调用的是 BufferInterface 的函数  
    //       tf.lookupTransform(global_frame,    // 目标坐标系 
    //                                                 ros::Time(),
    //                                                 plan_pose.header.frame_id,      // 原坐标系
    //                                                 plan_pose.header.stamp, 
    //                                                 plan_pose.header.frame_id, 
    //                                                 ros::Duration(0.5));
    //   // std::cout << "global_frame: " << global_frame << std::endl;
    //   // std::cout << "plan_pose.header.frame_id: " << plan_pose.header.frame_id << std::endl;
    //   //let's get the pose of the robot in the frame of the plan
    //   // 将机器人的位姿转换到全局路径规划的坐标系上
    //   // 将global_pose转换为plan_pose.header.frame_id指定的坐标系下的姿态，
    //   // 并将结果存储在robot_pose中
    //   geometry_msgs::PoseStamped robot_pose;
    //   tf.transform(global_pose, robot_pose, plan_pose.header.frame_id);

    //   //we'll discard points on the plan that are outside the local costmap
    //   // 这行代码设置了一个距离阈值dist_threshold，它基于局部代价地图（costmap）的大小和分辨率。
    //   // 该阈值是代价地图尺寸的一半（以米为单位）。这通常用于确定哪些路径点（在全局计划中）是“接近”机器人的
    //   double dist_threshold = std::max(costmap.getSizeInCellsX() * costmap.getResolution() / 2.0,
    //                                    costmap.getSizeInCellsY() * costmap.getResolution() / 2.0);

    //   static unsigned int start_idx = 0;
    //   double sq_dist_threshold = dist_threshold * dist_threshold;
    //   double sq_dist = 0;
    //   double old_sq_dist = 1000; 
    //   //we need to loop to a point on the plan that is within a certain distance of the robot
    //   // 遍历全局计划（global_plan）中的点，并找到第一个距离机器人不超过dist_threshold的点
    //   while(start_idx < (unsigned int)global_plan.size()) {
    //     double x_diff = robot_pose.pose.position.x - global_plan[start_idx].pose.position.x;
    //     double y_diff = robot_pose.pose.position.y - global_plan[start_idx].pose.position.y;
    //     sq_dist = x_diff * x_diff + y_diff * y_diff;
    //     if (sq_dist <= sq_dist_threshold && sq_dist > old_sq_dist) {
    //       break;
    //     }
    //     old_sq_dist = sq_dist; 
    //     ++start_idx;
    //   }

    // //   geometry_msgs::PoseStamped newer_pose;
    // //   unsigned int i = start_idx; 
    // //   //now we'll transform until points are outside of our distance threshold
    // //   // 将全局路径所有距离机器人距离小于阈值的路径点提取出来并转换到局部规划器的全局坐标下 
    // //   // 放置于transformed_plan
    // //   while(i < (unsigned int)global_plan.size() && sq_dist <= sq_dist_threshold) {
    // //     const geometry_msgs::PoseStamped& pose = global_plan[i];
    // //     tf2::doTransform(pose, newer_pose, plan_to_global_transform);

    // //     transformed_plan.push_back(newer_pose);

    // //     double x_diff = robot_pose.pose.position.x - global_plan[i].pose.position.x;
    // //     double y_diff = robot_pose.pose.position.y - global_plan[i].pose.position.y;
    // //     sq_dist = x_diff * x_diff + y_diff * y_diff;

    // //     ++i;
    // //   }
    // }
    // catch(tf2::LookupException& ex) {
    //   ROS_ERROR("No Transform available Error: %s\n", ex.what());
    //   return false;
    // }
    // catch(tf2::ConnectivityException& ex) {
    //   ROS_ERROR("Connectivity Error: %s\n", ex.what());
    //   return false;
    // }
    // catch(tf2::ExtrapolationException& ex) {
    //   ROS_ERROR("Extrapolation Error: %s\n", ex.what());
    //   if (!global_plan.empty())
    //     ROS_ERROR("Global Frame: %s Plan Frame size %d: %s\n", global_frame.c_str(), (unsigned int)global_plan.size(), global_plan[0].header.frame_id.c_str());

    //   return false;
    // }

    // return true;
  }

} // namespace
