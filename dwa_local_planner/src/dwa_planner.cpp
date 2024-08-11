/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2009, Willow Garage, Inc.
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
#include <dwa_local_planner/dwa_planner.h>
#include <base_local_planner/goal_functions.h>
#include <cmath>

//for computing path distance
#include <queue>

#include <angles/angles.h>

#include <ros/ros.h>
#include <tf2/utils.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include "dwa_local_planner/color.hpp"

namespace dwa_local_planner {
  /**
   * @brief 在DWAPlannerROS::reconfigureCB中调用
   * 
   * @param config 
   */
  void DWAPlanner::reconfigure(DwaPlannerConfig &config)
  {
    std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!DWAPlanner::reconfigure" << std::endl;
    boost::mutex::scoped_lock l(configuration_mutex_);
    
    generator_.setParameters(
        config.sim_time,
        config.sim_granularity,
        config.angular_sim_granularity,
        config.use_dwa,
        sim_period_    // 控制周期，由局部规划的频率决定  
        );     

    double resolution = planner_util_->getCostmap()->getResolution();
    path_distance_bias_ = resolution * config.path_distance_bias;
    // pdistscale used for both path and alignment, set  forward_point_distance to zero to discard alignment
    path_costs_.setScale(path_distance_bias_);
    alignment_costs_.setScale(path_distance_bias_);
    motionDirection_costs_.setScale(path_distance_bias_);

    goal_distance_bias_ = resolution * config.goal_distance_bias;
    goal_costs_.setScale(goal_distance_bias_);
    goal_front_costs_.setScale(goal_distance_bias_);

    occdist_scale_ = config.occdist_scale;
    obstacle_costs_.setScale(occdist_scale_);

    stop_time_buffer_ = config.stop_time_buffer;
    oscillation_costs_.setOscillationResetDist(config.oscillation_reset_dist, config.oscillation_reset_angle);
    forward_point_distance_ = config.forward_point_distance;
    goal_front_costs_.setXShift(forward_point_distance_);
    alignment_costs_.setXShift(forward_point_distance_);
 
    // obstacle costs can vary due to scaling footprint feature
    obstacle_costs_.setParams(config.max_vel_trans, config.max_scaling_factor, config.scaling_speed);
    velocity_costs_.setScale(0.01);  
    twirling_costs_.setScale(config.twirling_scale);
    std::cout << "config.twirling_scale: " << config.twirling_scale << std::endl;

    int vx_samp, vy_samp, vth_samp;
    vx_samp = config.vx_samples;
    vy_samp = config.vy_samples;
    vth_samp = config.vth_samples;
 
    if (vx_samp <= 0) {
      ROS_WARN("You've specified that you don't want any samples in the x dimension. We'll at least assume that you want to sample one value... so we're going to set vx_samples to 1 instead");
      vx_samp = 1;
      config.vx_samples = vx_samp;
    }
 
    if (vy_samp <= 0) {
      ROS_WARN("You've specified that you don't want any samples in the y dimension. We'll at least assume that you want to sample one value... so we're going to set vy_samples to 1 instead");
      vy_samp = 1;
      config.vy_samples = vy_samp;
    }
 
    if (vth_samp <= 0) {
      ROS_WARN("You've specified that you don't want any samples in the th dimension. We'll at least assume that you want to sample one value... so we're going to set vth_samples to 1 instead");
      vth_samp = 1;
      config.vth_samples = vth_samp;
    }
 
    vsamples_[0] = vx_samp;
    vsamples_[1] = vy_samp;
    vsamples_[2] = vth_samp;
    std::cout << "vx_samp: " << vx_samp << std::endl;
    std::cout << "vy_samp: " << vy_samp << std::endl;
    std::cout << "vth_samp: " << vth_samp << std::endl;
  }

  DWAPlanner::DWAPlanner(std::string name, base_local_planner::LocalPlannerUtil *planner_util) :
      planner_util_(planner_util),
      obstacle_costs_(planner_util->getCostmap()),
      path_costs_(planner_util->getCostmap()),
      goal_costs_(planner_util->getCostmap(), 0.0, 0.0, true),
      goal_front_costs_(planner_util->getCostmap(), 0.0, 0.0, true),
      alignment_costs_(planner_util->getCostmap())
  {
    ros::NodeHandle private_nh("~/" + name);

    goal_front_costs_.setStopOnFailure( false );
    alignment_costs_.setStopOnFailure( false );

    //Assuming this planner is being run within the navigation stack, we can
    //just do an upward search for the frequency at which its being run. This
    //also allows the frequency to be overwritten locally.
    std::string controller_frequency_param_name;
    if(!private_nh.searchParam("controller_frequency", controller_frequency_param_name)) {
      sim_period_ = 0.05;
    } else {
      double controller_frequency = 0;
      private_nh.param(controller_frequency_param_name, controller_frequency, 20.0);
      if(controller_frequency > 0) {
        sim_period_ = 1.0 / controller_frequency;
      } else {
        ROS_WARN("A controller_frequency less than 0 has been set. Ignoring the parameter, assuming a rate of 20Hz");
        sim_period_ = 0.05;
      }
    }
    ROS_INFO("Sim period is set to %.2f", sim_period_);

    oscillation_costs_.resetOscillationFlags();

    bool sum_scores;
    private_nh.param("sum_scores", sum_scores, false);
    obstacle_costs_.setSumScores(sum_scores);


    private_nh.param("publish_cost_grid_pc", publish_cost_grid_pc_, false);
    map_viz_.initialize(name, planner_util->getGlobalFrame(), boost::bind(&DWAPlanner::getCellCosts, this, _1, _2, _3, _4, _5, _6));

    private_nh.param("global_frame_id", frame_id_, std::string("odom"));

    traj_cloud_pub_ = private_nh.advertise<sensor_msgs::PointCloud2>("trajectory_cloud", 1);
    markers_pub = private_nh.advertise<visualization_msgs::MarkerArray>("/front_target", 1);  
    private_nh.param("publish_traj_pc", publish_traj_pc_, false);

    // set up all the cost functions that will be applied in order
    // (any function returning negative values will abort scoring, so the order can improve performance)
    std::vector<base_local_planner::TrajectoryCostFunction*> critics;
    critics.push_back(&motionDirection_costs_);   
    critics.push_back(&velocity_costs_); 
    critics.push_back(&oscillation_costs_); // 运动打分判断，是否震荡，是：代价大
    critics.push_back(&obstacle_costs_); // 障碍物碰撞检测打分，碰到障碍物，代价值增大
    // critics.push_back(&goal_front_costs_); // 局部轨迹与局部路径的最终点的朝向一致
    // critics.push_back(&alignment_costs_); // 局部轨迹与局部路径的朝向一致
    // critics.push_back(&path_costs_); // 局部轨迹（根据当前速度外推出的轨迹）与局部路径（规划的路径）对比，局部轨迹离局部路径的横向偏差小，其代价值就小
    // critics.push_back(&goal_costs_); // 局部轨迹与局部路径的终点进行对比，希望距离小
    
    // trajectory generators
    std::vector<base_local_planner::TrajectorySampleGenerator*> generator_list;
    generator_list.push_back(&generator_);

    scored_sampling_planner_ = base_local_planner::SimpleScoredSamplingPlanner(generator_list, critics);

    private_nh.param("cheat_factor", cheat_factor_, 1.0);
    look_index_ = 5;
    look_incre_ = 1;
  }

  // used for visualization only, total_costs are not really total costs
  bool DWAPlanner::getCellCosts(int cx, int cy, float &path_cost, float &goal_cost, float &occ_cost, float &total_cost) {

    path_cost = path_costs_.getCellCosts(cx, cy);
    goal_cost = goal_costs_.getCellCosts(cx, cy);
    occ_cost = planner_util_->getCostmap()->getCost(cx, cy);
    if (path_cost == path_costs_.obstacleCosts() ||
        path_cost == path_costs_.unreachableCellCosts() ||
        occ_cost >= costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
      return false;
    }

    total_cost =
        path_distance_bias_ * path_cost +
        goal_distance_bias_ * goal_cost +
        occdist_scale_ * occ_cost;
    return true;
  }

  bool DWAPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan) {
    oscillation_costs_.resetOscillationFlags();
    look_index_ = 5;
    return planner_util_->setPlan(orig_global_plan);    // 将orig_global_plan设置给LocalPlannerUtil的global_plan_
  }

  /**
   * This function is used when other strategies are to be applied,
   * but the cost functions for obstacles are to be reused.
   */
  bool DWAPlanner::checkTrajectory(
      Eigen::Vector3f pos,
      Eigen::Vector3f vel,
      Eigen::Vector3f vel_samples) {
    oscillation_costs_.resetOscillationFlags();
    base_local_planner::Trajectory traj;
    geometry_msgs::PoseStamped goal_pose = global_plan_.back();
    Eigen::Vector3f goal(goal_pose.pose.position.x, goal_pose.pose.position.y, tf2::getYaw(goal_pose.pose.orientation));
    base_local_planner::LocalPlannerLimits limits = planner_util_->getCurrentLimits();
    generator_.initialise(pos,
        vel,
        goal,
        &limits,
        vsamples_);
    generator_.generateTrajectory(pos, vel, vel_samples, traj);
    double cost = scored_sampling_planner_.scoreTrajectory(traj, -1);
    //if the trajectory is a legal one... the check passes
    if(cost >= 0) {
      return true;
    }
    ROS_WARN("Invalid Trajectory %f, %f, %f, cost: %f", vel_samples[0], vel_samples[1], vel_samples[2], cost);

    //otherwise the check fails
    return false;
  }

  void DWAPlanner::updatePlanAndLocalCosts(
      const geometry_msgs::PoseStamped& global_pose,
      const std::vector<geometry_msgs::PoseStamped>& new_plan,
      const std::vector<geometry_msgs::Point>& footprint_spec) {
    global_plan_.resize(new_plan.size());
    for (unsigned int i = 0; i < new_plan.size(); ++i) {
      global_plan_[i] = new_plan[i];
    }
    // 下面是对代价函数进行各种设置        没有太看懂     
    obstacle_costs_.setFootprint(footprint_spec);
    // costs for going away from path
    path_costs_.setTargetPoses(global_plan_);
    // costs for not going towards the local goal as much as possible
    goal_costs_.setTargetPoses(global_plan_);
    motionDirection_costs_.setTargetPoses(global_plan_);
    // alignment costs
    geometry_msgs::PoseStamped goal_pose = global_plan_.back();
    Eigen::Vector3f pos(global_pose.pose.position.x, global_pose.pose.position.y, tf2::getYaw(global_pose.pose.orientation));
    double sq_dist =
        (pos[0] - goal_pose.pose.position.x) * (pos[0] - goal_pose.pose.position.x) +
        (pos[1] - goal_pose.pose.position.y) * (pos[1] - goal_pose.pose.position.y);
    // we want the robot nose to be drawn to its final position
    // (before robot turns towards goal orientation), not the end of the
    // path for the robot center. Choosing the final position after
    // turning towards goal orientation causes instability when the
    // robot needs to make a 180 degree turn at the end
    std::vector<geometry_msgs::PoseStamped> front_global_plan = global_plan_;
    double angle_to_goal = atan2(goal_pose.pose.position.y - pos[1], goal_pose.pose.position.x - pos[0]);
    front_global_plan.back().pose.position.x = front_global_plan.back().pose.position.x +
      forward_point_distance_ * cos(angle_to_goal);
    front_global_plan.back().pose.position.y = front_global_plan.back().pose.position.y + forward_point_distance_ *
      sin(angle_to_goal);

    goal_front_costs_.setTargetPoses(front_global_plan);
    // keeping the nose on the path
    if (sq_dist > forward_point_distance_ * forward_point_distance_ * cheat_factor_) {
      alignment_costs_.setScale(path_distance_bias_);
      // costs for robot being aligned with path (nose on path, not ju
      alignment_costs_.setTargetPoses(global_plan_);
    } else {
      // once we are close to goal, trying to keep the nose close to anything destabilizes behavior.
      alignment_costs_.setScale(0.0);
    }
  }

  /*
   * given the current state of the robot, find a good trajectory
   */
  base_local_planner::Trajectory DWAPlanner::findBestPath(
      const geometry_msgs::PoseStamped& global_pose,
      const geometry_msgs::PoseStamped& global_vel,
      geometry_msgs::PoseStamped& drive_velocities) {

    //make sure that our configuration doesn't change mid-run
    boost::mutex::scoped_lock l(configuration_mutex_);

    Eigen::Vector3f pos(global_pose.pose.position.x, global_pose.pose.position.y, tf2::getYaw(global_pose.pose.orientation));
    Eigen::Vector3f vel(global_vel.pose.position.x, global_vel.pose.position.y, tf2::getYaw(global_vel.pose.orientation));
    // 将全局轨迹最后的点作为目标(即距离机器人最远的点 )
    geometry_msgs::PoseStamped goal_pose = global_plan_.back();
    Eigen::Vector3f goal(goal_pose.pose.position.x, goal_pose.pose.position.y, tf2::getYaw(goal_pose.pose.orientation));
    base_local_planner::LocalPlannerLimits limits = planner_util_->getCurrentLimits();
    // std::cout << "global_plan_ size: " << global_plan_.size() << ", global_plan_.back() x: " << goal_pose.pose.position.x
    //   << ", y: " << goal_pose.pose.position.y << ",global_plan_.front() x:  " << global_plan_.front().pose.position.x
    //   << ", y: " << global_plan_.front().pose.position.y  << std::endl;
    std::cout << "robot pos x: " << global_pose.pose.position.x << ",y: "
      << global_pose.pose.position.y << "\n"; 
    // std::cout << "robot pos : " << pos.transpose() << std::endl;
    // prepare cost functions and generators for this run
    generator_.initialise(
        pos,
        vel,  // 根据当前速度和加速度限制获取可行速度区间
        goal,      // 这里传入目标点只是为了求一个速度，只有当  use_dwa_ = 0 时才有用
        &limits,
        vsamples_,
        true);

    // 计算目标点
    // 计算机器人起始朝向与轨迹目标点之间的夹角，
    // 夹角越大，说明机器人需要的旋转越多，则预瞄点越近
    double robot_th = pos[2];
    if (robot_th > M_PI) {
        robot_th -= 2 * M_PI;
    } else if (robot_th < -M_PI) {
        robot_th += 2 * M_PI;
    }
    // int look_index = 40; 
    // int res_look_index = look_index;
    // bool up_flag = 0;
    // bool down_flag = 0;  
    // // std::cout << "路径长度："  << global_plan_.size() << "\n"; 
    // while (look_index < global_plan_.size() && look_index >= 10) {
    //   // 轨迹参考点 与 轨迹起点连线的倾角     
    //   // 根据轨迹的速度规划选择轨迹参考点index
    //   double direct1 = std::atan2(global_plan_[look_index].pose.position.y - global_plan_[0].pose.position.y, 
    //                                                           global_plan_[look_index].pose.position.x - global_plan_[0].pose.position.x);  // [-pi, pi]
    //   double direct2 = std::atan2(global_plan_[look_index / 2].pose.position.y - global_plan_[0].pose.position.y, 
    //                                                           global_plan_[look_index / 2].pose.position.x - global_plan_[0].pose.position.x);  // [-pi, pi]
    //   // std::cout << "direct: " << direct << std::endl;
    //   // std::cout << "traj_end_th: " << traj_end_th << std::endl;
    //   double diff = std::fabs(direct1 - direct2);
    //   if (diff > M_PI) {
    //     diff = 2 * M_PI - diff;   
    //   }
    //   std::cout << "look_index: " << look_index << ", diff: " << diff << "\n";
    //   // 如果角度差小于10  
    //   if (diff > 0.3) {
    //     look_index /= 2;
    //   } else {
    //     break;  
    //   }
    // }
    // std::cout << "路径长度："  << global_plan_.size() << "\n"; 
    // while (look_index < global_plan_.size() && look_index >= 10) {
    // 轨迹参考点 与 轨迹起点连线的倾角     
    // 根据轨迹的速度规划选择轨迹参考点index
    double direct1 = std::atan2(global_plan_[look_index_].pose.position.y - global_plan_[0].pose.position.y, 
                                                            global_plan_[look_index_].pose.position.x - global_plan_[0].pose.position.x);  // [-pi, pi]
    double direct2 = std::atan2(global_plan_[look_index_ / 2].pose.position.y - global_plan_[0].pose.position.y, 
                                                            global_plan_[look_index_ / 2].pose.position.x - global_plan_[0].pose.position.x);  // [-pi, pi]
    // std::cout << "direct: " << direct << std::endl;
    // std::cout << "traj_end_th: " << traj_end_th << std::endl;
    double diff = std::fabs(direct1 - direct2);
    if (diff > M_PI) {
      diff = 2 * M_PI - diff;   
    }
    std::cout << "look_index: " << look_index_ << ", diff: " << diff << "\n";
    // 如果角度差大于20 则拉近前视点  
    if (diff > 0.35) {
      if (look_index_ >= 10) {
        look_index_ -= look_incre_;
      }
    } else if (diff < 0.1745) {
      // 如果角度差小于10  则拉远前视点
      if (look_index_ < global_plan_.size() - look_incre_ && look_index_ <= 40) {
        look_index_ += look_incre_;
      }
    } 
    // }
    // look_index = global_plan_.size() - 1;
    motionDirection_costs_.SetGlobalTrajTargetIndex(look_index_);

    result_traj_.cost_ = -7;
    // find best trajectory by sampling and scoring the samples
    std::vector<base_local_planner::Trajectory> all_explored;
    // generator_在初始化时和scored_sampling_planner_进行了关联
    scored_sampling_planner_.findBestTrajectory(result_traj_, &all_explored);

    if(publish_traj_pc_)
    {
        // 将DWA搜索的路径可视化出来  ， 这些路径都在all_explored中 
        // std::cout << "publish_traj_pc_" << std::endl;
        sensor_msgs::PointCloud2 traj_cloud;
        traj_cloud.header.frame_id = frame_id_;
        traj_cloud.header.stamp = ros::Time::now();

        sensor_msgs::PointCloud2Modifier cloud_mod(traj_cloud);
        cloud_mod.setPointCloud2Fields(5, "x", 1, sensor_msgs::PointField::FLOAT32,
                                          "y", 1, sensor_msgs::PointField::FLOAT32,
                                          "z", 1, sensor_msgs::PointField::FLOAT32,
                                          "theta", 1, sensor_msgs::PointField::FLOAT32,
                                          "cost", 1, sensor_msgs::PointField::FLOAT32);

        unsigned int num_points = 0;
        for(std::vector<base_local_planner::Trajectory>::iterator t=all_explored.begin(); t != all_explored.end(); ++t)
        {
            if (t->cost_< 0)
              continue;
            num_points += t->getPointsSize();
        }

        cloud_mod.resize(num_points);
        sensor_msgs::PointCloud2Iterator<float> iter_x(traj_cloud, "x");
        for(std::vector<base_local_planner::Trajectory>::iterator t=all_explored.begin(); t != all_explored.end(); ++t)
        {
            if(t->cost_< 0)
                continue;
            // Fill out the plan
            for(unsigned int i = 0; i < t->getPointsSize(); ++i) {
                double p_x, p_y, p_th;
                t->getPoint(i, p_x, p_y, p_th);
                iter_x[0] = p_x;
                iter_x[1] = p_y;
                iter_x[2] = 0.0;
                iter_x[3] = p_th;
                iter_x[4] = t->cost_;
                ++iter_x;
            }
        }
        traj_cloud_pub_.publish(traj_cloud);

        // 可视化前视点
        visualization_msgs::MarkerArray markers;
        markers.markers.resize(1);
        ros::Time stamp = ros::Time::now();  
        // node markers    位姿节点
        visualization_msgs::Marker& traj_marker = markers.markers[0];
        traj_marker.header.frame_id = "odom";
        traj_marker.header.stamp = stamp;
        traj_marker.ns = "nodes";
        traj_marker.id = 0;
        traj_marker.type = visualization_msgs::Marker::SPHERE_LIST;
        traj_marker.pose.orientation.w = 1.0;
        traj_marker.scale.x = traj_marker.scale.y = traj_marker.scale.z = 0.1;
        // 数量
        traj_marker.points.resize(1);
        // 颜色
        traj_marker.colors.resize(1);
        // 设置位置
        traj_marker.points[0].x = global_plan_[look_index_].pose.position.x;
        traj_marker.points[0].y = global_plan_[look_index_].pose.position.y;
        traj_marker.points[0].z = 0;
        // 颜色
        traj_marker.colors[0].r = 1.0;
        traj_marker.colors[0].g = 0;
        traj_marker.colors[0].b = 0.0;
        traj_marker.colors[0].a = 1.0;

        markers_pub.publish(markers);
    }

    // verbose publishing of point clouds
    if (publish_cost_grid_pc_) {
      // std::cout << "publish_cost_grid_pc_" << std::endl;
      //we'll publish the visualization of the costs to rviz before returning our best trajectory
      map_viz_.publishCostCloud(planner_util_->getCostmap());
    }

    // debrief stateful scoring functions
    oscillation_costs_.updateOscillationFlags(pos, &result_traj_, planner_util_->getCurrentLimits().min_vel_trans);

    //if we don't have a legal trajectory, we'll just command zero
    if (result_traj_.cost_ < 0) {
      drive_velocities.pose.position.x = 0;
      drive_velocities.pose.position.y = 0;
      drive_velocities.pose.position.z = 0;
      drive_velocities.pose.orientation.w = 1;
      drive_velocities.pose.orientation.x = 0;
      drive_velocities.pose.orientation.y = 0;
      drive_velocities.pose.orientation.z = 0;
    } else {
      drive_velocities.pose.position.x = result_traj_.xv_;
      drive_velocities.pose.position.y = result_traj_.yv_;
      drive_velocities.pose.position.z = 0;
      tf2::Quaternion q;
      q.setRPY(0, 0, result_traj_.thetav_);
      if (result_traj_.thetav_ < -0.2 || result_traj_.thetav_ > 0.2) {
        std::cout << color::RED << "result_traj_.thetav_ error !" << result_traj_.thetav_
          << color::RESET << "\n";
      } else {
        std::cout << color::GREEN << "result_traj_.thetav_ OK!" << color::RESET << "\n";
      }
      tf2::convert(q, drive_velocities.pose.orientation);
    }

    return result_traj_;
  }

  void DWAPlanner::enableReverse() {
      planner_util_->enableReverse();  
  }

  void DWAPlanner::disableReverse() {
      planner_util_->disableReverse();  
  }
};
