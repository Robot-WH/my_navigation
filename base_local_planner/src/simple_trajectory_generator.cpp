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
 * Author: TKruse
 *********************************************************************/

#include <base_local_planner/simple_trajectory_generator.h>
#include <iostream>
#include <cmath>

#include <base_local_planner/velocity_iterator.h>

namespace base_local_planner {

void SimpleTrajectoryGenerator::initialise(
    const Eigen::Vector3f& pos,
    const Eigen::Vector3f& vel,
    const Eigen::Vector3f& goal,
    base_local_planner::LocalPlannerLimits* limits,
    const Eigen::Vector3f& vsamples,
    std::vector<Eigen::Vector3f> additional_samples,
    bool discretize_by_time) {
  initialise(pos, vel, goal, limits, vsamples, discretize_by_time);
  // add static samples if any
  sample_params_.insert(sample_params_.end(), additional_samples.begin(), additional_samples.end());
}

// dwa每次执行的时候都会调用，用于生成候选轨迹
//  DWAPlannerROS::computeVelocityCommands -> 
// DWAPlannerROS::dwaComputeVelocityCommands -> 
// DWAPlanner::findBestPath
void SimpleTrajectoryGenerator::initialise(
    const Eigen::Vector3f& pos,
    const Eigen::Vector3f& vel,
    const Eigen::Vector3f& goal,
    base_local_planner::LocalPlannerLimits* limits,
    const Eigen::Vector3f& vsamples,
    bool discretize_by_time) {
  /*
   * We actually generate all velocity sample vectors here, from which to generate trajectories later on
   */
  double max_vel_th = limits->max_vel_theta;
  // std::cout << "max_vel_th: " << max_vel_th << std::endl;
  double min_vel_th = -1.0 * max_vel_th;
  // std::cout << "min_vel_th: " << min_vel_th << std::endl;
  discretize_by_time_ = discretize_by_time;
  Eigen::Vector3f acc_lim = limits->getAccLimits();
  acc_lim[2] = 10000;
  acc_lim[0] = 10000;
  pos_ = pos;
  vel_ = vel;     // 需要当前速度是为了后面根据最大加速度判断采样速度可不可取
  limits_ = limits;
  next_sample_index_ = 0;
  sample_params_.clear();

  double min_vel_x = limits->min_vel_x;
  double max_vel_x = limits->max_vel_x;
  // std::cout << "limits->max_vel_x: " << limits->max_vel_x << std::endl;
  // std::cout << "limits->min_vel_x: " << limits->min_vel_x << std::endl;
  double min_vel_y = limits->min_vel_y;
  double max_vel_y = limits->max_vel_y;

  // if sampling number is zero in any dimension, we don't generate samples generically
  if (vsamples[0] * vsamples[1] * vsamples[2] > 0) {
    //compute the feasible velocity space based on the rate at which we run
    Eigen::Vector3f max_vel = Eigen::Vector3f::Zero();
    Eigen::Vector3f min_vel = Eigen::Vector3f::Zero();

    if (!use_dwa_) {
      std::cout << "!use_dwa_" << "\n";
      // there is no point in overshooting the goal, and it also may break the
      // robot behavior, so we limit the velocities to those that do not overshoot in sim_time
      double dist = hypot(goal[0] - pos[0], goal[1] - pos[1]);
      max_vel_x = std::max(std::min(max_vel_x, dist / sim_time_), min_vel_x);
      max_vel_y = std::max(std::min(max_vel_y, dist / sim_time_), min_vel_y);

      // if we use continous acceleration, we can sample the max velocity we can reach in sim_time_
      max_vel[0] = std::min(max_vel_x, vel[0] + acc_lim[0] * sim_time_);
      max_vel[1] = std::min(max_vel_y, vel[1] + acc_lim[1] * sim_time_);
      max_vel[2] = std::min(max_vel_th, vel[2] + acc_lim[2] * sim_time_);

      min_vel[0] = std::max(min_vel_x, vel[0] - acc_lim[0] * sim_time_);
      min_vel[1] = std::max(min_vel_y, vel[1] - acc_lim[1] * sim_time_);
      min_vel[2] = std::max(min_vel_th, vel[2] - acc_lim[2] * sim_time_);
    } else {
      // std::cout << "use_dwa_" << "\n";
      // 根据当前速度与最大加速度 确定 采样速度的最大值   
      // with dwa do not accelerate beyond the first step, we only sample within velocities we reach in sim_period
      max_vel[0] = std::min(max_vel_x, vel[0] + acc_lim[0] * sim_period_);
      std::cout << "max_vel[0]: " << max_vel[0] 
        << ",acc_lim[0]:" << acc_lim[0] 
        << ",max_vel_x: " << max_vel_x 
        << " ,vel[0] + acc_lim[0] * sim_period_: " << vel[0] + acc_lim[0] * sim_period_ << std::endl;
      max_vel[1] = std::min(max_vel_y, vel[1] + acc_lim[1] * sim_period_);
      max_vel[2] = std::min(max_vel_th, vel[2] + acc_lim[2] * sim_period_);
      // std::cout << "max_vel[1]: " << max_vel[1] << std::endl;
      // std::cout << "max_vel_y: " << max_vel_y << std::endl;
      // std::cout << "max_vel_x: " << max_vel_x << std::endl;
      std::cout << "max_vel_th: " << max_vel_th << std::endl;
      std::cout << "vel[2]: " << vel[2] << std::endl;
      std::cout << "acc_lim[2]: " << acc_lim[2] << std::endl;
      std::cout << "sim_period: " << sim_period_ << std::endl;
      std::cout << "max_vel[2]: " << max_vel[2] << std::endl;

      min_vel[0] = std::max(min_vel_x, vel[0] - acc_lim[0] * sim_period_);
      // std::cout << "min_vel[0]: " << min_vel[0] << std::endl;
      min_vel[1] = std::max(min_vel_y, vel[1] - acc_lim[1] * sim_period_);
      // std::cout << "min_vel[1]: " << min_vel[1] << ",vel[1]: " << vel[1] << std::endl;
      min_vel[2] = std::max(min_vel_th, vel[2] - acc_lim[2] * sim_period_);
      std::cout << "min_vel_th: " << min_vel_th << std::endl;
      std::cout << "min_vel[2]: " << min_vel[2] << std::endl;
    }

    Eigen::Vector3f vel_samp = Eigen::Vector3f::Zero();
    // vsamples 为采样数，生成样本 
    // 这里 VelocityIterator 即使  min_vel > max_vel 也可以，那么采样就是从大到小
    VelocityIterator x_it(min_vel[0], max_vel[0], vsamples[0]);
    VelocityIterator y_it(min_vel[1], max_vel[1], vsamples[1]);
    VelocityIterator th_it(min_vel[2], max_vel[2], vsamples[2]);
    // std::cout << "vsamples[0]: " << vsamples[0] << "\n";

    for(; !x_it.isFinished(); x_it++) {
      vel_samp[0] = x_it.getVelocity();
      for(; !y_it.isFinished(); y_it++) {
        vel_samp[1] = y_it.getVelocity();
        for(; !th_it.isFinished(); th_it++) {
          vel_samp[2] = th_it.getVelocity();
          //ROS_DEBUG("Sample %f, %f, %f", vel_samp[0], vel_samp[1], vel_samp[2]);
          sample_params_.push_back(vel_samp);
          // std::cout << "vel_samp: " << vel_samp.transpose() << std::endl;
        }
        th_it.reset();
      }
      y_it.reset();
    }
  }
}

void SimpleTrajectoryGenerator::setParameters(
    double sim_time,
    double sim_granularity,
    double angular_sim_granularity,
    bool use_dwa,
    double sim_period) {
  std::cout << "设置轨迹生成器参数：" << std::endl;
  sim_time_ = sim_time;
  sim_granularity_ = sim_granularity;    // 细分粒度
  angular_sim_granularity_ = angular_sim_granularity;
  use_dwa_ = use_dwa;
  continued_acceleration_ = ! use_dwa_;
  sim_period_ = sim_period;   // 控制周期
  std::cout << "sim_time_: " << sim_time_ << std::endl;
  std::cout << "sim_granularity_: " << sim_granularity_ << std::endl;
  std::cout << "angular_sim_granularity_: " << angular_sim_granularity_ << std::endl;
  std::cout << "use_dwa_: " << use_dwa_ << std::endl;
  std::cout << "sim_period_: " << sim_period_ << std::endl;
}

/**
 * Whether this generator can create more trajectories
 */
bool SimpleTrajectoryGenerator::hasMoreTrajectories() {
  return next_sample_index_ < sample_params_.size();
}

/**
 * Create and return the next sample trajectory
 */
bool SimpleTrajectoryGenerator::nextTrajectory(Trajectory &comp_traj) {
  bool result = false;
  if (hasMoreTrajectories()) {
    // pos_，vel_在initialise中设置 
    if (generateTrajectory(
            pos_,   
            vel_,
            sample_params_[next_sample_index_],   // 轨迹参数，initialise中构建
            comp_traj)) {
      result = true;
    }
  }
  next_sample_index_++;
  return result;
}

/**
 * @brief 
 * 
 * @param pos pos current position of robot
 * @param vel 机器人当前速度
 * @param sample_target_vel 未来设定轨迹的运动参数
 * @param[out] traj 
 * @return true 
 * @return false 
 */
bool SimpleTrajectoryGenerator::generateTrajectory(
      Eigen::Vector3f pos,
      Eigen::Vector3f vel,
      Eigen::Vector3f sample_target_vel,
      base_local_planner::Trajectory& traj) {
  // hypot(a, b)计算的是sqrt(a*a + b*b)
  double vmag = hypot(sample_target_vel[0], sample_target_vel[1]);
  double eps = 1e-4;
  traj.cost_   = -1.0; // placed here in case we return early
  //trajectory might be reused so we'll make sure to reset it
  traj.resetPoints();
  // std::cout << "traj param: " << sample_target_vel.transpose() << "\n";
  // 确保设定的轨迹的运动速度大于最小运动速度
  if ((limits_->min_vel_trans >= 0 && vmag + eps < limits_->min_vel_trans) &&
      (limits_->min_vel_theta >= 0 && fabs(sample_target_vel[2]) + eps < limits_->min_vel_theta)) {
    // std::cout << "速度太低！！！！" << "\n";
    return false;
  }
  // 确保设定的轨迹的运动速度小于最大运动速度
  if (limits_->max_vel_trans >=0 && vmag - eps > limits_->max_vel_trans) {
    // std::cout << "速度超标！！！！vx: " << sample_target_vel[0] << "\n";
    return false;
  }

  int num_steps;
  //// discretize_by_time_为true，则按时间进行离散化，sim_granularity_为最小间隔时间
  // discretize_by_time_为false, 则按距离进行离散化，sim_granularity_为最小间隔距离
  // 默认discretize_by_time_为false
  if (discretize_by_time_) {
    num_steps = ceil(sim_time_ / sim_granularity_);
  } else {
    // compute the number of steps we must take along this trajectory to be "safe"
    double sim_time_distance = vmag * sim_time_; // the distance the robot would travel in sim_time if it did not change velocity
    double sim_time_angle = fabs(sample_target_vel[2]) * sim_time_; // the angle the robot would rotate in sim_time
    num_steps =
        ceil(std::max(sim_time_distance / sim_granularity_,
            sim_time_angle  / angular_sim_granularity_));
  }

  if (num_steps == 0) {
    // std::cout << "num_steps == 0    return false !!!!!!!!!!!!!!!!!" << "\n";
    return false;
  }

  // compute a timestep
  double dt = sim_time_ / num_steps;
  traj.time_delta_ = dt;

  Eigen::Vector3f loop_vel;
  // use_dwa_默认为true，则continued_acceleration_默认为false
  if (continued_acceleration_) {
    // 传入当前速度是为了考虑最大加速度的限制对速度进行选择
    loop_vel = computeNewVelocities(sample_target_vel, vel, limits_->getAccLimits(), dt);
    traj.xv_     = loop_vel[0];
    traj.yv_     = loop_vel[1];
    traj.thetav_ = loop_vel[2];
    // std::cout << "continued_acceleration_ true" << "\n"; 
  } else {
    // assuming sample_vel is our target velocity within acc limits for one timestep
    loop_vel = sample_target_vel;
    traj.xv_     = sample_target_vel[0];
    traj.yv_     = sample_target_vel[1];
    traj.thetav_ = sample_target_vel[2];
    // std::cout << "------------------------------------------continued_acceleration_ false" << "\n"; 
  }

  // 根据分辨率计算轨迹离散化后的每个点
  for (int i = 0; i < num_steps; ++i) {

    //add the point to the trajectory so we can draw it later if we want
    traj.addPoint(pos[0], pos[1], pos[2]);
    // continued_acceleration_默认为false
    if (continued_acceleration_) {
      //calculate velocities
      loop_vel = computeNewVelocities(sample_target_vel, loop_vel, limits_->getAccLimits(), dt);
      //ROS_WARN_NAMED("Generator", "Flag: %d, Loop_Vel %f, %f, %f", continued_acceleration_, loop_vel[0], loop_vel[1], loop_vel[2]);
    }

    //update the position of the robot using the velocities passed in
    pos = computeNewPositions(pos, loop_vel, dt);

  } // end for simulation steps

  return true; // trajectory has at least one point
}

Eigen::Vector3f SimpleTrajectoryGenerator::computeNewPositions(const Eigen::Vector3f& pos,
    const Eigen::Vector3f& vel, double dt) {
  Eigen::Vector3f new_pos = Eigen::Vector3f::Zero();
  new_pos[0] = pos[0] + (vel[0] * cos(pos[2]) + vel[1] * cos(M_PI_2 + pos[2])) * dt;
  new_pos[1] = pos[1] + (vel[0] * sin(pos[2]) + vel[1] * sin(M_PI_2 + pos[2])) * dt;
  new_pos[2] = pos[2] + vel[2] * dt;
  return new_pos;
}

/**
 * change vel using acceleration limits to converge towards sample_target-vel
 */
Eigen::Vector3f SimpleTrajectoryGenerator::computeNewVelocities(const Eigen::Vector3f& sample_target_vel,
    const Eigen::Vector3f& vel, Eigen::Vector3f acclimits, double dt) {
  Eigen::Vector3f new_vel = Eigen::Vector3f::Zero();
  for (int i = 0; i < 3; ++i) {
    if (vel[i] < sample_target_vel[i]) {
      // 加速
      new_vel[i] = std::min(double(sample_target_vel[i]), vel[i] + acclimits[i] * dt);
    } else {
      // 减速
      new_vel[i] = std::max(double(sample_target_vel[i]), vel[i] - acclimits[i] * dt);
    }
  }
  return new_vel;
}

} /* namespace base_local_planner */
