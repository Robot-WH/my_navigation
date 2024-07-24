/**
 * @file twirling_cost_function copy.cpp
 * @author lwh ()
 * @brief 
 * @version 0.1
 * @date 2024-05-26
 * 
 * @copyright Copyright (c) 2024
 * 
 */
#include <iostream>
#include <base_local_planner/velocity_cost_function.h>

#include <math.h>

namespace base_local_planner {
/**
 * @brief 线速度越快越好 ，且线速度越大越惩罚角速度
 * 
 * @param traj 
 * @return double 
 */
double VelocityCostFunction::scoreTrajectory(Trajectory &traj) {
  // std::cout << "VelocityCostFunction" << std::endl;
  // 线速度 0.1-1m左右       数量级别  1-100
  // float rot_factor = 50 * log10(std::fabs(traj.thetav_)) / log10(std::fabs(traj.xv_));
  float rot_factor = 0;
  // float rot_factor = std::fabs(traj.thetav_) * std::fabs(traj.xv_);
  // 角速度 0 - 1左右     
  if (traj.xv_ < 0) {
    return (5 / (traj.xv_ * traj.xv_)) * (1 + rot_factor);  
  }
  return (1 / (traj.xv_ * traj.xv_)) * (1 + rot_factor);  // add cost for making the robot spin
}

} /* namespace base_local_planner */
