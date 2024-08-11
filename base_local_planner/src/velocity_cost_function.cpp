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
  float rot_factor = std::pow(1 + std::fabs(traj.xv_), 2 * std::fabs(traj.thetav_));
  // 速度 0 - 1左右 
  // 线速度为0.1时，速度项 大约 100 
  // 线速度为1时，速度项 大约 1
  // 线速度为1时，角速度为0.1(10度),  角速度项  约 5    
  // 线速度为1时，角速度为0.5(30度), 角速度项  约 600
  // 线速度为0.1时，角速度为0.1, 角速度项  约1.38
  // 线速度为0.1时，角速度为0.5   角速度项   约  5
  // if (traj.xv_ < 0) {
  //   return (0.1 / (traj.xv_ * traj.xv_)) + 0.1 * rot_factor;  
  // }
  // return (0.01 / (traj.xv_ * traj.xv_)) + 0.1 * rot_factor;  // add cost for making the robot spin
  // return 0.1 * rot_factor;  
  if (traj.xv_ < 0) {
    return (0.5 / (traj.xv_ * traj.xv_)) + rot_factor;  
  }
  return (0.1 / (traj.xv_ * traj.xv_)) + rot_factor;  // add cost for making the robot spin
}

} /* namespace base_local_planner */
