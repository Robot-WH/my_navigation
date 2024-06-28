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
#include <base_local_planner/linearVelocity_cost_function.h>

#include <math.h>

namespace base_local_planner {

double LinearVelocityCostFunction::scoreTrajectory(Trajectory &traj) {
  // std::cout << "LinearVelocityCostFunction" << std::endl;
  return 1 / (traj.xv_ * traj.xv_);  // add cost for making the robot spin
}

} /* namespace base_local_planner */
