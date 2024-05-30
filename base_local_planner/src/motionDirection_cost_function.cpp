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
#include <base_local_planner/motionDirection_cost_function.h>

#include <math.h>

namespace base_local_planner {

double MotionDirectionCostFunction::scoreTrajectory(Trajectory &traj) {
  std::cout << "MotionDirectionCostFunction" << std::endl;
  std::cout << "traj vx:" << traj.xv_ << ", traj.yv_: " << traj.yv_ << ", traj.thetav_" << traj.thetav_ << std::endl;
  // 以目标轨迹1 / 3 位置处的点作为目标点 
  int target_idx = 1;
  if (target_poses_.size() >= 40) {
    // std::cout << "target_poses_.size(): " << target_poses_.size() << std::endl;
    if (traj.xv_ > 0.2) {
      target_idx = target_poses_.size() / 3;  
    } else if (traj.xv_ > 0.1) {
      target_idx = target_poses_.size() / 5;  
    } else {
      target_idx = target_poses_.size() / 10;  
    }
    if (target_idx < 15) {
      target_idx = 15;  
    }
  } 
  else {
    // std::cout << "到末端了!!!!!!!!" << std::endl;
    target_idx = target_poses_.size() - 1;  
  }

  const auto& target_point = target_poses_[target_idx];
  std::cout << "target_point x: " << target_point.pose.position.x  << ",y: " << target_point.pose.position.y  << std::endl; 
  // 获取该轨迹末端的位姿
  if (traj.getPointsSize() == 0) return 0;   
  double traj_end_x, traj_end_y, traj_end_th;
  traj.getEndpoint(traj_end_x, traj_end_y, traj_end_th);
  NormalizationAngle(traj_end_th); 
  std::cout << "traj_end_x: " << traj_end_x  << ",y: " << traj_end_y  << std::endl; 
  // 轨迹末端与target点 连线的倾角
  double direct = std::atan2(target_point.pose.position.y - traj_end_y, 
                                                          target_point.pose.position.x - traj_end_x);
  std::cout << "direct: " << direct << std::endl;
  std::cout << "traj_end_th: " << traj_end_th << std::endl;
  double diff = std::fabs(direct - traj_end_th);
  if (diff > M_PI) {
    diff = 2 * M_PI - diff;   
  }
  std::cout << "diff: " << diff << std::endl;
  return diff;
}

void MotionDirectionCostFunction::setTargetPoses(std::vector<geometry_msgs::PoseStamped> target_poses) {
  target_poses_ = target_poses;  
}

} /* namespace base_local_planner */
