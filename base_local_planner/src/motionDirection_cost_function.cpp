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

void MotionDirectionCostFunction::SetGlobalTrajTargetIndex(const int& index) {
  target_idx_ = index; 
  const auto& target_point = target_poses_[target_idx_];
  std::cout << "重值DWA目标点，x: " << target_point.pose.position.x << ",y: "
    << target_point.pose.position.y << "\n"; 
}

double MotionDirectionCostFunction::scoreTrajectory(Trajectory &traj) {                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     
  std::cout << "MotionDirectionCostFunction" << std::endl;
  std::cout << "traj vx:" << traj.xv_ << ", traj.yv_: " << traj.yv_ << ", traj.thetav_" << traj.thetav_ << std::endl;
 
  const auto& target_point = target_poses_[target_idx_];
  std::cout << "target_point x: " << target_point.pose.position.x  << ",y: " << target_point.pose.position.y  << std::endl; 
  // 获取该轨迹末端的位姿
  if (traj.getPointsSize() == 0) return 0;   
  double traj_end_x, traj_end_y, traj_end_th;
  traj.getEndpoint(traj_end_x, traj_end_y, traj_end_th);
  NormalizationAngle(traj_end_th); 

  std::cout << "traj_end_x: " << traj_end_x  << ",traj_end_y: " << traj_end_y << std::endl; 
  // 轨迹末端与target点 连线的倾角     [-pi, pi]
  double direct = std::atan2(target_point.pose.position.y - traj_end_y, 
                                                          target_point.pose.position.x - traj_end_x);
  std::cout << "direct: " << direct << std::endl;

    // double direct = std::atan2( - traj_end_y, 
    //                                                       -4 - traj_end_x);
  // std::cout << "target_point.pose.position.y: " << target_point.pose.position.y << std::endl;
  // std::cout << "traj_end_y: " << traj_end_y << std::endl;
  // std::cout << "target_point.pose.position.y - traj_end_y: " << target_point.pose.position.y - traj_end_y << std::endl;
  // std::cout << "target_point.pose.position.x: " << target_point.pose.position.x << std::endl;
  // std::cout << "traj_end_x: " << traj_end_x << std::endl;
  // std::cout << "target_point.pose.position.x - traj_end_x: " << target_point.pose.position.x - traj_end_x << std::endl;

  std::cout << "traj_end_th: " << traj_end_th << std::endl;
  double diff = std::fabs(direct - traj_end_th);
  if (diff > M_PI) {
    diff = 2 * M_PI - diff;   
  }
  std::cout << "diff: " << diff << std::endl;
  // 计算轨迹末端与目标点的距离
  double dis = std::sqrt((target_point.pose.position.y - traj_end_y) * (target_point.pose.position.y - traj_end_y) + 
                              (target_point.pose.position.x - traj_end_x) * (target_point.pose.position.x - traj_end_x));
    // double dis = std::fabs(- traj_end_y) + 
    //                           std::fabs(-4 - traj_end_x);
  
  // return 10 * dis;
  // const auto& target_point = target_poses_[target_idx_];
  // if (traj.getPointsSize() == 0) return 0;   
  // 获取机器人起始位姿
  double traj_begin_x, traj_begin_y, traj_begin_th;
  traj.getPoint(0, traj_begin_x, traj_begin_y, traj_begin_th);
  std::cout << "traj_begin_x: " << traj_begin_x  << ",traj_end_y: " << traj_begin_y 
    << ", traj_begin_th: " << traj_begin_th << std::endl; 
  // NormalizationAngle(traj_begin_th); 
  // // 机器人与target点 连线的倾角     [-pi, pi]
  // double direct_robot_to_target = std::atan2(target_point.pose.position.y - traj_begin_y, 
  //                                                                                             target_point.pose.position.x - traj_begin_x);
  // // 轨迹末端与机器人连线的倾角
  // // 获取该轨迹末端的位姿
  // double traj_end_x, traj_end_y, traj_end_th;
  // traj.getEndpoint(traj_end_x, traj_end_y, traj_end_th);
  // NormalizationAngle(traj_end_th); 
  // double direct_robot_to_traj_tail = std::atan2(traj_end_y - traj_begin_y, 
  //                                                         traj_end_x - traj_begin_x);

  // double diff = std::fabs(direct_robot_to_target - direct_robot_to_traj_tail);
  // if (diff > M_PI) {
  //   diff = 2 * M_PI - diff;   
  // }
  // // 计算轨迹末端与目标点的距离
  // double dis = std::sqrt((target_point.pose.position.y - traj_end_y) * (target_point.pose.position.y - traj_end_y) + 
  //                             (target_point.pose.position.x - traj_end_x) * (target_point.pose.position.x - traj_end_x));

  // return diff + 10 * dis;
  // return dis;
  return diff;
}

void MotionDirectionCostFunction::setTargetPoses(std::vector<geometry_msgs::PoseStamped> target_poses) {
  target_poses_ = target_poses;  
}
} /* namespace base_local_planner */
