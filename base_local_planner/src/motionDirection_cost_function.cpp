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
  // std::cout << "MotionDirectionCostFunction" << std::endl;
  // std::cout << "traj vx:" << traj.xv_ << ", traj.yv_: " << traj.yv_ << ", traj.thetav_" << traj.thetav_ << std::endl;
  // 以目标轨迹1 / 3 位置处的点作为目标点 
  int target_idx = 1;
  if (target_poses_.size() >= 40) {
    // 评估轨迹的曲率 
    // 一种简单的方法：计算中间点到起点与终点的夹角 基于余弦定理
  //   int mid_index = target_poses_.size() / 2; 
  //   int end_index = target_poses_.size() - 1; 
  //   float d_start_mid_2 = 
  //     std::pow(target_poses_[0].pose.position.y - target_poses_[mid_index].pose.position.y, 2) 
  //     + std::pow(target_poses_[0].pose.position.x - target_poses_[mid_index].pose.position.x, 2) ;
  //   float d_start_mid = std::sqrt(d_start_mid_2); 

  //   float d_start_end_2 = 
  //     std::pow(target_poses_[0].pose.position.y - target_poses_[end_index].pose.position.y, 2) 
  //     + std::pow(target_poses_[0].pose.position.x - target_poses_[end_index].pose.position.x, 2) ;
  //   float d_start_end = std::sqrt(d_start_end_2); 

  //   float d_mid_end_2 = 
  //     std::pow(target_poses_[end_index].pose.position.y - target_poses_[mid_index].pose.position.y, 2) 
  //     + std::pow(target_poses_[end_index].pose.position.x - target_poses_[mid_index].pose.position.x, 2) ;

  //   float cos_angle = (d_start_mid_2 + d_start_end_2 - d_mid_end_2) / (2 * d_start_mid * d_start_end);
  //   cos_angle = std::fabs(cos_angle);   // 越小曲率越大，越大曲率越小  
    

  //   // std::cout << "target_poses_.size(): " << target_poses_.size() << std::endl;
  //   if (cos_angle > 0.9) {
  //     target_idx = target_poses_.size() / 2;  
  //   } else if (cos_angle > 0.7) {
  //     target_idx = target_poses_.size() / 3;  
  //   } else {
  //     target_idx = target_poses_.size() / 10;
  //   }
  //   if (target_idx < 15) {
  //     target_idx = 15;  
  //   }
  // } 
  // else {
  //   // std::cout << "到末端了!!!!!!!!" << std::endl;
  //   target_idx = target_poses_.size() - 1;  
  // }
  // 计算轨迹中点与起点的连线与机器人起始朝向的夹角，夹角越大，说明机器人需要的旋转越多，预瞄点越近
    // 获取该轨迹末端的位姿
    if (traj.getPointsSize() == 0) {
      return 0;   
    }
    double traj_start_x, traj_start_y, traj_start_th;
    traj.getPoint(0, traj_start_x, traj_start_y, traj_start_th);
    NormalizationAngle(traj_start_th); 
    // 轨迹参考点 与 轨迹起点连线的倾角     [-pi, pi]
    // 根据轨迹的速度规划选择轨迹参考点index
    int ref_index = 10; 
    double direct = std::atan2(target_poses_[ref_index].pose.position.y - traj_start_x, 
                                                            target_poses_[ref_index].pose.position.x - traj_start_y);
    // std::cout << "direct: " << direct << std::endl;
    // std::cout << "traj_end_th: " << traj_end_th << std::endl;
    double diff = std::fabs(direct - traj_start_th);
    if (diff > M_PI) {
      diff = 2 * M_PI - diff;   
    }
    // std::cout << "误差: " << diff << std::endl;
    // 当前朝向与轨迹终点朝向偏差越大，则预瞄点应该越近
    if (diff > 0.7854) {   
      // 大于 45度
      target_idx = target_poses_.size() / 5;  
    } else if (diff > 0.3491) {
      // 大于20度
      target_idx = target_poses_.size() / 3;  
    } else {
      target_idx = target_poses_.size() / 2;
    }
    if (target_idx < 15) {
      target_idx = 15;  
    }
  } 
  else {
    // std::cout << "到末端了!!!!!!!!" << std::endl;
    target_idx = target_poses_.size() - 1;  
  }
  // if (target_poses_.size() >= 40) {
  //   // std::cout << "target_poses_.size(): " << target_poses_.size() << std::endl;
  //   if (traj.xv_ > 0.2) {
  //     target_idx = target_poses_.size() / 3;  
  //   } else if (traj.xv_ > 0.1) {
  //     target_idx = target_poses_.size() / 5;  
  //   } else {
  //     target_idx = target_poses_.size() / 10;  
  //   }
  //   if (target_idx < 15) {
  //     target_idx = 15;  
  //   }
  // } 
  // else {
  //   // std::cout << "到末端了!!!!!!!!" << std::endl;
  //   target_idx = target_poses_.size() - 1;  
  // }

  const auto& target_point = target_poses_[target_idx];
  // std::cout << "target_point x: " << target_point.pose.position.x  << ",y: " << target_point.pose.position.y  << std::endl; 
  // 获取该轨迹末端的位姿
  if (traj.getPointsSize() == 0) return 0;   
  double traj_end_x, traj_end_y, traj_end_th;
  traj.getEndpoint(traj_end_x, traj_end_y, traj_end_th);
  NormalizationAngle(traj_end_th); 
  //如果处与倒车状态
  // if (traj.xv_ < 0) {
  //   std::cout << "倒车, traj_end_th: " << traj_end_th << "\n"; 
  //   if (traj_end_th < 0) {
  //     traj_end_th += M_PI;  
  //   } else {
  //     traj_end_th -= M_PI;  
  //   }
  //   std::cout << "倒车, traj_end_th: " << traj_end_th << "\n"; 
  // }
  // std::cout << "traj_end_x: " << traj_end_x  << ",y: " << traj_end_y  << std::endl; 
  // 轨迹末端与target点 连线的倾角     [-pi, pi]
  double direct = std::atan2(target_point.pose.position.y - traj_end_y, 
                                                          target_point.pose.position.x - traj_end_x);
  // std::cout << "target_point.pose.position.y: " << target_point.pose.position.y << std::endl;
  // std::cout << "traj_end_y: " << traj_end_y << std::endl;
  // std::cout << "target_point.pose.position.y - traj_end_y: " << target_point.pose.position.y - traj_end_y << std::endl;
  // std::cout << "target_point.pose.position.x: " << target_point.pose.position.x << std::endl;
  // std::cout << "traj_end_x: " << traj_end_x << std::endl;
  // std::cout << "target_point.pose.position.x - traj_end_x: " << target_point.pose.position.x - traj_end_x << std::endl;

  // std::cout << "direct: " << direct << std::endl;
  // std::cout << "traj_end_th: " << traj_end_th << std::endl;
  double diff = std::fabs(direct - traj_end_th);
  if (diff > M_PI) {
    diff = 2 * M_PI - diff;   
  }
  // std::cout << "diff: " << diff << std::endl;
  return diff;
}

void MotionDirectionCostFunction::setTargetPoses(std::vector<geometry_msgs::PoseStamped> target_poses) {
  target_poses_ = target_poses;  
}
} /* namespace base_local_planner */
