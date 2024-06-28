#include <pure_pursuit_local_planner/pure_pursuit_planner.h>
#include <base_local_planner/goal_functions.h>
#include <cmath>
//for computing path distance
#include <queue>
#include <angles/angles.h>
#include <ros/ros.h>
#include <tf2/utils.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
namespace pure_pursuit_local_planner {
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void PurePursuitPlanner::reconfigure(PurePursuitPlannerConfig &config)
{
  std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!PurePursuitPlanner::reconfigure" << std::endl;
  boost::mutex::scoped_lock l(configuration_mutex_);
  forward_point_distance_ = config.forward_point_distance;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
PurePursuitPlanner::PurePursuitPlanner(std::string name, tf2_ros::Buffer* tf, base_local_planner::LocalPlannerUtil *planner_util) :
    tf_(tf), linear_v_max_(0.2)
{
  ros::NodeHandle private_nh("~/" + name);

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

  private_nh.param("global_frame_id", frame_id_, std::string("odom"));

  state_ = State::begin_align;    
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool PurePursuitPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan) {
  global_plan_ = orig_global_plan;
  front_target_point_index_ = -1;  
  state_ = State::begin_align;
  return true;   
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void PurePursuitPlanner::UpdateFrontTargetPoint(const float& curr_pos_x, const float& curr_pos_y) {
  std::cout << "UpdateFrontTargetPoint" << "\n"; 
  bool update = true;  
  // 先判断是否需要更新前视点
  if (front_target_point_index_ >= 0) {
    std::cout << "front_target_point_index_: " << front_target_point_index_ << "\n"; 
    std::cout << "global_plan_.size(): " <<  global_plan_.size() << "\n"; 
    // 如果当前位置距离前视点的距离小于前视距离的一半  则需要更新前视点
    float diff_x = global_plan_[front_target_point_index_].pose.position.x - curr_pos_x;
    float diff_y = global_plan_[front_target_point_index_].pose.position.y - curr_pos_y;
    float distance = std::sqrt(diff_x * diff_x + diff_y * diff_y);
    std::cout << "distance: " << distance << "\n"; 
    if (distance > 0.5 * front_view_distance_) {
      std::cout << " 不需要更新前视点"  << "\n"; 
      update = false;  
    }
  }
  if (update) {
    int num = global_plan_.size() - 1;
    std::cout << " 更新前视点, global_plan_.size(): " <<  num << "  ,front_target_point_index_: " << front_target_point_index_ << "\n"; 
    // 向后寻找新的前视点  
    if (front_target_point_index_  < num) {
      ++front_target_point_index_;
      std::cout << "++front_target_point_index_: " << front_target_point_index_ << "\n"; 
        for (; front_target_point_index_ < global_plan_.size() - 1; ++front_target_point_index_) {
        float diff_x = global_plan_[front_target_point_index_].pose.position.x - curr_pos_x;
        float diff_y = global_plan_[front_target_point_index_].pose.position.y - curr_pos_y;
        float distance = std::sqrt(diff_x * diff_x + diff_y * diff_y);
        if (distance > front_view_distance_) {
          std::cout << "找到新的前视点，front_target_point_index_： " << front_target_point_index_
            << ", distance: " << distance << "\n";  
          break;
        }
      }
    }
  }
  std::cout << "front_target_point_index_: " << front_target_point_index_ << "\n"; 
  // 获得前视点位于当前机器人坐标系下的坐标
  front_target_point_in_base_ = goalToBaseFrame(global_plan_[front_target_point_index_]); 
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
geometry_msgs::PoseStamped PurePursuitPlanner::goalToBaseFrame(
          const geometry_msgs::PoseStamped& goal_pose_msg) {
  #if ROS_VERSION_GE(ROS_VERSION_MAJOR, ROS_VERSION_MINOR, ROS_VERSION_PATCH, 1, 14, 0)
    geometry_msgs::PoseStamped goal_pose, base_pose_msg;
    goal_pose = goal_pose_msg;
    goal_pose.header.stamp = ros::Time();
    std::cout << "前视点在global系，x：" << goal_pose.pose.position.x
      << ",y: " << goal_pose.pose.position.y << "\n"; 

    try {
      base_pose_msg = tf_->transform(goal_pose, "base");
      std::cout << "前视点在local系，x：" << base_pose_msg.pose.position.x
      << ",y: " << base_pose_msg.pose.position.y << "\n"; 
    } catch (tf2::TransformException& ex) {
      ROS_WARN("goalToBaseFrame transform error");
      return base_pose_msg;
    }
#else
  geometry_msgs::PoseStamped base_pose_msg;
  tf::Stamped<tf::Pose> goal_pose, base_pose;
  poseStampedMsgToTF(goal_pose_msg, goal_pose);
  goal_pose.stamp_ = ros::Time();

  try {
    tf_.transformPose(costmap_ros_->getBaseFrameID(), goal_pose, base_pose);
  } catch (tf::TransformException& ex) {
    ROS_WARN("transform err");
    return base_pose_msg;
  }

  tf::poseStampedTFToMsg(base_pose, base_pose_msg);
#endif
  return base_pose_msg;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool PurePursuitPlanner::CalculateMotion(geometry_msgs::Twist& cmd_vel) {
  float rotation_v = 0.0;
  static float linear_v = 0.0; 
  // 初始对齐状态时，纯旋转，让机器人对齐目标点
  if (state_ == State::begin_align) {
    // std::cout << "state_ == State::begin_align" << "\n"; 
    float angle = std::atan2(front_target_point_in_base_.pose.position.y, 
                                                      front_target_point_in_base_.pose.position.x);
    // std::cout << "angle: " << angle << "\n"; 
    // 误差在10度以上   则全速旋转
    if (angle > 0.1745) {
      rotation_v = 0.5;      // 30度 / s 
    } else if (angle < -0.1745) {
      rotation_v = -0.5;  
    } else if (std::fabs(angle) < 0.01745) {    // 角度 < 1时  对齐完成
      state_ = State::mid_run;    // 切换到中途跑模式  
    } else {
      rotation_v = 3 * angle;        // P控制  
    }
  } else if (state_ == State::mid_run) {
    std::cout << "state_ == State::mid_run" << "\n"; 
    // 确定线速度
    float l_2 = front_target_point_in_base_.pose.position.x * front_target_point_in_base_.pose.position.x 
                    + front_target_point_in_base_.pose.position.y * front_target_point_in_base_.pose.position.y; 
    // 更新速度，与目标点越近速度越慢  
    if (front_target_point_in_base_.pose.position.x < 0.001) {
      // 冲出去就停下
      linear_v = 0;  
    } else {
      // 这里是0.1m 内  开始减速
      if (l_2 < 0.01) {
        // 继续分段
        if (l_2 < 0.0001) {     // 0.01m距离时  
          linear_v = 100 * l_2; 
        } else {
          linear_v = std::sqrt(l_2); 
        }
      } else {
        // 加速时需要限制加速度
        if (linear_v < linear_v_max_) {
          linear_v += 0.02;  
        }
      }
    }
    // 确定角速度
    // 确保front_target_point_in_base_.pose.position.y不为0，避免除以0的错误  
    // 线速度很慢时关闭旋转
    if (front_target_point_in_base_.pose.position.y  == 0 || linear_v < 0.05) {  
      rotation_v = 0; 
    } else {
      float r = l_2 / (2 * front_target_point_in_base_.pose.position.y);  
      rotation_v = linear_v / r;    
      std::cout << "rotation_v: " << rotation_v << "\n";
      while (rotation_v > 0.7 || rotation_v < -0.7) {
        linear_v *= 0.8;  
        rotation_v = linear_v / r;    
        std::cout << "降低 rotation_v: " << rotation_v << "\n";
      }
    }
    std::cout << "linear_v: " << linear_v << "\n"; 
    // 如果速度足够小，进入下一个状态  
    if (linear_v < 0.003) {
      state_ = State::end_align;
    }
  } else if (state_ == State::end_align) {
    std::cout << "state_ == State::end_align" << "\n"; 
    // 将ROS的Quaternion转换为tf2的Quaternion  
    tf2::Quaternion q(front_target_point_in_base_.pose.orientation.x, front_target_point_in_base_.pose.orientation.y,  
                      front_target_point_in_base_.pose.orientation.z, front_target_point_in_base_.pose.orientation.w);  
    // 使用tf2的函数将Quaternion转换为欧拉角  
    // tf2::Vector3 euler_angles = q.getEuler();  
    double roll, pitch, yaw;  
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);  
    std::cout << "yaw: " << yaw << "\n"; 
    // euler_angles.z就是yaw角（绕Z轴的旋转角）  
    // double yaw = euler_angles.z();  
    // 误差在10度以上   则全速旋转
    if (yaw > 0.1745) {
      rotation_v = 0.5;      // 30度 / s 
    } else if (yaw < -0.1745) {
      rotation_v = -0.5;  
    } else if (std::fabs(yaw) < 0.01745) {    // 角度 < 1时  对齐完成
      if (front_target_point_index_ == (global_plan_.size() - 1)) {
        std::cout << "导航结束，到达目标点!" << "\n";  
        state_ = State::finish;    // 本次导航结束
      } else {
        state_ = State::begin_align;
      }
    } else {
      rotation_v = 3 * yaw;        // P控制  
    }
  }
  // 设置cmd_vel的线速度和角速度  
  cmd_vel.linear.x = linear_v;  // 设置线速度  
  cmd_vel.angular.z = rotation_v; // 设置角速度（假设是绕Z轴旋转）  
  return true;  
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool PurePursuitPlanner::IsGoalReached() {
  if (state_ == State::finish) {
    return true;
  }
  return false;   
}
};
