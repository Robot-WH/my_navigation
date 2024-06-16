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
  void PurePursuitPlanner::reconfigure(PurePursuitPlannerConfig &config)
  {
    std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!PurePursuitPlanner::reconfigure" << std::endl;
    boost::mutex::scoped_lock l(configuration_mutex_);

    generator_.setParameters(
        config.sim_time,
        config.sim_granularity,
        config.angular_sim_granularity,
        config.use_dwa,
        sim_period_);

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
    linearVelocity_costs_.setScale(0.001);  
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

  PurePursuitPlanner::PurePursuitPlanner(std::string name, tf2_ros::Buffer* tf, base_local_planner::LocalPlannerUtil *planner_util) :
      tf_(tf), linear_v_(0.3),
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
    map_viz_.initialize(name, planner_util->getGlobalFrame(), boost::bind(&PurePursuitPlanner::getCellCosts, this, _1, _2, _3, _4, _5, _6));

    private_nh.param("global_frame_id", frame_id_, std::string("odom"));

    traj_cloud_pub_ = private_nh.advertise<sensor_msgs::PointCloud2>("trajectory_cloud", 1);
    private_nh.param("publish_traj_pc", publish_traj_pc_, false);

    // set up all the cost functions that will be applied in order
    // (any function returning negative values will abort scoring, so the order can improve performance)
    std::vector<base_local_planner::TrajectoryCostFunction*> critics;
    critics.push_back(&oscillation_costs_); // 运动打分判断，是否震荡，是：代价大
    critics.push_back(&obstacle_costs_); // 障碍物碰撞检测打分，碰到障碍物，代价值增大
    // critics.push_back(&goal_front_costs_); // 局部轨迹与局部路径的最终点的朝向一致
    // critics.push_back(&alignment_costs_); // 局部轨迹与局部路径的朝向一致
    // critics.push_back(&path_costs_); // 局部轨迹（根据当前速度外推出的轨迹）与局部路径（规划的路径）对比，局部轨迹离局部路径的横向偏差小，其代价值就小
    // critics.push_back(&goal_costs_); // 局部轨迹与局部路径的终点进行对比，希望距离小
    critics.push_back(&linearVelocity_costs_); 
    // critics.push_back(&twirling_costs_); // 机器人旋转不要太大
    critics.push_back(&motionDirection_costs_);   

    // trajectory generators
    std::vector<base_local_planner::TrajectorySampleGenerator*> generator_list;
    generator_list.push_back(&generator_);

    scored_sampling_planner_ = base_local_planner::SimpleScoredSamplingPlanner(generator_list, critics);

    private_nh.param("cheat_factor", cheat_factor_, 1.0);
  }

  // used for visualization only, total_costs are not really total costs
  bool PurePursuitPlanner::getCellCosts(int cx, int cy, float &path_cost, float &goal_cost, float &occ_cost, float &total_cost) {

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

  bool PurePursuitPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan) {
    global_plan_ = orig_global_plan;
    front_target_point_index_ = -1;  
    return true;   
  }

  void PurePursuitPlanner::UpdateFrontTargetPoint(const float& curr_pos_x, const float& curr_pos_y) {
    std::cout << "UpdateFrontTargetPoint" << "\n"; 
    bool update = true;  
    // 先判断是否需要更新前视点
    if (front_target_point_index_ >= 0) {
      std::cout << "front_target_point_index_: " << front_target_point_index_ << "\n"; 
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
      // 向后寻找新的前视点  
      ++front_target_point_index_;

      for (; front_target_point_index_ < global_plan_.size(); ++front_target_point_index_) {
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
    // 获得前视点位于当前机器人坐标系下的坐标
    front_target_point_in_base_ = goalToBaseFrame(global_plan_[front_target_point_index_]); 
  }


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

bool PurePursuitPlanner::CalculateMotion(geometry_msgs::Twist& cmd_vel) {
  // 确保front_target_point_in_base_.pose.position.y不为0，避免除以0的错误  
  if (front_target_point_in_base_.pose.position.y == 0.0) {  
    // 处理错误情况，例如返回false或设置默认速度  
    ROS_ERROR("Target Y position is zero, cannot calculate radius.");  
    cmd_vel.linear.x = 0.0;  
    cmd_vel.angular.z = 0.0;  
    return false;  
  }  
  float l_2 = front_target_point_in_base_.pose.position.x * front_target_point_in_base_.pose.position.x 
                  + front_target_point_in_base_.pose.position.y * front_target_point_in_base_.pose.position.y; 
  float r = l_2 / (2 * front_target_point_in_base_.pose.position.y);  
  // 注意：这里假设linear_v_已经被正确设置，并且不为0  
  float rotation_v = linear_v_ / r;    
  // 设置cmd_vel的线速度和角速度  
  cmd_vel.linear.x = linear_v_;  // 设置线速度  
  cmd_vel.angular.z = rotation_v; // 设置角速度（假设是绕Z轴旋转）  
  return true;  
}

};
