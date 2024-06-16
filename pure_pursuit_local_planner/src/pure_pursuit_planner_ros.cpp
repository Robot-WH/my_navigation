#include <pure_pursuit_local_planner/pure_pursuit_planner_ros.h>
#include <Eigen/Core>
#include <cmath>
#include <ros/console.h>
#include <pluginlib/class_list_macros.h>
#include <base_local_planner/goal_functions.h>
#include <nav_msgs/Path.h>
#include <tf2/utils.h>
#include <nav_core/parameter_magic.h>

//register this planner as a BaseLocalPlanner plugin
PLUGINLIB_EXPORT_CLASS(pure_pursuit_local_planner::PurePursuitPlannerROS, nav_core::BaseLocalPlanner)

namespace pure_pursuit_local_planner {

  void PurePursuitPlannerROS::reconfigureCB(PurePursuitPlannerConfig &config, uint32_t level) {
      std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!PurePursuitPlannerROS::reconfigureCB" << std::endl;
      if (setup_ && config.restore_defaults) {
        config = default_config_;
        config.restore_defaults = false;
      }
      if ( ! setup_) {
        default_config_ = config;
        setup_ = true;
      }

      // update generic local planner params
      base_local_planner::LocalPlannerLimits limits;
      limits.max_vel_trans = config.max_vel_trans;
      limits.min_vel_trans = config.min_vel_trans;
      limits.max_vel_x = config.max_vel_x;
      limits.min_vel_x = config.min_vel_x;
      limits.max_vel_y = config.max_vel_y;
      limits.min_vel_y = config.min_vel_y;
      limits.max_vel_theta = config.max_vel_theta;
      limits.min_vel_theta = config.min_vel_theta;
      limits.acc_lim_x = config.acc_lim_x;
      limits.acc_lim_y = config.acc_lim_y;
      limits.acc_lim_theta = config.acc_lim_theta;
      limits.acc_lim_trans = config.acc_lim_trans;
      limits.xy_goal_tolerance = config.xy_goal_tolerance;
      limits.yaw_goal_tolerance = config.yaw_goal_tolerance;
      limits.prune_plan = config.prune_plan;
      limits.trans_stopped_vel = config.trans_stopped_vel;
      limits.theta_stopped_vel = config.theta_stopped_vel;
      planner_util_.reconfigureCB(limits, config.restore_defaults);

      // update dwa specific configuration
      dp_->reconfigure(config);
  }

  PurePursuitPlannerROS::PurePursuitPlannerROS() : initialized_(false),
      odom_helper_("odom"), setup_(false) {

  }

  void PurePursuitPlannerROS::initialize(
      std::string name,
      tf2_ros::Buffer* tf,
      costmap_2d::Costmap2DROS* costmap_ros) {
    if (!initialized_) {

      ros::NodeHandle private_nh("~/" + name);
      g_plan_pub_ = private_nh.advertise<nav_msgs::Path>("global_plan", 1);
      l_plan_pub_ = private_nh.advertise<nav_msgs::Path>("local_plan", 1);
      tf_ = tf;
      costmap_ros_ = costmap_ros;
      costmap_ros_->getRobotPose(current_pose_);

      // make sure to update the costmap we'll use for this cycle
      costmap_2d::Costmap2D* costmap = costmap_ros_->getCostmap();
      // 在局部规划中   costmap_ros_的global_frame_是 odom 
      planner_util_.initialize(tf, costmap, costmap_ros_->getGlobalFrameID());

      //create the actual planner that we'll use.. it'll configure itself from the parameter server
      dp_ = boost::shared_ptr<PurePursuitPlanner>(new PurePursuitPlanner(name, tf_, &planner_util_));

      if( private_nh.getParam( "odom_topic", odom_topic_ ))
      {
        odom_helper_.setOdomTopic( odom_topic_ );
      }
      
      initialized_ = true;

      // Warn about deprecated parameters -- remove this block in N-turtle
      nav_core::warnRenamedParameter(private_nh, "max_vel_trans", "max_trans_vel");
      nav_core::warnRenamedParameter(private_nh, "min_vel_trans", "min_trans_vel");
      nav_core::warnRenamedParameter(private_nh, "max_vel_theta", "max_rot_vel");
      nav_core::warnRenamedParameter(private_nh, "min_vel_theta", "min_rot_vel");
      nav_core::warnRenamedParameter(private_nh, "acc_lim_trans", "acc_limit_trans");
      nav_core::warnRenamedParameter(private_nh, "theta_stopped_vel", "rot_stopped_vel");

      dsrv_ = new dynamic_reconfigure::Server<PurePursuitPlannerConfig>(private_nh);
      dynamic_reconfigure::Server<PurePursuitPlannerConfig>::CallbackType cb = boost::bind(&PurePursuitPlannerROS::reconfigureCB, this, _1, _2);
      dsrv_->setCallback(cb);
    }
    else{
      ROS_WARN("This planner has already been initialized, doing nothing.");
    }
  }
  
  bool PurePursuitPlannerROS::setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan) {
    if (!initialized_) {
      ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
      return false;
    }
    //when we get a new plan, we also want to clear any latch we may have on goal tolerances
    latchedStopRotateController_.resetLatching();

    ROS_INFO("Got new plan");
    return dp_->setPlan(orig_global_plan);
  }

  bool PurePursuitPlannerROS::isGoalReached() {
    if (!initialized_) {
      ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
      return false;
    }
    if (!costmap_ros_->getRobotPose(current_pose_)) {
      ROS_ERROR("Could not get robot pose");
      return false;
    }

    if(latchedStopRotateController_.isGoalReached(&planner_util_, odom_helper_, current_pose_)) {
      ROS_INFO("Goal reached");
      return true;
    } else {
      return false;
    }
  }


  PurePursuitPlannerROS::~PurePursuitPlannerROS(){
    //make sure to clean things up
    delete dsrv_;
  }


  bool PurePursuitPlannerROS::computeVelocityCommands(geometry_msgs::Twist& cmd_vel) {
    // dispatches to either dwa sampling control or stop and rotate control, depending on whether we have been close enough to goal
    // 获取当前位姿
    if (!costmap_ros_->getRobotPose(current_pose_)) {
      ROS_ERROR("Could not get robot pose");
      return false;
    }
    // 判断是否需要更新预瞄点
    dp_->UpdateFrontTargetPoint(current_pose_.pose.position.x, current_pose_.pose.position.y);  

    // // 使用latchedStopRotateController_来检查是否到达了目标位置。如果到达了，则执行停止和旋转的逻辑。
    // if (latchedStopRotateController_.isPositionReached(&planner_util_, current_pose_)) {

    // } else {
      // 计算DWA规划器的速度命令
      bool isOk = dp_->CalculateMotion(cmd_vel);
      // if (isOk) {
      //   publishGlobalPlan(transformed_plan);
      // } else {
      //   ROS_WARN_NAMED("dwa_local_planner", "DWA planner failed to produce path.");
      //   std::vector<geometry_msgs::PoseStamped> empty_plan;
      //   publishGlobalPlan(empty_plan);
      // }
    //   return isOk;
    // }
    return true; 
  }
};
