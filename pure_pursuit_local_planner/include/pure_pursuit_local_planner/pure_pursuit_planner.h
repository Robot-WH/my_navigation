#ifndef PURE_PURSUIT_LOCAL_PLANNER_H_
#define PURE_PURSUIT_LOCAL_PLANNER_H_

#include <vector>
#include <Eigen/Core>

#include <pure_pursuit_local_planner/PurePursuitPlannerConfig.h>

//for creating a local cost grid
#include <base_local_planner/map_grid_visualizer.h>

//for obstacle data access
#include <costmap_2d/costmap_2d.h>

#include <base_local_planner/trajectory.h>
#include <base_local_planner/local_planner_limits.h>
#include <base_local_planner/local_planner_util.h>
#include <base_local_planner/simple_trajectory_generator.h>

#include <base_local_planner/oscillation_cost_function.h>
#include <base_local_planner/map_grid_cost_function.h>
#include <base_local_planner/obstacle_cost_function.h>
#include <base_local_planner/twirling_cost_function.h>
#include <base_local_planner/velocity_cost_function.h>
#include <base_local_planner/motionDirection_cost_function.h>
#include <base_local_planner/simple_scored_sampling_planner.h>

#include <nav_msgs/Path.h>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace pure_pursuit_local_planner {
  /**
   * @class PurePursuitPlanner
   * @brief A class implementing a local planner using the Dynamic Window Approach
   */
  class PurePursuitPlanner {
    public:
      /**
       * @brief  Constructor for the planner
       * @param name The name of the planner 
       * @param costmap_ros A pointer to the costmap instance the planner should use
       * @param global_frame the frame id of the tf frame to use
       */
      PurePursuitPlanner(std::string name, tf2_ros::Buffer* tf, base_local_planner::LocalPlannerUtil *planner_util);

      /**
       * @brief Reconfigures the trajectory planner
       */
      void reconfigure(PurePursuitPlannerConfig &cfg);


      /**
       * @brief Get the period at which the local planner is expected to run
       * @return The simulation period
       */
      double getSimPeriod() { return sim_period_; }


      /**
       * sets new plan and resets state
       */
      bool setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan);

      geometry_msgs::PoseStamped goalToBaseFrame(
          const geometry_msgs::PoseStamped& goal_pose_msg);

      void UpdateFrontTargetPoint(const float& curr_pos_x, const float& curr_pos_y, const geometry_msgs::Quaternion& curr_pos_rot);

      bool CalculateMotion(geometry_msgs::Twist& cmd_vel);

      bool IsGoalReached();  

      const geometry_msgs::PoseStamped& GetFrontViewPoint();
    private:
      enum class State {begin_align, mid_run, end_align, finish} state_;
      base_local_planner::LocalPlannerUtil *planner_util_;

      double sim_period_;///< @brief The number of seconds to use to compute max/min vels for dwa
      base_local_planner::Trajectory result_traj_;

      double forward_point_distance_;

      boost::mutex configuration_mutex_;
      std::string frame_id_;
      int front_target_point_index_ = -1;     // 前向点在全局路径下的index
      float front_view_distance_ = 0;   
      float max_front_view_distance_ = 1.0;
      float min_front_view_distance_ = 0.1;
      std::vector<geometry_msgs::PoseStamped> global_plan_;
      geometry_msgs::PoseStamped front_target_point_in_base_; 

      tf2_ros::Buffer* tf_;
      float linear_v_max_;  
  };
};
#endif
