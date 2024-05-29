#ifndef MOTIONDIRECTION_COST_FUNCTION_H
#define MOTIONDIRECTION_COST_FUNCTION_H
#include <geometry_msgs/PoseStamped.h>
#include <base_local_planner/trajectory_cost_function.h>

namespace base_local_planner {


class MotionDirectionCostFunction: public base_local_planner::TrajectoryCostFunction {
public:

  MotionDirectionCostFunction() {}
  ~MotionDirectionCostFunction() {}

  double scoreTrajectory(Trajectory &traj);
  void setTargetPoses(std::vector<geometry_msgs::PoseStamped> target_poses);

  bool prepare() {return true;};
private:
  double NormalizationAngle(double &angle) {
      if (angle > M_PI)
          angle -= 2 * M_PI;
      else if (angle < -M_PI)
          angle += 2 * M_PI;

      return angle;
  }
  std::vector<geometry_msgs::PoseStamped> target_poses_;

};

} /* namespace base_local_planner */
#endif /* MOTIONDIRECTION_COST_FUNCTION_H */
