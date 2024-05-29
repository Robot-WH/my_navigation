#ifndef LINEARVELOCITY_COST_FUNCTION_H
#define LINEARVELOCITY_COST_FUNCTION_H

#include <base_local_planner/trajectory_cost_function.h>

namespace base_local_planner {


class LinearVelocityCostFunction: public base_local_planner::TrajectoryCostFunction {
public:

  LinearVelocityCostFunction() {}
  ~LinearVelocityCostFunction() {}

  double scoreTrajectory(Trajectory &traj);

  bool prepare() {return true;};
};

} /* namespace base_local_planner */
#endif /* LINEARVELOCITY_COST_FUNCTION_H */
