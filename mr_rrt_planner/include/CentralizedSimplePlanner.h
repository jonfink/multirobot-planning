#ifndef _CENTRALIZED_SIMPLE_PLANNER_H_
#define _CENTRALIZED_SIMPLE_PLANNER_H_

#include "ros/ros.h"

#include "CentralizedRRTPlanner.h"

#include "armadillo"
using namespace arma;

class CentralizedSimplePlanner : public CentralizedRRTPlanner
{
 public:
  CentralizedSimplePlanner(ros::NodeHandle *n);

  virtual bool PlanConfiguration(map<int, pair<double, double> > &_current_state, double time_budget=-1.0);
  virtual bool EvaluateState(Col<double> &state, bool extend_state, bool debug=false);
};

#endif
