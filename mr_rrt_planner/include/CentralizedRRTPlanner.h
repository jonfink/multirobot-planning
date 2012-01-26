#ifndef _CENTRALIZED_RRT_PLANNER_H_
#define _CENTRALIZED_RRT_PLANNER_H_

#include <boost/thread.hpp>
#include "ros/ros.h"

#include "CentralizedPlanner.h"

extern "C" {
#include "rrts/opttree.h"
#include "rrts/optsystem.h"
}

#include <map>
using namespace std;

#include "armadillo"
using namespace arma;

class CentralizedRRTPlanner : public CentralizedPlanner
{
 public:
  CentralizedRRTPlanner(ros::NodeHandle *n, string map_topic="/map");

  virtual bool PlanConfiguration(map<int, pair<double, double> > &_current_state, double time_budget=-1.0);

  virtual void SetGoal(int num_nodes, map<int, pair<double, double> > &node_goals);
  virtual bool GetPath(list<map<int, pair<double, double> > > &path, bool append=true);
  virtual void GetNodes(list<map<int, pair<double, double> > > &nodes);
  virtual double GetCostToGo();
  virtual void TakeCurrentBest();

  virtual void PausePlanning();
  virtual void UnPausePlanning();

 protected:
  // Variables for planning process
  opttree_t *opttree;
  boost::recursive_mutex opttree_lock;
  void PlanningThread();

  double prob_sample_target_before_soln;
  double prob_sample_dynamic_domain_before_soln;
  double prob_sample_custom_before_soln;

  double prob_sample_target_after_soln;
  double prob_sample_dynamic_domain_after_soln;
  double prob_sample_custom_after_soln;

  double steer_distance;
};

#endif
