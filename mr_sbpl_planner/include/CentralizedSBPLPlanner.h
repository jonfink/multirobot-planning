#ifndef _SBPL_PLANNER_H_
#define _SBPL_PLANNER_H_

#include "ros/ros.h"

// My planner base class (pulls in costmap as well)
#include "CentralizedPlanner.h"

// sbpl headers
#include <sbpl/headers.h>

#include "armadillo"
using namespace arma;

class CentralizedSBPLPlanner : public CentralizedPlanner
{
 public:
  CentralizedSBPLPlanner(ros::NodeHandle *n);

  bool PlanConfiguration(map<int, pair<double, double> > &_current_state, double time_budget=-1.0);
  bool PlanConfigurationBlock(map<int, pair<double, double> > &_current_state, double time_budget=-1.0);
  bool EvaluateState(Col<double> &state, bool extend_state, bool debug=false);

  void SetGoal(int num_nodes, map<int, pair<double, double> > &node_goals);
  bool GetPath(list<map<int, pair<double, double> > > &path, bool append=true);
  void GetNodes(list<map<int, pair<double, double> > > &nodes);
  double GetCostToGo();
  void TakeCurrentBest();

  void PausePlanning();
  void UnPausePlanning();

 private:
  bool initialized_;
	MDPConfig MDPCfg;
  bool searchuntilfirstsolution;
  vector<int> solution_stateIDs;

  unsigned char costMapCostToSBPLCost(unsigned char newcost);

  void PlanningThread();

  SBPLPlanner* planner_;
  EnvironmentNAV2D* env_;

  std::string planner_type_; /**< sbpl method to use for planning.  choices are ARAPlanner and ADPlanner */

  double initial_epsilon_; /**< initial epsilon for beginning the anytime search */

  std::string environment_type_; /** what type of environment in which to plan.  choices are 2D and XYThetaLattice. */

  bool forward_search_; /** whether to use forward or backward search */
  int force_scratch_limit_; /** the number of cells that have to be changed in the costmap to force the planner to plan from scratch even if its an incremental planner */

  unsigned char lethal_obstacle_;
  unsigned char inscribed_inflated_obstacle_;
  unsigned char sbpl_cost_multiplier_;

  boost::recursive_mutex solution_state_lock;
};

#endif
