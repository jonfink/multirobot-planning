#ifndef _CENTRALIZED_PLANNER_H_
#define _CENTRALIZED_PLANNER_H_

#include <boost/thread.hpp>
#include "ros/ros.h"

#include "nav_msgs/OccupancyGrid.h"

#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>

#include <map>
using namespace std;

#include "armadillo"
using namespace arma;

int64_t ts_now ();

class CentralizedPlanner
{
 public:
  CentralizedPlanner(ros::NodeHandle *n);

  virtual bool PlanConfiguration(map<int, pair<double, double> > &_current_state, double time_budget=-1.0);
  virtual bool PlanConfigurationBlock(map<int, pair<double, double> > &_current_state, double time_budget=-1.0);
  virtual bool EvaluateState(Col<double> &state, bool extend_state, bool debug=false);
  virtual void DonePlanningHook();

  virtual void SetGoal(int num_nodes, map<int, pair<double, double> > &node_goals);
  virtual bool IsPlanning();
  virtual bool GetPath(list<map<int, pair<double, double> > > &path, bool append=true);
  virtual void GetNodes(list<map<int, pair<double, double> > > &nodes);
  virtual double GetCostToGo();
  virtual void TakeCurrentBest();

  virtual void StartPlanning();
  virtual void PausePlanning();
  virtual void UnPausePlanning();

  virtual void SetPrintProgress(bool state);

  virtual bool PlanFailed();

 protected:
  bool print_progress;
  Col<double> goal;
  Col<double> goalK;

  bool planning_failed;

  // Variables for planning process
  boost::shared_ptr<boost::thread> m_thread;
  bool planning;
  bool stop_planning;
  boost::recursive_mutex planning_state_lock;
  virtual void PlanningThread();
  double time_budget;

  ros::Subscriber map_sub;
  bool map_set;
  void handle_map(const nav_msgs::OccupancyGrid::ConstPtr &msg);
  nav_msgs::OccupancyGrid static_map;

  costmap_2d::Costmap2D *costmap;
  std::vector<unsigned char> input_data_;
};

#endif
