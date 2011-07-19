#include "CentralizedPlanner.h"

#include <typeinfo>

// Returns current time
int64_t
ts_now () {
  struct timeval tv;
  gettimeofday (&tv,NULL);
  return (int64_t) tv.tv_sec * 1000000 + tv.tv_usec;
}

CentralizedPlanner::CentralizedPlanner(ros::NodeHandle *n)
{
  this->map_set = false;
  ROS_INFO("Requesting the map...\n");
  map_sub = n->subscribe("/map", 1, &CentralizedPlanner::handle_map, this);

  ros::Rate r(1.0);
  while(!map_set && ros::ok()){
    ros::spinOnce();
    ROS_INFO("[CP] Still waiting on map...\n");
    r.sleep();
   }

   ROS_INFO("Got map continuing\n");

   unsigned int map_width, map_height;
   double map_resolution;
   double map_origin_x, map_origin_y;

   map_width = (unsigned int)static_map.info.width;
   map_height = (unsigned int)static_map.info.height;
   map_resolution = static_map.info.resolution;
   map_origin_x = static_map.info.origin.position.x;
   map_origin_y = static_map.info.origin.position.y;

   double inscribed_radius, circumscribed_radius, inflation_radius;
   inscribed_radius = 0.46;

   if(n->hasParam("robot_radius")){
     n->param("robot_radius", inscribed_radius, 0.15);
   }

   circumscribed_radius = inscribed_radius;
   n->param("inflation_radius", inflation_radius, 0.3);

   double raytrace_range = 3.0;
   double max_obstacle_height = 0.1;
   double obstacle_range = 2.5;

   double cost_scale;
   n->param("cost_scaling_factor", cost_scale, 10.0);

   int temp_lethal_threshold, temp_unknown_cost_value;
   n->param("lethal_cost_threshold", temp_lethal_threshold, int(100));
   n->param("unknown_cost_value", temp_unknown_cost_value, int(0));

   unsigned char lethal_threshold = std::max(std::min(temp_lethal_threshold, 255), 0);
   unsigned char unknown_cost_value = std::max(std::min(temp_unknown_cost_value, 255), 0);

   bool track_unknown_space;
   n->param("track_unknown_space", track_unknown_space, false);

   costmap = new costmap_2d::Costmap2D(map_width, map_height,
				       map_resolution, map_origin_x, map_origin_y,
				       inscribed_radius, circumscribed_radius, inflation_radius,
				       obstacle_range, max_obstacle_height, raytrace_range,
				       cost_scale, input_data_, lethal_threshold,
				       track_unknown_space, unknown_cost_value);
 }

 void CentralizedPlanner::handle_map(const nav_msgs::OccupancyGrid::ConstPtr &msg)
 {
   if(!map_set) {
     ROS_INFO("Saving map with resolution %f ...\n", msg->info.resolution);
     this->static_map = *msg;

     // We need to cast to unsigned chars from int
     unsigned int numCells = static_map.info.width * static_map.info.height;
     for(unsigned int i = 0; i < numCells; i++){
       input_data_.push_back((unsigned char) static_map.data[i]);
     }
   }
   this->map_set = true;

   this->print_progress = true;
}

void CentralizedPlanner::SetPrintProgress(bool state)
{
  this->print_progress = state;
}

double CentralizedPlanner::GetCostToGo()
{
  return 0;
}

void CentralizedPlanner::
SetGoal(int num_nodes, map<int, pair<double, double> > &node_goals)
{
  this->goal = zeros<Col<double> >(2*num_nodes);
  this->goalK = zeros<Col<double> >(2*num_nodes);
  int n = this->goal.n_elem;

  for(int i=0; i < num_nodes; ++i) {
    if(node_goals.count(i) > 0) {
      goal(i*2 + 0) = node_goals[i].first;
      goal(i*2 + 1) = node_goals[i].second;
      goalK(i*2 + 0) = 1;
      goalK(i*2 + 1) = 1;
    }
  }

  bool planning_state;
  {
    boost::recursive_mutex::scoped_lock lock(planning_state_lock);
    planning_state = this->planning;
  }

  // If we are in the middle of an existing planning process
  if(planning_state) {
    // Stop current planning process
    {
      boost::recursive_mutex::scoped_lock lock(planning_state_lock);
      this->planning = false;
    }
    m_thread->join();
  }
}

void CentralizedPlanner::
StartPlanning()
{
  m_thread = boost::shared_ptr<boost::thread>
    (new boost::thread(boost::bind(&CentralizedPlanner::PlanningThread, this)));

  {
    boost::recursive_mutex::scoped_lock lock(planning_state_lock);
    this->planning = true;
  }
}

bool CentralizedPlanner::
IsPlanning()
{
  bool ret;
  {
    boost::recursive_mutex::scoped_lock lock(planning_state_lock);
    ret = this->planning;
  }
  return ret;
}

bool CentralizedPlanner::
PlanConfiguration(map<int, pair<double, double> > &_current_state, double time_budget)
{
  int n = this->goal.n_elem;
  this->time_budget = time_budget;
  this->stop_planning = true;

  if(m_thread) {
    m_thread->join();
  }

  return true;
}

bool CentralizedPlanner::
PlanConfigurationBlock(map<int, pair<double, double> > &_current_state, double time_budget)
{
  int n = this->goal.n_elem;
  this->time_budget = time_budget;
  this->stop_planning = true;

  return true;
}

void CentralizedPlanner::
GetNodes(list<map<int, pair<double, double> > > &nodes)
{
  nodes.clear();
}

bool CentralizedPlanner::
GetPath(list<map<int, pair<double, double> > > &path, bool append)
{
  return false;
}

void CentralizedPlanner::PausePlanning()
{
}

void CentralizedPlanner::UnPausePlanning()
{
}

void CentralizedPlanner::
PlanningThread()
{

  this->DonePlanningHook();

  printf("Done planning thread\n");
}

void CentralizedPlanner::DonePlanningHook()
{

}

bool CentralizedPlanner::
EvaluateState(Col<double> &state, bool extend_state, bool debug)
{
  // This function *MUST* be overwritten by inherited classes to work properly
  return true;
}

void CentralizedPlanner::TakeCurrentBest()
{
}
