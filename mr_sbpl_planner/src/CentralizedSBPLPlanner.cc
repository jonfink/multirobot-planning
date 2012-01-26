#include "CentralizedSBPLPlanner.h"

class LatticeSCQ : public StateChangeQuery{
  public:
    LatticeSCQ(EnvironmentNAV2D* env, std::vector<nav2dcell_t> const & changedcellsV)
      : env_(env), changedcellsV_(changedcellsV) {
    }

    // lazy init, because we do not always end up calling this method
    virtual std::vector<int> const * getPredecessors() const{
      if(predsOfChangedCells_.empty() && !changedcellsV_.empty())
        env_->GetPredsofChangedEdges(&changedcellsV_, &predsOfChangedCells_);
      return &predsOfChangedCells_;
    }

    // lazy init, because we do not always end up calling this method
    virtual std::vector<int> const * getSuccessors() const{
      if(succsOfChangedCells_.empty() && !changedcellsV_.empty())
        env_->GetSuccsofChangedEdges(&changedcellsV_, &succsOfChangedCells_);
      return &succsOfChangedCells_;
    }

    EnvironmentNAV2D * env_;
    std::vector<nav2dcell_t> const & changedcellsV_;
    mutable std::vector<int> predsOfChangedCells_;
    mutable std::vector<int> succsOfChangedCells_;
};

CentralizedSBPLPlanner::CentralizedSBPLPlanner(ros::NodeHandle *n, string map_topic) : CentralizedPlanner(n, map_topic)
{
  n->param("sbpl/planner_type", planner_type_, string("ARAPlanner"));
  n->param("sbpl/time_budget", time_budget, 10.0);
  n->param("sbpl/initial_epsilon",initial_epsilon_,3.0);
  n->param("sbpl/environment_type", environment_type_, string("2D"));
  n->param("sbpl/forward_search", forward_search_, bool(false));
  //n->param("sbpl/primitive_filename",primitive_filename_,string(""));
  n->param("sbpl/force_scratch_limit",force_scratch_limit_,500);

  searchuntilfirstsolution = true;

  int lethal_obstacle;
  n->param("sbpl/lethal_obstacle",lethal_obstacle,20);
  lethal_obstacle_ = (unsigned char) lethal_obstacle;
  inscribed_inflated_obstacle_ = lethal_obstacle_-1;
  sbpl_cost_multiplier_ = (unsigned char) (costmap_2d::INSCRIBED_INFLATED_OBSTACLE/inscribed_inflated_obstacle_ + 1);
  ROS_DEBUG("SBPL: lethal: %uz, inscribed inflated: %uz, multiplier: %uz",lethal_obstacle,inscribed_inflated_obstacle_,sbpl_cost_multiplier_);

  env_ = new EnvironmentNAV2D();

  /////////////////////////////////////
  // Don't need these for 2D navigation
  /////////////////////////////////////
  // if(!env_->SetEnvParameter("cost_inscribed_thresh",costMapCostToSBPLCost(costmap_2d::INSCRIBED_INFLATED_OBSTACLE))){
  //   ROS_ERROR("Failed to set cost_inscribed_thresh parameter");
  //   exit(1);
  // }
  // if(!env_->SetEnvParameter("cost_possibly_circumscribed_thresh", costMapCostToSBPLCost(costmap->getCircumscribedCost()))){
  //   ROS_ERROR("Failed to set cost_possibly_circumscribed_thresh parameter");
  //   exit(1);
  // }
  int obst_cost_thresh = costMapCostToSBPLCost(costmap_2d::LETHAL_OBSTACLE);

  bool ret;
  try{
    ret = env_->InitializeEnv(costmap->getSizeInCellsX(), // width
                              costmap->getSizeInCellsY(), // height
                              0, // mapdata
                              obst_cost_thresh);
  }
  catch(SBPL_Exception e){
    ROS_ERROR("SBPL encountered a fatal exception!");
    ret = false;
  }
  if(!ret){
    ROS_ERROR("SBPL initialization failed!");
    exit(1);
  }

  try {
    ret = env_->InitializeMDPCfg(&MDPCfg);
  }
  catch(SBPL_Exception e){
    ROS_ERROR("SBPL encountered a fatal exception!");
    ret = false;
  }
  if(!ret){
    ROS_ERROR("SBPL initialization failed on MDPCfg!");
    exit(1);
  }

  // Do I need this?
  for (ssize_t ix(0); ix < costmap->getSizeInCellsX(); ++ix)
    for (ssize_t iy(0); iy < costmap->getSizeInCellsY(); ++iy)
      env_->UpdateCost(ix, iy, costMapCostToSBPLCost(costmap->getCost(ix,iy)));

  if ("ARAPlanner" == planner_type_){
    ROS_INFO("Planning with ARA*");
    planner_ = new ARAPlanner(env_, forward_search_);
  }
  else if ("ADPlanner" == planner_type_){
    ROS_INFO("Planning with AD*");
    planner_ = new ADPlanner(env_, forward_search_);
  }
  else{
    ROS_ERROR("ARAPlanner and ADPlanner are currently the only supported planners!\n");
    exit(1);
  }

  if(planner_->set_start(MDPCfg.startstateid) == 0)
    {
      ROS_ERROR("ERROR: failed to set start state\n");
      exit(1);
    }

  if(planner_->set_goal(MDPCfg.goalstateid) == 0)
    {
      ROS_ERROR("ERROR: failed to set goal state\n");
      exit(1);
    }

  planner_->set_search_mode(searchuntilfirstsolution);
  planner_->set_initialsolution_eps(initial_epsilon_);

}

unsigned char CentralizedSBPLPlanner::costMapCostToSBPLCost(unsigned char newcost){
  if(newcost == costmap_2d::LETHAL_OBSTACLE)
    return lethal_obstacle_;
  else if(newcost == costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
    return inscribed_inflated_obstacle_;
  else if(newcost == 0 || newcost == costmap_2d::NO_INFORMATION)
    return 0;
  else
    return (unsigned char) (newcost/sbpl_cost_multiplier_ + 0.5);
}

void CentralizedSBPLPlanner::SetGoal(int num_nodes, map<int, pair<double, double> > &node_goals)
{
  if(num_nodes != 1) {
    ROS_ERROR("SBPL Planner only works for one robot for now\n");
  }

  this->goal.zeros(2);
  this->goal(0) = node_goals[0].first;
  this->goal(1) = node_goals[0].second;
}

bool CentralizedSBPLPlanner::
PlanConfigurationBlock(map<int, pair<double, double> > &_current_state, double time_budget)
{
  int n = this->goal.n_elem;
  this->time_budget = time_budget;
  this->stop_planning = true;
  this->planning_failed = false;

  if(m_thread) {
    m_thread->join();
  }

  // Setup initial
  try{
    unsigned int start_ix, start_iy;
    costmap->worldToMap(_current_state[0].first, _current_state[0].second,
                        start_ix, start_iy);
    ROS_DEBUG("Setting start %2.2f, %2.2f (%d, %d)", _current_state[0].first, _current_state[0].second, start_ix, start_iy);
    int ret = env_->SetStart(start_ix, start_iy);
    if(ret < 0 || planner_->set_start(ret) == 0){
      ROS_ERROR("ERROR: failed to set start state\n");
      return false;
    }
  }
  catch(SBPL_Exception e){
    ROS_ERROR("SBPL encountered a fatal exception while setting the start state");
    return false;
  }

  // Setup goal
  try{
    unsigned int goal_ix, goal_iy;
    ROS_DEBUG("Setting goal %2.2f, %2.2f (%d, %d)", this->goal(0), this->goal(1), goal_ix, goal_iy);
    costmap->worldToMap(this->goal(0), this->goal(1), goal_ix, goal_iy);
    int ret = env_->SetGoal(goal_ix, goal_iy);
    if(ret < 0 || planner_->set_goal(ret) == 0){
      ROS_ERROR("ERROR: failed to set goal state\n");
      return false;
    }
  }
  catch(SBPL_Exception e){
    ROS_ERROR("SBPL encountered a fatal exception while setting the goal state");
    return false;
  }

  int ret;

  ROS_DEBUG("[sbpl_lattice_planner] run planner");
  {
    boost::recursive_mutex::scoped_lock lock(solution_state_lock);
    solution_stateIDs.clear();
    int solution_cost;
    try{
      ret = planner_->replan(time_budget, &solution_stateIDs, &solution_cost);
      if(ret) {
        ROS_DEBUG("Solution is found\n");
        planning_failed = false;
      }
      else{
        ROS_INFO("Solution not found\n");
        planning_failed = true;
        {
          boost::recursive_mutex::scoped_lock lock(planning_state_lock);
          this->planning = false;
        }
        solution_stateIDs.clear();
        return false;
      }
    }
    catch(SBPL_Exception e){
      ROS_ERROR("SBPL encountered a fatal exception while planning");
      {
        boost::recursive_mutex::scoped_lock lock(planning_state_lock);
        this->planning = false;
      }
      return false;
    }

    ROS_DEBUG("size of solution=%d", (int)solution_stateIDs.size());
  }

  {
    boost::recursive_mutex::scoped_lock lock(planning_state_lock);
    this->planning = false;
  }
  this->DonePlanningHook();

  if(ret)
    return true;

  return false;
}

bool CentralizedSBPLPlanner::
PlanConfiguration(map<int, pair<double, double> > &_current_state, double time_budget)
{
  int n = this->goal.n_elem;
  this->time_budget = time_budget;
  this->stop_planning = true;
  this->planning_failed = false;

  if(m_thread) {
    m_thread->join();
  }

  // Setup initial
  try{
    unsigned int start_ix, start_iy;
    costmap->worldToMap(_current_state[0].first, _current_state[0].second,
                        start_ix, start_iy);
    ROS_DEBUG("Setting start %2.2f, %2.2f (%d, %d)", _current_state[0].first, _current_state[0].second, start_ix, start_iy);
    int ret = env_->SetStart(start_ix, start_iy);
    if(ret < 0 || planner_->set_start(ret) == 0){
      ROS_ERROR("ERROR: failed to set start state\n");
      return false;
    }
  }
  catch(SBPL_Exception e){
    ROS_ERROR("SBPL encountered a fatal exception while setting the start state");
    return false;
  }

  // Setup goal
  try{
    unsigned int goal_ix, goal_iy;
    ROS_DEBUG("Setting goal %2.2f, %2.2f (%d, %d)", this->goal(0), this->goal(1), goal_ix, goal_iy);
    costmap->worldToMap(this->goal(0), this->goal(1), goal_ix, goal_iy);
    int ret = env_->SetGoal(goal_ix, goal_iy);
    if(ret < 0 || planner_->set_goal(ret) == 0){
      ROS_ERROR("ERROR: failed to set goal state\n");
      return false;
    }
  }
  catch(SBPL_Exception e){
    ROS_ERROR("SBPL encountered a fatal exception while setting the goal state");
    return false;
  }


  // Ensure that planning variable is set when we leave this function
  {
    boost::recursive_mutex::scoped_lock lock(planning_state_lock);
    this->planning = true;
  }

  this->StartPlanning();

  return true;
}

double CentralizedSBPLPlanner::GetCostToGo()
{
  return 0;
}

bool CentralizedSBPLPlanner::
GetPath(list<map<int, pair<double, double> > > &path, bool append)
{
  bool result = true;

  if(!append)
    path.clear();

  double x, y;
  int ix, iy;

  boost::recursive_mutex::scoped_lock lock(solution_state_lock);

  for(int i=0; i < solution_stateIDs.size(); ++i) {
	  path.push_back(map<int, pair<double,double> >());
    env_->GetCoordFromState(solution_stateIDs[i], ix, iy);
    costmap->mapToWorld(ix, iy, x, y);
    path.back().insert(make_pair(0, make_pair(x, y)));
  }

  return result;
}

void CentralizedSBPLPlanner::PausePlanning()
{

}

void CentralizedSBPLPlanner::UnPausePlanning()
{

}

void CentralizedSBPLPlanner::PlanningThread()
{
  bool planning_state;
  {
    boost::recursive_mutex::scoped_lock lock(planning_state_lock);
    this->planning = true;
    planning_state = this->planning;
  }

  ROS_DEBUG("[sbpl_lattice_planner] run planner");
  {
    boost::recursive_mutex::scoped_lock lock(solution_state_lock);
    solution_stateIDs.clear();
    int solution_cost;
    try{
      int ret = planner_->replan(time_budget, &solution_stateIDs, &solution_cost);
      if(ret) {
        ROS_DEBUG("Solution is found\n");
        planning_failed = false;
      }
      else{
        ROS_INFO("Solution not found\n");
        planning_failed = true;
      }
    }
    catch(SBPL_Exception e){
      ROS_ERROR("SBPL encountered a fatal exception while planning");
      {
        boost::recursive_mutex::scoped_lock lock(planning_state_lock);
        this->planning = false;
      }
      return;
    }

    ROS_DEBUG("size of solution=%d", (int)solution_stateIDs.size());
  }

  {
    boost::recursive_mutex::scoped_lock lock(planning_state_lock);
    this->planning = false;
  }
  this->DonePlanningHook();
}

void CentralizedSBPLPlanner::TakeCurrentBest()
{

}

void CentralizedSBPLPlanner::GetNodes(list<map<int, pair<double, double> > > &nodes)
{

}

bool CentralizedSBPLPlanner::
EvaluateState(Col<double> &state, bool extend_state, bool debug)
{
  int N = this->goal.n_elem/2;

  double xi, yi, xj, yj;

  //printf(".");

  if(debug) {
    printf("State: \n");
    for(int i=0; i < N; ++i) {
      printf("\t Node %d: %2.2f, %2.2f\n", i, state(i*2+0), state(i*2+1));
    }
  }

  // printf("\t");
  // for(unsigned int i=0; i < state.n_elem; ++i)
  //   printf("%2.2f ", state(i));
  // printf("\n");

  // Check for feasibility of state against costmap
  for(int i=0; i < N; ++i) {
    xi = state(i*2+0);
    yi = state(i*2+1);

    unsigned int mx, my;
    costmap->worldToMap(xi, yi, mx, my);

    unsigned char cost = costmap->getCost(mx, my);

    if(cost != costmap_2d::FREE_SPACE) {
      //printf("OBSTACLE: %2.2f %2.2f\n", xi, yi);
      //printf("x");
      return false;
    }
    // else if(cost != costmap_2d::NO_INFORMATION) {
    //   //printf("OBSTACLE: %2.2f %2.2f\n", xi, yi);
    //   //printf("x");
    //   return false;
    // }
  }

  return true;
}
