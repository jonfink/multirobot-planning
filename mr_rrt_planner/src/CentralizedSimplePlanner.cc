#include "CentralizedSimplePlanner.h"

// extern "C" {
// #include "rrts/opttree.h"
// #include "rrts/optsystem.h"
// }

int centralized_simple_planner_wrapper(void *class_obj, state_t *state, state_t *initial_state)
{
  CentralizedSimplePlanner *cp = reinterpret_cast<CentralizedSimplePlanner *>(class_obj);

  Col<double> state_col(state->x_count);
  for(unsigned int i=0; i < state->x_count; ++i)
    state_col[i] = state->x[i];

  if(cp->EvaluateState(state_col, (initial_state != NULL)))
    return 0;

  return 1;
}

CentralizedSimplePlanner::CentralizedSimplePlanner(ros::NodeHandle *n) : CentralizedRRTPlanner(n)
{
  n->param("cp_simple/prob_sample_target_before_soln", prob_sample_target_before_soln, 0.0);
  n->param("cp_simple/prob_sample_dynamic_domain_before_soln", prob_sample_dynamic_domain_before_soln, 0.0);
  n->param("cp_simple/prob_sample_custom_before_soln", prob_sample_custom_before_soln, 0.0);

  n->param("cp_simple/prob_sample_target_after_soln", prob_sample_target_after_soln, 0.0);
  n->param("cp_simple/prob_sample_dynamic_domain_after_soln", prob_sample_dynamic_domain_after_soln, 0.0);
  n->param("cp_simple/prob_sample_custom_after_soln", prob_sample_custom_after_soln, 0.0);

  n->param("cp_simple/steer_distance", steer_distance, 0.5);
}

bool CentralizedSimplePlanner::
PlanConfiguration(map<int, pair<double, double> > &_current_state, double time_budget)
{
  int n = this->goal.n_elem;
  this->time_budget = time_budget;
  this->stop_planning = true;

  if(m_thread) {
    m_thread->join();
  }

  {
    boost::recursive_mutex::scoped_lock lock(opttree_lock);
    if(opttree != NULL)
      opttree_destroy(opttree);
    this->opttree = opttree_create (n, 1);
  }

  if(n == 0)
    return false;

  if (!opttree) {
    printf ("Memory allocation error\n");
    exit (1);
  }

  {
    // TODO: Make sure the 'anchor' concept is unused or removed entirely
    boost::recursive_mutex::scoped_lock lock(opttree_lock);
    opttree->optsys->anchor_x = 0;
    opttree->optsys->anchor_y = 0;
  }

  double map_center_x, map_center_y, map_size_x, map_size_y;
  map_size_x = costmap->getSizeInMetersX();
  map_size_y = costmap->getSizeInMetersY();

  map_center_x = costmap->getOriginX() + 0.5*map_size_x;
  map_center_y = costmap->getOriginY() + 0.5*map_size_y;

  region_2d_t operating_region;
  operating_region.center[0] = map_center_x;
  operating_region.center[1] = map_center_y;
  operating_region.size[0] = map_size_x;
  operating_region.size[1] = map_size_y;

  // Test case
  // operating_region.center[0] = 2.5;
  // operating_region.center[1] = 2.0;
  // operating_region.size[0] = 14;
  // operating_region.size[1] = 12;

  // Levine
  // operating_region.center[0] = -1.44;
  // operating_region.center[1] = 7.4;
  // operating_region.size[0] = 17;
  // operating_region.size[1] = 7;

  // L457 + hallway
  // operating_region.center[0] = 1.3;
  // operating_region.center[1] = 3.0;
  // operating_region.size[0] = 12;
  // operating_region.size[1] = 12;

  printf("Operating region: centered at (%2.2f, %2.2f) with dimensions (%2.2f, %2.2f)\n",
	 operating_region.center[0], operating_region.center[1],
	 operating_region.size[0], operating_region.size[1]);

  {
    boost::recursive_mutex::scoped_lock lock(opttree_lock);
    optsystem_update_operating_region (opttree->optsys, &operating_region);
  }

  this->goal.print_trans("Goal:");
  region_2d_t goal_region;
  goal_region.center[0] = this->goal(n-2);
  goal_region.center[1] = this->goal(n-1);
  // TODO: make goal region size a parameter
  goal_region.size[0] = 0.5;
  goal_region.size[1] = 0.5;
  {
    boost::recursive_mutex::scoped_lock lock(opttree_lock);
    optsystem_update_goal_region (opttree->optsys, &goal_region);
  }

  printf("Goal region: %2.2f, %2.2f (%2.2f x %2.2f)\n",
	 goal_region.center[0], goal_region.center[1],
	 goal_region.size[0], goal_region.size[1]);

  // 2.d create the root state
  state_t *root_state = optsystem_new_state(opttree->optsys);
  for(int i=0; i < this->goal.n_elem/2; ++i) {
    root_state->x[i*2 + 0] = _current_state[i].first;
    root_state->x[i*2 + 1] = _current_state[i].second;
  }

  {
    boost::recursive_mutex::scoped_lock lock(opttree_lock);
    opttree_set_root_state (opttree, root_state);
  }

  printf("Root state: ");
  for(unsigned int i=0; i < root_state->x_count; ++i)
    printf("%2.2f ", root_state->x[i]);
  printf("\n");

  {
    boost::recursive_mutex::scoped_lock lock(opttree_lock);
    opttree->optsys->obstacle_func = &(centralized_simple_planner_wrapper);
    opttree->optsys->obstacle_func_param = (void*)(this);

    opttree->optsys->steer_distance = 0.5;

    opttree->run_rrtstar = 1;  // Run the RRT* algorithm
    opttree->ball_radius_constant = 30;
    opttree->ball_radius_max = 1.0;

    opttree->target_sample_prob_before_first_solution = prob_sample_target_before_soln;
    opttree->dynamic_domain_sample_before_first_solution = prob_sample_dynamic_domain_before_soln;
    opttree->custom_sample_before_first_solution = prob_sample_custom_before_soln;
    // uniform sampling with prob 1 - 0.3 - 0.3

    opttree->target_sample_prob_after_first_solution = prob_sample_target_after_soln;
    opttree->dynamic_domain_sample_after_first_solution = prob_sample_dynamic_domain_after_soln;
    opttree->custom_sample_after_first_solution = prob_sample_custom_after_soln;
    // uniform sampling with prob 1 - 0.3 - 0.3

    if(opttree->dynamic_domain_sample_before_first_solution < 1e-5 &&
       opttree->dynamic_domain_sample_after_first_solution < 1e-5) {
      KD_FUNC_PREFIX_kd_free(opttree->kdtree2);
      opttree->kdtree2 = NULL;
    }
  }

  // Ensure that planning variable is set when we leave this function
  {
    boost::recursive_mutex::scoped_lock lock(planning_state_lock);
    this->planning = true;
  }

  this->StartPlanning();

  return true;
}


bool CentralizedSimplePlanner::
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
  }

  return true;
}
