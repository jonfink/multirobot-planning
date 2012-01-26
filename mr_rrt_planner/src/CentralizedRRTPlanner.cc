#include "CentralizedRRTPlanner.h"

#include <typeinfo>

extern "C" {
#include "rrts/opttree.h"
#include "rrts/optsystem.h"
}

// Extracts the optimal path and writes it into a file
  int
  optmain_write_optimal_path_to_file (opttree_t *opttree) {

    FILE *f_ptr = fopen ("/tmp/optpath.txt", "wt");

    GSList *optstates_list = NULL;
    {
      node_t *node_curr = opttree->lower_bound_node;
      while (node_curr) {
	optstates_list = g_slist_prepend (optstates_list, node_curr);
	node_curr = node_curr->parent;
      }
    }

    if (optstates_list) {
      GSList *optstates_ptr = optstates_list;
      while (optstates_ptr) {
	node_t *node_curr = (node_t *)(optstates_ptr->data);

	GSList *traj_from_parent_ptr = node_curr->traj_from_parent;
	while (traj_from_parent_ptr) {
	  state_t *state_this = (state_t *)(traj_from_parent_ptr->data);
	  for(unsigned int i=0; i < state_this->x_count; ++i)
	    fprintf (f_ptr, "%3.5lf, ", state_this->x[i]);
	  fprintf(f_ptr, "\n");
	  traj_from_parent_ptr = g_slist_next (traj_from_parent_ptr);
	}
	state_t *state_curr = node_curr->state;
	for(unsigned int i=0; i < state_curr->x_count; ++i)
	  fprintf (f_ptr, "%3.5lf, ", state_curr->x[i]);
	fprintf(f_ptr, "\n");
	optstates_ptr = g_slist_next (optstates_ptr);
      }
    }

    fclose (f_ptr);

    return 1;
  }


  // Writes the whole tree into a file
  int
  optmain_write_tree_to_file (opttree_t *opttree) {

    FILE *f_nodes_ptr = fopen ("/tmp/nodes.txt", "wt");
    FILE *f_edges_ptr = fopen ("/tmp/edges.txt", "wt");

    GSList *node_ptr = opttree->list_nodes;

    while (node_ptr) {

      node_t *node_curr = (node_t*)node_ptr->data;
      for(unsigned int i=0; i < node_curr->state->x_count; ++i)
	fprintf(f_nodes_ptr, "%5.5lf,", node_curr->state->x[i]);
      fprintf(f_nodes_ptr, "\n");

      node_t *node_parent = node_curr->parent;
      if (node_parent) {
	for(unsigned int i=0; i < node_curr->state->x_count; ++i)
	  fprintf(f_edges_ptr, "%5.5lf,", node_curr->state->x[i]);
	for(unsigned int i=0; i < node_parent->state->x_count; ++i)
	  fprintf(f_edges_ptr, "%5.5lf,", node_parent->state->x[i]);
	fprintf(f_edges_ptr, "\n");
      }

      node_ptr = g_slist_next (node_ptr);
    }

    fclose (f_nodes_ptr);
    fclose (f_edges_ptr);

    return 1;
  }

int centralized_rrt_planner_wrapper(void *class_obj, state_t *state, state_t *initial_state)
{
  CentralizedRRTPlanner *cp = reinterpret_cast<CentralizedRRTPlanner *>(class_obj);

  Col<double> state_col(state->x_count);
  for(unsigned int i=0; i < state->x_count; ++i)
    state_col[i] = state->x[i];

  if(cp->EvaluateState(state_col, (initial_state != NULL)))
    return 0;

  return 1;
}

CentralizedRRTPlanner::CentralizedRRTPlanner(ros::NodeHandle *n, string map_topic)
: CentralizedPlanner(n, map_topic)
{
  // Call this so that glib stuff works in multiple threads
  g_thread_init(NULL);

  this->opttree = NULL;
  {
      boost::recursive_mutex::scoped_lock lock(planning_state_lock);
      this->planning = false;
  }

   // RRT planner parameters
   n->param("prob_sample_target_before_soln", prob_sample_target_before_soln, 0.0);
   n->param("prob_sample_dynamic_domain_before_soln", prob_sample_dynamic_domain_before_soln, 0.0);
   n->param("prob_sample_custom_before_soln", prob_sample_custom_before_soln, 0.0);

   n->param("prob_sample_target_after_soln", prob_sample_target_after_soln, 0.0);
   n->param("prob_sample_dynamic_domain_after_soln", prob_sample_dynamic_domain_after_soln, 0.0);
   n->param("prob_sample_custom_after_soln", prob_sample_custom_after_soln, 0.0);

   n->param("steer_distance", steer_distance, 0.5);
 }

double CentralizedRRTPlanner::GetCostToGo()
{
  return this->opttree->cost_to_go;
}

void CentralizedRRTPlanner::
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
  if(planning_state && this->opttree) {

    // Stop current planning process and setup goal region
    {
      boost::recursive_mutex::scoped_lock lock(planning_state_lock);
      this->planning = false;
    }
    m_thread->join();

    this->goal.print_trans("Goal:");
    region_2d_t goal_region;
    goal_region.center[0] = this->goal(n-2);
    goal_region.center[1] = this->goal(n-1);
    goal_region.size[0] = 1.5;
    goal_region.size[1] = 1.5;
    {
      boost::recursive_mutex::scoped_lock lock(opttree_lock);
      optsystem_update_goal_region (opttree->optsys, &goal_region);
    }

    printf("Goal region: %2.2f, %2.2f (%2.2f x %2.2f)\n",
	   goal_region.center[0], goal_region.center[1],
	   goal_region.size[0], goal_region.size[1]);
  }
}


bool CentralizedRRTPlanner::
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
    opttree->optsys->anchor_x = 0.0;
    opttree->optsys->anchor_y = 0.0;
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
    opttree->optsys->obstacle_func = &(centralized_rrt_planner_wrapper);
    opttree->optsys->obstacle_func_param = (void*)(this);

    opttree->optsys->steer_distance = steer_distance;

    opttree->run_rrtstar = 0;  // Run the RRT* algorithm
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
  }

  this->StartPlanning();

  return true;
}

void CentralizedRRTPlanner::
GetNodes(list<map<int, pair<double, double> > > &nodes)
{
  nodes.clear();
  unsigned int k = 0;

  {
    //boost::recursive_mutex::scoped_lock lock(opttree_lock);

    GSList *node_ptr = opttree->list_nodes;
    //GSList *node_ptr = opttree->list_samples;

    while (node_ptr) {

      node_t *node_curr = (node_t*)node_ptr->data;
      // Insert empty map into nodes list
      nodes.push_back(map<int, pair<double,double> >());

      // Insert node positions into map
      for(unsigned int i=0; i < node_curr->state->x_count/2; ++i) {
	nodes.back().insert(make_pair(i, make_pair(node_curr->state->x[2*i],
						   node_curr->state->x[2*i+1])));
      }

      ++k;
      node_ptr = g_slist_next (node_ptr);
    }
  }

}

bool CentralizedRRTPlanner::
GetPath(list<map<int, pair<double, double> > > &path, bool append)
{
  bool result = true;

  if(!append)
    path.clear();

  if(opttree && opttree->lower_bound_node) {
    result = true;
  }
  else {
    result = false;
    return result;
  }

  {
    boost::recursive_mutex::scoped_lock lock(opttree_lock);
    optmain_write_optimal_path_to_file (opttree);
    optmain_write_tree_to_file (opttree);
  }


  {
    boost::recursive_mutex::scoped_lock lock(opttree_lock);
    GSList *optstates_list = NULL;
    {
      node_t *node_curr = opttree->lower_bound_node;

      // Check validity of lower_bound_node
      Col<double> state_col(node_curr->state->x_count);
	for(unsigned int i=0; i < node_curr->state->x_count; ++i)
	  state_col[i] = node_curr->state->x[i];

	printf("Lower bound ");
	this->EvaluateState(state_col, false, true);

      while (node_curr) {
	optstates_list = g_slist_prepend (optstates_list, node_curr);
	node_curr = node_curr->parent;
      }
    }

    if (optstates_list) {
      GSList *optstates_ptr = optstates_list;
      int k=0;
      while (optstates_ptr) {
	node_t *node_curr = (node_t *)(optstates_ptr->data);
	GSList *traj_from_parent_ptr = node_curr->traj_from_parent;
	while (traj_from_parent_ptr) {
	  if(this->print_progress) {
	    printf("waypoint %d: ", path.size());
	  }
	  path.push_back(map<int, pair<double,double> >());
	  k = 0;
	  state_t *state_this = (state_t *)(traj_from_parent_ptr->data);
	  for(int i=0; i < state_this->x_count/2; ++i) {
	    path.back().insert(make_pair(k++,
					 make_pair(state_this->x[i*2 + 0],
						   state_this->x[i*2 + 1])));
	    if(this->print_progress) {
	      printf("(%d, %2.2f, %2.2f), ", k-1,
		     state_this->x[i*2 + 0], state_this->x[i*2 + 1]);
	    }
	  }
	  if(this->print_progress) {
	    printf("\n");
	  }
	  traj_from_parent_ptr = g_slist_next (traj_from_parent_ptr);
	}
	if(this->print_progress) {
	  printf("waypoint %d: ", path.size());
	}
	path.push_back(map<int, pair<double,double> >());
	k = 0;
	state_t *state_curr = node_curr->state;
	for(int i=0; i < state_curr->x_count/2; ++i) {
	  path.back().insert(make_pair(k++,
				       make_pair(state_curr->x[i*2 + 0],
						 state_curr->x[i*2 + 1])));

	  if(this->print_progress) {
	    printf("(%d, %2.2f, %2.2f), ", k-1,
		   state_curr->x[i*2 + 0], state_curr->x[i*2 + 1]);
	  }
	}
	if(this->print_progress) {
	  printf("\n");
	}
	optstates_ptr = g_slist_next (optstates_ptr);
      }
    }
  }

  return result;
}

void CentralizedRRTPlanner::PausePlanning()
{
  opttree_lock.lock();
}

void CentralizedRRTPlanner::UnPausePlanning()
{
  opttree_lock.unlock();
}

void CentralizedRRTPlanner::
PlanningThread()
{
  //////////////////////////////////
  // Planning Thread
  int num_iterations = 32760;
  int64_t time_start = ts_now(); // Record the start time
  double acc_time = 0;
  bool planning_state;
  this->stop_planning = false;

  {
    boost::recursive_mutex::scoped_lock lock(planning_state_lock);
    this->planning = true;
    planning_state = this->planning;
  }

  opttree->cost_to_go = 10000;

  for (int i = 0;
       (i < num_iterations) && (this->time_budget > 0 || (!opttree->lower_bound_node)) && (planning_state) && (!this->stop_planning);
       i++) {

    {
      boost::recursive_mutex::scoped_lock lock(opttree_lock);
      opttree_iteration (opttree);

      acc_time = ((double)(ts_now() - time_start))/1000000.0;

      if ( this->print_progress && (i != 0 ) && (i%1 == 0) )
        if(opttree->lower_bound_node) {
          printf ("\nTime: %5.5lf/%2.2f, Cost: %5.5lf ",
                  acc_time, this->time_budget, opttree->lower_bound);
        }
        else {
          printf ("\nTime: %5.5lf, Cost: inf (%2.2f, %2.2f \%) ",
                  ((double)(ts_now() - time_start))/1000000.0,
                  this->GetCostToGo(),
                  100.0*(double)i/(double)num_iterations);
        }

      if(this->time_budget > 0.0 && (acc_time > this->time_budget)) {
        if(opttree->lower_bound_node)
          break;
      }

      {
        boost::recursive_mutex::scoped_lock lock(planning_state_lock);
        planning_state = this->planning;
      }
    }
  }

  {
    boost::recursive_mutex::scoped_lock lock(planning_state_lock);
    this->planning = false;
  }

  this->DonePlanningHook();

  printf("Done planning thread\n");

  // End Planning Thread
  /////////////////////////////////
}

void CentralizedRRTPlanner::TakeCurrentBest()
{
  {
    boost::recursive_mutex::scoped_lock lock(planning_state_lock);
    opttree_take_current_best(opttree);
  }
}
