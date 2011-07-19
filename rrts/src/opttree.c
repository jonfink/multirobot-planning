/*
This file is a part of ``RRT(*)'', an incremental
sampling-based optimal motion planning library.
Copyright (c) 2010 Sertac Karaman <sertac@mit.edu>, Emilio Frazzoli <frazzoli@mit.edu>

Permission is hereby granted, free of charge, to any person
obtaining a copy of this software and associated documentation
files (the "Software"), to deal in the Software without
restriction, including without limitation the rights to use,
copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the
Software is furnished to do so, subject to the following
conditions:

The above copyright notice and this permission notice shall be
included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
OTHER DEALINGS IN THE SOFTWARE.
*/

/*
RRT* algorithm is explained in the following paper:
http://arxiv.org/abs/1005.0416
*/

#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <sys/time.h>

#include <glib.h>

#include "rrts/opttree.h"


// Allocate memory for a new node
node_t* opttree_new_node (opttree_t *self) {
    node_t *node_new = (node_t *) malloc (sizeof (node_t));
    node_new->parent = NULL;
    node_new->children = NULL;
    node_new->state = optsystem_new_state (self->optsys);
    node_new->reaches_target = FALSE;
    node_new->distance_from_root = -1.0;
    node_new->distance_from_parent = -1.0;
    node_new->traj_from_parent = NULL;
    node_new->inputs_from_parent = NULL;
    return node_new;
}


// Allocate memory for a new node, but does not allocate memory for the state
node_t* opttree_new_node_no_state () {
    node_t *node_new = (node_t *) malloc (sizeof (node_t));
    node_new->parent = NULL;
    node_new->children = NULL;
    node_new->state = NULL;
    node_new->reaches_target = FALSE;
    node_new->distance_from_root = -1.0;
    node_new->distance_from_parent = -1.0;
    node_new->traj_from_parent = NULL;
    node_new->inputs_from_parent = NULL;
    return node_new;
}


// Free node
int opttree_free_node (opttree_t *self, node_t *node) {

    optsystem_free_state (self->optsys, node->state);
    g_slist_free (node->children);

    GSList *traj_ptr = node->traj_from_parent;
    while (traj_ptr) {
        optsystem_free_state (self->optsys, (state_t *)(traj_ptr->data));
        traj_ptr = g_slist_next (traj_ptr);
    }
    g_slist_free (node->traj_from_parent);

    GSList *inputs_ptr = node->inputs_from_parent;
    while (inputs_ptr) {
        optsystem_free_input (self->optsys, (input_t*)(inputs_ptr->data));
        inputs_ptr = g_slist_next (inputs_ptr);
    }
    g_slist_free (node->inputs_from_parent);

    free (node);

    return 1;
}


// Find the nearest neighbor in the tree
node_t* opttree_find_nearest_neighbor (opttree_t *self, state_t *state_from) {

    node_t *min_node = NULL;

    kdres_t *kdres = kd_nearest (self->kdtree, optsystem_get_state_key (self->optsys, state_from));
    if (kd_res_end (kdres))  {
        printf ("ERROR: No nearest neighbors\n");
        exit(1);
    }
    min_node = kd_res_item_data (kdres);
    kd_res_free (kdres);

    return min_node;
}


// Recursively update the distances
void opttree_update_distance_from_root (opttree_t *self, node_t *node_parent, node_t *node_curr) {

    node_curr->distance_from_root = node_parent->distance_from_root + node_curr->distance_from_parent;

    // Check for reachability of the target, if so update the lower_bound
    if (optsystem_is_reaching_target (self->optsys, node_curr->state)) {
        if (node_curr->distance_from_root < self->lower_bound) {
            self->lower_bound = node_curr->distance_from_root;
            self->lower_bound_node = node_curr;
        }
        node_curr->reaches_target = 1;
    }
    else
        node_curr->reaches_target = 0;

    GSList *node_child_list = node_curr->children;
    while (node_child_list){
        opttree_update_distance_from_root (self, node_curr, (node_t *)(node_child_list->data));
        node_child_list = g_slist_next (node_child_list);
    }
}


// Adds a given trajectory to the graph and returns a pointer to the the last node
node_t *opttree_add_traj_to_graph (opttree_t *self, node_t *node_start, node_t *node_end,
                                   GSList *trajectory, int num_node_states, int *node_states, GSList *inputs) {

    node_t *node_prev = node_start;   // This variable maintains the node last added to the graph to update parents

    int trajectory_count = 0;
    int node_states_count = 0;

    GSList *subtraj_curr = NULL;
    GSList *inputs_curr = NULL;

    GSList *inputs_ptr = inputs;
    GSList *trajectory_ptr = trajectory;
    while (trajectory_ptr) {

        state_t *state_curr = (state_t *) (trajectory_ptr->data);

        // Check whether this the end of the trajectory
        if (!g_slist_next (trajectory_ptr)) {
            // This must be the last node in the traj.

            node_t *node_curr;
            if (node_end) {    // If node_end is given, then modify node end accordingly

                node_curr = node_end;
                node_t *node_parent = node_curr->parent;
                node_parent->children = g_slist_remove (node_parent->children, (gpointer) node_curr);
                // Free traj_from_parent
                GSList *traj_from_parent_ptr = node_curr->traj_from_parent;
                while (traj_from_parent_ptr) {
                    optsystem_free_state (self->optsys, (state_t *) (traj_from_parent_ptr->data));
                    traj_from_parent_ptr = g_slist_next (traj_from_parent_ptr);
                }
                g_slist_free (node_curr->traj_from_parent);
                node_curr->traj_from_parent = NULL;
                // Free input_from_parent
                GSList *inputs_from_parent_ptr = node_curr->inputs_from_parent;
                while (inputs_from_parent_ptr) {
                    optsystem_free_input (self->optsys, (input_t *) (inputs_from_parent_ptr->data));
                    inputs_from_parent_ptr = g_slist_next (inputs_from_parent_ptr);
                }
                g_slist_free (node_curr->inputs_from_parent);
                node_curr->inputs_from_parent = NULL;

                // Free this state
                optsystem_free_state (self->optsys, state_curr);

            }
            else {   // If node_end is not given, then insert this state into the graph
                node_curr = opttree_new_node_no_state ();
                node_curr->children = NULL;
                node_curr->state = state_curr;
                node_curr->reaches_target = optsystem_is_reaching_target (self->optsys, node_curr->state);

		double cost_to_go = optsystem_evaluate_cost_to_go(self->optsys, node_curr->state);
		if(cost_to_go < self->cost_to_go) {
		  self->closest_node_outside_target = node_curr;
		  self->cost_to_go = cost_to_go;
		}

                kd_insert (self->kdtree, optsystem_get_state_key (self->optsys, node_curr->state), node_curr);
		if(self->kdtree2)
		  KD_FUNC_PREFIX_kd_insert (self->kdtree2, optsystem_get_state_key (self->optsys, node_curr->state), node_curr);

                // Add node to the graph
                self->list_nodes = g_slist_prepend (self->list_nodes, (gpointer) (node_curr));
                self->num_nodes++;

            }
            node_curr->parent = node_prev;
            if (inputs) {
                input_t *input_this = (input_t *)(inputs_ptr->data);
                inputs_curr = g_slist_prepend (inputs_curr, input_this);
            }
            node_curr->inputs_from_parent = g_slist_reverse (inputs_curr);
            node_curr->traj_from_parent = g_slist_reverse (subtraj_curr);
            node_curr->distance_from_parent = optsystem_evaluate_distance_for_cost (self->optsys,
                                                                                    node_curr->inputs_from_parent);
            opttree_update_distance_from_root (self, node_prev, node_curr);

            // Add this node to the children of the previous node
            node_prev->children = g_slist_prepend (node_prev->children, node_curr);

            // Reevaluate reaches_target variables
            if (node_curr->reaches_target) {
                if (! (node_prev->reaches_target) ) {
                    node_t *node_temp = node_prev;
                    while (node_temp )  {
                        node_temp->reaches_target = TRUE;
                        node_temp = node_temp->parent;
                    }
                }

                if (node_curr->reaches_target) {
                    if (node_curr->distance_from_root < self->lower_bound) {
                        self->lower_bound = node_curr->distance_from_root;
                        self->lower_bound_node = node_curr;
                    }
                }
            }

            // Reset the pointers
            subtraj_curr = NULL;
            node_prev = node_curr;

            goto end_iteration;

        }

        if (node_states) {
            if ( trajectory_count == node_states[node_states_count] ) {

                // Create the new node
                node_t *node_curr = opttree_new_node_no_state ();
                node_curr->state =state_curr;
                node_curr->parent = node_prev;
                node_curr->children = NULL;
                node_curr->reaches_target = optsystem_is_reaching_target (self->optsys, node_curr->state);
                if (inputs) {
                    inputs_curr = g_slist_prepend (inputs_curr, inputs_ptr->data);
                }
                node_curr->inputs_from_parent = g_slist_reverse (inputs_curr);
                node_curr->traj_from_parent = g_slist_reverse (subtraj_curr);
                node_curr->distance_from_parent = optsystem_evaluate_distance_for_cost (self->optsys,
                                                                                        node_curr->inputs_from_parent);
                node_curr->distance_from_root = node_prev->distance_from_root + node_curr->distance_from_parent;

		double cost_to_go = optsystem_evaluate_cost_to_go(self->optsys, node_curr->state);
		if(cost_to_go < self->cost_to_go) {
		  self->closest_node_outside_target = node_curr;
		  self->cost_to_go = cost_to_go;
		}

                kd_insert (self->kdtree, optsystem_get_state_key (self->optsys, node_curr->state), node_curr);
		if(self->kdtree2)
		  KD_FUNC_PREFIX_kd_insert (self->kdtree2, optsystem_get_state_key (self->optsys, node_curr->state), node_curr);

                // Update the parent node children pointer
                node_prev->children = g_slist_prepend (node_prev->children, node_curr);

                // Reevaluate reaches_target variables
                if (node_curr->reaches_target) {
                    if (! (node_prev->reaches_target) ) {
                        node_t *node_temp = node_prev;
                        while (node_temp )  {
                            node_temp->reaches_target = TRUE;
                            node_temp = node_temp->parent;
                        }
                    }

                    if (node_curr->distance_from_root < self->lower_bound) {
                        self->lower_bound = node_curr->distance_from_root;
                        self->lower_bound_node = node_curr;
                    }
                }

                self->list_nodes = g_slist_prepend (self->list_nodes, node_curr);
                self->num_nodes++;

                // Reset the pointers
                subtraj_curr = NULL;
                inputs_curr = NULL;

                node_prev = node_curr;

                // Increase the node_states counter
                node_states_count++;

                goto end_iteration;

            }
        }

        // Add current state to the subtrajectory
        subtraj_curr = g_slist_prepend (subtraj_curr, state_curr);
        if (inputs)
            inputs_curr = g_slist_prepend (inputs_curr, inputs_ptr->data);

    end_iteration:

        trajectory_ptr = g_slist_next (trajectory_ptr);
        if (inputs) {
            inputs_ptr = g_slist_next (inputs_ptr);
        }
        trajectory_count++;
    }

    g_slist_free (trajectory);
    g_slist_free (inputs);

    free (node_states);

    return node_prev;
}


// Extend node towards a given sample
// If node can not be extended , e.g., a collision occurs, then returns FALSE
node_t *opttree_extend_towards_sample (opttree_t *self, node_t *node_from, state_t *state_towards) {

    // Extend the node towards the sample
    int fully_extends = 0;
    GSList *trajectory = NULL;
    int num_node_states = 0;
    int  *node_states = NULL;
    GSList *inputs = NULL;
    if (optsystem_extend_to (self->optsys, node_from->state, state_towards,
                             &fully_extends, &trajectory, &num_node_states, &node_states, &inputs) == 0) {
        return NULL;
    }

    // Add all the nodes in the trajectory to the tree
    node_t *node_curr = opttree_add_traj_to_graph (self, node_from, NULL, trajectory, num_node_states, node_states, inputs);

    /* printf("---> adding node ["); */
    /* for(int i=0; i < node_curr->state->x_count; ++i) */
    /*   printf("%2.2f ", node_curr->state->x[i]); */
    /* printf("]\n"); */

    // Check for reachability of the target, if so update the lower_bound
    if (optsystem_is_reaching_target (self->optsys, node_curr->state)) {
        if (node_curr->distance_from_root < self->lower_bound) {
            self->lower_bound = node_curr->distance_from_root;
            self->lower_bound_node = node_curr;
        }
        node_curr->reaches_target = 1;
    }
    else
        node_curr->reaches_target = 0;

    return node_curr; // Return the last node
}


// Extends a given node towards state_towards and returns the resulting state
//    does not generate a new node or populate the tree
state_t *opttree_extend_towards_sample_no_create_node (opttree_t *self, node_t *node_from, state_t *state_towards) {

    state_t *state = NULL;

    int fully_extends = 0;
    GSList *trajectory = NULL;
    int num_node_states = 0;
    int *node_states = NULL;
    GSList *inputs = NULL;
    if (optsystem_extend_to (self->optsys, node_from->state, state_towards,
                             &fully_extends, &trajectory, &num_node_states, &node_states, &inputs) == 0)
        return NULL;

    {
        // Get the last state in the trajectory
        GSList *state_trajectory = trajectory;
        while (state_trajectory) {
            state_t *state_curr = state_trajectory->data;
            if (!g_slist_next (state_trajectory))
                state = optsystem_clone_state (self->optsys, state_curr);
            state_trajectory = g_slist_next (state_trajectory);
        }
    }

    { // Define some local variables
        GSList *state_trajectory = trajectory;
        while (state_trajectory) {
            state_t *state_curr = (state_t *)(state_trajectory->data);
            optsystem_free_state (self->optsys, state_curr);
            state_trajectory = g_slist_next (state_trajectory);
        }
        g_slist_free (trajectory);
        GSList *inputs_ptr = inputs;
        while (inputs_ptr) {
            input_t *input_curr = (input_t *)(inputs_ptr->data);
            optsystem_free_input (self->optsys, input_curr);
            inputs_ptr = g_slist_next (inputs_ptr);
        }
        g_slist_free (inputs);

        // Free node_states
        free (node_states);
    }

    return state;
}


// Goes through all the nodes in node_list and returns a pointer to the node that
//    gets to state_towards with minimum cost
node_t* opttree_find_min_node_in_set (opttree_t *self, state_t *state_towards, GSList *list_nodes) {

    node_t *min_node = NULL;
    double min_cost = DBL_MAX;

    GSList *node_curr_list = list_nodes;
    while (node_curr_list ){
        node_t *node_curr = (node_t *) node_curr_list->data;


        // Extend this node towards state_towards and evaluate the cost incurred
        //    -- if the node does not extend, then move on
        int fully_extends = 0;
        GSList *trajectory = NULL;
        int num_node_states = 0;
        int *node_states = NULL;
        GSList *inputs = NULL;

        if (optsystem_extend_to (self->optsys, node_curr->state, state_towards,
                                 &fully_extends, &trajectory, &num_node_states, &node_states, &inputs) != 0) {


            if (fully_extends)
            {
                // Evaluate the total cost
                double total_cost = node_curr->distance_from_root;

                double incremental_cost = optsystem_evaluate_distance_for_cost (self->optsys, inputs);

                total_cost += incremental_cost;

                if (total_cost < min_cost) {
                    min_node = node_curr;
                    min_cost = total_cost;
                }
            }

            // Free the states
            {
                GSList* traj_temp = trajectory;
                while (traj_temp) {
                    state_t *state_this = (state_t *)(traj_temp->data);
                    optsystem_free_state (self->optsys, state_this);
                    traj_temp = g_slist_next (traj_temp);
                }
            }
            // Free the trajectory
            g_slist_free (trajectory);
            // Free Inputs
            {
                GSList *inputs_temp = inputs;
                while (inputs_temp) {
                    input_t *input_this = (input_t *)(inputs_temp->data);
                    optsystem_free_input (self->optsys, input_this);
                    inputs_temp = g_slist_next (inputs_temp);
                }
            }
            // Free the inputs list
            g_slist_free (inputs);
            // Free node_states
            free (node_states);

        }
        node_curr_list = g_slist_next (node_curr_list);
    }


    return min_node;
}


// Takes a kdres set and returns a list of nodes
GSList *opttree_kdtree_to_gslist (state_t * state, kdres_t *kdres) {

    GSList *node_list = NULL;

    kd_res_rewind (kdres);
    while (!kd_res_end(kdres)) {
        node_t * node_curr = kd_res_item_data (kdres);
        node_list = g_slist_prepend (node_list, node_curr);
        kd_res_next (kdres);
    }

    return node_list;
}


// Finds the set of nodes with max cost
GSList *opttree_find_nodes_in_ball (opttree_t *self, state_t *state, double ball_radius) {
    GSList *nodes_in_ball = NULL;

    kdres_t *kdres = kd_nearest_range (self->kdtree, optsystem_get_state_key (self->optsys, state), ball_radius);
    nodes_in_ball = opttree_kdtree_to_gslist (state, kdres);
    kd_res_free (kdres);

    return nodes_in_ball;
}


// Extends the node back to the tree
int opttree_extend_back_to_tree (opttree_t *self, node_t *node_from, GSList *node_list) {


    state_t *state_from = node_from->state;

    GSList *node_curr_list = node_list;
    while (node_curr_list) {
        node_t *node_curr = (node_t *) (node_curr_list->data);

        if (node_curr == node_from){ // If current node is the same node_from, then continue normal operation
            node_curr_list = g_slist_next (node_curr_list);
            continue;
        }

        if (node_curr == self->root) {
            node_curr_list = g_slist_next (node_curr_list);
            continue;
        }

        state_t *state_towards = node_curr->state;


        gboolean free_states = FALSE;

        // Try to extend the state towards the sample
        int fully_extends = 0;
        GSList *trajectory = NULL;
        int num_node_states = 0;
        int *node_states = NULL;
        GSList *inputs = NULL;
        if (optsystem_extend_to (self->optsys, state_from, state_towards,
                                 &fully_extends, &trajectory, &num_node_states, &node_states, &inputs) == 0) {
            fully_extends = 0;
        }

        if (fully_extends) {   // Consider rewiring the tree

            // Calculate the cummulative distance from the root till the end of the trajectory
            double dist = node_from->distance_from_root;
            dist += optsystem_evaluate_distance_for_cost (self->optsys, inputs);

            // Check whether the new branch is shorter than the existing one
            if (dist < node_curr->distance_from_root) {

                // Add the trajectory to the tree
                node_t *node_new;
                node_new = opttree_add_traj_to_graph (self, node_from, node_curr, trajectory, num_node_states, node_states, inputs);
            }
            else {                   // If the new distance from the root is not less than the current one
                free_states = TRUE;  // remember to free up the memory used by the states in the trajectory
            }
        }
        else{                      // If it does not fully extend
            free_states = TRUE;    // remember to free up the memory used by the states in the trajectory
        }                          //     OBSTACLES CAUSE NO EXTENSIONS, IN WHICH CASE fully_extend = 0

        // Free up the memory used by the trajectory if the trajectory is not registered into the graph
        if (free_states) {
            GSList *trajectory_ptr = trajectory;
            while (trajectory_ptr) {
                optsystem_free_state (self->optsys, (state_t *)(trajectory_ptr->data));
                trajectory_ptr = g_slist_next (trajectory_ptr);
            }
            g_slist_free (trajectory);
            GSList *inputs_ptr = inputs;
            while (inputs_ptr) {
                optsystem_free_input (self->optsys, (input_t *)(inputs_ptr->data));
                inputs_ptr = g_slist_next (inputs_ptr);
            }
            g_slist_free (inputs);
            free (node_states);
        }

        node_curr_list = g_slist_next (node_curr_list);
    }
    return 1;
}


int opttree_iteration (opttree_t *self) {

  // 1. Sample a state
  state_t *state_random = optsystem_new_state(self->optsys);

  int good_sample = 0;

  while(good_sample == 0) {

    double prob = 0;
    double value = rand()/(RAND_MAX + 1.0);

    if (self->lower_bound_node) {

      prob = self->target_sample_prob_after_first_solution;
      if(value < prob) {
	// Target sampling
        if (optsystem_sample_target_state (self->optsys, state_random) != 0) {
          good_sample = 1;
	  break;
        }
      }
      prob += self->dynamic_domain_sample_after_first_solution;
      if(value < prob && self->kdtree2) {
	// Dynamic domain sampling
	KD_FUNC_PREFIX_kd_sample_pos(self->kdtree2, state_random->x);
	good_sample = 1;
	break;
      }
      prob += self->custom_sample_after_first_solution;
      if(value < prob) {
	if(self->custom_sample_func) {
	  self->custom_sample_func(self->custom_sample_func_param, state_random);
	  good_sample = 1;
	  break;
	}
      }

      // Otherwise, use uniform sampling
      if (optsystem_sample_state (self->optsys, state_random) != 0) {
	good_sample = 1;
	break;
      }
    }
    else {

      prob = self->target_sample_prob_before_first_solution;
      if(value < prob) {
	// Target sampling
        if (optsystem_sample_target_state (self->optsys, state_random) != 0) {
          good_sample = 1;
	  break;
        }
      }
      prob += self->dynamic_domain_sample_before_first_solution;
      if(value < prob && self->kdtree2) {
	// Dynamic domain sampling
	KD_FUNC_PREFIX_kd_sample_pos(self->kdtree2, state_random->x);
	good_sample = 1;
	break;
      }
      prob += self->custom_sample_before_first_solution;
      if(value < prob) {
	if(self->custom_sample_func) {
	  self->custom_sample_func(self->custom_sample_func_param, state_random);
	  good_sample = 1;
	  break;
	}
      }

      // Otherwise, use uniform sampling
      if (optsystem_sample_state (self->optsys, state_random) != 0) {
	good_sample = 1;
	break;
      }
    }
  }

  node_t *node_sample = opttree_new_node_no_state();
  node_sample->state = optsystem_clone_state(self->optsys, state_random);
  self->list_samples = g_slist_prepend (self->list_samples, (gpointer) (node_sample));
  self->num_samples++;

  // A) RRT* ALGORITHM
  if (self->run_rrtstar) {

    // A.1. Calculate the ball radius constant
    self->ball_radius_last = self->ball_radius_constant
      * (sqrt(log(1+(double)(self->num_nodes))/((double)(self->num_nodes))));
    if (self->ball_radius_last >= self->ball_radius_max)
      self->ball_radius_last = self->ball_radius_max;

    // A.2. Find the nearest node
    node_t *nearest_node = opttree_find_nearest_neighbor (self, state_random);
    if (!nearest_node) {
      optsystem_free_state(self->optsys, state_random);
      return 0;
    }

    // A.3. Extend the node towards the sampled state -- just computer the extended state
    state_t *extended_state = opttree_extend_towards_sample_no_create_node (self, nearest_node, state_random);
    if (!extended_state) {
      optsystem_free_state(self->optsys, state_random);
      return 0;
    }

    // A.4. Compute the set of all close nodes around extended_state
    GSList *close_nodes = NULL;
    close_nodes = opttree_find_nodes_in_ball (self, extended_state, self->ball_radius_last);

    // A.5. Pick the node to be extended
    node_t *node_from = nearest_node;  // If close_nodes is empty,     then extend nearest_nodes
    if (close_nodes) {                 // If close_nodes is non-empty, then extend the min_node in close_nodes
      node_from = opttree_find_min_node_in_set (self, extended_state, close_nodes);
      if (!node_from)                //   If no close node can be extended, then fall back to the nearest
	node_from = nearest_node;
    }

    // A.6. Extend the appropriate node
    node_t *extended_node = opttree_extend_towards_sample (self, node_from, extended_state);
    if (!extended_node) {
      g_slist_free (close_nodes);
      optsystem_free_state(self->optsys, state_random);
      return 0;
    }

    // A.7. Rewire the tree if possible
    opttree_extend_back_to_tree (self, extended_node, close_nodes);

    optsystem_free_state (self->optsys, extended_state);

    g_slist_free (close_nodes);
  }

  // B) RRT ALGORITHM
  else {

    // B.1. Find the nearest node
    node_t *nearest_node = opttree_find_nearest_neighbor (self, state_random);
    if (!nearest_node) {
      optsystem_free_state(self->optsys, state_random);
      return 0;
    }

    // B.2. Extend the nearest towards the sample
    node_t *extended_node = opttree_extend_towards_sample (self, nearest_node, state_random);
    if (!extended_node) {
      optsystem_free_state(self->optsys, state_random);
      return 0;
    }
  }

  optsystem_free_state(self->optsys, state_random);
  return 1;
}

int opttree_take_current_best (opttree_t *self)
{
  if(self->closest_node_outside_target) {
    self->lower_bound = self->closest_node_outside_target->distance_from_root;
    self->lower_bound_node = self->closest_node_outside_target;
    return 1;
  }


  return 0;
}

int opttree_reinitialize (opttree_t *self) {

    // Clear the tree
    GSList *node_curr_list  = self->list_nodes;
    while (node_curr_list) {
        node_t *node_curr = node_curr_list->data;

        opttree_free_node (self, node_curr);

        node_curr_list = g_slist_next (node_curr_list);
    }

    node_curr_list  = self->list_samples;
    while (node_curr_list) {
        node_t *node_curr = node_curr_list->data;

        opttree_free_node (self, node_curr);

        node_curr_list = g_slist_next (node_curr_list);
    }


    g_slist_free (self->list_nodes);
    g_slist_free (self->list_samples);

    self->list_nodes = NULL;
    self->list_samples = NULL;

    // Reinitialize the basics
    self->lower_bound = DBL_MAX;
    self->lower_bound_node = NULL;

    // Reinitialize the kdtree
    kd_clear (self->kdtree);

    // Initialize the root node
    self->root = opttree_new_node (self);
    optsystem_get_initial_state (self->optsys, self->root->state);
    self->root->distance_from_root = 0.0;
    self->root->distance_from_parent = 0.0;
    self->cost_to_go = optsystem_evaluate_cost_to_go(self->optsys, self->root->state);

    kd_insert (self->kdtree, optsystem_get_state_key (self->optsys, self->root->state), self->root);

    if(self->kdtree2)
      KD_FUNC_PREFIX_kd_insert (self->kdtree2, optsystem_get_state_key (self->optsys, self->root->state), self->root);

    // Initialize the list of all nodes
    self->list_nodes = NULL;
    self->list_nodes = g_slist_prepend (self->list_nodes, (gpointer) (self->root));
    self->num_nodes = 1;

    return 1;
}


opttree_t* opttree_create (unsigned int num_states, unsigned int num_inputs) {

    opttree_t *self = (opttree_t *) calloc (sizeof (opttree_t), 1);

    // Set up the dynamical system
    self->optsys = (optsystem_t *) calloc (sizeof (optsystem_t), 1);
    self->optsys->num_states = num_states;
    self->optsys->num_inputs = num_inputs;
    optsystem_new_system (self->optsys);

    // Initialize the kdtree
    self->kdtree = kd_create (optsystem_get_num_states(self->optsys));

    int m = 10;
    double r = 3.5;
    self->kdtree2 = KD_FUNC_PREFIX_kd_create (optsystem_get_num_states(self->optsys), m, r);

    // Set the lower bound to infinity
    self->lower_bound = DBL_MAX;
    self->lower_bound_node = NULL;

    // Initialize the root node
    self->root = opttree_new_node (self);
    optsystem_get_initial_state (self->optsys, self->root->state);
    self->root->distance_from_root = 0.0;
    self->root->distance_from_parent = 0.0;

    self->cost_to_go = optsystem_evaluate_cost_to_go(self->optsys, self->root->state);

    kd_insert (self->kdtree, optsystem_get_state_key (self->optsys, self->root->state), self->root);
    if(self->kdtree2)
      KD_FUNC_PREFIX_kd_insert (self->kdtree2, optsystem_get_state_key (self->optsys, self->root->state), self->root);

    // Initialize the list of all nodes
    self->list_nodes = NULL;
    self->list_nodes = g_slist_prepend (self->list_nodes, (gpointer) (self->root));
    self->num_nodes = 1;

    self->list_samples = NULL;
    node_t *node_sample = opttree_new_node_no_state();
    node_sample->state = optsystem_clone_state(self->optsys, self->root->state);
    self->list_samples = g_slist_prepend (self->list_samples, (gpointer) (node_sample));
    self->num_samples = 1;

    // Initialize parameters to default values
    self->run_rrtstar = 1;
    self->ball_radius_constant = 30.0;
    self->ball_radius_max = 1.0;

    self->target_sample_prob_before_first_solution = 0.0;
    self->dynamic_domain_sample_before_first_solution = 0.0;
    self->custom_sample_before_first_solution = 0.0;

    self->target_sample_prob_after_first_solution = 0.0;
    self->dynamic_domain_sample_after_first_solution = 0.0;
    self->custom_sample_after_first_solution = 0.0;

    return self;
}


// Frees the memory allocated for the nodes of the tree
int opttree_free_tree (opttree_t *self) {

    GSList *list_node_curr = self->list_nodes;
    while (list_node_curr) {
      opttree_free_node (self, (node_t *)(list_node_curr->data) );
      list_node_curr = g_slist_next (list_node_curr);
    }
    g_slist_free (self->list_nodes);

    list_node_curr = self->list_samples;
    while (list_node_curr) {
        opttree_free_node (self, (node_t *)(list_node_curr->data) );
        list_node_curr = g_slist_next (list_node_curr);
    }
    g_slist_free (self->list_samples);

    return 1;
}


int opttree_destroy (opttree_t *self) {

    optsystem_free_system (self->optsys);

    opttree_free_tree (self);

    return 1;
}


int opttree_set_root_state (opttree_t *self, state_t *state) {

    optsystem_set_initial_state (self->optsys, state);
    optsystem_get_initial_state (self->optsys, self->root->state);

    return 1;
}
