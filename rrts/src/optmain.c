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

#define PUBLISH_NODES_EDGES 0

#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <glib.h>
#include <sys/time.h>

#include "opttree.h"
#include "kdtree.h"

// Returns current time 
int64_t
ts_now () {
  struct timeval tv;
  gettimeofday (&tv,NULL);
  return (int64_t) tv.tv_sec * 1000000 + tv.tv_usec;
}


// Extracts the optimal path and writes it into a file
int 
optmain_write_optimal_path_to_file (opttree_t *opttree) {
    
    FILE *f_ptr = fopen ("optpath.txt", "wt"); 

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
		for(int i=0; i < state_this->x_count; ++i)
                  fprintf (f_ptr, "%3.5lf, ", state_this->x[i]);
		fprintf(f_ptr, "\n");
                traj_from_parent_ptr = g_slist_next (traj_from_parent_ptr);
            }
            state_t *state_curr = node_curr->state;
	    for(int i=0; i < state_curr->x_count; ++i)
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

    FILE *f_nodes_ptr = fopen ("nodes.txt", "wt");
    FILE *f_edges_ptr = fopen ("edges.txt", "wt");
    
    GSList *node_ptr = opttree->list_nodes; 

    while (node_ptr) {
        
        node_t *node_curr = node_ptr->data;
	for(int i=0; i < node_curr->state->x_count; ++i)
	  fprintf(f_nodes_ptr, "%5.5lf, ", node_curr->state->x[i]);
	fprintf(f_nodes_ptr, "\n");

        node_t *node_parent = node_curr->parent;        
        if (node_parent) {
	  for(int i=0; i < node_curr->state->x_count; ++i)
	    fprintf(f_edges_ptr, "%5.5lf, ", node_curr->state->x[i]);
	  for(int i=0; i < node_parent->state->x_count; ++i)
	    fprintf(f_edges_ptr, "%5.5lf, ", node_parent->state->x[i]);
	  fprintf(f_edges_ptr, "\n");
        }
        
        node_ptr = g_slist_next (node_ptr);
    }

    fclose (f_nodes_ptr);
    fclose (f_edges_ptr);

    return 1;
}

int obstacle_func(void *class_obj, state_t *state)
{
  return 0;

  /*
  double dist = sqrt( (state->x[2] - state->x[0])*(state->x[2] - state->x[0]) +
                      (state->x[3] - state->x[1])*(state->x[3] - state->x[1]) );

  if(dist > 1.0)
    return 1;

  return 0;
  */
}


int main () {

    // Setup the parameters
    int num_iterations = 4000;

    // 1. Create opttree structure
    opttree_t *opttree = opttree_create (2, 1);
    if (!opttree) {
        printf ("Memory allocation error\n");
        exit (1);
    }

    opttree->optsys->anchor_x = 0;
    opttree->optsys->anchor_y = 0;
    
    // 2. Setup the environment
    // 2.a. create the operating region
    region_2d_t operating_region = {
        .center = {0.0, 0.0},
        .size = {20.0, 20.0}
    };
    optsystem_update_operating_region (opttree->optsys, &operating_region);
    // 2.b. create the goal region
    region_2d_t goal_region = {
        .center = {8.0, 8.0},
        .size = {0.5, 0.5}
    };
    optsystem_update_goal_region (opttree->optsys, &goal_region);
    // 2.c create obstacles
    GSList *obstacle_list = NULL;
    region_2d_t *obstacle;
    obstacle = malloc (sizeof (region_2d_t));
    obstacle->center[0] = 4.0;
    obstacle->center[1] = 5.0;
    obstacle->size[0] = 4.0;
    obstacle->size[1] = 15.0;
    obstacle_list = g_slist_prepend (obstacle_list, obstacle);
    obstacle = malloc (sizeof (region_2d_t));
    obstacle->center[0] = 4.0;
    obstacle->center[1] = -7.05;
    obstacle->size[0] = 5.0;
    obstacle->size[1] = 9.0;
    obstacle_list = g_slist_prepend (obstacle_list, obstacle);
    optsystem_update_obstacles (opttree->optsys, obstacle_list);

    // 2.d create the root state
    state_t *root_state = optsystem_new_state(opttree->optsys);
    opttree_set_root_state (opttree, root_state);

    opttree->optsys->obstacle_func = &(obstacle_func);
    opttree->optsys->obstacle_func_param = NULL;

    opttree->run_rrtstar = 0;  // Run the RRT* algorithm
    opttree->ball_radius_constant = 30;
    opttree->ball_radius_max = 1.0;
    opttree->target_sample_prob_before_first_solution = 0.0;
    opttree->target_sample_prob_after_first_solution = 0.0;
    opttree->dynamic_domain_sample_before_first_solution = 0.5;
    
    // 3. Run opttree in iterations
    int64_t time_start = ts_now(); // Record the start time
    int i;
    for (i = 0; (i < num_iterations) && (!opttree->lower_bound_node); i++) {
        opttree_iteration (opttree);
        
        if ( (i != 0 ) && (i%1000 == 0) ) 
            printf ("Time: %5.5lf, Cost: %5.5lf\n", 
                    ((double)(ts_now() - time_start))/1000000.0, opttree->lower_bound); 
        


    }

    printf("Iterations to solution: %d\n", i);
    
    // 4. Save the results to the files
    optmain_write_optimal_path_to_file (opttree);
    optmain_write_tree_to_file (opttree);
        
    // 5. Destroy the opttree structure
    opttree_destroy (opttree);
    
    return 1;
}
