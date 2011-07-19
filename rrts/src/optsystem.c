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

#include "rrts/optsystem.h"


// Returns 1 iff (state) is on an obstacle
gboolean
optsystem_on_obstacle (optsystem_t *self, state_t *state, state_t *initial_state);

// Returns 1 iff the line connecting (state_initial) and (state_final) lies on an obstacle
int
optsystem_segment_on_obstacle (optsystem_t *self, state_t *state_initial, state_t *state_final, int num_steps);

// Allocates memory for and initializes an empty dynamical system
int optsystem_new_system (optsystem_t *self) {

  self->initial_state = optsystem_new_state(self);

  self->obstacle_list = NULL;
  self->steer_distance = 1.0; // default steer distance

  self->obstacle_func = NULL;

  return 1;
}


// Frees up the memory used by optsystem_t *self
int optsystem_free_system (optsystem_t *self) {

  optsystem_free_state(self, self->initial_state);

  return 1;
}


// Allocates memory for a new state
state_t* optsystem_new_state (optsystem_t *self) {
    state_t *state = (state_t *) malloc (sizeof (state_t));
    state->x_count = self->num_states;
    state->x = (double *) malloc (sizeof (double)*state->x_count);

    for (int i = 0; i < state->x_count; i++)
        state->x[i] = 0.0;

#ifdef STORE_ROUTE
    state->route_count = (self->num_states + 1) * (self->num_states + 1);
    if(state->route_count > 0) {
      state->route = (double *) malloc (sizeof (double)*state->route_count);

      for (int i = 0; i < state->x_count + 1; i++)
	for (int j = 0; j < state->x_count + 1; j++)
	  state->route[i*(state->x_count+1) + j] = 0.0;
    }

    state->prob_margin = 0;
#endif

    return state;
}


// Frees up the memory used by a given state
int optsystem_free_state (optsystem_t *self, state_t *state) {
  state->x_count = 0;
  free(state->x);
#ifdef STORE_ROUTE
  state->route_count = 0;
  free(state->route);
  state->prob_margin = 0;
#endif
  free (state);
  return 1;
}


// Allocates memory for a new state
input_t* optsystem_new_input (optsystem_t *self) {
    input_t *input = (input_t *) malloc (sizeof (input_t));
    input->x_count = self->num_inputs;
    input->x = (double *) malloc (sizeof (double)*input->x_count);

    for (int i = 0; i < input->x_count; i++)
        input->x[i] = 0.0;
    return input;
}


// Frees up the memory used by a given state
int optsystem_free_input (optsystem_t *self, input_t *input) {
  free(input->x);
  free (input);
  return 1;
}


// Create a copy of the given state
state_t *optsystem_clone_state (optsystem_t *self, state_t *state) {
    state_t *stateNew = (state_t *) malloc (sizeof (state_t));
    stateNew->x_count = state->x_count;
    stateNew->x = (double *) malloc( sizeof(double)*stateNew->x_count);

    for (int i = 0; i < stateNew->x_count; i++)
      stateNew->x[i] = state->x[i];

#ifdef STORE_ROUTE
    stateNew->route_count = (self->num_states + 1) * (self->num_states + 1);
    stateNew->route = (double *) malloc (sizeof (double)*state->route_count);

    stateNew->prob_margin = state->prob_margin;

    for (int i = 0; i < state->route_count; i++)
      stateNew->route[i] = state->route[i];
#endif

    return stateNew;
}


// Sets the initial state to a particular value
int optsystem_set_initial_state (optsystem_t *self, state_t *state) {

    for (int i = 0; i < self->num_states; i++)
        self->initial_state->x[i] = state->x[i];

#ifdef STORE_ROUTE
    for (int i = 0; i < state->route_count; i++)
        self->initial_state->route[i] = state->route[i];

    self->initial_state->prob_margin = state->prob_margin;
#endif
    return 1;
}


// Returns the initial state of the system
int  optsystem_get_initial_state (optsystem_t *self, state_t *state) {
    for (int i = 0; i < self->num_states; i++)
        state->x[i] = self->initial_state->x[i];

#ifdef STORE_ROUTE
    for (int i = 0; i < state->route_count; i++)
        state->route[i] = self->initial_state->route[i];

    state->prob_margin = self->initial_state->prob_margin;
#endif
    return 1;
}


// Returns the ``number of dimensions''
int optsystem_get_num_states (optsystem_t *self) {
    return self->num_states;
}


// Returns a double array (of size optsystem_get_num_states)
//  used for storing the state in a spatial data structure
double* optsystem_get_state_key (optsystem_t *self, state_t *state) {
    return  state->x;
}

double center[2];
double size[2];

//#define RESTRICTED_SAMPLING

// Creates a random state
int optsystem_sample_state (optsystem_t *self, state_t *random_state) {

#ifdef RESTRICTED_SAMPLING
  // Sample x[n-2], x[n-1]
  for(int i = self->num_states-2; i < self->num_states; ++i) {
    random_state->x[i] =
      self->operating_region.size[i%2]*rand()/(RAND_MAX + 1.0)
      + self->operating_region.center[i%2] -
      self->operating_region.size[i%2]/2.0;
  }

  // Find center of {n-2, n-1} node and anchor node
  center[0] = (random_state->x[self->num_states-2] + self->anchor_x)/2.0;
  center[1] = (random_state->x[self->num_states-1] + self->anchor_y)/2.0;
  size[0]=1.25*fabs(random_state->x[self->num_states-2] - self->anchor_x);
  size[1]=1.25*fabs(random_state->x[self->num_states-1] - self->anchor_y);

  if(size[0] > size[1])
    size[1] = size[0];
  else
    size[0] = size[1];

  // Sample in bounding box of anchor and last node
  for (int i = 0; i < self->num_states-2; i++) {
    random_state->x[i] = size[i%2]*rand()/(RAND_MAX + 1.0)
      + center[i%2] - size[i%2]/2.0;
  }

#else
  for(int i = 0; i < self->num_states; ++i) {
    random_state->x[i] =
      self->operating_region.size[i%2]*rand()/(RAND_MAX + 1.0)
      + self->operating_region.center[i%2] -
      self->operating_region.size[i%2]/2.0;
  }

#endif

  /* printf("Sampled state: ["); */
  /* for(int i=0; i < self->num_states; ++i) */
  /*   printf("%2.2f ", random_state->x[i]); */
  /* printf("]\n"); */

  /* if ( optsystem_on_obstacle (self, random_state) ) */
  /*   return 0; */

  return 1;
}


// Creates a random state
int optsystem_sample_target_state (optsystem_t *self, state_t *random_state) {

#ifdef RESTRICTED_SAMPLING
  for (int i = self->num_states-2; i < self->num_states; ++i) {
    if(i < self->num_states-2) {
     random_state->x[i] =
       self->operating_region.size[i%2]*rand()/(RAND_MAX + 1.0)
       + self->operating_region.center[i%2]
       - self->operating_region.size[i%2]/2.0;
    }
    else {
      random_state->x[i] = self->goal_region.size[i%2]*rand()/(RAND_MAX + 1.0)
        + self->goal_region.center[i%2] - self->goal_region.size[i%2]/2.0;
    }
  }

  // Find center of {n-2, n-1} node and anchor node
  center[0] =(random_state->x[self->num_states-2] + self->anchor_x)/2.0;
  center[1] = (random_state->x[self->num_states-1] + self->anchor_y)/2.0;
  size[0]=1.25*fabs(random_state->x[self->num_states-2] - self->anchor_x);
  size[1]=1.25*fabs(random_state->x[self->num_states-1] - self->anchor_y);

  if(size[0] > size[1])
    size[1] = size[0];
  else
    size[0] = size[1];

  // Sample in bounding box of anchor and last node
  for (int i = 0; i < self->num_states-2; i++) {
    random_state->x[i] = size[i%2]*rand()/(RAND_MAX + 1.0)
      + center[i%2] - size[i%2]/2.0;
  }

#else
  for (int i = 0; i < self->num_states; ++i) {
    if(i < self->num_states-2) {
     random_state->x[i] =
       self->operating_region.size[i%2]*rand()/(RAND_MAX + 1.0)
       + self->operating_region.center[i%2]
       - self->operating_region.size[i%2]/2.0;
    }
    else {
      random_state->x[i] = self->goal_region.size[i%2]*rand()/(RAND_MAX + 1.0)
        + self->goal_region.center[i%2] - self->goal_region.size[i%2]/2.0;
    }
  }
#endif

  /* if ( optsystem_on_obstacle (self, random_state) ) */
  /*   return 0; */

  /* printf("Sampled goal state: ["); */
  /* for(int i=0; i < self->num_states; ++i) */
  /*   printf("%2.2f ", random_state->x[i]); */
  /* printf("]\n"); */

  return 1;
}


// Evaluates the Euclidean distance between two states -- used mainly for the Nearest and CloseNodes procedures
double optsystem_evaluate_distance (optsystem_t *self, state_t *state_from, state_t *state_to) {

    double dist = 0;
    for (int i = 0; i < self->num_states; i++) {
        double dist_this = state_to->x[i] - state_from->x[i];
        dist += dist_this * dist_this;
    }

    return sqrt (dist);
}


// Evaluates the cost when traversing a given set of inputs
double optsystem_evaluate_distance_for_cost (optsystem_t *self, GSList *inputs) {

    // Calculates the Euclidean distance to get from state_from to state_to using the intermediate trajectory traj

    double time = 0.0;

    GSList *inputs_ptr = inputs;
    while (inputs_ptr) {
        input_t *input_curr = inputs_ptr->data;
        time += input_curr->x[0];
        inputs_ptr = g_slist_next (inputs_ptr);
    }

    return time;
}

#define INDIVIDUAL_ROBOT_STEPS

// Checks whether the line segment between (state_initial) and (state_final) lies on an obstacle
int optsystem_segment_on_obstacle (optsystem_t *self, state_t *state_initial, state_t *state_final, int num_steps) {
    double *delta = (double*)malloc(sizeof(double)*state_initial->x_count);

    // Start search along segment at state_initial (moving towards state_final)
    for(int i=0; i < state_initial->x_count; ++i) {
      delta[i] = (state_final->x[i] - state_initial->x[i])/((double)num_steps);
      state_final->x[i] = state_initial->x[i];
    }

    /* printf("[segment_on_obstacle]\n"); */
    /* printf("\t ["); */
    /* for(int k=0; k < state_final->x_count; ++k) */
    /*   printf("%2.2f ", state_final->x[k]); */
    /* printf("]\n"); */

#ifdef INDIVIDUAL_ROBOT_STEPS

    /* int *freeze_robot = malloc(sizeof(int)*state_initial->x_count); */
    /* for(int i=0; i < state_initial->x_count; ++i) */
    /*   freeze_robot[i] = 0; */

    /* // For each step */
    /* for(int j=0; j < num_steps; ++j) { */
    /*     int init_idx = rand() % state_initial->x_count; */
    /*     int i = init_idx; */
    /*     do { */
    /*         //for(int i=0; i < state_initial->x_count/2; i++) { */

    /*         // NEW: check feasibility after each robot is moved one delta forward */
    /*         if(freeze_robot[i] == 0) { */
    /*             state_final->x[i] += delta[i]; */
    /*         } */
    /*         else { */
    /*             // Increment counter with a wrap-around */
    /*             i++; */
    /*             i = i % (state_initial->x_count); */
    /*             continue; */
    /*         } */

    /*         // If this step is on an obstacle, step back one and return that as the */
    /*         // final point on the segment */
    /*         if(optsystem_on_obstacle(self, state_final, (j==0 ? state_initial : NULL) )) { */

    /*             state_final->x[i] -= delta[i]; */
    /*             freeze_robot[i] = 1; */
    /*         } */

    /*         // Increment counter with a wrap-around */
    /*         i++; */
    /*         i = i % (state_initial->x_count); */
    /*     } while (i != init_idx ); // Stop when we return to the "beginning" */

    /*     // Check if we've "frozen" all robots */
    /*     int total = 0; */
    /*     for(int k=0; k < state_initial->x_count; ++k) { */
    /*         total += freeze_robot[k]; */
    /*     } */
    /*     if(total == state_initial->x_count) { */
    /*         // Should return that segment *is* on an obstacle if the only feasible */
    /*         // point is the initial point */
    /*         free(delta); */
    /*         free(freeze_robot); */
    /*         if(j == 0) */
    /*             return 1; */

    /*         // Otherwise, return the new final point of the segment */
    /*         return 0; */
    /*     } */
    /* } */

    /* free(freeze_robot); */

    int *freeze_robot = malloc(sizeof(int)*state_initial->x_count/2);
    for(int i=0; i < state_initial->x_count/2; ++i)
      freeze_robot[i] = 0;

    // For each step
    for(int j=0; j < num_steps; ++j) {
        int init_idx = rand() % state_initial->x_count/2;
        int i = init_idx;
        do {
            //for(int i=0; i < state_initial->x_count/2; i++) {

            // NEW: check feasibility after each robot is moved one delta forward
            if(freeze_robot[i] == 0) {
                state_final->x[2*i] += delta[2*i];
                state_final->x[2*i+1] += delta[2*i+1];
            }
            else {
                // Increment counter with a wrap-around
                i++;
                i = i % (state_initial->x_count/2);
                continue;
            }

            // If this step is on an obstacle, step back one and return that as the
            // final point on the segment
            if(optsystem_on_obstacle(self, state_final, (j==0 ? state_initial : NULL) )) {

                state_final->x[2*i] -= delta[2*i];
                state_final->x[2*i+1] -= delta[2*i+1];

                freeze_robot[i] = 1;
            }

            // Increment counter with a wrap-around
            i++;
            i = i % (state_initial->x_count/2);
        } while (i != init_idx ); // Stop when we return to the "beginning"

        // Check if we've "frozen" all robots
        int total = 0;
        for(int k=0; k < state_initial->x_count/2; ++k) {
            total += freeze_robot[k];
        }
        if(total == state_initial->x_count/2) {
            // Should return that segment *is* on an obstacle if the only feasible
            // point is the initial point
            free(delta);
            free(freeze_robot);
            if(j == 0)
                return 1;

            // Otherwise, return the new final point of the segment
            return 0;
        }
    }

    free(freeze_robot);

#else
    // For each step
    for(int j=0; j < num_steps; ++j) {
        for(int i=0; i < state_initial->x_count; ++i) {
            state_final->x[i] += delta[i];
        }
        /* printf("\t ["); */
        /* for(int k=0; k < state_final->x_count; ++k) */
        /* 	printf("%2.2f ", state_final->x[k]); */
        /* printf("]\n"); */

        // If this step is on an obstacle, step back one and return that as the
        // final point on the segment
        if(optsystem_on_obstacle(self, state_final, (j==0 ? state_initial : NULL) )) {
            for(int i=0; i < state_initial->x_count; ++i) {
                state_final->x[i] -= delta[i];
            }
            free(delta);

            // Should return that segment *is* on an obstacle if the only feasible
            // point is the initial point
            if(j == 0)
                return 1;

            // Otherwise, return the new final point of the segment
            return 0;
        }
    }
#endif

    free(delta);

    // If we get all the way through the segment, just return the final point as
    // feasible
    return 0;
}


// Extends a given state towards another state
int optsystem_extend_to (optsystem_t *self, state_t *state_from, state_t *state_towards,
                         int *fully_extends, GSList **trajectory,
                         int *num_node_states, int **nodes_states, GSList **inputs) {

    int discretization_num_steps = 10;

    GSList *trajectory_curr = NULL; // Start with an empty trajectory
    GSList *inputs_curr = NULL;

    double dist = 0;
    for(int i=0; i < self->num_states; ++i) {
      dist +=
        (state_towards->x[i] - state_from->x[i])*
        (state_towards->x[i] - state_from->x[i]);
    }
    dist = sqrt(dist);

    /* printf("Extending from ["); */
    /* for(int i=0; i < self->num_states; ++i) { */
    /*   printf("%2.2f ", state_from->x[i]); */
    /* } */
    /* printf("] to ["); */
    /* for(int i=0; i < self->num_states; ++i) { */
    /*   printf("%2.2f ", state_towards->x[i]); */
    /* } */
    /* printf("]\n"); */

    if (dist < self->steer_distance) {
        //printf("within steer distance (%2.2f < %2.2f)\n", dist, self->steer_distance);

        /* printf("Extend towards: "); */
        /* for(int i=0; i < self->num_states; ++i) { */
        /* 	printf("%2.2f ", state_towards->x[i]); */
        /* } */
        /* printf("(from "); */
        /* for(int i=0; i < self->num_states; ++i) { */
        /* 	printf("%2.2f ", state_from->x[i]); */
        /* }       */
        /* printf(")\n"); */

        // Find maximum individual robot trajectory
        double max_dist = 0;
        double tmp_dist = 0;
        for(int i=0; i < self->num_states/2; ++i) {
            tmp_dist = sqrt( (state_towards->x[2*i] - state_from->x[2*i])*
                             (state_towards->x[2*i] - state_from->x[2*i]) +
                             (state_towards->x[2*i+1] - state_from->x[2*i+1])*
                             (state_towards->x[2*i+1] - state_from->x[2*i+1]) );
            if(tmp_dist > max_dist)
                max_dist = tmp_dist;
        }
        double desired_delta = 0.3;
        discretization_num_steps = (int)(round(max_dist/desired_delta));

        if (optsystem_segment_on_obstacle (self, state_from, state_towards, discretization_num_steps) ) {
            *fully_extends = 0;
            return 0;
        }

        state_t *state_new = optsystem_clone_state(self, state_towards);
        trajectory_curr = g_slist_prepend (trajectory_curr, state_new);
        input_t *input_new = optsystem_new_input (self);
        input_new->x[0] = dist;
        inputs_curr = g_slist_prepend (inputs_curr, input_new);
        *fully_extends = 1;
    }
    else {
      //printf("not within steer distance (%2.2f > %2.2f)\n", dist, self->steer_distance);
        *fully_extends = 0;
        state_t *state_new = optsystem_new_state (self);
	for(int i=0; i < self->num_states; ++i)
	  state_new->x[i] = self->steer_distance*(state_towards->x[i] - state_from->x[i])/dist + state_from->x[i];

      /* printf("Extend towards: "); */
      /* for(int i=0; i < self->num_states; ++i) { */
      /* 	printf("%2.2f ", state_new->x[i]); */
      /* } */
      /* printf("(from "); */
      /* for(int i=0; i < self->num_states; ++i) { */
      /* 	printf("%2.2f ", state_from->x[i]); */
      /* }       */
      /* printf(")\n"); */

      // Find maximum individual robot trajectory
      double max_dist = 0;
      double tmp_dist = 0;
      for(int i=0; i < self->num_states/2; ++i) {
	tmp_dist = sqrt( (state_new->x[2*i] - state_from->x[2*i])*
			 (state_new->x[2*i] - state_from->x[2*i]) +
			 (state_new->x[2*i+1] - state_from->x[2*i+1])*
			 (state_new->x[2*i+1] - state_from->x[2*i+1]) );
	if(tmp_dist > max_dist)
	  max_dist = tmp_dist;
      }
      double desired_delta = 0.3;
      discretization_num_steps = (int)(round(max_dist/desired_delta));

      if (optsystem_segment_on_obstacle(self, state_from, state_new, discretization_num_steps)) {
	optsystem_free_state (self, state_new);
	return 0;
      }

      dist = 0;
      for(int i=0; i < self->num_states; ++i) {
	dist +=
	  (state_new->x[i] - state_from->x[i])*
	  (state_new->x[i] - state_from->x[i]);
      }
      dist = sqrt(dist);

      if(dist < 0.01) {
	printf("Adding a new node _close_ to old one (%f)\n", dist);
      }



        trajectory_curr = g_slist_prepend (trajectory_curr, state_new);
        input_t *input_new = optsystem_new_input (self);
        input_new->x[0] = 1.0;
        inputs_curr = g_slist_prepend (inputs_curr, input_new);
    }

    *trajectory = trajectory_curr;
    *inputs = inputs_curr;
    *num_node_states = 0;
    *nodes_states = NULL;

    return 1;
}


// Checks whether a given state is on an obstacle
gboolean optsystem_on_obstacle (optsystem_t *self, state_t *state, state_t *initial_state) {

    GSList *obstacle_list_curr = self->obstacle_list;

    // For each obstacle
    while (obstacle_list_curr) {
        region_2d_t *obstacle_region = (region_2d_t *) (obstacle_list_curr->data);

	// For each 2D agent
	for(int i=0; i < self->num_states; i += 2) {
	  if ( (fabs(obstacle_region->center[0] - state->x[i]) <=
		obstacle_region->size[0]/2.0) &&
	       (fabs(obstacle_region->center[1] - state->x[i+1]) <=
		obstacle_region->size[1]/2.0) )
            return 1;
	}

        obstacle_list_curr = g_slist_next (obstacle_list_curr);
    }

    if(self->obstacle_func) {
      return self->obstacle_func(self->obstacle_func_param, state, initial_state);
    }

    return 0;
}


// Checks whether a given state is inside the target region
gboolean optsystem_is_reaching_target (optsystem_t *self, state_t *state) {

  // We only care about the position of the 'last' 2D component of the state
    if ( (fabs(self->goal_region.center[0] - state->x[self->num_states-2]) <= self->goal_region.size[0]/2.0) &&
         (fabs(self->goal_region.center[1] - state->x[self->num_states-1]) <= self->goal_region.size[1]/2.0) )  {
        return 1;
    }

    return 0;
}


double optsystem_evaluate_cost_to_go (optsystem_t *self, state_t *state) {

    // If state is in the goal region then return zero
    if (optsystem_is_reaching_target (self, state))
        return 0.0;

    // Otherwise calculate a lower bound on the cost to go
    // TODO: do the exact cost to go calculation

    double min_side = self->goal_region.size[0]/2.0;
    if (self->goal_region.size[1] < min_side)
        min_side = self->goal_region.size[1]/2.0;

    double dist_x = state->x[self->num_states-2] - self->goal_region.center[0];
    double dist_y = state->x[self->num_states-1] - self->goal_region.center[1];

    dist_x = fabs(dist_x);
    dist_y = fabs(dist_y);

    double dist = sqrt(dist_x*dist_x + dist_y*dist_y);

    /* printf("min_side: %2.2f, goal_region_center: %2.2f %2.2f, dist: %2.2f\n",  */
    /* 	   min_side, self->goal_region.center[0], self->goal_region.center[1], dist); */

    /* if (dist_x < dist_y) { */
    /*     dist -= sqrt(1 + (dist_x/dist_y)*(dist_x/dist_y))*self->goal_region.size[0]/2.0; */
    /* } */
    /* else { */
    /*     dist -= sqrt(1 + (dist_y/dist_x)*(dist_y/dist_x))*self->goal_region.size[0]/2.0; */
    /* } */

    /* dist -= min_side; */
    /* if (dist < 0.0) */
    /*     dist = 0.0; */

    return dist;


    double sat[4] = { 0, 0, 0, 0};

    if (state->x[self->num_states-2] <= self->goal_region.center[0] + self->goal_region.size[0]/2.0)
        sat[0] = 1;

    if (state->x[self->num_states-2] >= self->goal_region.center[0] - self->goal_region.size[0]/2.0)
        sat[1] = 1;

    if (state->x[self->num_states-1] <= self->goal_region.center[1] + self->goal_region.size[1]/2.0)
        sat[2] = 1;

    if (state->x[self->num_states-1] >= self->goal_region.center[1] - self->goal_region.size[1]/2.0)
        sat[3] = 1;

    int sum = sat[0] + sat[1] + sat[2] + sat[3];

    if (sum == 2) { // Check distance to corners
        double dist_min;
        double dist_this;
        double dist_x;
        double dist_y;

        dist_x = state->x[self->num_states-2] - (self->goal_region.center[0] + self->goal_region.size[0]/2.0);
        dist_y = state->x[self->num_states-1] - (self->goal_region.center[1] + self->goal_region.size[1]/2.0);
        dist_this = sqrt(dist_x*dist_x + dist_y*dist_y);
        dist_min = dist_this;

        dist_x = state->x[self->num_states-2] - (self->goal_region.center[0] - self->goal_region.size[0]/2.0);
        dist_y = state->x[self->num_states-1] - (self->goal_region.center[1] + self->goal_region.size[1]/2.0);
        dist_this = sqrt(dist_x*dist_x + dist_y*dist_y);
        if (dist_this < dist_min)
            dist_min = dist_this;

        dist_x = state->x[self->num_states-2] - (self->goal_region.center[0] - self->goal_region.size[0]/2.0);
        dist_y = state->x[self->num_states-1] - (self->goal_region.center[1] - self->goal_region.size[1]/2.0);
        dist_this = sqrt(dist_x*dist_x + dist_y*dist_y);
        if (dist_this < dist_min)
            dist_min = dist_this;

        dist_x = state->x[self->num_states-2] - (self->goal_region.center[0] + self->goal_region.size[0]/2.0);
        dist_y = state->x[self->num_states-1] - (self->goal_region.center[1] - self->goal_region.size[1]/2.0);
        dist_this = sqrt(dist_x*dist_x + dist_y*dist_y);
        if (dist_this < dist_min)
            dist_min = dist_this;

        return dist_min;
    }
    else if (sum == 3) { // Check distance to edges

        if (sat[0] == 0)
            return fabs(state->x[self->num_states-2] - (self->goal_region.center[0] + self->goal_region.size[0]/2.0));

        if (sat[1] == 0)
            return fabs(state->x[self->num_states-2] - (self->goal_region.center[0] - self->goal_region.size[0]/2.0));

        if (sat[2] == 0)
            return fabs(state->x[self->num_states-1] - (self->goal_region.center[1] + self->goal_region.size[1]/2.0));

        if (sat[3] == 0)
            return fabs(state->x[self->num_states-1] - (self->goal_region.center[1] - self->goal_region.size[1]/2.0));

        printf ("??Sum : %d\n", sum);
        exit(1);

    }
    else {
        printf ("Sum : %d\n", sum);
        exit(1);
    }

}


gboolean optsystem_update_goal_region (optsystem_t *self, region_2d_t *goal_region) {

    self->goal_region.center[0] = goal_region->center[0];
    self->goal_region.center[1] = goal_region->center[1];
    self->goal_region.size[0] = goal_region->size[0];
    self->goal_region.size[1] = goal_region->size[1];

    return TRUE;
}


gboolean optsystem_update_operating_region (optsystem_t *self, region_2d_t *operating_region) {

    self->operating_region.center[0] = operating_region->center[0];
    self->operating_region.center[1] = operating_region->center[1];
    self->operating_region.size[0] = operating_region->size[0];
    self->operating_region.size[1] = operating_region->size[1];

    return TRUE;
}


gboolean optsystem_update_obstacles (optsystem_t *self, GSList *obstacle_list) {

    // Clear the previous obstacles
    while (self->obstacle_list) {
        region_2d_t *region_curr = (region_2d_t *) (self->obstacle_list->data);
        self->obstacle_list = g_slist_remove (self->obstacle_list, region_curr);
        free (region_curr);
    }

    // Add new obstacles
    GSList *obstacle_list_curr = obstacle_list;
    while (obstacle_list_curr) {
        region_2d_t *region_curr = (region_2d_t *) (obstacle_list_curr->data);
        region_2d_t *region_new = (region_2d_t *) malloc (sizeof (region_2d_t));
        region_new->center[0] = region_curr->center[0];
        region_new->center[1] = region_curr->center[1];
        region_new->size[0] = region_curr->size[0];
        region_new->size[1] = region_curr->size[1];

        self->obstacle_list = g_slist_prepend (self->obstacle_list, (gpointer)region_new);

        obstacle_list_curr = g_slist_next (obstacle_list_curr);
    }

    return TRUE;
}
