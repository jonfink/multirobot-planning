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

#ifndef __OPTSYSTEM_H_
#define __OPTSYSTEM_H_

#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <glib.h>

// Type definitions
typedef struct _region_2d_t region_2d_t;
typedef struct _state_t state_t;
typedef struct _input_t input_t;
typedef struct _optsystem_t optsystem_t;

#define STORE_ROUTE

// Initializes the optsystem_t structure
int
optsystem_new_system (optsystem_t *self);

// Frees the memory allocated for elements of the optsystem_t structure
int
optsystem_free_system (optsystem_t *self);

// Allocates memory for a new state and returns a pointer to the new state
state_t*
optsystem_new_state (optsystem_t *self);

// Frees the memory allocated for a state
int
optsystem_free_state (optsystem_t *self, state_t *state);

// Allocates memory for a new input and returns a pointer to the new input
input_t*
optsystem_new_input (optsystem_t *self);

// Frees the memory allocated for an input
int
optsystem_free_input (optsystem_t *self, input_t *input);

// Allocates memory for a new state. Copies a given state into the new state
//   and returns a pointer to the new state
state_t*
optsystem_clone_state (optsystem_t *self, state_t *state);

// Sets the initial state to a given state_t structure
int
optsystem_set_initial_state (optsystem_t *self, state_t *state);

// Copies the initial state to the given state_t structure
int
optsystem_get_initial_state (optsystem_t *self, state_t *state);

// Returns the number of states
int
optsystem_get_num_states (optsystem_t *self);

// Returns a key to the state for the kd-tree implementation. The return
//   value must be a double array of size equal to the number of states
double*
optsystem_get_state_key (optsystem_t *self, state_t *state);

// Returns a new sample state from the free-space
int
optsystem_sample_state (optsystem_t *self, state_t *random_state);

// Returns a new sample state from the target state
int
optsystem_sample_target_state (optsystem_t *self, state_t *random_state);

// Evaluates the distance between two given states for nearest neighbors computation
double
optsystem_evaluate_distance (optsystem_t *self, state_t *state_from, state_t *state_to);

// Evaluates the cost of a given trajectory
double
optsystem_evaluate_distance_for_cost (optsystem_t *self, GSList *inputs);

// Extends a given state (state_from) towards a given state (state_towards). Sets
//   (fully_extend) to 1 if the extension exactly reaches to state_towards and to 0
//   if the extension falls short. The resulting sequence of states is returned in
//   (trajectory) and the resulting inputs is returned in (inputs). The intermediate
//   nodes to be put into the tree is returned in (node_states), while the number of
//   such nodes is teruned in (num_node_states).
int
optsystem_extend_to (optsystem_t *self, state_t *state_from, state_t *state_towards,
                     int *fully_extends, GSList **trajectory, int *num_node_states, int **node_states, GSList **inputs);

// Returns true iff the given state reaches the goal
gboolean
optsystem_is_reaching_target (optsystem_t *self, state_t *state);

double optsystem_evaluate_cost_to_go (optsystem_t *self, state_t *state);

// ===========   Environment interface    =============================================
// Updates the operating region
gboolean
optsystem_update_operating_region (optsystem_t *self, region_2d_t *operating_region);

// Updates the goal region
gboolean
optsystem_update_goal_region (optsystem_t *self, region_2d_t *goal_region);

// Updates the list of obstacles
gboolean
optsystem_update_obstacles (optsystem_t *self, GSList *obstacle_list);

// A 2D region to describe operating and goal regions as well as obstacles
struct _region_2d_t {
    double center[2];
    double size[2];
};
// ====================================================================================

// State structure
struct _state_t {
  unsigned int x_count;
  double *x;
#ifdef STORE_ROUTE
  unsigned int route_count;
  double *route;
  double prob_margin;
#endif
};


// Input structure
struct _input_t {
  unsigned int x_count;
  double *x;
};

// System structure
struct _optsystem_t {
  unsigned int num_states;
  unsigned int num_inputs;
  state_t *initial_state;        // Initial state
  region_2d_t operating_region;  // Operating region
  region_2d_t goal_region;       // Goal region
  GSList *obstacle_list;         // A list of obstacles
  double steer_distance;         // Max distance for "steer" operation
  double anchor_x, anchor_y;     // Position of 'anchor' node

  int (*obstacle_func)(void*, state_t*, state_t*); // Function pointer
  void *obstacle_func_param;
};


#endif
