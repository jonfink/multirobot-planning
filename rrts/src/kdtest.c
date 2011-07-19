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

#include "rrts/kdtree2.h"

// Returns current time
int64_t
ts_now () {
  struct timeval tv;
  gettimeofday (&tv,NULL);
  return (int64_t) tv.tv_sec * 1000000 + tv.tv_usec;
}

int main () {

  struct KD_FUNC_PREFIX_kdtree *kd = KD_FUNC_PREFIX_kd_create(3, 3, 0.5);

  double pos[3] = {0.0, 0.0, 0.0};

  double t_start = ts_now();
  for(double x=0.0; x < 10; x += 0.5) {
    for(double y=0.0; y < 10; y += 0.5) {
      for(double z=0.0; z < 10; z += 0.5) {
	pos[0] = x;
	pos[1] = y;
	pos[2] = z;
	KD_FUNC_PREFIX_kd_insert(kd, pos, NULL);
      }
    }
  }
  double t_end = ts_now();

  printf("Inserted %d nodes in %f seconds (%f seconds/node)\n",
	 20*20*20, (t_end-t_start)/1000000.0, ((t_end-t_start)/1000000.0)/(20*20*20));

  pos[0] = 12.1;
  pos[1] = 12.1;
  pos[2] = 12.1;

  struct KD_FUNC_PREFIX_kdres *ret = KD_FUNC_PREFIX_kd_nearest(kd, pos);

  if(ret) {
    printf("%2.2f %2.2f %2.2f\n",
	   ret->rlist->next->item->points[0],
	   ret->rlist->next->item->points[1],
	   ret->rlist->next->item->points[2]);
  }

  double total_x = 0;
  double total_y = 0;
  double total_z = 0;

  for(int i=0; i < 10000; ++i) {
    KD_FUNC_PREFIX_kd_sample_pos(kd, pos);
    total_x += pos[0];
    total_y += pos[1];
    total_z += pos[2];
  }

  printf("Average sample: %2.2f, %2.2f, %2.2f\n",
	 total_x/10000.0,
	 total_y/10000.0,
	 total_z/10000.0);

  KD_FUNC_PREFIX_kd_free(kd);
  return 1;
}
