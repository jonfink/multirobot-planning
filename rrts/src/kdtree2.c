/*
  This file is part of ``kdtree'', a library for working with kd-trees.
  Copyright (C) 2007-2009 John Tsiombikas <nuclear@siggraph.org>

  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions are met:

  1. Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.
  2. Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.
  3. The name of the author may not be used to endorse or promote products
  derived from this software without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR IMPLIED
  WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO
  EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
  EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
  OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
  IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
  OF SUCH DAMAGE.
*/
/* single nearest neighbor search written by Tamas Nepusz <tamas@cs.rhul.ac.uk> */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <float.h>
#include "rrts/kdtree2.h"



#if defined(WIN32) || defined(__WIN32__)
#include <malloc.h>
#endif

#ifdef USE_LIST_NODE_ALLOCATOR

#ifndef NO_PTHREADS
#include <pthread.h>
#else

#ifndef I_WANT_THREAD_BUGS
#error "You are compiling with the fast list node allocator, with pthreads disabled! This WILL break if used from multiple threads."
#endif	/* I want thread bugs */

#endif	/* pthread support */
#endif	/* use list node allocator */


#define SQ(x)			((x) * (x))

void KD_FUNC_PREFIX_print_kdnode(struct KD_FUNC_PREFIX_kdnode *node)
{
  return;
  if(node->k == -1) {
    printf("=== Leaf Node (%x) {m=%d, num_points=%d, points={", node, node->m, node->num_points);
    for(int i=0; i < node->num_points; ++i) {
      printf("[");
      for(int j=0; j < node->bbox->dim; ++j)
	printf("%2.2f ", node->points[i*node->bbox->dim + j]);
      printf("], ");
    }
    printf("} area=%2.2f, height=%d, ", node->area, node->height);
    printf("bbox=[");
    for(int j=0; j < node->bbox->dim; ++j)
      printf("%2.2f ", node->bbox->min[j]);
    printf("]->[");
    for(int j=0; j < node->bbox->dim; ++j)
      printf("%2.2f ", node->bbox->max[j]);
    printf("], ");
    printf("rbbox=[");
    for(int j=0; j < node->rbbox->dim; ++j)
      printf("%2.2f ", node->rbbox->min[j]);
    printf("]->[");
    for(int j=0; j < node->rbbox->dim; ++j)
      printf("%2.2f ", node->rbbox->max[j]);
    printf("]}\n");
  }
  else {
    printf("=== Split Node (%x) {v1=%x, v2=%x, area=%2.2f, height=%d, k=%d, l=%2.2f, ", node, node->v1, node->v2, node->area, node->height, node->k, node->l);
    printf("bbox=[");
    for(int j=0; j < node->bbox->dim; ++j)
      printf("%2.2f ", node->bbox->min[j]);
    printf("]->[");
    for(int j=0; j < node->bbox->dim; ++j)
      printf("%2.2f ", node->bbox->max[j]);
    printf("]}\n");
  }
}


static void KD_FUNC_PREFIX_clear_rec(struct KD_FUNC_PREFIX_kdnode *node, void (*destr)(void*));

static int KD_FUNC_PREFIX_rlist_insert(struct KD_FUNC_PREFIX_res_node *list, struct KD_FUNC_PREFIX_kdnode *item, double dist_sq);
static void KD_FUNC_PREFIX_clear_results(struct KD_FUNC_PREFIX_kdres *set);

static struct KD_FUNC_PREFIX_kdhyperrect* KD_FUNC_PREFIX_hyperrect_create(int dim, const double *min, const double *max);
static struct KD_FUNC_PREFIX_kdhyperrect* KD_FUNC_PREFIX_hyperrect_auto_create(int dim, int num_points, const double *points);
static struct KD_FUNC_PREFIX_kdhyperrect* KD_FUNC_PREFIX_hyperrect_auto_create_r(int dim, int num_points, const double *points, double r);
static void KD_FUNC_PREFIX_hyperrect_free(struct KD_FUNC_PREFIX_kdhyperrect *rect);
static struct KD_FUNC_PREFIX_kdhyperrect* KD_FUNC_PREFIX_hyperrect_duplicate(const struct KD_FUNC_PREFIX_kdhyperrect *rect);
static void KD_FUNC_PREFIX_hyperrect_extend(struct KD_FUNC_PREFIX_kdhyperrect *rect, const double *pos);
static void KD_FUNC_PREFIX_hyperrect_extend_r(struct KD_FUNC_PREFIX_kdhyperrect *rect, const double *pos, double r);
static double KD_FUNC_PREFIX_hyperrect_dist_sq(struct KD_FUNC_PREFIX_kdhyperrect *rect, const double *pos);

#ifdef USE_LIST_NODE_ALLOCATOR
static struct KD_FUNC_PREFIX_res_node *KD_FUNC_PREFIX_alloc_resnode(void);
static void KD_FUNC_PREFIX_free_resnode(struct KD_FUNC_PREFIX_res_node*);
#else
#define KD_FUNC_PREFIX_alloc_resnode()		malloc(sizeof(struct KD_FUNC_PREFIX_res_node))
#define KD_FUNC_PREFIX_free_resnode(n)		free(n)
#endif

struct KD_FUNC_PREFIX_kdtree *KD_FUNC_PREFIX_kd_create(int k, int m, double r)
{
  struct KD_FUNC_PREFIX_kdtree *tree;

  if(!(tree = malloc(sizeof *tree))) {
    return 0;
  }

  tree->dim = k;
  tree->m = m;
  tree->r = r;
  tree->root = 0;
  tree->destr = 0;
  return tree;
}

void KD_FUNC_PREFIX_kd_free(struct KD_FUNC_PREFIX_kdtree *tree)
{
  if(tree) {
    KD_FUNC_PREFIX_kd_clear(tree);
    free(tree);
  }
}

static void KD_FUNC_PREFIX_clear_rec(struct KD_FUNC_PREFIX_kdnode *node, void (*destr)(void*))
{
  if(!node) return;

  if(node->v1)
    KD_FUNC_PREFIX_clear_rec(node->v1, destr);

  if(node->v2)
    KD_FUNC_PREFIX_clear_rec(node->v2, destr);

  if(destr) {
    for(int i=0; i < node->num_points; ++i)
      destr(node->data[i]);

    free(node->data);
  }
  free(node->points);
  free(node);
}

void KD_FUNC_PREFIX_kd_clear(struct KD_FUNC_PREFIX_kdtree *tree)
{
  KD_FUNC_PREFIX_clear_rec(tree->root, tree->destr);
  tree->root = 0;
}

void KD_FUNC_PREFIX_kd_data_destructor(struct KD_FUNC_PREFIX_kdtree *tree, void (*destr)(void*))
{
  tree->destr = destr;
}

static struct KD_FUNC_PREFIX_kdnode* KD_FUNC_PREFIX_build_kd_tree(int num_points, const double *points, void **data, int dim, int m, struct KD_FUNC_PREFIX_kdhyperrect *bbox, double r);

static void KD_FUNC_PREFIX_split_node(struct KD_FUNC_PREFIX_kdnode **v1,
				      struct KD_FUNC_PREFIX_kdnode **v2,
				      double *l, int *k,
				      int num_points,
				      const double *points, void **data, int dim, int m,
				      struct KD_FUNC_PREFIX_kdhyperrect *bbox,
				      double r,
				      int *depth)
{
  double *tmp = NULL;
  tmp = malloc(sizeof(double)*num_points);
  for(int i=0; i < num_points; ++i) {
    tmp[i] = points[dim*i + *k]; // pull k^th dimension points
  }
  // Sort k^th dimension points
  int ii=0;
  double tmp_val;
  int all_vals_equal = 1;
  for(int ii=0; ii < num_points; ++ii) {
    for(int i=ii; i < num_points; ++i) {
      if(tmp[ii] > tmp[i]) {
	tmp_val = tmp[ii];
	tmp[ii] = tmp[i];
	tmp[i] = tmp_val;
	all_vals_equal = 0;
      }
    }
  }

  if(all_vals_equal) {
    free(tmp);
    *k = (*k+1) % dim; // try the next dimension for splitting
    *depth = *depth + 1;

    if(*depth > dim) {
      printf("tmp: [");
      for(int i=0; i < num_points; ++i) {
        printf("%2.2f ", tmp[i]);
      }
      printf("]\n");

      return;
    }


    // TODO: what if we have N copies of the same point (i.e. no split possible?)
    KD_FUNC_PREFIX_split_node(v1, v2, l, k, num_points, points, data, dim, m, bbox, r,
			      depth);
    return;
  }

  // Try picking l such that the points are split evenly along the k-axis
  *l = tmp[num_points/2];

  // Ensure that l will actually split the points somewhere
  ii=0;
  while(tmp[num_points-1] <= *l) {
    if(num_points/2 - ii < 0) {
      fprintf(stderr, "Out of bounds!\n");
      exit(-1);
    }
    *l = tmp[num_points/2 - ii];
    ++ii;
  }

  struct KD_FUNC_PREFIX_kdhyperrect *b1 = KD_FUNC_PREFIX_hyperrect_duplicate(bbox);
  struct KD_FUNC_PREFIX_kdhyperrect *b2 = KD_FUNC_PREFIX_hyperrect_duplicate(bbox);
  b1->max[*k] = *l;
  b2->min[*k] = *l;

  // Collect points that belong to each bbox
  int num_points1 = 0;
  int num_points2 = 0;
  for(int i=0; i < num_points; ++i) {
    if(points[dim*i + *k] <= *l) {
      num_points1++;
    }
    else {
      num_points2++;
    }
  }
  double *points1 = (double*)malloc(dim*num_points1*sizeof(double));
  double *points2 = (double*)malloc(dim*num_points2*sizeof(double));
  void **data1 = (void**)malloc(num_points1*sizeof(void*));
  void **data2 = (void**)malloc(num_points2*sizeof(void*));
  num_points1 = 0;
  num_points2 = 0;
  for(int i=0; i < num_points; ++i) {
    if(points[dim*i + *k] <= *l) {
      memcpy(&(points1[num_points1*dim]),
	     &(points[dim*i]), dim*sizeof(double));
      data1[num_points1] = data[i];
      num_points1++;
    }
    else {
      memcpy(&(points2[num_points2*dim]),
	     &(points[dim*i]), dim*sizeof(double));
      data2[num_points2] = data[i];
      num_points2++;
    }
  }

  // Create two children nodes
  *v1 = KD_FUNC_PREFIX_build_kd_tree(num_points1, points1, data1, dim, m, b1, r);
  *v2 = KD_FUNC_PREFIX_build_kd_tree(num_points2, points2, data2, dim, m, b2, r);

  // build_kd_tree will allocate/memcpy, so we can free the point data now
  free(points1);
  free(points2);
  free(data1);
  free(data2);
  free(tmp);
}

static struct KD_FUNC_PREFIX_kdnode* KD_FUNC_PREFIX_build_kd_tree(int num_points, const double *points, void **data, int dim, int m, struct KD_FUNC_PREFIX_kdhyperrect *bbox, double r)
{
  if(num_points <= m) {
    // Allocate leaf node
    struct KD_FUNC_PREFIX_kdnode *node;
    if(!(node = malloc(sizeof(*node)))) {
      return NULL;
    }

    // Copy points (with data) into storage
    node->num_points = num_points;
    node->m = m;
    node->points = 0;
    node->data = 0;

    node->points = (double *)malloc(dim * m * sizeof(double));
    memcpy(node->points, points, dim * num_points * sizeof(double));
    node->data = (void**)malloc(m * sizeof(void*));
    for(int i=0; i < num_points; ++i) {
      node->data[i] = data[i];
    }

    // Setup as leaf node
    node->k = -1;  // Signify leaf node
    node->v1 = NULL;
    node->v2 = NULL;
    // with bounding box
    node->bbox = bbox;
    // and r-bounding box
    node->rbbox = KD_FUNC_PREFIX_hyperrect_auto_create_r(dim, num_points, points, r);

    // and area computed
    node->area = 0;
    for(int i=0; i < dim; ++i) {
      node->area += (node->rbbox->max[i] - node->rbbox->min[i]);
    }
    // height of a leaf is zero (?)
    node->height = 0;

    KD_FUNC_PREFIX_print_kdnode(node);

    return node;
  }
  else {
    if(!bbox)
      return NULL;

    // Split bounding box at mid-point of longest dimension
    int k;      /* splitting dimension */
    double l=0;   /* splitting plane */
    for(int i=0; i < dim; ++i) {
      if((bbox->max[i] - bbox->min[i]) > l) {
	k = i;
	l = (bbox->max[i] - bbox->min[i]);
      }
    }

    struct KD_FUNC_PREFIX_kdnode *v1 = NULL;
    struct KD_FUNC_PREFIX_kdnode *v2 = NULL;

    int depth = 0;
    KD_FUNC_PREFIX_split_node(&v1, &v2, &l, &k, num_points, points, data, dim, m, bbox, r, &depth);

    struct KD_FUNC_PREFIX_kdnode *v;
    if(!(v = malloc(sizeof(*v)))) {
      return NULL;
    }
    v->num_points = 0;
    v->points = 0;
    v->data = 0;
    v->k = k;
    v->l = l;
    v->v1 = v1;
    v->v2 = v2;
    v->bbox = bbox;
    v->area = v1->area + v2->area;
    v->height = (v1->height > v2->height ? v1->height : v2->height) + 1;

    KD_FUNC_PREFIX_print_kdnode(v);
    return v;
  }
}

static struct KD_FUNC_PREFIX_kdnode* KD_FUNC_PREFIX_update_kd_tree(const double *point, void *data, int dim, int m, struct KD_FUNC_PREFIX_kdnode* v, struct KD_FUNC_PREFIX_kdhyperrect *bbox, double r)
{
  if(!v)
    printf("Node 'v' not defined?!?\n");

  // If v is a leaf
  if(v->k == -1) {

    // Check that point does not already exist in this leaf node
    for(int j=0; j < v->num_points; ++j) {
      double dist_j = 0;
      for(int i=0; i < dim; ++i) {
	dist_j +=
	  (v->points[j*dim + i] - point[i])*
	  (v->points[j*dim + i] - point[i]);
      }
      dist_j = sqrt(dist_j);
      // If this point is too similar to existing point,
      // return node without changes
      if(dist_j < 1e-4)
	return v;
    }

    if(v->num_points + 1 <= m) {
      // Simple add 'point' to leaf node
      memcpy(&(v->points[v->num_points*dim]), point, dim*sizeof(double));
      v->data[v->num_points] = data;
      v->num_points++;
      KD_FUNC_PREFIX_hyperrect_extend(v->bbox, point);
      KD_FUNC_PREFIX_hyperrect_extend_r(v->rbbox, point, r);
      // recompute area
      v->area = 0;
      for(int i=0; i < dim; ++i)
	v->area += (v->rbbox->max[i] - v->rbbox->min[i]);

      v->v1 = 0;
      v->v2 = 0;

      KD_FUNC_PREFIX_print_kdnode(v);
      return v;
    }
    else {
      KD_FUNC_PREFIX_print_kdnode(v);
      // Call build_kd_tree with points
      double *points = (double*)malloc(dim*(v->num_points+1)*sizeof(double));
      void **data = (void**)malloc((v->num_points+1)*sizeof(void*));

      memcpy(points, v->points, dim*v->num_points*sizeof(double));
      memcpy(&(points[dim*v->num_points]), point, dim*sizeof(double));
      int num_points = v->num_points + 1;

      /* if(v)  */
      /* 	KD_FUNC_PREFIX_clear_rec(v, NULL); */

      v = KD_FUNC_PREFIX_build_kd_tree(num_points, points, data, dim, m,
			KD_FUNC_PREFIX_hyperrect_auto_create(dim, num_points, points), r);

      KD_FUNC_PREFIX_print_kdnode(v);
      return v;
    }
  }
  else {
    // Consider the sub-boxes of v (v->v1->bbox, v->v2->bbox)
    if(point[v->k] < v->l) {
      v->v1 = KD_FUNC_PREFIX_update_kd_tree(point, data, dim, m, v->v1, v->v1->bbox, r);
    }
    else {
      v->v2 = KD_FUNC_PREFIX_update_kd_tree(point, data, dim, m, v->v2, v->v2->bbox, r);
    }
    // Keep tree balanced
    if( (v->v1->height > 2*v->v2->height) ||
	(v->v2->height > 2*v->v1->height) ) {
      //printf("Unbalanced tree -- not dealing with it now\n");
    }

    v->area = v->v1->area + v->v2->area;
    v->height = (v->v1->height > v->v2->height ? v->v1->height : v->v2->height) + 1;

    KD_FUNC_PREFIX_print_kdnode(v);
    return v;
  }
}

int KD_FUNC_PREFIX_kd_insert(struct KD_FUNC_PREFIX_kdtree *tree, const double *pos, void *data)
{
  if(!tree->root) {
    tree->root = KD_FUNC_PREFIX_build_kd_tree(1, pos, &data,
			       tree->dim, tree->m,
			       KD_FUNC_PREFIX_hyperrect_auto_create(tree->dim, 1, pos), tree->r);
  }
  else {
    tree->root = KD_FUNC_PREFIX_update_kd_tree(pos, data, tree->dim, tree->m,
				tree->root, tree->root->bbox, tree->r);
  }

  return 0;
}

static int KD_FUNC_PREFIX_find_nearest(struct KD_FUNC_PREFIX_kdnode *node, const double *pos, double range, struct KD_FUNC_PREFIX_res_node *list, int ordered, int dim)
{
  double dist_sq, dx;
  int i, ret, added_res = 0;

  if(!node) return 0;

  // If node is a leaf
  if(node->k == -1) {

    for(int j=0; j < node->num_points; ++j) {

      dist_sq = 0;
      for(i=0; i<dim; i++) {
	dist_sq += SQ(node->points[dim*j+i] - pos[i]);
      }
      if(dist_sq <= SQ(range)) {
	if(KD_FUNC_PREFIX_rlist_insert(list, node, ordered ? dist_sq : -1.0) == -1) {
	  return -1;
	}
	added_res += 1;
      }
    }

    return added_res;
  }
  else {

    dx = pos[node->k] - node->l;
    ret = KD_FUNC_PREFIX_find_nearest(dx <= 0.0 ? node->v1 : node->v2, pos, range, list, ordered, dim);

    if(ret >= 0 && fabs(dx) < range) {
      added_res += ret;
      ret = KD_FUNC_PREFIX_find_nearest(dx <= 0.0 ? node->v2 : node->v1, pos, range, list, ordered, dim);
    }
    if(ret == -1) {
      return -1;
    }
    added_res += ret;

    return added_res;
  }

}

static void KD_FUNC_PREFIX_kd_nearest_i(struct KD_FUNC_PREFIX_kdnode *node, const double *pos, int dim, double **closest_point, void *data)
{
  if(!node)
    return;

  double min_dist = DBL_MAX;
  double dist;

  KD_FUNC_PREFIX_print_kdnode(node);

  // If on a leaf
  if(node->k == -1) {
    // Compare points to pos
    for(int i=0; i < node->num_points; ++i) {
      dist = 0;
      for(int j=0; j < dim; ++j)
	dist += SQ(node->points[i*dim + j] - pos[j]);

      if(dist < min_dist) {
	min_dist = dist;
	*closest_point = &(node->points[i*dim]);
	data = node->data[i];
      }
    }
  }
  else {
    // Descend into tree on correct side
    double dx = pos[node->k] - node->l;
    KD_FUNC_PREFIX_kd_nearest_i(dx <= 0.0 ? node->v1 : node->v2, pos, dim, closest_point, data);
  }
}

struct KD_FUNC_PREFIX_kdres *KD_FUNC_PREFIX_kd_nearest(struct KD_FUNC_PREFIX_kdtree *kd, const double *pos)
{
  double *closest_point = malloc(sizeof(double*));
  void **data = malloc(sizeof(void*));
  struct KD_FUNC_PREFIX_kdres *rset;

  if (!kd) return 0;
  if (!kd->root) return 0;

  /* Allocate result set */
  if(!(rset = malloc(sizeof *rset))) {
    return 0;
  }
  if(!(rset->rlist = KD_FUNC_PREFIX_alloc_resnode())) {
    free(rset);
    return 0;
  }
  rset->rlist->next = 0;
  rset->tree = kd;

  /* Search for the nearest neighbour recursively */
  KD_FUNC_PREFIX_kd_nearest_i(kd->root, pos, kd->dim, &closest_point, &data);

  struct KD_FUNC_PREFIX_kdnode *result;
  result = malloc(sizeof(*result));
  result->k = -1;
  result->num_points = 1;
  result->points = malloc(kd->dim*sizeof(double));
  memcpy(result->points, closest_point, kd->dim*sizeof(double));
  result->data = malloc(sizeof(void*));
  result->data[0] = *data;

  /* Store the result */
  if (result) {
    if (KD_FUNC_PREFIX_rlist_insert(rset->rlist, result, -1.0) == -1) {
      KD_FUNC_PREFIX_kd_res_free(rset);
      return 0;
    }
    rset->size = 1;
    KD_FUNC_PREFIX_kd_res_rewind(rset);
    return rset;
  } else {
    KD_FUNC_PREFIX_kd_res_free(rset);
    return 0;
  }
}

struct KD_FUNC_PREFIX_kdres *KD_FUNC_PREFIX_kd_nearest_range(struct KD_FUNC_PREFIX_kdtree *kd, const double *pos, double range)
{
  int ret;
  struct KD_FUNC_PREFIX_kdres *rset;

  if(!(rset = malloc(sizeof *rset))) {
    return 0;
  }
  if(!(rset->rlist = KD_FUNC_PREFIX_alloc_resnode())) {
    free(rset);
    return 0;
  }
  rset->rlist->next = 0;
  rset->tree = kd;

  if((ret = KD_FUNC_PREFIX_find_nearest(kd->root, pos, range, rset->rlist, 0, kd->dim)) == -1) {
    KD_FUNC_PREFIX_kd_res_free(rset);
    return 0;
  }
  rset->size = ret;
  KD_FUNC_PREFIX_kd_res_rewind(rset);
  return rset;
}


void KD_FUNC_PREFIX_kd_res_free(struct KD_FUNC_PREFIX_kdres *rset)
{
  KD_FUNC_PREFIX_clear_results(rset);
  KD_FUNC_PREFIX_free_resnode(rset->rlist);
  free(rset);
}

int KD_FUNC_PREFIX_kd_res_size(struct KD_FUNC_PREFIX_kdres *set)
{
  return (set->size);
}

void KD_FUNC_PREFIX_kd_res_rewind(struct KD_FUNC_PREFIX_kdres *rset)
{
  rset->riter = rset->rlist->next;
}

int KD_FUNC_PREFIX_kd_res_end(struct KD_FUNC_PREFIX_kdres *rset)
{
  return rset->riter == 0;
}

int KD_FUNC_PREFIX_kd_res_next(struct KD_FUNC_PREFIX_kdres *rset)
{
  rset->riter = rset->riter->next;
  return rset->riter != 0;
}

void *KD_FUNC_PREFIX_kd_res_item(struct KD_FUNC_PREFIX_kdres *rset, double *pos)
{
  if(rset->riter) {
    if(pos) {
      memcpy(pos, rset->riter->item->points, rset->tree->dim * sizeof *pos);
    }
    return rset->riter->item->data;
  }
  return 0;
}

void *KD_FUNC_PREFIX_kd_res_item_data(struct KD_FUNC_PREFIX_kdres *set)
{
  return KD_FUNC_PREFIX_kd_res_item(set, 0);
}

/* ---- hyperrectangle helpers ---- */
static struct KD_FUNC_PREFIX_kdhyperrect* KD_FUNC_PREFIX_hyperrect_create(int dim, const double *min, const double *max)
{
  size_t size = dim * sizeof(double);
  struct KD_FUNC_PREFIX_kdhyperrect* rect = 0;

  if (!(rect = malloc(sizeof(struct KD_FUNC_PREFIX_kdhyperrect)))) {
    return 0;
  }

  rect->dim = dim;
  rect->min = malloc(size);
  rect->max = malloc(size);

  memcpy(rect->min, min, size);
  memcpy(rect->max, max, size);

  return rect;
}

static struct KD_FUNC_PREFIX_kdhyperrect* KD_FUNC_PREFIX_hyperrect_auto_create(int dim, int num_points, const double *points)
{
  size_t size = dim * sizeof(double);
  struct KD_FUNC_PREFIX_kdhyperrect* rect = 0;

  if (!(rect = malloc(sizeof(struct KD_FUNC_PREFIX_kdhyperrect)))) {
    return 0;
  }

  rect->dim = dim;
  rect->min = malloc(size);
  rect->max = malloc(size);

  for(int i=0; i < dim; ++i) {
    rect->min[i] = DBL_MAX;
    rect->max[i] = -DBL_MAX;
  }

  for(int i=0; i < num_points; ++i) {
    for(int j=0; j < dim; ++j) {
      if(points[i*dim+j] < rect->min[j])
	rect->min[j] = points[i*dim+j];
      if(points[i*dim+j] > rect->max[j])
	rect->max[j] = points[i*dim+j];
    }
  }

  return rect;
}

static struct KD_FUNC_PREFIX_kdhyperrect* KD_FUNC_PREFIX_hyperrect_auto_create_r(int dim, int num_points, const double *points, double r)
{
  size_t size = dim * sizeof(double);
  struct KD_FUNC_PREFIX_kdhyperrect* rect = 0;

  if (!(rect = malloc(sizeof(struct KD_FUNC_PREFIX_kdhyperrect)))) {
    return 0;
  }

  rect->dim = dim;
  rect->min = malloc(size);
  rect->max = malloc(size);

  for(int i=0; i < dim; ++i) {
    rect->min[i] = DBL_MAX;
    rect->max[i] = -DBL_MAX;
  }

  for(int i=0; i < num_points; ++i) {
    for(int j=0; j < dim; ++j) {
      if(points[i*dim+j] - r < rect->min[j])
	rect->min[j] = points[i*dim+j] - r;
      if(points[i*dim+j]+r > rect->max[j])
	rect->max[j] = points[i*dim+j] + r;
    }
  }

  return rect;
}

static void KD_FUNC_PREFIX_hyperrect_free(struct KD_FUNC_PREFIX_kdhyperrect *rect)
{
  free(rect->min);
  free(rect->max);
  free(rect);
}

static struct KD_FUNC_PREFIX_kdhyperrect* KD_FUNC_PREFIX_hyperrect_duplicate(const struct KD_FUNC_PREFIX_kdhyperrect *rect)
{
  return KD_FUNC_PREFIX_hyperrect_create(rect->dim, rect->min, rect->max);
}

static void KD_FUNC_PREFIX_hyperrect_extend(struct KD_FUNC_PREFIX_kdhyperrect *rect, const double *pos)
{
  int i;

  for (i=0; i < rect->dim; i++) {
    if (pos[i] < rect->min[i]) {
      rect->min[i] = pos[i];
    }
    if (pos[i] > rect->max[i]) {
      rect->max[i] = pos[i];
    }
  }
}

static void KD_FUNC_PREFIX_hyperrect_extend_r(struct KD_FUNC_PREFIX_kdhyperrect *rect, const double *pos, double r)
{
  int i;

  for (i=0; i < rect->dim; i++) {
    if (pos[i]-r < rect->min[i]) {
      rect->min[i] = pos[i] - r;
    }
    if (pos[i]+r > rect->max[i]) {
      rect->max[i] = pos[i]+r;
    }
  }
}

static double KD_FUNC_PREFIX_hyperrect_dist_sq(struct KD_FUNC_PREFIX_kdhyperrect *rect, const double *pos)
{
  int i;
  double result = 0;

  for (i=0; i < rect->dim; i++) {
    if (pos[i] < rect->min[i]) {
      result += SQ(rect->min[i] - pos[i]);
    } else if (pos[i] > rect->max[i]) {
      result += SQ(rect->max[i] - pos[i]);
    }
  }

  return result;
}

/* ---- static helpers ---- */

#ifdef USE_LIST_NODE_ALLOCATOR
/* special list node allocators. */
static struct KD_FUNC_PREFIX_res_node *free_nodes;

#ifndef NO_PTHREADS
static pthread_mutex_t alloc_mutex = PTHREAD_MUTEX_INITIALIZER;
#endif

static struct KD_FUNC_PREFIX_res_node *KD_FUNC_PREFIX_alloc_resnode(void)
{
  struct KD_FUNC_PREFIX_res_node *node;

#ifndef NO_PTHREADS
  pthread_mutex_lock(&alloc_mutex);
#endif

  if(!free_nodes) {
    node = malloc(sizeof *node);
  } else {
    node = free_nodes;
    free_nodes = free_nodes->next;
    node->next = 0;
  }

#ifndef NO_PTHREADS
  pthread_mutex_unlock(&alloc_mutex);
#endif

  return node;
}

static void KD_FUNC_PREFIX_free_resnode(struct KD_FUNC_PREFIX_res_node *node)
{
#ifndef NO_PTHREADS
  pthread_mutex_lock(&alloc_mutex);
#endif

  node->next = free_nodes;
  free_nodes = node;

#ifndef NO_PTHREADS
  pthread_mutex_unlock(&alloc_mutex);
#endif
}
#endif	/* list node allocator or not */


/* inserts the item. if dist_sq is >= 0, then do an ordered insert */
static int KD_FUNC_PREFIX_rlist_insert(struct KD_FUNC_PREFIX_res_node *list, struct KD_FUNC_PREFIX_kdnode *item, double dist_sq)
{
  struct KD_FUNC_PREFIX_res_node *rnode;

  if(!(rnode = KD_FUNC_PREFIX_alloc_resnode())) {
    return -1;
  }
  rnode->item = item;
  rnode->dist_sq = dist_sq;

  if(dist_sq >= 0.0) {
    while(list->next && list->next->dist_sq < dist_sq) {
      list = list->next;
    }
  }
  rnode->next = list->next;
  list->next = rnode;
  return 0;
}

static void KD_FUNC_PREFIX_clear_results(struct KD_FUNC_PREFIX_kdres *rset)
{
  struct KD_FUNC_PREFIX_res_node *tmp, *node = rset->rlist->next;

  while(node) {
    tmp = node;
    node = node->next;
    KD_FUNC_PREFIX_free_resnode(tmp);
  }

  rset->rlist->next = 0;
}

static void KD_FUNC_PREFIX_kd_sample_i(struct KD_FUNC_PREFIX_kdnode *node, double *pos)
{
  // If this is a leaf node, sample inside r-bounding box
  if(node->k == -1) {
    for(int i=0; i < node->rbbox->dim; ++i) {
      pos[i] = (node->rbbox->max[i]-node->rbbox->min[i])*rand()/(RAND_MAX+1.0)
	+ node->rbbox->min[i];
    }
  }
  // Otherwise, chose randomly between edges based on weight (area)
  else {
    double val = rand()/(RAND_MAX + 1.0);
    if( (node->v1->area + node->v2->area)*val < node->v1->area) {
      KD_FUNC_PREFIX_kd_sample_i(node->v1, pos);
    }
    else {
      KD_FUNC_PREFIX_kd_sample_i(node->v2, pos);
    }
  }
}


int KD_FUNC_PREFIX_kd_sample_pos(struct KD_FUNC_PREFIX_kdtree *kd, double *pos)
{
  KD_FUNC_PREFIX_kd_sample_i(kd->root, pos);

  /* Result should be in pos */
  return 0;
}
