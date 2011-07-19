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
#ifndef _KDTREE2_H_
#define _KDTREE2_H_

#ifdef __cplusplus
extern "C" {
#endif

struct KD_FUNC_PREFIX_kdhyperrect {
  int dim;
  double *min, *max;              /* minimum/maximum coords */
};

struct KD_FUNC_PREFIX_kdnode {
  int num_points;   /* number of points (only for leaf node) */
  int m;
  double *points;  /* points */
  void **data;      /* arbitrary data */

  int k;    /* splitting dimension */
  double l; /* splitting plane */

  struct KD_FUNC_PREFIX_kdnode *v1, *v2;	/* negative/positive side of splitting hyperplane */

  struct KD_FUNC_PREFIX_kdhyperrect *bbox; /* Bounding box */
  struct KD_FUNC_PREFIX_kdhyperrect *rbbox; /* r-bounding box */

  double area;    /* area of this node's bounding box */
  int height;  
};

struct KD_FUNC_PREFIX_res_node {
  struct KD_FUNC_PREFIX_kdnode *item;
  double dist_sq;
  struct KD_FUNC_PREFIX_res_node *next;
};

struct KD_FUNC_PREFIX_kdtree {
  int dim;
  int m;
  double r;
  struct KD_FUNC_PREFIX_kdnode *root;
  void (*destr)(void*);
};

struct KD_FUNC_PREFIX_kdres {
  struct KD_FUNC_PREFIX_kdtree *tree;
  struct KD_FUNC_PREFIX_res_node *rlist, *riter;
  int size;
};


  /* create a kd-tree for "k"-dimensional data */
  struct KD_FUNC_PREFIX_kdtree *KD_FUNC_PREFIX_kd_create(int k, int m, double r);

  /* free the struct KD_FUNC_PREFIX_kdtree */
  void KD_FUNC_PREFIX_kd_free(struct KD_FUNC_PREFIX_kdtree *tree);

  /* remove all the elements from the tree */
  void KD_FUNC_PREFIX_kd_clear(struct KD_FUNC_PREFIX_kdtree *tree);

  /* if called with non-null 2nd argument, the function provided
   * will be called on data pointers (see kd_insert) when nodes
   * are to be removed from the tree.
   */
  void KD_FUNC_PREFIX_kd_data_destructor(struct KD_FUNC_PREFIX_kdtree *tree, void (*destr)(void*));

  /* insert a node, specifying its position, and optional data */
  int KD_FUNC_PREFIX_kd_insert(struct KD_FUNC_PREFIX_kdtree *tree, const double *pos, void *data);

  /* Find one of the nearest nodes from the specified point.
   *
   * This function returns a pointer to a result set with at most one element.
   */
  struct KD_FUNC_PREFIX_kdres *KD_FUNC_PREFIX_kd_nearest(struct KD_FUNC_PREFIX_kdtree *tree, const double *pos);

  /* Sample from tree */
  int KD_FUNC_PREFIX_kd_sample_pos(struct KD_FUNC_PREFIX_kdtree *tree, double *pos);


  /* Find any nearest nodes from the specified point within a range.
   *
   * This function returns a pointer to a result set, which can be manipulated
   * by the kd_res_* functions.
   * The returned pointer can be null as an indication of an error. Otherwise
   * a valid result set is always returned which may contain 0 or more elements.
   * The result set must be deallocated with kd_res_free, after use.
   */
  struct KD_FUNC_PREFIX_kdres *KD_FUNC_PREFIX_kd_nearest_range(struct KD_FUNC_PREFIX_kdtree *tree, const double *pos, double range);

  /* frees a result set returned by kd_nearest_range() */
  void KD_FUNC_PREFIX_kd_res_free(struct KD_FUNC_PREFIX_kdres *set);

  /* returns the size of the result set (in elements) */
  int KD_FUNC_PREFIX_kd_res_size(struct KD_FUNC_PREFIX_kdres *set);

  /* rewinds the result set iterator */
  void KD_FUNC_PREFIX_kd_res_rewind(struct KD_FUNC_PREFIX_kdres *set);

  /* returns non-zero if the set iterator reached the end after the last element */
  int KD_FUNC_PREFIX_kd_res_end(struct KD_FUNC_PREFIX_kdres *set);

  /* advances the result set iterator, returns non-zero on success, zero if
   * there are no more elements in the result set.
   */
  int KD_FUNC_PREFIX_kd_res_next(struct KD_FUNC_PREFIX_kdres *set);

  /* returns the data pointer (can be null) of the current result set item
   * and optionally sets its position to the pointers(s) if not null.
   */
  void *KD_FUNC_PREFIX_kd_res_item(struct KD_FUNC_PREFIX_kdres *set, double *pos);

  /* equivalent to kd_res_item(set, 0) */
  void *KD_FUNC_PREFIX_kd_res_item_data(struct KD_FUNC_PREFIX_kdres *set);


#ifdef __cplusplus
}
#endif

#endif	/* _KDTREE_H_ */
