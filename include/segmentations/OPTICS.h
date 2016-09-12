#ifndef _OPTICS_H_Tony_29_AUG_2016_
#define _OPTICS_H_Tony_29_AUG_2016_


#include <limits.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>


#define UNCLASSIFIED -1
#define NOISE 0


#define CORE_POINT 1
#define NOT_CORE_POINT 0

#define SUCCESS 0
#define FAILURE -3

#define UNDEFINED FLT_MAX

#define PCL_NO_PRECOMPILE

struct EIGEN_ALIGN16 opt_point_s              //定义点类型结构

{

	float x, y, z;
	int cluster_id;
	double core_dis;        

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW// 确保new操作符对齐操作

}EIGEN_ALIGN16;// 强制SSE对齐

//}

POINT_CLOUD_REGISTER_POINT_STRUCT(opt_point_s,// 注册点类型宏

	(float,x,x)

	(float,y,y)

	(float,z,z)

	(int,cluster_id,cluster_id)

	(double,core_dis,core_dis)
	)

// struct opt_point_s 
// {
// 	float x, y, z;
// 	int cluster_id;
// 	double core_dis;
// };
typedef struct opt_point_s opt_point_t;

typedef struct opt_node_s opt_node_t;
struct opt_node_s 
{
	unsigned int index;

	//used to order the neighbourhood points 
	double dis_to_core;	//distance to the core point  

	double reach_dis;

	opt_node_t *next;
};



struct opt_epsilon_neighbours_s
{
	unsigned int num_members;
	opt_node_t *head, *tail;
};
typedef struct opt_epsilon_neighbours_s opt_epsilon_neighbours_t;

static pcl::KdTreeFLANN<opt_point_t>::Ptr nn_search;

opt_node_t *create_node(unsigned int index, double dis_to_core);
void destroy_node(opt_node_t *node);

opt_epsilon_neighbours_t* create_epsilon_neighbours();
void destroy_epsilon_neighbours(opt_epsilon_neighbours_t *en);

int append_at_end(
	opt_epsilon_neighbours_t *en,
	opt_node_t *p);

int insert_by_distance(
	unsigned int index,
	double dis,
	opt_epsilon_neighbours_t *en);

int insert_by_reach_distance(
	opt_node_t *node,
	opt_epsilon_neighbours_t *en);

int moveup_reach_distance(
	opt_node_t *node,
	double reach_dis,
	opt_epsilon_neighbours_t *en);

opt_epsilon_neighbours_t *get_epsilon_neighbours(
	unsigned int index,
	opt_point_t *points,
	unsigned int num_points,
	double epsilon,
	double (*dist)(opt_point_t *a, opt_point_t *b));



int expand(
	unsigned int index,
	unsigned int cluster_id,
	opt_point_t *points,
	unsigned int num_points,
	double epsilon,
	unsigned int minpts,
	double (*dist)(opt_point_t *a, opt_point_t *b));

// int spread(
// 	unsigned int index,
// 	opt_epsilon_neighbours_t *seeds,
// 	unsigned int cluster_id,
// 	opt_point_t *points,
// 	unsigned int num_points,
// 	double epsilon,
// 	unsigned int minpts,
// 	double (*dist)(opt_point_t *a, opt_point_t *b));

double euclidean_dist(opt_point_t *a, opt_point_t *b);

double adjacent_intensity_dist(opt_point_t *a, opt_point_t *b);

float core_distance(opt_epsilon_neighbours_t *en, 
	double eps, 
	unsigned int minpts);

void update_seeds(unsigned int index,
	opt_point_t *points,
	unsigned int num_points,
	opt_epsilon_neighbours_t *en,  
	opt_epsilon_neighbours_t *seeds,
	double eps, unsigned int minpts);


opt_node_t *create_node(unsigned int index, double dis_to_core)
{
	opt_node_t *n = (opt_node_t *) calloc(1, sizeof(opt_node_t));
	if (n == NULL)
		perror("Failed to allocate node.");
	else
	{
		n->index = index;
		n->dis_to_core = dis_to_core;
		n->next = NULL;
		n->reach_dis = UNDEFINED;
	}
	return n;
}

void destroy_node(opt_node_t *node)
{
	if(node)
		free(node);
}

opt_epsilon_neighbours_t* create_epsilon_neighbours()
{
	opt_epsilon_neighbours_t *en = (opt_epsilon_neighbours_t *)
		calloc(1, sizeof(opt_epsilon_neighbours_t));
	if (en == NULL) 
	{
		perror("Failed to allocate epsilon neighbours.");
		return NULL;
	}
	en->num_members = 0;
	en->head = NULL;
	en->tail = NULL;

	return en;
}

void destroy_epsilon_neighbours(opt_epsilon_neighbours_t *en)
{
	if (en) 
	{
		opt_node_t *t, *h = en->head;
		while (h) 
		{
			t = h->next;
			destroy_node(h);
			h = t;
		}
		free(en);
	}
}
opt_node_t *pop_node(opt_epsilon_neighbours_t *en, opt_epsilon_neighbours_t *dummy)
{
	opt_node_t *head = en->head;

	if(head == NULL)
		return NULL;

	en->head = head->next;
	append_at_end(dummy, head);

	return head;
}

int append_at_end(
	opt_epsilon_neighbours_t *en,
	opt_node_t *p)
{
	if (en->head == NULL)
	{
		en->head = p;
		en->tail = p;
	}
	else
	{
		en->tail->next = p;
		en->tail = p;
	}
	++(en->num_members);
	return SUCCESS;
}

int insert_by_distance(
	unsigned int index,
	double dis,
	opt_epsilon_neighbours_t *en)
{
	opt_node_t *n = create_node(index, dis);
	if (n == NULL) {
		free(en);
		return FAILURE;
	}

	if (en->head == NULL) 
	{
		en->head = n;
		en->tail = n;
	}
	else
	{
		//ascending by distance to core
		//insert the node to the proper position
		
		if(en->head->dis_to_core > n->dis_to_core)
		{
			opt_node_t *tmp_node = en->head;
			en->head = n;
			n->next = tmp_node;
		}
		else
		{
			opt_node_t *head = en->head;
			opt_node_t *next = head->next;

			while(next && next->dis_to_core < n->dis_to_core)
			{
				head = next;
				next = head->next;
			}
			head->next = n;
			n->next = next;

			if(next == NULL)
				en->tail = n;
		}
	}

	++(en->num_members);
	return SUCCESS;
}

int insert_by_reach_distance(
	opt_node_t *node,
	opt_epsilon_neighbours_t *en)
{
	if (en->head == NULL) 
	{
		en->head = node;
		en->tail = node;
	}
	else
	{
		//ascending by distance to core
		//insert the node to the proper position

		if(en->head->reach_dis > node->reach_dis)
		{
			opt_node_t *tmp_node = en->head;
			en->head = node;
			node->next = tmp_node;
		}
		else
		{
			opt_node_t *head = en->head;
			opt_node_t *next = head->next;

			while(next && next->reach_dis < node->reach_dis)
			{
				head = next;
				next = head->next;
			}
			head->next = node;
			node->next = next;

			if(next == NULL)
				en->tail = node;
		}
	}

	++(en->num_members);
	return SUCCESS;
}

int moveup_reach_distance(
	opt_node_t *node,
	double reach_dis,
	opt_epsilon_neighbours_t *en)
{
	opt_node_t *h = en->head;

	while(h)
	{
		if(node == h)
		{
			h->reach_dis = reach_dis;
			break;
		}

		h = h->next;
	}

	if(h == NULL)
		return FAILURE;
	else
		return SUCCESS;
}


//ordered by distance
int get_epsilon_neighbours(
	unsigned int index,
	opt_point_t *points,
	unsigned int num_points,
	double epsilon,
	double (*dist)(opt_point_t *a, opt_point_t *b),
	opt_epsilon_neighbours_t *en)
{
	
	std::vector<int> ptIdxSearched;
	std::vector<float> ptSqrDist;

	if ( nn_search->radiusSearch (points[index], epsilon, ptIdxSearched, ptSqrDist) > 1 )
	{
		for(int i=1; i < ptIdxSearched.size (); ++i)
		{
			double dis_to_core = sqrt(ptSqrDist[i]);
			if (insert_by_distance(ptIdxSearched[i], dis_to_core, en) == FAILURE) 
			{
				destroy_epsilon_neighbours(en);
				en = NULL;
				break;
			}
		}

	}

	return 1;
}


int expand(
	unsigned int index,
	unsigned int cluster_id,
	opt_point_t *points,
	unsigned int num_points,
	double epsilon,
	unsigned int minpts,
	double (*dist)(opt_point_t *a, opt_point_t *b))
{
	int return_value = NOT_CORE_POINT;

	opt_epsilon_neighbours_t *en = create_epsilon_neighbours();
	if (en == NULL) {
		perror("Failed to allocate epsilon neighbours.");
		return FAILURE;
	}

	opt_epsilon_neighbours_t *seeds = create_epsilon_neighbours();
	if (seeds == NULL) {
		perror("Failed to allocate epsilon neighbours.");
		return FAILURE;
	}

	opt_epsilon_neighbours_t *dummy = create_epsilon_neighbours();
	if (dummy == NULL) {
		perror("Failed to allocate epsilon neighbours.");
		return FAILURE;
	}

	get_epsilon_neighbours(index, points,
		num_points, epsilon,
		dist,
		en);


	if (en->num_members < minpts)
		points[index].cluster_id = NOISE;
	else 
	{
//		opt_node_t *core = create_node(index, 0);
		points[index].cluster_id = cluster_id;

		if((points[index].core_dis = core_distance(en, epsilon, minpts)) != UNDEFINED)
		{
			update_seeds(index, points, num_points, en, seeds, epsilon, minpts);

			opt_node_t *q = pop_node(seeds, dummy);

			while(q)
			{
				en->num_members=0;
				en->head=NULL;
				en->tail=NULL;

				get_epsilon_neighbours(q->index, points,
					num_points, epsilon,
					dist,
					en);

				points[q->index].cluster_id = cluster_id;

				points[q->index].core_dis = core_distance(en, epsilon, minpts);
				if (points[q->index].core_dis != UNDEFINED)
						update_seeds(q->index, points, num_points, en, seeds, epsilon, minpts);
			}
			
		}

		return_value = CORE_POINT;
	}

		
	destroy_epsilon_neighbours(seeds);
	destroy_epsilon_neighbours(dummy);

	en->head=NULL;
	en->tail=NULL;
	destroy_epsilon_neighbours(en);
	return return_value;
}

/*
int spread(
	unsigned int index,
	opt_epsilon_neighbours_t *seeds,
	unsigned int cluster_id,
	opt_point_t *points,
	unsigned int num_points,
	double epsilon,
	unsigned int minpts,
	double (*dist)(opt_point_t *a, opt_point_t *b))
{
	opt_epsilon_neighbours_t *spread =
		get_epsilon_neighbours(index, points,
		num_points, epsilon,
		dist);

	if (spread == NULL)
		return FAILURE;
	if (spread->num_members >= minpts) {
		opt_node_t *n = spread->head;
		opt_point_t *d;
		while (n) {
			d = &points[n->index];
			if (d->cluster_id == NOISE ||
				d->cluster_id == UNCLASSIFIED) {
					if (d->cluster_id == UNCLASSIFIED) {
						if (append_at_end(n->index, seeds)
							== FAILURE) {
								destroy_epsilon_neighbours(spread);
								return FAILURE;
						}
					}
					d->cluster_id = cluster_id;
			}
			n = n->next;
		}
	}

	destroy_epsilon_neighbours(spread);
	return SUCCESS;
}
*/
double euclidean_dist(opt_point_t *a, opt_point_t *b)
{
	return sqrt(pow(a->x - b->x, 2) +
		pow(a->y - b->y, 2) +
		pow(a->z - b->z, 2));
}

float core_distance(opt_epsilon_neighbours_t *en, double eps, unsigned int minpts)
{
	if(en->num_members < minpts)
		return UNDEFINED;
	else
	{
		int i=1;
		opt_node_t *head = en->head;
		while(i<minpts)
		{
			i++;
			head = head->next;
		}
		return head->dis_to_core;
	}
}

void update_seeds(unsigned int index,
	opt_point_t *points,
	unsigned int num_points,
	opt_epsilon_neighbours_t *en,  
	opt_epsilon_neighbours_t *seeds,
	double eps, 
	unsigned int minpts)
{
	double coredist = points[index].core_dis;

	opt_node_t *t, *h = en->head;

	while(h)
	{
		if(points[h->index].cluster_id == UNCLASSIFIED)
		{
			double new_reach_dist = std::max(h->dis_to_core, coredist);
			if(h->reach_dis == UNDEFINED)
			{
				h->reach_dis = new_reach_dist;
				insert_by_reach_distance(h, seeds);
			}
			else
			{
				if (new_reach_dist < h->reach_dis)
				{
					moveup_reach_distance(h, new_reach_dist, seeds);
				}
			}
		}

		h = h->next;
	}

}

void optics(
	opt_point_t *points,
	unsigned int num_points,
	double epsilon,
	unsigned int minpts,
	double (*dist)(opt_point_t *a, opt_point_t *b))
{
	unsigned int i, cluster_id = 1;


	pcl::KdTreeFLANN<opt_point_t>::Ptr kdtree (new pcl::KdTreeFLANN<opt_point_t>);

	pcl::PointCloud<opt_point_t>::Ptr cloud (new pcl::PointCloud<opt_point_t>); 

	cloud->height = 1;
	cloud->width = num_points;
	cloud->points.assign(points, points+num_points);
	kdtree->setInputCloud (cloud);

	nn_search = kdtree;

	for (i = 0; i < num_points; ++i)
	{
		if (points[i].cluster_id == UNCLASSIFIED)
		{
			if (expand(i, cluster_id, points,
				num_points, epsilon, minpts,
				dist) == CORE_POINT)

				++cluster_id;
		}
	}
}





#endif