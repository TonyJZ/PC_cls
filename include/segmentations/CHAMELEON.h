#ifndef _CHAMELEON_H_Tony_23_AUG_2016_
#define _CHAMELEON_H_Tony_23_AUG_2016_


/******************************************************
 * This is my CHAMELEON clustering program            *
 * Author: Weinan Wang                                *
 ******************************************************/

#include &lt;stdio.h> /* for sscanf, FILE* */
#include &lt;stdlib.h> /* for calloc, malloc, recalloc, free */
#include &lt;math.h> /* for sqrt */
#include &lt;string.h> /* for strlen */
#include &lt;sys/time.h>

/* !!!!!! */
/* Define the maximum size of the data points */
#define MAXSIZE 10000

/* maxmum length of the file name */
#define MAX_fileName 14

/* !!!!!! */
/* Define the K_nearest as 10,
 * this is according to the CHAMELEON paper p.13.
 */
/* #define K_nearest 10 */
#define K_nearest 15
/*#define K5_nearest 50 *//* 5*K_nearest */
#define K5_nearest 50

/* !!!!!! */
/* this is the balance constraint to be used in hMETIS.
 * this selection is according to page 10 of the CHAMELEON paper
 */
#define BC 25

/*!!!!!!*/
/* This is the MINSIZE over all the points, to be used in hMETIS.
 * this selection is according to page 13 of the CHAMELEON paper
 */
/* #define MINSIZE 0.025 */
#define MINSIZE 0.04

/* !!!!!! */
/* This is the domain width and height of the data set */
/* This set of WIDTH and HEIGHT is for t7.10k.dat */
#define WIDTH 700
#define HEIGHT 500
/* this is the diagonal distance */
#define DIAG 860.0

/* this is used for the maxmum number of groups
 * after phase 1's partition.
 */
#define MAXGROUPS 100

/* this alpha is used for the criterian function (p11 of the CHAMELEON paper)
 * for merging in phase 2.
 * This selection is according to p13 of the CHAMELEON paper.
 */
#define ALPHA 300.0

/* 
 * This data type represent a hyper edge for a point.
 */
typedef struct
{
  int pointNO; /* the other point number */
  double similarity; /* similarity (DIAG-distance)/DIAG */
} Hyperedge;


/*
 * This data type contains information of
 * one point.
 */
typedef struct
{
  float x;
  float y;
  Hyperedge edges[K5_nearest]; 
  /* length cannot be bigger than K5_nearest */
  int length; /* current number of hyperedges */
} Point;


/* 
 * This structure represent
 * a node in the binary tree of 
 * graph partition.
 */
struct node {
  int * points; /* list of point numbers */
  int numberPoints; /* number of points belongs to this node */
  struct node *left; /* left subtree */
  struct node *right; /* right subtree */
};

/**************************************************
 * Declare static functions                       *
 **************************************************/
static int parsingInput(int argc, char *argv[]);
static int readData();
static int initialize();
static int establish_hyperGraph();
static double computeSimilarity(int i, int j);
static int cutNode(struct node * source, struct node * left, struct node * right, int ub);
static int partition(struct node* source, struct node * left, struct node * right);
static int belongs(struct node *, int point);
static int phase2();
static int merge(int group0, int group1);
static float computeGoodness(int group0, int group1);
static int computeAIC_AC(struct node * node0, struct node * node1, float * aic, float * ac);
static int clusterResult();
static long compute_time(struct timeval start_time, struct timeval end_time);


/*************************************************
 * Declare global variables                      *
 *************************************************/
/* data points */
Point points[MAXSIZE];

FILE * fp; /* file pointer pointing to the data file */
FILE * out_fp; /* file pointer pointing to the output file */

/* N is the total number of data points in the data file */
int N;
/* fileName is the data file's name */
char fileName[MAX_fileName + 1];
/* stopClusterNO is the number of remainning clusters after
 * the whole cluster process.
 */
int stopClusterNO;

/* partition tree root */
struct node *root = NULL;

/* threshold to decide whether stop partition */
int threshold;

/* this variable is for output matlab file */
int index_matlab=0;

/* 
 * this variable is used for indexing point groups 
 * produced in phase 1.
 */
int groupIndex;

int * groups[MAXGROUPS];
int groupLength[MAXGROUPS];


/* following varaibles are for merging phase in phase 2 */
float bestGoodness; /* best goodness */
int mergeGroups[2] = {0, 0}; /* the two groups' number should be merged */

/* for timing */
long time_period;
struct timeval time_start, time_end;


/****************************************************************
 * Name    : main                                               *
 * Function: main function                                      *
 * Input   : argc, argv                                         *
 * Output  : void                                               *
 *           The usage of the rock program should be of the     *
 *           form:                                              *
 *             chameleon N fileName stopClusterNO               *
 *           Each parameter's meanning is :                     *
 *           chameleon -- program's name                        *
 *           int N    -- how many data points does the file has *
 *           string fileName -- indicate the data file          *
 *                              name which                      *
 *                               contains all the data vectors. *
 *           int stopClusterNO -- number of remaining clusters  *
 *                                at the end of clustering.     *
 ****************************************************************/
int main(int argc, char * argv[]) {
  struct node * left;
  struct node * right;

  int i;

  if(gettimeofday(&time_start, NULL)&lt;0)
    {
      printf("time error \n");
      return -1;
    }

  printf("ALPHA is %f \n", ALPHA);

  /* parse the command line */
 if (parsingInput(argc, argv)&lt;0)
   {
     return -1;
   }

  /* read in data file */
 if ( readData()&lt;0 )
   {
      fclose(fp);
      return -1;
   }

 /* close read in data file */
 fclose(fp);

 /* for debugging purpose */
 /* printData(); */

 /* initialize data structure */
 if ( initialize()&lt;0 )
   {
     return -1;
   }

 /* establish hyperGraph */
 if (establish_hyperGraph()&lt;0)
   {
     return -1;
   }


 /* Now begin partition */
 left = (struct node *)calloc(1, sizeof(struct node));
 if(left == NULL)
   {
     printf("cannot allocate memory for left! \n");
     return -1;
   }
 right = (struct node *)calloc(1, sizeof(struct node));
 if(right == NULL)
   {
     printf("cannot allocate memory for right! \n");
     return -1;
   }

 /* begin partitionning, this is a recursive program */
 if(partition(root, left, right)&lt;0)
   {
     printf("partition error! \n");
     return -1;
   }

 if(phase2()&lt;0)
   {
     printf("phase2 error! \n");
     return -1;
   }

 if(clusterResult()&lt;0)
   {
     fclose(out_fp);
     printf("clusterResult error! \n");
     return -1;
   }
 fclose(out_fp);

 for(i=0; i&lt;groupIndex; i++)
   {
     free(groups[i]);
     groups[i] = NULL;
   }

 if(gettimeofday(&time_end, NULL)&lt;0){
   printf("time error \n");
   return -1;
 }
 
  /* Now compute the time for sequential sorting */
 time_period = compute_time(time_start, time_end);
  
 printf("time spend is : %d \n", time_period);

 return 1;
}/* end main */

/****************************************************************
 * Name    : compute_time                                       *
 * Function: compute the time interval between two              *
 *           given time structures.                             *
 * Input   : (struct timeval start_time),                       *
 *           (struct timeval end_time)                          *
 * Output  : long                                               *
 ****************************************************************/

static long compute_time(struct timeval start_time, struct timeval end_time)
{
  long sec, msec, time;

  /* time interval in sec */
  sec = (long)(end_time.tv_sec - start_time.tv_sec);

  /* time interval in msec */
  msec = (long)(end_time.tv_usec - start_time.tv_usec);

  /* compute time difference in msec */
  time = sec*1000000+msec;

  return time;

}/* end compute_time */



/****************************************************************
 * Name    : clusterResult                                      *
 * Function: output cluster result.                             *
 * Input   : void                                               *
 * Output  : int                                                *
 ****************************************************************/
static int clusterResult(){
  int i, j, count;
  
  /* open hyperGraph file to write output */
  if ((out_fp = fopen("output.m", "w")) == NULL )
    {
      printf("cannot open output file correctly! \n");
      return -1;
    }

  count = 0;
  for(i=0; i&lt;groupIndex; i++){
    if(groupLength[i]>0){
      /* printf("c%d =[\n", count);*/
      fprintf(out_fp, "%% The cluster NO is %d \n", i);
      fprintf(out_fp, "c%d =[", count);
      for(j=0; j&lt;groupLength[i]; j++){
	/* printf("%f %f \n", points[groups[i][j]].x, points[groups[i][j]].y);*/
	fprintf(out_fp, "%f %f \n", points[groups[i][j]].x, points[groups[i][j]].y);
      }/* end for */
      /* printf("];\n\n");*/
      fprintf(out_fp, "];\n");
      count++;
    }/* end if */
  }/* end for */

  if(count != stopClusterNO){
    printf("existing cluster number is not equal to required cluster numer! \n");
    return -1;
  }/* end if */

  return 1;
}/* end clusterResult */



/****************************************************************
 * Name    : computeAIC_AC                                      *
 * Function: compute absolute inter-connectivity and absolute   *
 *           closeness of two nodes.                            *
 * Input   : struct node * node0 -- pointer to node0.           *
 *           struct node * node1 -- pointer to node1.           *
 *           float * aic -- pointer to the return value for     *
 *                          absolute inter-connectivity.        *
 *           float * ac  -- pointer to the return value for     *
 *                          absolute closeness.                 *
 * Output  :  float                                             *
 ****************************************************************/
static int computeAIC_AC(struct node * node0, struct node * node1, float * aic, float * ac){
  int count, i, j;

  count = 0;
  (* aic) = 0.0;
  (* ac) = 0.0;
  /* count the edges from node0 */
  for(i=0; i&lt;node0->numberPoints; i++){
    for(j=0; j&lt;points[node0->points[i]].length; j++){
      if(belongs(node1, points[node0->points[i]].edges[j].pointNO)>0){
	(* aic) = (* aic) + points[node0->points[i]].edges[j].similarity;
	count++;
      }/* end if */
    }/* end for j */
  }/* end for i */
  /*  printf("!!! aic is %f,", *aic);*/
  /* count the edges from node1 */
  for(i=0; i&lt;node1->numberPoints; i++){
    for(j=0; j&lt;points[node1->points[i]].length; j++){
      if(belongs(node0, points[node1->points[i]].edges[j].pointNO)>0){
	(* aic) = (* aic) + points[node1->points[i]].edges[j].similarity;
	count++;
      }/* end if */
    }/* end for j */
  }/* end for i */
  /* printf(" aic is %f !!!", *aic);*/

  /*  printf("count is %d, ", count);*/

  if(count>0)
    {
      (* ac) = (* aic)/((float)count);
    }
  else
    {
      (*ac) = 0;
      (* aic) = 0;
    }

  if(*ac > 10000 || *aic > 10000){
    printf("ac is %e, aic is %e, count is %d \n", (*ac), (*aic), count);
  }

  return 1;
}/* end computeAIC_AC */


/****************************************************************
 * Name    : computeGoodness                                    *
 * Function: compute goodness for merging two clusters.         *
 * Input   : int group0 -- group0's index.                      *
 *           int group1 -- group1's index.                      *
 * Output  :  float                                             *
 ****************************************************************/
static float computeGoodness(int group0, int group1){
  float goodness;
  float ri, rc;
  int ubFactor;

  /* form two nodes according to two groups */
  struct node * node0;
  struct node * node1;
  /* left and right is for cutting the node0 and node 1 to
   * compute internal connectivity and internal closeness.
   */
  struct node * left0;
  struct node * right0;
  struct node * left1;
  struct node * right1;

  int i;
  /* absolute inter-connectivity and absolute closeness */
  float aic, ac, aic0, ac0, aic1, ac1;

  /*!!!!!!!*/
  ubFactor = 6;

  /* First create node0 and node1 according to group0 and group1 */
  node0 = (struct node *)calloc(1, sizeof(struct node));
  node1 = (struct node *)calloc(1, sizeof(struct node));
  if(node0==NULL || node1==NULL){
    printf("cannot allocate memory for node0 and node1. \n");
    return -1;
  }/* end if */

  node0->left=NULL;
  node0->right=NULL;
  node0->numberPoints = groupLength[group0];
  node0->points = (int *)calloc(node0->numberPoints, sizeof(int));
  if(node0->points == NULL){
    printf("cannot allocate memory for node0->points. \n");
    return -1;
  }
  for(i=0; i&lt;groupLength[group0]; i++){
    node0->points[i]=groups[group0][i];
  }/* end for */


  node1->left=NULL;
  node1->right=NULL;
  node1->numberPoints = groupLength[group1];
  node1->points = (int *)calloc(node1->numberPoints, sizeof(int));
  if(node1->points == NULL){
    printf("cannot allocate memory for node0->points. \n");
    return -1;
  }
  for(i=0; i&lt;groupLength[group1]; i++){
    node1->points[i]=groups[group1][i];
  }/* end for */

  /* Now compute absolute inter-connectivity and 
   * absolute closeness.
   */
  computeAIC_AC(node0, node1, &aic, &ac);


  if(ac > 0.0 && aic > 0.0){
    /* now need to compute the internal connectivity of 
     * node0 and node1.
     */
    left0 = (struct node *)calloc(1, sizeof(struct node));
    right0 = (struct node *)calloc(1, sizeof(struct node));
    if(left0==NULL || right0==NULL){
      printf("cannot allocate memory for node0 and node1. \n");
      return -1;
    }/* end if */
    
    if(cutNode(node0, left0, right0, ubFactor)&lt;0)
      {
	printf("cut node0 error! \n");
	return -1;
      }
    
    /* Now compute internal inter-connectivity and 
     * absolute closeness of node0.
     */
    computeAIC_AC(left0, right0, &aic0, &ac0);
    
    left1 = (struct node *)calloc(1, sizeof(struct node));
    right1 = (struct node *)calloc(1, sizeof(struct node));
    if(left1==NULL || right1==NULL){
      printf("cannot allocate memory for node0 and node1. \n");
      return -1;
    }/* end if */
    
    if(cutNode(node1, left1, right1, ubFactor)&lt;0)
      {
	printf("cut node0 error! \n");
	return -1;
      }
    
    /* Now compute internal inter-connectivity and 
     * absolute closeness of node0.
     */
    computeAIC_AC(left1, right1, &aic1, &ac1);
    
    
    /* Now compute Relative Inter-Connectivity */
    ri = (aic*2)/(aic0+aic1);

    /* Now compute Relative Closeness */
    rc = ac*(node0->numberPoints+node1->numberPoints)/((node0->numberPoints)*ac0 + 
						       (node1->numberPoints)*ac1);
  
    /* 
       printf("aic is %f, aic0 is %f, aic1 is %f \n", aic, aic0, aic1);
       printf("ac is %f, ac0 is %f, ac1 is %f \n", ac, ac0, ac1);
       printf("node0->numberPoints is %d, node1->numberPoints is %d \n", 
       node0->numberPoints, node1->numberPoints);
    */

    if(ri>10000 || rc>10000){
      printf("ri or ic is wrong! ri= %f, rc= %f \n", ri, rc);
      return -1;
    }
    /* The final goodness according to p11 of the CHAMELEON paper */
    /* printf("ri is %f, rc is %f. \n", ri, rc);*/
    goodness = ri*pow((double)rc, ALPHA);


    free(node0->points);
    free(node1->points);
    free(left0->points);
    free(right0->points);
    free(left1->points);
    free(right1->points);
    free(left0);
    free(right0);
    free(left1);
    free(right1);
    free(node0);
    free(node1);
  }
  else{
    ri=0.0;
    rc=0.0;
    /* printf("ri is %f, rc is %f. \n", ri, rc);*/
    goodness = 0.0;

    free(node0->points);
    free(node1->points);
    free(node0);
    free(node1);
  }/* end if...else...*/

  return goodness;
}/* end computeGoodness */

/****************************************************************
 * Name    : merge                                              *
 * Function: merge 2 clusters.                                  *
 * Input   : int group0 -- group0's index.                      *
 *           int group1 -- group1's index.                      *
 * Output  :  int                                               *
 ****************************************************************/
static int merge(int group0, int group1){
  int i, j;

  groups[group0] = (int *)realloc(groups[group0], 
				  (groupLength[group0]+groupLength[group1])*sizeof(int)) ;
  if(groups[group0]==NULL){
    printf("cannot enlarge groups[group0]; \n");
    return -1;
  }
  /* move elements from group1 to group0 */
  for(i=groupLength[group0], j=0; i&lt;(groupLength[group0]+groupLength[group1]); i++, j++){
    groups[group0][i] = groups[group1][j];
  }/* end for */

  free(groups[group1]);
  groups[group1]=NULL;

  groupLength[group0] = groupLength[group0]+groupLength[group1];
  groupLength[group1] = 0;

  return 1;
}/* end merge */


/****************************************************************
 * Name    : phase2                                             *
 * Function: phase 2, merging clusters.                         *
 * Input   : void                                               *
 * Output  :  int                                               *
 ****************************************************************/
static int phase2(){
  int clusterNO;
  float goodness;
  int group0, group1;
  int i, j;

  /* 
   * groupIndex at the end of phase 1 gives the
   * number of initial clusters after phase 1.
   */
  clusterNO = groupIndex;

#ifdef Debug_phase2
  printf("At the beginning of phase 2, clusterNO is %d, stopClusterNO is %d \n", 
	 clusterNO, stopClusterNO);
#endif

  if(clusterNO&lt;stopClusterNO){
    printf("at the beginning of phase 2, clusterNO is already smaller than stopClusterNO! error\n");
    return -1;
  }/* end if */

  bestGoodness = 0.0;
  while(clusterNO>stopClusterNO){
    /* the for loop decide which two groups need to be merged */
    for(i=0; i&lt;(groupIndex-1); i++){
      if(groupLength[i]!=0){
	group0 = i;
	for(j=(group0+1); j&lt;groupIndex; j++){
	  if(groupLength[j]!=0){
	    group1 = j;
	    
	    /* Now compute the goodness for merging the two clusters */
	    goodness = computeGoodness(group0, group1);
	    if(goodness > 10000 || goodness &lt; 0){
	      printf("group0 is %d, group1 is %d,!!!\n", 
		     group0, group1);
	      printf("goodness is %f, some thing wrong!\n", goodness);

	      return -1;
	    }
	    /* printf("group0 is %d, group1 is %d, goodness is %f !!! \n", 
	       group0, group1, goodness);
	    */
	    if(goodness > bestGoodness)
	      {
		bestGoodness = goodness;
		mergeGroups[0] = group0;
		mergeGroups[1] = group1;
	      }
	  }/* end if */
	}/* end for j*/
      }/* end if */
    }/* end for i*/
    
    /* now merge the two selected groups */
    if(mergeGroups[0]==mergeGroups[1]){
      printf("mergeGroups[0]==mergeGroups[1]==%d, something wrong! \n", 
	     mergeGroups[0]);
      return -1;
    }

    if(bestGoodness == 0.0){
      printf("bestGoodness reached 0 \n");
      return 1;
    }

    printf("best goodness is %f \n", bestGoodness);

    if(merge(mergeGroups[0], mergeGroups[1])&lt;0)
      {
	printf("merge error! \n");
	return -1;
      }

    clusterNO--;
    
#ifdef Debug_phase2
    printf("In the while loop,  clusterNO is %d, stopClusterNO is %d, mergeGroups[0] is %d, mergeGroups[1] is %d. \n", 
	   clusterNO, stopClusterNO, mergeGroups[0], mergeGroups[1]);
#endif

    bestGoodness = 0.0;
    mergeGroups[0]=0;
    mergeGroups[1]=0;

  }/* end while */

  return 1;

}/* end phase2 */


/****************************************************************
 * Name    : belongs                                            *
 * Function: decide whether a point belongs to a node.          *
 * Input   : struct node * sourceNode --  the node              *
 *           int point -- the point                             *
 * Output  :  int                                               *
 ****************************************************************/
static int belongs(struct node * sourceNode, int point){
  int i, tmp;

  tmp = 0;

  for(i=0; i&lt;sourceNode->numberPoints; i++){
    if(sourceNode->points[i] == point)
      {
	tmp = 1;
	break;
      }/* end if */
  }

  return tmp;

}/* end belongs */

/****************************************************************
 * Name    : cutNode                                            *
 * Function: cut a node of the tree into two parts, and thus    *
 *           return the left pointer and the right pointer.     *
 * Input   : struct node * source --  the source node to be cut.*
 *           struct node * left -- the left resulting node.     *
 *           struct node * right -- the right resulting node.   *
 * Output  :  int                                               *
 ****************************************************************/
/* May 30th, Weinan: one problem I have just found is that I need to 
 * establish a kind of checking table to connect my global vertex 
 * number with local vertex number to be used in the HMETIS_PartRecursive
 * function calling.
 */
static int cutNode(struct node * source, struct node * left, struct node * right, int ub){
  int nvtxs, nhedges;
  int * vwgts = NULL;
  int * eptr = NULL;
  int * eind = NULL;
  int * hewgts = NULL;
  int nparts;
  int ubfactor;
  int * options = NULL;
  int * part = NULL;
  int * edgecut = NULL;

  int tmp, i, j, k, l, tmpNO, flag, found, group0, group1, index0, index1;
  /* this is the table used to correspond global vertice number with 
   * local vertice number.
   */
  int * checkTable;

  /* number of vertices */
  nvtxs = source->numberPoints;
  checkTable = (int *)calloc(nvtxs, sizeof(int));
  if(checkTable == NULL)
    {
      printf("cannot allocate memory for checkTable! \n");
      return -1;
    }
  /* load the vertice's number into the checkTable */
  for(i=0; i&lt;nvtxs; i++){
    checkTable[i] = source->points[i];
  }/* end for */

  /* number of edges */
  tmp=0;
  for(i=0; i&lt;nvtxs; i++){
    for(j=0; j&lt;points[source->points[i]].length; j++){
      /* decide a point whether belongs to a node */
      if(belongs(source, points[source->points[i]].edges[j].pointNO)==1) /* !!!!!! */
	tmp++;
    }/* end for j*/
  }/* end for i*/
  nhedges = tmp;

  /* weight of the vertices, because 
   * the vertices are not weighted, so
   * according to p13 of the hMETIS manual,
   * vwgts can be NULL.
   */
  vwgts = NULL;

  /* eptr and eind */
  /* because all my edges are of
   * size 2, so it makes things 
   * easier.
   */
  tmp = nhedges+1;
  eptr = (int *)calloc(tmp, sizeof(int));
  if(eptr == NULL)
    {
      printf("cannot allocate memory for eptr! \n");
      return -1;
    }
  for(i=0; i&lt;tmp; i++)
    eptr[i]=i*2;

  /* when loading eind, need to check the checkTable */
  eind = (int *)calloc(2*tmp, sizeof(int));
  if(eind == 0)
    {
      printf("cannot allocate memory for eind! \n");
      return -1;
    }
  k = 0;
  for(i=0; i&lt;nvtxs; i++){
    for(j=0; j&lt;points[source->points[i]].length; j++){
      /* decide a point whether belongs to a node */
      if(belongs(source, points[source->points[i]].edges[j].pointNO)==1){
	/* eind[k] = source->points[i];*/
	eind[k] = i;
	k++;
	/* eind[k] = points[source->points[i]].edges[j].pointNO; */
	tmpNO =  points[source->points[i]].edges[j].pointNO;
	flag = 0;
	for(l=0; l&lt;nvtxs; l++){
	  if (tmpNO == checkTable[l]){
	    flag = 1;
	    found = l;
	    break;
	  }
	}
	if(flag == 1)
	  eind[k] = found;
	else
	  {
	    printf("some thing wrong with checkTable! \n");
	    return -1;
	  }/* end if...else... */
	k++;      
      }/* end if */
    }/* end for j*/
  }/* end for i*/
  if(k != 2*nhedges){
    printf("some thing wrong with eind! \n");
    return -1;
  }/* end if */

  /* hewgts: an array of size nhedges that stores the weight
   * of the hyperedges.
   */
  hewgts = (int *)calloc(nhedges, sizeof(int));
  if(hewgts == 0)
    {
      printf("cannot allocate memory for hewgts! \n");
      return -1;
    }
  k = 0;
  for(i=0; i&lt;nvtxs; i++){
    for(j=0; j&lt;points[source->points[i]].length; j++){
      /* decide a point whether belongs to a node */
      if(belongs(source, points[source->points[i]].edges[j].pointNO)==1){
	/*!!!!!! here I have to do a cast becasue now similarity is a double */
	hewgts[k] = (int)(points[source->points[i]].edges[j].similarity*DIAG);
	k++;      
      }/* end if */
    }/* end for j*/
  }/* end for i*/
  if(k != nhedges){
    printf("some thing wrong with hewgts! \n");
    return -1;
  }/* end if */

  /* nparts: number of desired partitions */
  nparts = 2;

  /* ubfactor: relative imbalance factor */
  ubfactor = ub;

  /* options */
  /* note that is options[0]=0,
   * then default values are used.
   */
  options =  (int *)calloc(9, sizeof(int));
  if(options == 0)
    {
      printf("cannot allocate memory for options! \n");
      return -1;
    }
  options[0] = 0;

  /* part */
  part =  (int *)calloc(nvtxs, sizeof(int));
  if(part == 0)
    {
      printf("cannot allocate memory for part! \n");
      return -1;
    }

  /* edgecut */
  edgecut = (int *)calloc(1, sizeof(int));
  if(edgecut == 0)
    {
      printf("cannot allocate memory for edgecut! \n");
      return -1;
    }


  HMETIS_PartRecursive(nvtxs, nhedges, vwgts, eptr, eind, hewgts, 
		       nparts, ubfactor, options, part, edgecut);

  group0 = 0;
  group1 = 0;
  for(i=0; i&lt;nvtxs; i++){
    if(part[i] == 0)
      group0++;
    else if(part[i] == 1)
      group1++;
    else
      {
	printf("something wrong with part. \n");
	return -1;
      }/* end if..else...*/
  }/* end for */

  left->numberPoints = group0;
  right->numberPoints = group1;
  left->points = (int *)calloc(group0, sizeof(int));
  if(left->points == NULL)
    {
      printf("cannot allocate memory for left->points \n");
      return -1;
    }
  right->points = (int *)calloc(group1, sizeof(int));
  if(right->points == NULL)
    {
      printf("cannot allocate memory for right->points \n");
      return -1;
    }
  left->left = NULL;
  left->right = NULL;
  right->left = NULL;
  right->right = NULL;

  index0 = 0;
  index1 = 0;
  for(i=0; i&lt;nvtxs; i++){
    if(part[i] == 0)
      {
	left->points[index0] = checkTable[i];
	index0++;
      }
    else if(part[i] == 1)
     {
	right->points[index1] = checkTable[i];
	index1++;
      }
    else
      {
	printf("something wrong with part. \n");
	return -1;
      }/* end if..else...*/
  }/* end for */
  if(index0!=group0 || index1!=group1)
    {
      printf("some thing wrong with index0-1. \n");
      return -1;
    }

  free(vwgts);
  free(eptr);
  free(eind);
  free(hewgts);
  free(options);
  free(part);
  free(edgecut);

  free(checkTable);

  return 1;

}/* end cutNode */


/****************************************************************
 * Name    : partition                                          *
 * Function: partition the graph recursively.                   *
 * Input   : void                                               *
 * Output  :  int                                               *
 ****************************************************************/
static int partition(struct node * source, struct node * left, struct node * right){
  struct node * l_left;
  struct node * l_right;
  struct node * r_left;
  struct node * r_right;
  int i;

  /* cut the source node into left node and right node */
  if(cutNode(source, left, right, BC)&lt;0){
    printf("cutNode error! \n");
    return -1;
  }
  /* link the two sub-trees to the root */
  source->left = left;
  source->right = right;

  /* if left sub-tree is still bigger than threshold */
  if(left->numberPoints > threshold) {
    l_left = (struct node *)calloc(1, sizeof(struct node));
    if(l_left == NULL)
      {
	printf("cannot allocate memory for left! \n");
	return -1;
      }
    l_right = (struct node *)calloc(1, sizeof(struct node));
    if(l_right == NULL)
      {
	printf("cannot allocate memory for right! \n");
	return -1;
      }

    if(partition(left, l_left, l_right)&lt;0)
      return -1;

  }else{
    groupLength[groupIndex] = left->numberPoints;
    groups[groupIndex] =  (int *)calloc(left->numberPoints, sizeof(int));
    if(groups[groupIndex] == NULL)
      {
	printf("cannot allocate memory for groups! \n");
	return -1;
      }
    for(i=0;  i&lt;left->numberPoints; i++){
      groups[groupIndex][i] = left->points[i];
    }/* end for */
    groupIndex++;
    printf("groupIndex = %d \n", groupIndex);
    if(groupIndex >= MAXGROUPS)
      {
	printf("groupIndex is now bigger than MAXGROUPS \n");
	return -1;
      }

    /*following part is for output matlab file after first phase */
    /*
      printf("%Points in this group include: \n c%d = [", index_matlab);
      index_matlab++;
      for(i=0; i&lt;left->numberPoints; i++)
      {
      printf("%f %f \n ", points[left->points[i]].x, points[left->points[i]].y);
      }
      printf("];\n\n");
    */
  }/* end if...else... */
  
  if(right->numberPoints > threshold){
    r_left = (struct node *)calloc(1, sizeof(struct node));
    if(r_left == NULL)
      {
	printf("cannot allocate memory for left! \n");
	return -1;
      }
    r_right = (struct node *)calloc(1, sizeof(struct node));
    if(r_right == NULL)
      {
	printf("cannot allocate memory for right! \n");
	return -1;
      }

    if(partition(right, r_left, r_right)&lt;0)
      return -1;
  }else{
    groupLength[groupIndex] = right->numberPoints;
    groups[groupIndex] =  (int *)calloc(right->numberPoints, sizeof(int));
    if(groups[groupIndex] == NULL)
      {
	printf("cannot allocate memory for groups! \n");
	return -1;
      }
    for(i=0;  i&lt;right->numberPoints; i++){
      groups[groupIndex][i] = right->points[i];
    }/* end for */
    groupIndex++;
    printf("groupIndex = %d \n", groupIndex);
    if(groupIndex >= MAXGROUPS)
      {
	printf("groupIndex is now bigger than MAXGROUPS \n");
	return -1;
      }


    /*following part is for output matlab file after first phase */
    /*
      printf("%Points in this group include: \n c%d =[", index_matlab);
      index_matlab++;
      for(i=0; i&lt;right->numberPoints; i++)
      {
      printf("%f %f \n ", points[right->points[i]].x, points[right->points[i]].y);
      }
      printf("];\n\n");
    */
  }/* end if...else... */
  
  return 1;
}/* end partition */


/****************************************************************
 * Name    : computeSimilarity                                  *
 * Function: compute similarity between point i and point j     *
 * Input   :  int i -- index of point i                         *
 *            int j -- index of point j                         *
 * Output  :  double -- similarity in (DIAG-distance)/DIAG.     *
 ****************************************************************/
static double computeSimilarity(int i, int j) {
  float i_x, i_y, j_x, j_y;
  double simi;

  i_x = points[i].x;
  i_y = points[i].y;
  j_x = points[j].x;
  j_y = points[j].y;

  simi = (DIAG - sqrt((double)(i_x-j_x)*(i_x-j_x)+(double)(i_y-j_y)*(i_y-j_y)) )/DIAG;

  /*printf("simi is %f !!!", simi);*/

  return simi;
}/* end computeSimilarity */


/****************************************************************
 * Name    : establish_hyperGraph                               *
 * Function: establish hyperGraph based on the points           *
 *           information.                                       *
 * Input   :  void                                              *
 * Output  : int                                                *
 ****************************************************************/
static int establish_hyperGraph() {
  int i, j, k, l;
  double *similarity = NULL;
  double bestSimi;
  int indexBestSimi;
  int flag = 0;

  similarity = (double *)calloc(sizeof(double), N);
  if(similarity == NULL)
    return -1;

  /* find the k_nearest points for each of the N point */
  for (i=0; i&lt;N; i++){
    /* compute similarity of point i with each point j */
    for (j =0; j&lt;N; j++){
       similarity[j] = computeSimilarity(i,j);
    }/* end for j */

    /* find the K_nearest points around point i */
    for (k=0; k&lt;K_nearest; k++){

      bestSimi = 0.0;
      indexBestSimi = 0;

      for (j=0; j&lt;N; j++){
	if(j != i){
	  if (similarity[j]>bestSimi){
	    indexBestSimi = j;
	    bestSimi = similarity[j];
	  }/* end if */
	}/* end if */
      }/* end for j */
      
      /* add a new edge to one of the two points */
      if(i&lt;indexBestSimi){
	points[i].edges[points[i].length].pointNO = indexBestSimi;
	points[i].edges[points[i].length].similarity = bestSimi;
	points[i].length += 1;
	if(points[i].length > K5_nearest)
	  {
	    printf("length of the edges exceeds K5_nearest! bestSimi is %f \n", 
		   bestSimi);
	    return -1;
	  }
      }
      else {
	/* check whether this edge has already been added from the
	 * other point.
	 */
	flag = 0;
	for(l=0; l&lt;points[indexBestSimi].length; l++){
	  if(points[indexBestSimi].edges[l].pointNO == i){
	    flag = 1;
	    break;
	  }/* end if */
	}/* end for l */
	
	if(flag == 0){
	  points[indexBestSimi].edges[points[indexBestSimi].length].pointNO = i;
	  points[indexBestSimi].edges[points[indexBestSimi].length].similarity = bestSimi;
	  points[indexBestSimi].length += 1;
	  if(points[indexBestSimi].length > K5_nearest)
	  {
	    printf("length of the edges exceeds K5_nearest!bestSimi is %f \n", 
		   bestSimi);
	    return -1;
	  }
	}/* end if */
      }/* end else */

      similarity[indexBestSimi] = 0.0;

    }/* end for k */
    
  }/* end for i */

#ifdef Debug_establish_hyperGraph
  j =0;
  printf("hyper edge for point %d is: \n", j);
  for(i=0; i&lt;points[j].length; i++){
    printf("%d  %f \n", points[j].edges[i].pointNO,  points[j].edges[i].similarity);
  }/* end for */
  j = 36;
  printf("hyper edge for point %d is: \n", j);
  for(i=0; i&lt;points[j].length; i++){
    printf("%d  %f \n", points[j].edges[i].pointNO,  points[j].edges[i].similarity);
  }/* end for */
  j = 49;
  printf("hyper edge for point %d is: \n", j);
  for(i=0; i&lt;points[j].length; i++){
    printf("%d  %f \n", points[j].edges[i].pointNO,  points[j].edges[i].similarity);
  }/* end for */
#endif 

  free(similarity);
  similarity = NULL;
  return 1;

}/* end establish_hyperGraph */


/****************************************************************
 * Name    : initialize                                         *
 * Function: initialize data                                    *
 * Input   :  void                                              *
 * Output  : int                                                *
 ****************************************************************/
static int initialize() {
  int i;

  bestGoodness = 0.0;

  groupIndex = 0;

  index_matlab = 0;

  for(i=0; i&lt;MAXGROUPS; i++){
    groups[i]=NULL;
    groupLength[i]=0;
  }

  /* this is the threshold of size of node to stop partition */
  threshold = (int)(N*MINSIZE);
  if(threshold &lt; 1)
    {
      printf("threshold less than 1, error! \n");
      return -1;
    }

  root = (struct node *)calloc(1, sizeof(struct node));
  if(root == NULL)
    {
      printf("cannot allocate memory for root! \n");
      return -1;
    }

  root->numberPoints = N;
  root->points = (int *)calloc(N, sizeof(int));
  if(root->points == NULL)
    {
      printf("cannot allocate memory for root->points! \n");
      return -1;
    }
  /* for the root node, all the points belong to it */
  for(i=0; i&lt;N; i++)
    {
      (root->points)[i]=i;
    }

#ifdef Debug_initialize
  for(i=0; i&lt;N; i++)
    {
      printf("(root->points)[%d]=%d \n", i, (root->points)[i]);
    }
#endif

  /* initialize points */
  for(i=0; i&lt;N; i++) 
    {
      points[i].length = 0;
    }/* end for i */

  return 1;
}/* end initialize */



/****************************************************************
 * Name    : readData                                         *
 * Function: read in Data from the data file                    *
 * Input   :  void                                              *
 * Output  : void                                               *
 ****************************************************************/
/* 
 * May 23th, 2001. Weinan: readData() has been tested
 * being right.
 */
static int readData() {

  int i;

 /* open the data file */
  if ((fp = fopen(fileName, "r")) == NULL )
    {
      printf("cannot open input file correctly! \n");
      return -1;
    }

  /* Now start reading data */
  for (i=0; i&lt;N; i++)
    {
      fscanf(fp, "%f%f", &points[i].x, &points[i].y);
    }/* end for i */

  return 1;

}/*end readData */


/****************************************************************
 * Name    : parsingInput                                       *
 * Function: parsing input command line                         *
 * Input   : argc, argv                                         *
 * Output  : void                                               *
 ****************************************************************/
/* 
 * May 23th, 2001. Weinan: parsingInput() has been tested
 * being right.
 */
static int parsingInput(int argc, char *argv[]) {

  if(argc == 4)
    {
      sscanf(argv[1], "%d", &N);
      if(N>MAXSIZE)
	{
	  printf("The size of the data set is bigger than MAXSIZE! \n");
	  return -1;
	}

      if ((int)strlen(argv[2]) >= MAX_fileName)
	{
	  printf("the name of the file is too long. "
		 "It should be within %d characters \n", 
		 MAX_fileName);
	  return -1;
	}
      else
	{
	  sscanf(argv[2], "%s", fileName);
	}

      sscanf(argv[3], "%d", &stopClusterNO);
      if(stopClusterNO>N)
	{
	  printf("stopClusterNO is now bigger than N, impossible! \n");
	  return -1;
	}
    } /* end if */
  else
    {
      printf("argument error \n");
      printf(" Usage of the program should be of the form: \n");
      printf(" chameleon N fileName stopClusterNO \n");
      return -1;
    }/* end else */

#ifdef Debug_parsingInput
  printf("Input data is: \n ");
  printf("N = %d \n", N);
  printf("fileName = %s \n", fileName);
  printf("stopClusterNO = %d \n", stopClusterNO);
#endif

  return 1;

}/* end parsingInput */






#endif