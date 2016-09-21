#ifndef _Point_Cloud_Register_H_Tony_Sep_15_2016_
#define _Point_Cloud_Register_H_Tony_Sep_15_2016_

enum Trans_Type
{
	Non_Trans = 0,
	Rigid2D = 1,
	Rigid3D = 2
};


typedef struct  
{
	Trans_Type type;
	double coeffs[32];

}Transform_params;




#endif