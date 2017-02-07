
#ifndef PF_H_
#define PF_H_


#include <iostream>
#include<vector>

typedef struct PF_t_{
  double x;
  double y;
  double theta;
  double weight;
}pf_t_;

typedef struct Robot_position_{
  double x;
  double y;
  double theta;
}robot_position;

typedef struct Control{
	double translation;
	double rotate;
}control;


#endif
