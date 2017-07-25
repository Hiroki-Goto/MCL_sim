
#ifndef PF_H_
#define PF_H_


#include <iostream>
#include<vector>

typedef struct Robot_position_{
  double x;
  double y;
  double theta;
}robot_position;

typedef struct True_robot_position_{
  double x;
  double y;
  double theta;
}true_robot_position;

typedef struct Control{
	double translation;
	double rotate;
}control;

struct Sensor{
  double x;
  double y;
  double theta;
  double diss;
};

struct Pf_Sensor{
  double x;
  double y;
  double theta;
  double diss;
};

class Particle_filter{
private:
  double gause(double ave,double deviation,double x);
  double Uniform();
  double select_Uniform(double min, double max);
  double total_weight;
public:
  void draw_particle_pf();
  Particle_filter();
  ~Particle_filter();
  double deg_to_rad(double deg);
  void motion_update(control u_t);
  void sensor_update();
  void pf_sensor_measure();
};

#endif
