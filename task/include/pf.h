
#ifndef PF_H_
#define PF_H_
#include <vector>

#define PI 3.14159265

struct _pf_t_{
  double x,y,theta;
  double weight;
  std::vector<double> pf_senser_data;
};

typedef _pf_t_ pf_t_;

typedef struct _robot_position_t_{
  double x,y,theta;
  std::vector<double> robot_sensor_data;
}robot_position_t_;


class Amcl{
private:
  double Uniform();
  double Select_Uniform(double min, double max);
  double gause(double ave,double deviation,double x);
  double deg_to_rad(double deg);

public:
  Amcl();
  ~Amcl();
  void pf_init(robot_position_t_ *robot_position_t, pf_t_ *pf_t);
  //void pf_motion_updata(せいぎょ,robot_position_t_ *robot_position_t,pf_t_ *pf_t);
  //void pf_sensor_updata(robot_position_t_ *robot_position_t, pf_t_ *pf_t);
  //void pf_resampling(pf_t_ *pf_t);
  //void pf_get_laser_data(地図,robot_position_t_ *robot_position_t,pf_t_ *pf_t);
  
};

#endif
