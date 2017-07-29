#ifndef PF_AMCL_H
#define PF_AMCL_H

#include "pf.h"

class Amcl{
private:
    double setUniform(double min_number, double max_number);
    double gause(double average ,double sigma,double input);
    double degToRad(double degree);

public:
    Amcl();
    ~Amcl();
    void pfInit(Robot *robot_t,ParticleSet *particle_set_t);
    void pfMotionUpdata(Control *control_t, Robot *robot_t1, ParticleSet *particle_set_t1);
    //void pfSensorUpdata();
    //void pfResampling();

    void pfGetSensorData(Map map,Robot *robot_t_ber,ParticleSet *particle_set_t_bar);
};

#endif
