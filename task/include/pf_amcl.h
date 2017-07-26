#ifndef PF_AMCL_H
#define PF_AMCL_H

#include "pf.h"

class Amcl{
private:
    double setUniform(double min_number, double max_number);
    double gause(double average ,double sigma,double input);
public:
    Amcl();
    ~Amcl();
    void pfInit(Robot *robot_t,ParticleSet *particle_set_t);
    //void pfMotionUpdata();
    //void pfSensorUpdata();
    //void pfResampling();
};

#endif
