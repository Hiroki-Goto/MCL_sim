#ifndef PF_AMCL_H
#define PF_AMCL_H

#include "pf.h"

class Amcl{
private:
    double setUniform(double min_number, double max_number);
    double degToRad(double degree);
    double radToDeg(double radian);
    double pfMeasurementModel(double average ,double sigma, double input);
    double gaussian(double sigma);

public:
    Amcl();
    ~Amcl();
    void pfInit(Robot *robot_t,ParticleSet *particle_set_t);
    void pfMotionUpdata(Control *control_t, Robot *robot_t1, ParticleSet *particle_set_t1);
    void pfSensorUpdata(Robot *robot_t, ParticleSet *particle_set_t);
    void pfResampling(Robot *robot_t, ParticleSet *particle_set_t);

    void pfGetSensorData(Map map,Robot *robot_t_ber,ParticleSet *particle_set_t_bar);
};

#endif
