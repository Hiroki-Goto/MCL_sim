#ifndef DRAW_H
#define DRAW_H

#include "pf.h"

class GlDraw{
private:
    double degToRad(double degree);
public:
    GlDraw();
    ~GlDraw();
    void drawMap(Map map);
    void drawRobotAndParticle(Robot *robot_t,ParticleSet *particle_set_t);
};

#endif
