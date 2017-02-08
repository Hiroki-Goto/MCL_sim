
#ifndef _MY_GL_H_
#define _MY_GL_H_

#include "pf.h"

class My_gl:public Particle_filter{
private:

public:
  My_gl();
  ~My_gl();
  bool map_read();

  void draw_map();
  void motion(control u_t);
  void measure();
  void draw_robot();
  void draw_sensor();
  void draw_particle();
};

#endif
