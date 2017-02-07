
#ifndef MY_GL_H_
#define MY_GL_H_

#include "pf.h"

class My_gl{
private:
  double deg_to_rad(double deg);
public:
  My_gl();
  ~My_gl();
  bool map_read();
  void motion(control u_t);
  void draw_map();
  void draw_robot();
};

#endif
