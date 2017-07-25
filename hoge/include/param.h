#ifndef PARAM_H_
#define PARAM_H_

#define PI 3.1415926
#define THT_MAX 360

//ロボット関係
#define ROBO_RADIUS 5
#define SENSOR_RADIUS 2
#define MAX_RANGE 80
#define SENSOR_NUM 1


//地図関係
#define GLID 5
#define COLUMN 38
#define LINE 48

//パーティクルフィルタ関係
#define MAX_PARTICLE 50
#define PARTICLE_RADIUS 2
#define PARTICLE_ELEMENT 4
#define a1 5
#define a2 5
#define a3 5
#define a4 5
#define Z_ERROR 30


#define X_ERROR 2
#define Y_ERROR 2
#define THETA_ERROR 2

extern double robot_sensor_result;


#endif
