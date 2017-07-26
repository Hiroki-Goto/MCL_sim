
#ifndef PF_H
#define PF_H
#include <vector>

#define PI 3.14159265

typedef struct{
    double x, y, theta;
    std::vector<double> robotSensorData;
}Robot;

struct Particle{
    double x, y, theta;
    double weight;
    std::vector<double> particleSensorData;
};
typedef std::vector<Particle> ParticleSet;

typedef struct Control{
    double trance;
    double rotate;
}Control;

typedef std::vector< std::vector<int> > Map;


#endif
