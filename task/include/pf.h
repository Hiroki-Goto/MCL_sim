
#ifndef PF_H
#define PF_H
#include <vector>

#define PI 3.14159265

struct SensorData{
    double distance;
    double angle;
};

typedef struct Robot{
    double x, y, theta;
    std::vector<SensorData> robotSensorData;
}Robot;

struct Particle{
    double x, y, theta;
    double weight;
    std::vector<SensorData> particleSensorData;
};
typedef std::vector<Particle> ParticleSet;

typedef struct Control{
    double trance;
    double rotate;
}Control;

typedef std::vector< std::vector<int> > Map;

#endif
