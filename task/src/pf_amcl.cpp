#include "../include/pf.h"
#include "../include/pf_amcl.h"
#include "../include/param.h"

#include <iostream>
#include <stdlib.h>
#include <math.h>


Amcl::Amcl(){
    std::cout << "Initialized" << std::endl;
    //時間の初期化を行う？
}

Amcl::~Amcl(){
    std::cout << "Finshed" << std::endl;
}

double Amcl::setUniform(double min_number, double max_number){
    double number;
    number = (max_number-min_number)*rand()/RAND_MAX-(max_number-min_number)/2;
    return number;
}


double Amcl::gause(double average ,double sigma,double input){
    double result;
    result = 1 / (sqrt(2*PI)*sigma) * exp(-(input-average)*(input-average)/(2*sigma*sigma));
    return result;
}

double Amcl::degToRad(double degree){
    double radian;
    radian = degree / 180 * PI;
    return radian;
}

void Amcl::pfInit(Robot *robot_t,ParticleSet *particle_set_t){
    robot_t->x = INIT_X;
    robot_t->y = INIT_Y;
    robot_t->theta = INIT_THETA;
    SensorData sensorData;
    if(SENSOR_NUM == 1){
        sensorData.angle = INIT_THETA;
        robot_t->robotSensorData.push_back(sensorData);
    }
    else{
        double angle = 0;
        for(int i=0; i<SENSOR_NUM; i++){
            sensorData.angle = angle;
            angle += (180/(SENSOR_NUM-1));
            robot_t->robotSensorData.push_back(sensorData);
        }
    }
    Particle particle;
    particle_set_t->clear();
    for(int i=0; i<PARTICLE_NUM; i++){
        particle.x = INIT_X + setUniform(-INIT_POSITION_SIGMA,INIT_POSITION_SIGMA);
        particle.y = INIT_Y + setUniform(-INIT_POSITION_SIGMA,INIT_POSITION_SIGMA);
        particle.theta = INIT_THETA + (-INIT_ROTATE_SIGMA,INIT_ROTATE_SIGMA);
        if(SENSOR_NUM == 1){
            sensorData.angle = particle.theta;
            particle.particleSensorData.push_back(sensorData);
        }
        else{
            double angle = 0;
            for(int i=0; i<SENSOR_NUM; i++){
                sensorData.angle = angle;
                angle += (180/(SENSOR_NUM-1));
                particle.particleSensorData.push_back(sensorData);
            }
        }
        particle_set_t->push_back(particle);
    }
}

/*
 *   サンプリング（動作予測）
 *   引数：制御(Ut),ロボットの姿勢(x_t-1),パーティクルの姿勢(s_t-1)
 *   戻り値:ロボットの姿勢(x'_t),パーティクルの姿勢(s'_t)
 *   確率ロボティクスp-pを参照
 */
void Amcl::pfMotionUpdata(Control *control_t, Robot *robot_t1, ParticleSet *particle_set_t1){

    //引数では
    double deltaX = control_t->trance;
    double deltaY = control_t->trance;
    double deltaTheta = control_t->rotate;


    robot_t1->theta += deltaTheta;
    robot_t1->x += (deltaX)*(cos(degToRad(robot_t1->theta)));
    robot_t1->y += (deltaY)*(sin(degToRad(robot_t1->theta)));

}

/*
 *   計測更新（重み付け）
 *   引数：,ロボットの姿勢と計測(x'_t,Zt),パーティクルの姿勢,計測予測(s'_t,Z't)
 *   戻り値:重み付けされた各パーティクル(s_t)
 *   確率ロボティクスp-pを参照
 */

/*
 *   リサンプリング
 *   引数：重み付けされたパーティクル
 *   戻り値:リサンプリング後のパーティクル(s_t)
 *   確率ロボティクスp-pを参照
 */

void Amcl::pfGetSensorData(Map map,Robot *robot_t_ber,ParticleSet *particle_set_t_bar){

    for(int i=0; i<SENSOR_NUM; i++){
        for(double range=0; range<MAX_RANGE; range+=0.5){
            double sensorAngleT = robot_t_ber->theta - (INIT_THETA - robot_t_ber->robotSensorData.at(i).angle );
            int sensorX = robot_t_ber->x +range*cos(degToRad(sensorAngleT));
            int sensorY = robot_t_ber->y +range*sin(degToRad(sensorAngleT));

            if(map[-sensorY/MAP_GLID][sensorX/MAP_GLID] == 1){
                //std::cout << range << std::endl;
                robot_t_ber->robotSensorData.at(i).distance = range;
                break;
            }
            else{
                robot_t_ber->robotSensorData.at(i).distance = 0;
            }

        }
    }

}
