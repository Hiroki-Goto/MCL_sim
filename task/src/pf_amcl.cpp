#include "../include/pf.h"
#include "../include/pf_amcl.h"
#include "../include/param.h"

#include <iostream>
#include <stdlib.h>
#include <math.h>


Amcl::Amcl(){
    std::cout << "Initialized" << std::endl;
    srand48(time(NULL));
}

Amcl::~Amcl(){
    std::cout << "Finshed" << std::endl;
}

double Amcl::setUniform(double min_number, double max_number){
    double number;
    number = (max_number-min_number)*rand()/RAND_MAX-(max_number-min_number)/2;
    return number;
}

double Amcl::degToRad(double degree){
    double radian;
    radian = degree / 180 * PI;
    return radian;
}

double Amcl::radToDeg(double radian){
    double degree;
    degree = 180 /PI * radian;
    return degree;
}

double Amcl::pfMeasurementModel(double average ,double sigma,double input){
    double result;
    result = 1 / (sqrt(2*PI)*sigma) * exp(-(input-average)*(input-average)/(2*sigma*sigma));
    return result;
}


/*
 * 動作モデルにあるsample()の計算
 * 引数の値からランダムに1つ抽出するにすること
 */
double Amcl::sample(double sigma){
    double result;
    return result;
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
        particle.theta = INIT_THETA + setUniform(-INIT_ROTATE_SIGMA,INIT_ROTATE_SIGMA);
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
 */
void Amcl::pfMotionUpdata(Control *control_t, Robot *robot_t1, ParticleSet *particle_set_t1){

    double robotHatX,robotHatY,robotHatTheta;

    //制御による推定値の計算
    robotHatTheta = robot_t1->theta + control_t->rotate;
    //[0,π]の範囲にするため
    if(robotHatTheta >= 360){
        robotHatTheta -= 360;
    }else if(robotHatTheta < -360){
        robotHatTheta += 360;
    }
    robotHatX = robot_t1->x + (control_t->trance)*(cos(degToRad(robot_t1->theta)));
    robotHatY = robot_t1->y + (control_t->trance)*(sin(degToRad(robot_t1->theta)));

    //推定値からdelta_rotate1,delta_trance,delta_rotate1の計算
    //確率ロボティクスp124の2行目〜4行目


    //ロボットの位置を代入
    robot_t1->x = robotHatX;
    robot_t1->y = robotHatY;
    robot_t1->theta = robotHatTheta;

}

/*
 * 計測更新（重み付け）
 */
void Amcl::pfSensorUpdata(Robot *robot_t, ParticleSet *particle_set_t){

}

/*
 *  リサンプリング
 */
void Amcl::pfResampling(Robot *robot_t, ParticleSet *particle_set_t){
}

/*
 *センサデータの取得を行う関数
 *param.hのセンサのビーム数などを180にすると少し重くなってしまう
 *ビームの距離が荒くなっても良い場合はrange+=0.5の値を大きくしても良い
 */
void Amcl::pfGetSensorData(Map map,Robot *robot_t_ber,ParticleSet *particle_set_t_bar){

    for(int i=0; i<SENSOR_NUM; i++){
        for(double range=0; range<MAX_RANGE; range+=0.5){
            double sensorAngleT = robot_t_ber->theta - (INIT_THETA - robot_t_ber->robotSensorData.at(i).angle );
            int sensorX = robot_t_ber->x +range*cos(degToRad(sensorAngleT));
            int sensorY = robot_t_ber->y +range*sin(degToRad(sensorAngleT));

            if(map[-sensorY/MAP_GLID][sensorX/MAP_GLID] == 1 || map[-sensorY/MAP_GLID][sensorX/MAP_GLID] == 2){
                //std::cout << range << std::endl;
                robot_t_ber->robotSensorData.at(i).distance = range + setUniform(-SENSOE_ERROR,SENSOE_ERROR);
                break;
            }
            else{
                robot_t_ber->robotSensorData.at(i).distance = 0;
            }
        }
    }

    for(int i=0; i<particle_set_t_bar->size();i++){
        for(int j=0; j<SENSOR_NUM; j++){
            for(double range=0; range<MAX_RANGE; range+=0.5){
                double sensorAngleT = particle_set_t_bar->at(i).theta -(INIT_THETA -particle_set_t_bar->at(i).particleSensorData.at(j).angle);
                int sensorX = particle_set_t_bar->at(i).x + range*cos(degToRad(sensorAngleT));
                int sensorY = particle_set_t_bar->at(i).y + range*sin(degToRad(sensorAngleT));

                //計測予測なので，ボールの存在は考慮しない
                if(map[-sensorY/MAP_GLID][sensorX/MAP_GLID] == 1){
                    //std::cout << range << std::endl;
                    particle_set_t_bar->at(i).particleSensorData.at(j).distance = range;
                    break;
                }
                else{
                    particle_set_t_bar->at(i).particleSensorData.at(j).distance = 0;
                }
            }
        }
    }


}
