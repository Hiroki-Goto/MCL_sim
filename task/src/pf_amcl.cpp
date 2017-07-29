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



double Amcl::gaussian(double sigma){
    double result;
    for(int i=0; i<SAMPLE; i++){
        result += setUniform(-fabs(sigma),fabs(sigma));
    }
    result /= SAMPLE;
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
 *   引数：制御(Ut),ロボットの姿勢(x_t-1),パーティクルの姿勢(s_t-1)
 *   戻り値:ロボットの姿勢(x'_t),パーティクルの姿勢(s'_t)
 *   確率ロボティクスp123のサンプリングを参照
 */
void Amcl::pfMotionUpdata(Control *control_t, Robot *robot_t1, ParticleSet *particle_set_t1){

    //引数では
    double robotHatX,robotHatY,robotHatTheta;
    double deltaTrance,deltaRotate1,deltaRotate2;
    double deltaHatRotate1,deltaHatTrance,deltaHatRotate2;
    double deltaRotate1Noise,deltaRotate2Noise;

    //control_t->trance += setUniform(-ODOM_TRANSE_SIGMA,ODOM_TRANSE_SIGMA);
    //control_t->rotate += setUniform(-ODOM_ROTATE_SIGMA,ODOM_ROTATE_SIGMA);

    //制御による推定値の計算
    robotHatTheta = robot_t1->theta + control_t->rotate;
    if(robotHatTheta >= 360){
        robotHatTheta -= 360;
    }else if(robotHatTheta < -360){
        robotHatTheta += 360;
    }
    robotHatX = robot_t1->x + (control_t->trance)*(cos(degToRad(robot_t1->theta)));
    robotHatY = robot_t1->y + (control_t->trance)*(sin(degToRad(robot_t1->theta)));

    //推定値からdelta_rotate1,delta_trance,delta_rotate1の計算
    //確率ロボティクスp124の2行目〜4行目

    deltaRotate1 = atan2(robotHatY-robot_t1->y, robotHatX-robot_t1->x );
    deltaTrance = hypot(robotHatX-robot_t1->x, robotHatY-robot_t1->y);
    deltaRotate2 = degToRad(robotHatTheta - robot_t1->theta)- deltaRotate1;

    for(int i=0; i<particle_set_t1->size(); i++){

        deltaHatRotate1 = deltaRotate1 + gaussian(ALPHA_1*deltaRotate1*deltaRotate1 +
                                                  ALPHA_2*deltaTrance*deltaTrance);
        deltaHatTrance = deltaTrance + gaussian(ALPHA_3*deltaTrance*deltaTrance +
                                                ALPHA_4*deltaRotate1*deltaRotate1 +
                                                ALPHA_4*deltaRotate2*deltaRotate2);
        deltaHatRotate2= deltaRotate2 + gaussian(ALPHA_1*deltaRotate2*deltaRotate2 +
                                                  ALPHA_2*deltaTrance*deltaTrance);
        if(control_t->trance < 0){
            deltaHatTrance *= -1;
        }
        particle_set_t1->at(i).x += (deltaHatTrance)*(cos(degToRad(particle_set_t1->at(i).theta)));
        particle_set_t1->at(i).y += (deltaHatTrance)*(sin(degToRad(particle_set_t1->at(i).theta)));
        particle_set_t1->at(i).theta += radToDeg(deltaRotate1+deltaRotate2);
    }
    robot_t1->x = robotHatX;
    robot_t1->y = robotHatY;
    robot_t1->theta = robotHatTheta;
}

/*
 *   計測更新（重み付け）
 *   引数：,ロボットの姿勢と計測(x'_t,Zt),パーティクルの姿勢,計測予測(s'_t,Z't)
 *   戻り値:重み付けされた各パーティクル(s_t)
 *   確率ロボティクスp-pを参照
 */
void Amcl::pfSensorUpdata(Robot *robot_t, ParticleSet *particle_set_t){
    double weight;
    double total;
    for(int i=0;i<particle_set_t->size(); i++){
        for(int j=0; j<SENSOR_NUM;j++){
            weight += pfMeasurementModel(robot_t->robotSensorData.at(j).distance, SENSOE_ERROR,
                                            particle_set_t->at(i).particleSensorData.at(j).distance);
        }
        particle_set_t->at(i).weight = weight/SENSOR_NUM;
        total += weight/SENSOR_NUM;
        weight = 0;
    }
    robot_t->totalWeight = total;
}

/*
 *   リサンプリング
 *   引数：重み付けされたパーティクル
 *   戻り値:リサンプリング後のパーティクル(s_t)
 *   確率ロボティクスp-pを参照
 */
void Amcl::pfResampling(Robot *robot_t, ParticleSet *particle_set_t){
    double totalWeight;
    int particleNum=0;
    int totalParticleNum=0;
    bool particleFullTriger=false;
    Particle new_particle;
    ParticleSet new_sample;
    SensorData sensorData;
    new_sample.clear();
    for(int i=0; i<particle_set_t->size(); i++){
         particleNum = particle_set_t->at(i).weight/robot_t->totalWeight* PARTICLE_NUM;
         for(int j=0; j<particleNum; j++){
             new_particle.x = particle_set_t->at(i).x + setUniform(-RESAMPLE_POSITION_SIGMA,RESAMPLE_POSITION_SIGMA);
             new_particle.y = particle_set_t->at(i).y + setUniform(-RESAMPLE_POSITION_SIGMA,RESAMPLE_POSITION_SIGMA);
             new_particle.theta = particle_set_t->at(i).theta + setUniform(-RESAMPLE_ROTATE_SIGMA,RESAMPLE_ROTATE_SIGMA);
             new_particle.weight = particle_set_t->at(i).weight/robot_t->totalWeight;
             if(SENSOR_NUM == 1){
                 sensorData.angle = new_particle.theta;
                 new_particle.particleSensorData.push_back(sensorData);
             }
             else{
                 double angle = 0;
                 for(int i=0; i<SENSOR_NUM; i++){
                     sensorData.angle = angle;
                     angle += (180/(SENSOR_NUM-1));
                     new_particle.particleSensorData.push_back(sensorData);
                 }
             }
             totalParticleNum++;
             new_sample.push_back(new_particle);
             if(totalParticleNum >= PARTICLE_NUM){
                 particleFullTriger = true;
                break;
            }
         }
         if(particleFullTriger){
             particleFullTriger = false;
             break;
        }
    }
    if(totalParticleNum < PARTICLE_NUM){
        for(int i= totalParticleNum; i<PARTICLE_NUM; i++){
            new_particle.x = robot_t->x + setUniform(-RESAMPLE_POSITION_SIGMA,RESAMPLE_POSITION_SIGMA);
            new_particle.y = robot_t->y + setUniform(-RESAMPLE_POSITION_SIGMA,RESAMPLE_POSITION_SIGMA);
            new_particle.theta = robot_t->theta + setUniform(-RESAMPLE_ROTATE_SIGMA,RESAMPLE_ROTATE_SIGMA);
            new_particle.weight = particle_set_t->at(i).weight/robot_t->totalWeight;
            if(SENSOR_NUM == 1){
                sensorData.angle = new_particle.theta;
                new_particle.particleSensorData.push_back(sensorData);
            }
            else{
                double angle = 0;
                for(int i=0; i<SENSOR_NUM; i++){
                    sensorData.angle = angle;
                    angle += (180/(SENSOR_NUM-1));
                    new_particle.particleSensorData.push_back(sensorData);
                }
            }
            new_sample.push_back(new_particle);
            totalParticleNum++;
        }
    }
    particle_set_t->clear();
    for(int i=0; i<PARTICLE_NUM; i++){
        new_particle.x = new_sample.at(i).x;
        new_particle.y = new_sample.at(i).y;
        new_particle.theta = new_sample.at(i).theta;
        new_particle.weight = new_sample.at(i).weight;
        if(SENSOR_NUM == 1){
            sensorData.angle = new_particle.theta;
            new_particle.particleSensorData.push_back(sensorData);
        }
        else{
            double angle = 0;
            for(int i=0; i<SENSOR_NUM; i++){
                sensorData.angle = angle;
                angle += (180/(SENSOR_NUM-1));
                new_particle.particleSensorData.push_back(sensorData);
            }
        }
        particle_set_t->push_back(new_particle);
    }
    new_sample.clear();
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

    for(int i=0; i<particle_set_t_bar->size();i++){
        for(int j=0; j<SENSOR_NUM; j++){
            for(double range=0; range<MAX_RANGE; range+=0.5){
                double sensorAngleT = particle_set_t_bar->at(i).theta -(INIT_THETA -particle_set_t_bar->at(i).particleSensorData.at(j).angle);
                int sensorX = particle_set_t_bar->at(i).x + range*cos(degToRad(sensorAngleT));
                int sensorY = particle_set_t_bar->at(i).y + range*sin(degToRad(sensorAngleT));

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
