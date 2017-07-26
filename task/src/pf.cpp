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

/*
 *   ロボットの位置とパーティクルの初期化を行う
 */
void Amcl::pfInit(Robot *robot_t,ParticleSet *particle_set_t){
    robot_t->x = INIT_X;
    robot_t->y = INIT_Y;
    robot_t->theta = INIT_THETA;

    Particle particle;
    particle_set_t->clear();
    for(int i=0; i<PARTICLE_NUM; i++){
        particle.x = INIT_X + setUniform(-INIT_POSITION_SIGMA,INIT_POSITION_SIGMA);
        particle.y = INIT_Y + setUniform(-INIT_POSITION_SIGMA,INIT_POSITION_SIGMA);
        particle.theta = INIT_THETA + (-INIT_ROTATE_SIGMA,INIT_ROTATE_SIGMA);
        particle_set_t->push_back(particle);
    }
}

/*
 *   サンプリング（動作予測）
 *   引数：制御(Ut),ロボットの姿勢(x_t-1),パーティクルの姿勢(s_t-1)
 *   戻り値:ロボットの姿勢(x'_t),パーティクルの姿勢(s'_t)
 *   確率ロボティクスp-pを参照
 */

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
