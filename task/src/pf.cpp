#include "../include/pf.h"
#include "../include/param.h"
#include <stdlib.h>
#include <math.h>


Amcl::Amcl(){
//時間の初期化
}

Amcl::~Amcl(){

}

//一様乱数を発生させる
double Amcl::Uniform(){
  return ((double)rand()+1.0)/((double)RAND_MAX+2.0);
}

//min〜maxまでの範囲で乱数を発生させる
double Amcl::Select_Uniform(double min, double max){
  double result;
  result = (max-min)*rand()/RAND_MAX-(max-min)/2;
  return result;
}

double Amcl::gause(double ave,double deviation,double x){
  double result;
  result = 1 / (sqrt(2*PI)*deviation) * exp(-(x-ave)*(x-ave)/(2*deviation*deviation));
  return result;
}

double Amcl::deg_to_rad(double deg){
  return deg / 180.0 * PI;
}

//各種の初期化を行う
void Amcl::pf_init(robot_position_t_ *robot_position_t, pf_t_ *pf_t){

  //ロボットの位置の初期化
  robot_position_t->x = 34;
  robot_position_t->y = -210;
  robot_position_t->theta = 90;

  //パーティクルの初期化
  for(int i=0; i<PARTICLE_NUM; i++ ){
    pf_t->x = 34 + Select_Uniform(-INIT_POSITION_SIGMA, INIT_POSITION_SIGMA);
    pf_t->y = -210 + Select_Uniform(-INIT_POSITION_SIGMA, INIT_POSITION_SIGMA);
    pf_t->theta = 90 + Select_Uniform(-INIT_ROTATE_SIGMA, INIT_ROTATE_SIGMA);
  }
}

//ロボットの位置から各種センサデータの計測を行う
//センサの位置はロボットの中心点としている

//各パーティクルの位置から計測予測を行う


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//
//             チームでやる場合は                //
//            ここからのプログラムは               //
//       別々のファイルにした方がいいと思います　　　　 //
//              (競合防止のため)               //
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//

//オドメトリ動作モデルに従いパーティクルの更新を行う
//今回はロボットとパーティクルの更新を一気にしている
//詳しくは確率ロボティクスp120-p126を参照

//計測モデルに従い，各パーティクルの重み付けを行う
//今回計測モデルは正規分布にしているが，AMCLでの尤度場モデルにしてほしい
//詳しくは確率ロボティクスp139-p146を参照

//重み付けを行ったパーティクルを元にリサンプリングを行う
//このプログラムでは（面倒なので）KLDサンプルは行っていない
//リサンプリング方式もかなりテキトーなので各自で調べて実装してくだい
