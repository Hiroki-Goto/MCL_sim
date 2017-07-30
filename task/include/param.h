
//使用パーティクル
#define PARTICLE_NUM 100

//初期化する上でのパラメータ
#define INIT_X 110
#define INIT_Y -420
#define INIT_THETA 90
#define INIT_POSITION_SIGMA 50.0    //一方向の分散（スタート地点のコーナーにばら撒くようにしている）
#define INIT_ROTATE_SIGMA 10.0

//マップ関連
//地図の大きさは25分の１サイズになっている
#define MAP_LINE 98
#define MAP_COLUMN 85
#define MAP_GLID 5                  //地図1セルあたりの解像度[cm]

//描画関連
#define ROBOT_RADIUS 10
#define PARTICLE_RADIUS 2
#define SCAN_RADIUS 3

//センサ関連
#define ODOM_TRANSE_SIGMA 0.3       //実際のロボットのオドメトリ誤差（直進）
#define ODOM_ROTATE_SIGMA 0.3       //実際のロボットのオドメトリ誤差（回転）
#define SENSOR_NUM 180              //センサのビームの数
#define MAX_RANGE 90                //センサのビームのレンジ
#define SENSOE_ERROR 3              //この値＊MPA_GRIDが実際の計測誤差となる



//MCLでの行うパラメータ関係
#define ALPHA_1 0.3
#define ALPHA_2 0.3
#define ALPHA_3 0.4
#define ALPHA_4 0.2
#define SAMPLE 12                   //中心極限定理で使用するサンプル数
#define CONTROL_TRANCE 2
#define CONTROL_ROTATE 2

#define RESAMPLE_POSITION_SIGMA 8.0
#define RESAMPLE_ROTATE_SIGMA 5.0
