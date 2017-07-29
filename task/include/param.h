
//使用パーティクル
#define PARTICLE_NUM 50

//初期化する上でのパラメータ
#define INIT_X 34
#define INIT_Y -210
#define INIT_THETA 90
#define INIT_POSITION_SIGMA 20.0
#define INIT_ROTATE_SIGMA 10.0

//マップ関連
#define MAP_LINE 48
#define MAP_COLUMN 38
#define MAP_GLID 5

//描画関連
#define ROBOT_RADIUS 10
#define PARTICLE_RADIUS 2
#define SENSOR_RADIUS 1.5

//センサ関連
#define ODOM_TRANSE_SIGMA 0.1       //実際のロボットのオドメトリ誤差（直進）
#define ODOM_ROTATE_SIGMA 0.3       //実際のロボットのオドメトリ誤差（回転）
#define SENSOR_NUM 180
#define MAX_RANGE 80

//動作モデル関連
#define ALPHA_1 1
#define ALPHA_2 2
#define ALPHA_3 3
#define ALPHA_4 4
