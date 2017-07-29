# pro_robo_task

知能ロボットコンテストにおけるパーティクルフィルタを用いた自己位置推定のシミュレーション

コンパイル
g++ main.cpp src/pf_amcl.cpp  src/draw.cpp -lglut -lGLU -lGL -lm -O2
最適化オプションがないと重くなります

実行
./a.out


実際に動いていいる様子
https://youtu.be/2N2hHU9i01w
