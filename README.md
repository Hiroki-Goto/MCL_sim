# pro_robo_task

## Overview
確率ロボティクスの勉強会で使用する自己位置推定のプログラム  

## Description
知能ロボットコンテストで使用するフィールドを使用してMCLのシミュレーションを行う
また各自で動作モデル，計測モデル，リサンプリングを変更することでそれぞれの役割を確認する  

## Demo  
動いている時の様子  
[解答例](https://youtu.be/t_k74wJt0zU)  



## Usage
- openGLを用いているため，インストールしていない人は以下のコマンドでインストールする  
$ sudo apt-get install freeglut3-dev libglew1.5-dev

- コンパイル方法  
最適化オプションがないと重くなるので，必ず入れること    
$ g++ main.cpp src/pf_amcl.cpp  src/draw.cpp -lglut -lGLU -lGL -lm -O2  

- 実行  
./a.out

- 動かし方
    - 前進    w   
    - 後退    s  
    - 右回転   d  
    - 左回転   a  
    - 初期化   r  
