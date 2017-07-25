#include <iostream>
#include <math.h>
#include <vector>

#include <GL/glut.h>


#include "include/pf.h"

Amcl amcl;

robot_position_t_ *robot_position_t;
pf_t_ *pf_t;


void display(){
  glClear(GL_COLOR_BUFFER_BIT);
  //地図の描画
  //動作モデル
  //地図からそれぞれ更新
  //計測モデル
  //リサンプリング
  //ロボット，パーティクルの描画

  glFlush();
}

void resize(int w, int h){
	glViewport(0, 0, w, h);
	glLoadIdentity();
	glOrtho(-20,300,-250,30, -1.0, 1.0);   //画面サイズの変更
}


void keyboard(unsigned char key, int x, int y){
  switch (key) {
    case 'q':
    case 'Q':
    case '\033':  // '\033' は ESC の ASCII コード
      exit(0);
	  break;

    case 'w':   //前進
      glutPostRedisplay();
      std::cout << "up" << std::endl;
    break;

    case 'a':   //左回転
      glutPostRedisplay();
      std::cout << "left" << std::endl;
    break;

    case 's':   //後退
      glutPostRedisplay();
      std::cout << "back" << std::endl;
    break;

    case 'd':   //右回転
      glutPostRedisplay();
      std::cout << "right" << std::endl;
    break;
  }
}


int main(int argc, char **argv){
  glutInit(&argc, argv);
  glutInitWindowPosition(100, 100);
  glutInitWindowSize(400, 400);
  glutCreateWindow("Amcl_task");
  glutInitDisplayMode(GLUT_SINGLE | GLUT_RGBA);
  glClearColor(1.0, 1.0, 1.0, 1.0);
  glutDisplayFunc(display);
  glutReshapeFunc(resize);
  glutKeyboardFunc(keyboard);
  glutMainLoop();

  return 0;
}
