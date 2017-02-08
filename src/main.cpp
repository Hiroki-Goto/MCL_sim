#include <GL/glut.h>
#include <iostream>
#include <string>
#include <fstream>
#include <math.h>
#include "../include/param.h"
#include "../include/my_gl.h"
#include "../include/pf.h"

control con;
My_gl my_gl;

void display(){
	glClear(GL_COLOR_BUFFER_BIT);

	my_gl.draw_map();
	my_gl.motion(con);
	my_gl.measure();
	my_gl.draw_sensor();
	my_gl.draw_robot();
	glFlush();
	con.translation = con.rotate = 0;
}

void resize(int w, int h){
	glViewport(0, 0, w, h);
	glLoadIdentity();
	//画面サイズの変更
	glOrtho(-20,300,-250,30, -1.0, 1.0);
}


void keyboard(unsigned char key, int x, int y)
{

  switch (key) {
  case 'q':
  case 'Q':
  case '\033':  /* '\033' は ESC の ASCII コード */

    exit(0);
	break;
	case 'r':			//初期化
	break;
	case 'w':
		con.translation = 2;
		glutPostRedisplay();

		std::cout << "up" << std::endl;
	break;
	case 'a':
		con.rotate = 2;
		glutPostRedisplay();
		std::cout << "left" << std::endl;
	break;
	case 's':
		con.translation = -2;
		glutPostRedisplay();
		std::cout << "down" << std::endl;
	break;
	case 'd':
		con.rotate = -2;
		glutPostRedisplay();
		std::cout << "right" << std::endl;
	break;
  default:
    break;
  }
}

int main(int argc, char **argv){

	if(my_gl.map_read()){
		glutInit(&argc, argv);
		glutInitWindowPosition(100, 100);
		glutInitWindowSize(400, 400);

		glutCreateWindow("test");

		glutInitDisplayMode(GLUT_SINGLE | GLUT_RGBA);
		glClearColor(1.0, 1.0, 1.0, 1.0);

		glutDisplayFunc(display);
		glutReshapeFunc(resize);
		glutKeyboardFunc(keyboard);
		glutMainLoop();
		return 0;
	}else{
			return -1;
	}
}
