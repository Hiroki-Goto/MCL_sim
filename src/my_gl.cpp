#include <GL/glut.h>
#include <iostream>
#include <string>
#include <fstream>
#include <math.h>

#include "../include/param.h"
#include "../include/my_gl.h"

robot_position *x_t;
int Map[LINE][COLUMN];

My_gl::My_gl(){
	x_t = new robot_position;
}

My_gl::~My_gl(){
	delete x_t;
}
double My_gl::deg_to_rad(double deg){
	return deg / 180.0 * PI;
}
bool My_gl::map_read(){
	FILE *map_file;
	int num;
	char kaigyou;
	if((map_file =fopen("../map_wall.txt","r")) == NULL){
		std::cout << "Doesn't exist map file" << std::endl;
		return false;
	}else{
		int i=0,j=0;
		while (fscanf(map_file, "%d", &num) != EOF){
			if(j==(COLUMN-1)){
				Map[i][j] = num;
				j=0;
				i++;
    	}else{
				j++;
				Map[i][j] = num;
			}
		}
		return true;
	}
	fclose(map_file);
}

void My_gl::draw_map(){
	//Draw map
	for(int i=0;i<LINE;i++){
		for(int j=0;j<COLUMN;j++){
			if(Map[i][j] == 1){
				glBegin(GL_POLYGON);
				glColor3d(1.0, 0.0, 0.0);
				glVertex2d((i)*GLID, (j)*GLID);
				glVertex2d((i)*GLID, (j+1)*GLID);
				glVertex2d((i+1)*GLID, (j+1)*GLID);
				glVertex2d((i+1)*GLID, (j)*GLID);
				glEnd();
			}else{
				//glBegin(GL_POLYGON);
				glBegin(GL_LINE_LOOP);
				glColor3d(1.0, 1.0, 1.0);
				glVertex2d((i)*GLID, (j)*GLID);
				glVertex2d((i)*GLID, (j+1)*GLID);
				glVertex2d((i+1)*GLID, (j+1)*GLID);
				glVertex2d((i+1)*GLID, (j)*GLID);
				glEnd();
			}
		}
	}
}

void My_gl::draw_robot(){
	std::cout << x_t->x << "\t" << x_t->y << "\t" << x_t->theta << std::endl;
	glBegin(GL_LINE_LOOP);
	glColor3d(0.0, 1.0, 0.0);
	for(int th1=0; th1<THT_MAX; th1++){
		double th2 = th1 + 10.0;
		double th1_rad = deg_to_rad(th1);
		double th2_rad = deg_to_rad(th2);

		double x1 = RADIUS * cos(th1_rad);
   	double y1 = RADIUS * sin(th1_rad);
  	double x2 = RADIUS * cos(th2_rad);
  	double y2 = RADIUS * sin(th2_rad);
		glBegin(GL_LINES);
   		glVertex2f( x1+x_t->x, y1+x_t->y );
   		glVertex2f( x2+x_t->x, y2+x_t->y );
  	glEnd();
	}
	glEnd();
}

void My_gl::motion(control u_t){
	x_t->x += u_t.translation*cos(deg_to_rad(x_t->theta));
	x_t->y += u_t.translation*sin(deg_to_rad(x_t->theta));
	x_t->theta += u_t.rotate;
}
