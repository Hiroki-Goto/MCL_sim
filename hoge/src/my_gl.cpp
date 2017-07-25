#define GLOBAL_H


#include <GL/glut.h>
#include <iostream>
#include <string>
#include <fstream>
#include <math.h>
#include<vector>

#include "../include/param.h"
#include "../include/my_gl.h"

robot_position *x_t;
std::vector<Sensor> laser;
int Map[LINE][COLUMN];
double robot_sensor_result = 0;

enum Element{
  X,
  Y,
  THETA,
  W,
};

My_gl::My_gl(){
	x_t = new robot_position;
	x_t->x = 34;
	x_t->y = -210;
	x_t->theta = 90;

	Sensor sensor;
	for(int i=0;i != SENSOR_NUM;i++){
		sensor.x = x_t->x;
		sensor.y = x_t->y;
		sensor.theta = x_t->theta;
		sensor.diss = MAX_RANGE;
		laser.push_back(sensor);
	}
}

My_gl::~My_gl(){
	delete x_t;

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
				glColor3d(0.0, 0.0, 0.0);
				glVertex2d((j)*GLID, -(i)*GLID);
				glVertex2d((j)*GLID, -(i+1)*GLID);
				glVertex2d((j+1)*GLID, -(i+1)*GLID);
				glVertex2d((j+1)*GLID, -(i)*GLID);
				glEnd();
			}else{
				//glBegin(GL_POLYGON);
				glBegin(GL_LINE_LOOP);
				glColor3d(1.0, 1.0, 1.0);
				glVertex2d((j)*GLID, -(i)*GLID);
				glVertex2d((j)*GLID, -(i+1)*GLID);
				glVertex2d((j+1)*GLID, -(i+1)*GLID);
				glVertex2d((j+1)*GLID, -(i)*GLID);
				glEnd();
			}
		}
	}

}

void My_gl::draw_robot(){
	//std::cout << x_t->x << "\t" << x_t->y << "\t" << x_t->theta << std::endl;
	glBegin(GL_LINE_LOOP);
	glColor3d(0.0, 1.0, 0.0);
	for(int th1=0; th1<THT_MAX; th1++){
		double th2 = th1 + 10.0;
		double th1_rad = deg_to_rad(th1);
		double th2_rad = deg_to_rad(th2);

		double x1 = ROBO_RADIUS * cos(th1_rad);
   	double y1 = ROBO_RADIUS * sin(th1_rad);
  	double x2 = ROBO_RADIUS * cos(th2_rad);
  	double y2 = ROBO_RADIUS * sin(th2_rad);
		glBegin(GL_LINES);
   		glVertex2f( x1+x_t->x, y1+x_t->y );
   		glVertex2f( x2+x_t->x, y2+x_t->y );
  	glEnd();
	}
	int bou=10;
	glBegin(GL_LINES);
		glVertex2f( x_t->x, x_t->y );
		glVertex2f( x_t->x+bou*cos(deg_to_rad(x_t->theta)), x_t->y+bou*sin(deg_to_rad(x_t->theta)) );
	glEnd();
}

void My_gl::draw_sensor(){
	double range=0;

	Sensor sensor;
	laser.clear();
	for(int i=0;i != SENSOR_NUM;i++){
		sensor.x = x_t->x;
		sensor.y = x_t->y;
		sensor.theta = x_t->theta;
		for(range=0;range<MAX_RANGE;range+=0.5){
			int laser_x =  x_t->x + range*cos(deg_to_rad(x_t->theta) );
			int laser_y =  x_t->y + range*sin(deg_to_rad(x_t->theta) );
			if(Map[-laser_y/GLID][laser_x/GLID] == 1){
				sensor.diss = range;

				break;
			}else{
				sensor.diss = MAX_RANGE;
			}
		}
		robot_sensor_result = sensor.diss;
		laser.push_back(sensor);

		glColor3d(0.0, 0.0, 3.0);
		glBegin(GL_LINES);
			glVertex2f( x_t->x, x_t->y );
			glVertex2f( x_t->x+laser[0].diss*cos(deg_to_rad(sensor.theta)) , x_t->y+laser[0].diss*sin(deg_to_rad(sensor.theta)) );
		glEnd();

	//}
	}

}

void My_gl::draw_particle(){
	draw_particle_pf();
}


void My_gl::motion(control u_t){
	x_t->x += u_t.translation*cos(deg_to_rad(x_t->theta));
	x_t->y += u_t.translation*sin(deg_to_rad(x_t->theta));
	x_t->theta += u_t.rotate;
	motion_update(u_t);
}

void My_gl::measure(){
  pf_sensor_measure();
  sensor_update();
}
