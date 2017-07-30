
#include "../include/draw.h"
#include "../include/param.h"

#include <iostream>
#include <math.h>
#include <GL/glut.h>

GlDraw::GlDraw(){

}

GlDraw::~GlDraw(){

}

double GlDraw::degToRad(double degree){
    double radian;
    radian = degree / 180 * PI;
    return radian;
}


void GlDraw::drawMap(Map map){
    for(int i=0;i<MAP_LINE;i++){
		for(int j=0;j<MAP_COLUMN;j++){
			if(map[i][j] == 1){
				glBegin(GL_POLYGON);
				glColor3d(0.0, 0.0, 0.0);
				glVertex2d((j)*MAP_GLID, -(i)*MAP_GLID);
				glVertex2d((j)*MAP_GLID, -(i+1)*MAP_GLID);
				glVertex2d((j+1)*MAP_GLID, -(i+1)*MAP_GLID);
				glVertex2d((j+1)*MAP_GLID, -(i)*MAP_GLID);
				glEnd();
			}
            else if(map[i][j] == 2){
                glBegin(GL_POLYGON);
                glColor3d(0.0, 0.0, 1.0);
                glVertex2d((j)*MAP_GLID, -(i)*MAP_GLID);
                glVertex2d((j)*MAP_GLID, -(i+1)*MAP_GLID);
                glVertex2d((j+1)*MAP_GLID, -(i+1)*MAP_GLID);
                glVertex2d((j+1)*MAP_GLID, -(i)*MAP_GLID);
                glEnd();
            }
            else {
				//glBegin(GL_POLYGON);
				glBegin(GL_LINE_LOOP);
				glColor3d(1.0, 1.0, 1.0);
				glVertex2d((j)*MAP_GLID, -(i)*MAP_GLID);
				glVertex2d((j)*MAP_GLID, -(i+1)*MAP_GLID);
				glVertex2d((j+1)*MAP_GLID, -(i+1)*MAP_GLID);
				glVertex2d((j+1)*MAP_GLID, -(i)*MAP_GLID);
				glEnd();
			}
		}
	}

}

void GlDraw::drawRobotAndParticle(Robot *robot_t,ParticleSet *particle_set_t){
    //パーティクルの描画
    //ロボットを先に書くとパーティクルのせいで見えなくなるため

    for(int i=0; i<particle_set_t->size(); i++){
        glBegin(GL_LINE_LOOP);
        glColor3d(1.0, 0.0, 0.0);
    	for(int th1=0; th1<360; th1++){
    		double th2 = th1 + 10.0;
    		double th1_rad = degToRad(th1);
    		double th2_rad = degToRad(th2);

    		double x1 = PARTICLE_RADIUS * cos(th1_rad);
       	    double y1 = PARTICLE_RADIUS * sin(th1_rad);
      	    double x2 = PARTICLE_RADIUS * cos(th2_rad);
      	    double y2 = PARTICLE_RADIUS * sin(th2_rad);
    		glBegin(GL_LINES);
                glVertex2f( x1+particle_set_t->at(i).x ,y1+particle_set_t->at(i).y );
                glVertex2f( x2+particle_set_t->at(i).x, y2+particle_set_t->at(i).y );
      	    glEnd();
    	}
        int robotTheta = 5;    //姿勢を示すときの直線の長さ
        glBegin(GL_LINES);
            glVertex2f( particle_set_t->at(i).x, particle_set_t->at(i).y );
            glVertex2f( particle_set_t->at(i).x+robotTheta*cos(degToRad(particle_set_t->at(i).theta)),
                            particle_set_t->at(i).y+robotTheta*sin(degToRad(particle_set_t->at(i).theta)) );
        glEnd();
    }


    //ロボットの描画
    glBegin(GL_LINE_LOOP);
    glColor3d(0.0, 1.0, 0.0);
    for(int th1=0; th1<360; th1++){
        double th2 = th1 + 10.0;
        double th1_rad = degToRad(th1);
        double th2_rad = degToRad(th2);

        double x1 = ROBOT_RADIUS * cos(th1_rad);
        double y1 = ROBOT_RADIUS * sin(th1_rad);
        double x2 = ROBOT_RADIUS * cos(th2_rad);
        double y2 = ROBOT_RADIUS * sin(th2_rad);
        glBegin(GL_LINES);
            glVertex2f( x1+robot_t->x, y1+robot_t->y );
            glVertex2f( x2+robot_t->x, y2+robot_t->y );
        glEnd();
    }
    glEnd();
    int robotTheta = 20;    //姿勢を示すときの直線の長さ
    glBegin(GL_LINES);
        glVertex2f( robot_t->x, robot_t->y );
        glVertex2f( robot_t->x+robotTheta*cos(degToRad(robot_t->theta)), robot_t->y+robotTheta*sin(degToRad(robot_t->theta)) );
    glEnd();

    //ロボットのセンサの描画
    glColor3d(1.0, 0.0, 0.0);
    for(int i=0; i<SENSOR_NUM; i++){
        double sensorAngleT = robot_t->theta - (INIT_THETA - robot_t->robotSensorData.at(i).angle );

        /*
        //距離を線として表現したいとき（オススメはしない）
        glBegin(GL_LINES);
            glVertex2f( robot_t->x, robot_t->y );
            glVertex2f( robot_t->x + robot_t->robotSensorData.at(i).distance*cos(degToRad(sensorAngleT))
                            , robot_t->y + robot_t->robotSensorData.at(i).distance*sin(degToRad(sensorAngleT)) );
        glEnd();
        */


        glBegin(GL_POLYGON);
        for(int th1=0; th1<360; th1++){
            double th1_rad = degToRad(th1);
            double x1 = SCAN_RADIUS * cos(th1_rad);
            double y1 = SCAN_RADIUS * sin(th1_rad);
            glVertex3f( x1 + robot_t->x+robot_t->robotSensorData.at(i).distance*cos(degToRad(sensorAngleT)),
                            y1 + robot_t->y + robot_t->robotSensorData.at(i).distance*sin(degToRad(sensorAngleT))
                                , 0.0);
        }
        glEnd();


    }
}
