
#include "../include/pf.h"
#include "../include/param.h"

#include <GL/glut.h>
#include<stdlib.h>
#include<math.h>
#include <fstream>

robot_position *pf_x_t;

double pf_t[MAX_PARTICLE][PARTICLE_ELEMENT];
double hinan_pf[MAX_PARTICLE][PARTICLE_ELEMENT];
double Map_pf[LINE][COLUMN];
double pf_sensor_dis[MAX_PARTICLE][SENSOR_NUM];
std::vector<Pf_Sensor> pf_laser;



enum Element{
  X,
  Y,
  THETA,
  W,
};

double Particle_filter::Uniform(){
   return ((double)rand()+1.0)/((double)RAND_MAX+2.0);
}

double Particle_filter::select_Uniform(double min, double max){
  double result;
  result = (max-min)*rand()/RAND_MAX-(max-min)/2;
  //std::cout << result << std::endl;
  return result;
}

double Particle_filter::deg_to_rad(double deg){
	return deg / 180.0 * PI;
}

double Particle_filter::gause(double ave,double deviation,double x){
  return 1 / (sqrt(2*PI)*deviation) * exp(-(x-ave)*(x-ave)/(2*deviation*deviation));
}


Particle_filter::Particle_filter(){

  pf_x_t = new robot_position();
  for(int i=0; i<MAX_PARTICLE; i++){
    pf_t[i][X] = 34;
    pf_t[i][Y] = -210;
    pf_t[i][THETA] = 90;
    pf_t[i][W] = 1/MAX_PARTICLE;
  }

  FILE *map_file;
	int num;
	char kaigyou;
	if((map_file =fopen("../map_wall.txt","r")) == NULL){
		std::cout << "Doesn't exist map file" << std::endl;
	}else{
		int i=0,j=0;
		while (fscanf(map_file, "%d", &num) != EOF){
			if(j==(COLUMN-1)){
				Map_pf[i][j] = num;
				j=0;
				i++;
    	}else{
				j++;
				Map_pf[i][j] = num;
			}
		}
	}
	fclose(map_file);
}

Particle_filter::~Particle_filter(){
  delete pf_x_t;
}


void Particle_filter::motion_update(control u_t){

  for(int i=0; i<MAX_PARTICLE; i++){
    //std::cout << pf_t[i][X] << std::endl ;
  }
  for(int i=0; i<MAX_PARTICLE;i++){
    pf_t[i][THETA] += u_t.rotate+select_Uniform(-THETA_ERROR,THETA_ERROR);
    pf_t[i][X] += ((u_t.translation+select_Uniform(-X_ERROR,X_ERROR))*cos(deg_to_rad(pf_t[i][THETA])) );
    pf_t[i][Y] += ((u_t.translation+select_Uniform(-Y_ERROR,Y_ERROR))*sin(deg_to_rad(pf_t[i][THETA])) );

  }
}

void Particle_filter::draw_particle_pf(){
  glBegin(GL_LINE_LOOP);
	glColor3d(1.0, 0.0, 0.0);
  for(int i=0;i<MAX_PARTICLE;i++){
  int bou=2;
    //円描画
    for(int th1=0; th1<THT_MAX; th1++){
      double th2 = th1 + 10.0;
  		double th1_rad = deg_to_rad(th1);
  		double th2_rad = deg_to_rad(th2);

  		double x1 = PARTICLE_RADIUS * cos(th1_rad);
     	double y1 = PARTICLE_RADIUS * sin(th1_rad);
    	double x2 = PARTICLE_RADIUS * cos(th2_rad);
    	double y2 = PARTICLE_RADIUS * sin(th2_rad);

      glBegin(GL_LINES);
     		glVertex2f( x1+pf_t[i][X], y1+pf_t[i][Y] );
     		glVertex2f( x2+pf_t[i][X], y2+pf_t[i][Y] );
    	glEnd();

  	}

    //棒描画
    glBegin(GL_LINES);
  		glVertex2f( pf_t[i][X], pf_t[i][Y] );
  		glVertex2f( pf_t[i][X]+bou*cos(deg_to_rad(pf_t[i][THETA])), pf_t[i][Y]+bou*sin(deg_to_rad(pf_t[i][THETA])) );
  	glEnd();

  }
  glEnd();

}

void Particle_filter::pf_sensor_measure(){
  double range;
  Pf_Sensor pf_laser_result;
  pf_laser.clear();
  for(int i=0; i< MAX_PARTICLE; i++){
  	for(int j=0;j != SENSOR_NUM;j++){
  		pf_laser_result.x = pf_t[i][X];
  		pf_laser_result.y = pf_t[i][Y];
  		pf_laser_result.theta = pf_t[i][THETA];
      //std::cout << pf_laser_result.x << " ";
  		for(range=0;range<MAX_RANGE;range+=0.1){
  			int laser_x =  pf_t[i][X] + range*cos(deg_to_rad(pf_t[i][THETA]) );
  			int laser_y =  pf_t[i][Y] + range*sin(deg_to_rad(pf_t[i][THETA]) );
  			if(Map_pf[-laser_y/GLID][laser_x/GLID] == 1){
  				pf_laser_result.diss = range;
  				break;
  			}else{
  				pf_laser_result.diss = MAX_RANGE;
  			}
  		}

  		pf_laser.push_back(pf_laser_result);
    }
  }
}

void Particle_filter::sensor_update(){
  total_weight = 0;
  //std::cout << robot_sensor_result << std::endl;
  for(int i=0;i<MAX_PARTICLE;i++){
    pf_t[i][W] = gause(robot_sensor_result, Z_ERROR,pf_laser[i].diss);
    total_weight += pf_t[i][W];
  }

  //正規化
  for(int i=0; i<MAX_PARTICLE;i++){
    pf_t[i][W] /= total_weight;
  }
  double a[4];
  for(int i=0; i<MAX_PARTICLE;i++){
    for(int j=0; j<MAX_PARTICLE;j++){
        if((pf_t[j][W] >pf_t[i][W])){
          //saisyo dekai
        }else{
          a[0] = pf_t[i][X];
          a[1] = pf_t[i][Y];
          a[2] = pf_t[i][THETA];
          a[3] = pf_t[i][W];

          pf_t[i][X] = pf_t[j][X];
          pf_t[i][Y] = pf_t[j][Y];
          pf_t[i][THETA] = pf_t[j][THETA];
          pf_t[i][W] = pf_t[j][W];

          pf_t[j][X] = a[0];
          pf_t[j][Y] = a[1];
          pf_t[j][THETA] = a[2];
          pf_t[j][W] = a[3];
        }
    }
  }

  //hinan
  for(int i=0;i<MAX_PARTICLE;i++){
    hinan_pf[i][X] = pf_t[i][X];
    hinan_pf[i][Y] = pf_t[i][Y];
    hinan_pf[i][THETA] = pf_t[i][THETA];
    hinan_pf[i][W] = 0;

  }

  double w_itiyou = 1 / MAX_PARTICLE;
  int particle_count=0;
  int select_patricle=9;
  for(int i=0;i<7;i++){
      for(int j =0; j<select_patricle;j++){
        hinan_pf[particle_count][X] = pf_t[i][X];
        hinan_pf[particle_count][Y] = pf_t[i][Y];
        hinan_pf[particle_count][THETA] = pf_t[i][THETA];
        hinan_pf[particle_count][W] = w_itiyou;
        particle_count++;

      }
      select_patricle--;
  }

  for(int i = particle_count+1;i < MAX_PARTICLE+1;i++){
    //std::cout << i << " ";
    hinan_pf[i][X] = pf_t[i][X];
    hinan_pf[i][Y] = pf_t[i][Y];
    hinan_pf[i][THETA] = pf_t[i][THETA];
    hinan_pf[i][W] = w_itiyou;
  }

/*  for(int i=0;i<MAX_PARTICLE;i++){
    std::cout << hinan_pf[i][X] <<  " ";
  }
*/
  for(int i=0;i<MAX_PARTICLE;i++){
     pf_t[i][X]  = 0;
     pf_t[i][X] =0;
     pf_t[i][THETA] = 0;
     pf_t[i][W] = 0;
  }

  for(int i=0;i<MAX_PARTICLE;i++){
      pf_t[i][X]  = hinan_pf[i][X];
      pf_t[i][Y] = hinan_pf[i][Y];
      pf_t[i][THETA] = hinan_pf[i][THETA];
      pf_t[i][W] = w_itiyou;
  }

  for(int i=0;i<MAX_PARTICLE;i++){
      std::cout << pf_t[i][X] <<  " " << pf_t[i][Y] << std::endl;

  }

  draw_particle_pf();
}
