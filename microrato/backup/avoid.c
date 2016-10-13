#include <math.h>
#include <stdlib.h>
#include <string.h>
#include "rmi-mr32.h"
#include "avoid.h"
#include <stdbool.h>

//minimum distances of micro rato (avoid hit)
#define MIN_DIST_LEFT 20 //cm
#define MIN_DIST_RIGHT 20 //
#define MIN_DIST_FRONT 30 //

//velocities to control motors
#define VEL_CONTORNAR 30 //em pw 
#define VEL_MIN 10 
#define VEL_MIN_PESO 30 //definido com o peso em chão liso 
#define VEL_MIN_RUGOSO 40 //campo de futebol- alcatifa
#define VEL_MAX 35
#define RANGE (VEL_MAX- VEL_MIN)/4


#define FIND 0
#define R_LEFT 1
#define R_RIGHT 2
#define MIN_DIST 25






//iniciate
Avoid avoid_obst;

int C_state = 0, N_state = 0;


int parseSensors(){
	avoid_obst.STOP = false;
	if (tick40ms==1){
		tick40ms = 0;
		readAnalogSensors(); //update das variaveis
		                     //
		                     ////ver quanto tempo demora a ter isto????????
		                     
		//Máquina de estados interna
		//Está à procura de obstáculos:
		if(C_state == FIND){
			int sum ;
			if (( sum= bla(analogSensors.obstSensFront ,analogSensors.obstSensLeft, analogSensors.obstSensRight) )!= 0){
				avoid_obst.ON = true;

				//choose ...
				if (sum == 0)
					N_state = FIND;

				else if ( sum == 6 || sum == 7){
				 //order = STOP;
				 //o que fazer???????
				}
				else if(sum == 2 || sum==3){
					avoid_obst.angle = compute_angleL( analogSensors.obstSensFront ,analogSensors.obstSensLeft, analogSensors.obstSensRight );
					N_state = R_LEFT;
				}
					
				 
				else if(sum==4 || sum== 5 || sum == 1){

					N_state = R_RIGHT;
				}
					
				 
			}
		
		}else if(C_state == R_LEFT){
			avoid_obst.angle = compute_angleL( analogSensors.obstSensFront ,analogSensors.obstSensLeft, analogSensors.obstSensRight );
			printf("left %f -- %d, %d, %d\n", avoid_obst.angle, analogSensors.obstSensFront ,analogSensors.obstSensLeft, analogSensors.obstSensRight );
		}else if(C_state == R_RIGHT){
			//avoid_obst.angle = compute_angleR( analogSensors.obstSensFront ,analogSensors.obstSensLeft, analogSensors.obstSensRight );
			printf("right\n" );
			N_state = FIND;

		}

		C_state = N_state;
		

	}
	return 0;
}


//-------------auxiliares--------------------------
double map(double x, double in_min, double in_max, double out_min, double out_max){
    //printf("%f %f ", out_min, out_max);
    if(x < in_min )
    	return out_min;
    if(x> in_max)
    	return out_max;

  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


int bla(double front, double left, double right){

	int sum = 0;
	if(front <= MIN_DIST )
		sum+=1;

	if(left <= MIN_DIST )
		sum+=4;

	if(right <= MIN_DIST )
		sum+=2;

	return sum;
}

double compute_angleL(double front, double left, double right){

	double angle;
	int sum=0;
		//ver se o left mais pequeno deveria ser precedente
		if(front < MIN_DIST)
			sum+=1;
		if(right< MIN_DIST)
			sum+=2;
		if(left <=30 && left >=20)
			sum+= 4;
		else if(left <20)
			sum+=  8;
		else //left > 30
		    sum+= 16;

	
		if(sum==1 || sum==5 || sum==9 || sum==17){
			avoid_obst.angle = (-M_PI*45)/180;
		}
		else if(sum==4){
			avoid_obst.angle = 0;
		}
		else if(sum==8){
			avoid_obst.angle = map(left, 0 , MIN_DIST , -M_PI*30/180 ,0 );
		}
		else if(sum==16){
			avoid_obst.angle = map(left, 40 , 80 , 0 , M_PI*45 /180 );
		}
		else if(sum== 6){
			avoid_obst.angle = M_PI*10/180;
		}
		else if(sum==11 || sum== 7 || sum==10){
			avoid_obst.angle = 0;
			avoid_obst.STOP = true;
		}
		else if(sum==18 || sum==19){
			avoid_obst.angle = map(left, 40 , 80 , 10 , M_PI*55 /180 );	
		}
		
			//case 3 2:
			//algo esta mal
			
		/*

		//.....
		if(front < 60)
			angle = map(front, 30 , 60 , -M_PI*30 /180 , 0);
		else if(right <30 && left<30){
			avoid_obst.STOP = true;
			return -1;
		}
			
		else{
			if(left < 40)
				angle = map(left, 0 , 40 , -M_PI*20/180 ,0 );
			else
				angle = map(left, 40 , 80 , 0 , M_PI*45 /180 );
		}
			*/
		

	return avoid_obst.angle;


}

double compute_angleR(double front, double left, double right){

	double angle;

		//ver se o left mais pequeno deveria ser precedente
		//.....
		if(front < 60)
			angle = map(front, 30 , 60 , M_PI*30 /180 , 0);
		else if(right <30 && left<30){
			avoid_obst.STOP = true;
			return -1;
		}
			
		else{
			if(left < 40)
				angle = map(left, 0 , 40 , M_PI*20/180 ,0 );
			else
				angle = map(left, 40 , 80 , 0 , -M_PI*45 /180 );
		}
			
		

	return angle;


}
