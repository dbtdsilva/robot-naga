#include <math.h>
#include <stdlib.h>
#include <string.h>
#include "rmi-mr32.h"


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


double getRealDistance(double d);
int parseSensors();

struct Avoid {
   //ainda nao sei o que vai ser preciso
} ; 

//guarda o buffer e retir a mediana
int medianF(int newValue);
int medianL(int newValue);
int medianR(int newValue);

extern Avoid distanceSensors;

int parseSensors(){
	if (tick20ms==1){
		tick20ms = 0;
		readAnalogSensors();
		
		double dist_front = medianF(getRealDistance(analogSensors.obstSensFront));
		double dist_left = medianL(getRealDistance(analogSensors.obstSensLeft));
		double dist_right = medianR(getRealDistance(analogSensors.obstSensRight));

		//verify---------------------------------
		/*int left_wall=0, right_wall= 0, front_wall = 0;
		double tmp;

		if((tmp = getRealDistance(analogSensors.obstSensLeft)) <= MIN_DIST_LEFT)
		 left_wall = 4;
		//printf("%f\n",tmp );
		if((tmp =getRealDistance(analogSensors.obstSensRight)) <= MIN_DIST_RIGHT)
		 right_wall = 2;
		//rintf("%f\n",tmp );
		if((tmp = getRealDistance(analogSensors.obstSensFront)) <= MIN_DIST_FRONT) //analogSensors.obstSensFront
		 front_wall = 1;

		//printf("%f\n",tmp );

		int sum = left_wall + right_wall + front_wall;

		if (sum == 0){
		 //printf("WALK\n"); //não há obstaculos

		 avoid.overwrite = false;
		 avoid.turnLeft = false;
		 avoid.turnRight = false;
		}
		else if (sum == 1 || sum ==3 || sum == 5 || sum == 6 || sum == 7){
		 //order = STOP;
		 //printf("STOP\n");
		 setVel2(0,0);
		 avoid.overwrite = true;
		 avoid.turnLeft = false;
		 avoid.turnRight = false;
		}
		else if(sum == 2){
		 //order = TURN_LEFT; 
		 //printf("TURN_LEFT\n");

		 //*leftMotor = *rightMotor = 0.0;
		 avoid.overwrite = false;
		 avoid.turnLeft = true;
		 avoid.turnRight = false;
		}
		else if(sum==4){ // 4
		 //order = TURN_RIGHT;
		 //printf("TURN_RIGHT\n");
		 //*leftMotor = *rightMotor = 0.0;
		 avoid.turnRight = true;
		 avoid.overwrite = false;
		 avoid.turnLeft = false;
		}
		
*/
	}
	return 0;
}


//-------------auxiliares--------------------------

/*
	get distance in cm from the edge of microRato until object
	return cm
*/
double getRealDistance(double d){

	if (d <155) // a partir daqui é ruido
		return 80; //cm
	
	return 6200/(d-80);

}
