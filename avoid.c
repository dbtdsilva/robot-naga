#include <math.h>
#include <stdlib.h>
#include <string.h>
#include "rmi-mr32.h"


double getRealDistance(double d);
int parseSensors();

struct Avoid {
   //ainda nao sei o que vai ser preciso
} ; 

extern Avoid distanceSensors;

int parseSensors(){
	if (tick20ms==1){
		tick20ms = 0;
		readAnalogSensors();
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
