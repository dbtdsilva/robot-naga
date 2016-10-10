#include "rmi-mr32.h"
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include "functions.h"
#include "cmdAPI.c"
#include "pic32_uart_device_driver.h"




int GetMotorSpeed(double dist_front,double dist_left,double dist_right, double * leftMotor, double * rightMotor,  int * step, int * pos , bool* t);
int getSpeed(double dist);
double map_angle(int pos);

void walk_rotate(double deltaAngle);
void searchBecon(bool stoped);


//guarda o buffer e retir a mediana
int medianF(int newValue);
int medianL(int newValue);
int medianR(int newValue);

bool stoped;


	int i= 0;
	int C_state = 0, N_state = 0;
	bool foundBeacon = false;
	bool send_sensor= false;
	bool verify = false;
	int p = 0;
	double angle = 0;
	int inc = 0;
	
int pos_tmp;
	bool found_tmp;
	bool turning =false;
	bool inner = false;
	bool foundBeacon_before = false;
	int keepPos;
	bool count= false;


	int step = abs(POS_LEFT); //ate onde o servo pode ir -->15 no inicio
	int next_pos = POS_LEFT;  // position a que o servo está
	int pos ;

int main(void){

	

	initPIC32();
	closedLoopControl( true );
	setVel2(0, 0);
	enableObstSens();
	
	while(!startButton());
	
    

	
	EnableInterrupts();
	

	while(!stopButton()){

	
		
		walk_rotate(angle); //PID
		
		//rotate(angle);
		if(tick40ms==1){ // ciclo de avoid obstacles -> priority: high
			
			tick40ms= 0;
			
			//get orders
			

			//printf("REAL distance: front = %f , left = %f , right = %f\n",dist_front, dist_left, dist_right );
			/*
			double leftMotor, rightMotor;

			
			GetMotorSpeed(dist_front, dist_left, dist_right, &leftMotor, &rightMotor, &step, &pos, &turning);
			//printf("%f %f\n",leftMotor, rightMotor);
			if(turning){
				setVel2(leftMotor, rightMotor);
			}
			*/
			


		}

			



	
	}

	//disableSensors();

	return 0;
}


/*
	return error : 0 for success
	return speed of motors in arguments leftMotor and rightMotor 

int GetMotorSpeed(double dist_front,double dist_left,double dist_right, double * leftMotor, double * rightMotor , int * step, int * pos, bool* turning){


	if (dist_right<0 || dist_left<0 || dist_front<0)
		return -1;

	int left_wall = 0;
	int right_wall = 0;
	int front_wall = 0;

	//distance = getRealDistance(distance);
	stoped = false;

	if(dist_left<= MIN_DIST_LEFT)
		left_wall = 4;
	if(dist_right<= MIN_DIST_RIGHT)
		right_wall = 2;
	if(dist_front<= MIN_DIST_FRONT)
		front_wall = 1;

	int sum = left_wall + right_wall + front_wall;

	if (sum == 0){
		//order = WALK;
		//printf("WALK\n");
		*turning=false;
		
	}
	else if (sum == 1 || sum ==3 || sum == 5 || sum == 6 || sum == 7){
		//order = STOP;
		//printf("STOP\n");
		//*turning=false;
		*turning = true;
		*leftMotor = *rightMotor = 0.0;
		setVel2(0,0);
		stoped = true;
	}
	else if(sum == 2){
		//order = TURN_LEFT; 
		//printf("TURN_LEFT\n");
		*turning=true;
		//rotate(M_PI/6);
		*pos = -15;
		*step = 15;
		int vel = map(dist_right,10,35, 25 , 5); 
		*rightMotor = 20 + vel;
		*leftMotor = 20; //continuam a andar ate serem corrigidos novamente
	}
	else{ // 4
		
		*turning = true;
		*pos = -15;
		*step = 15;
		int vel = map(dist_left,10,35, 25 , 5); 
		*rightMotor = 20;
		*leftMotor = 20 + vel;
	}
	

	return 0;
	
}

*/

/*
	get speed acording to distance
	return speed

int getSpeed(double distance){
	
	int speed;
	//note: dist is in cm and already the 7cm were taken
	
	if(distance >=70.0) //between 70 and bigger cm
	{
		speed = map(distance,70,80,3*RANGE + VEL_MIN,4*RANGE + VEL_MIN); 
	}
	else if(distance >=60.0) //between 60 e 70
	{
		speed = map(distance,60,69,2*RANGE + VEL_MIN,3*RANGE + VEL_MIN);
	}
	else if(distance >=50.0) //between 50 e 60
	{
		speed = map(distance,50,59,RANGE + VEL_MIN,2*RANGE + VEL_MIN);
	}
	else if(distance >=40.0) //between 40 e 50
	{
		speed = map(distance,40,49,VEL_MIN, RANGE + VEL_MIN);
	}
	else if(distance >=30.0) //between 30 e 40 cm
	{
		//minima velocidade
		speed = VEL_MIN;
	}
	else  //30 cm and less
		speed = 0;

	return speed;
}




*/







int sort(int *array, int size)
{
   int flag, i, j, aux;
   j = size;
   do 
   {
      flag = FALSE;
      for (i=0; i < j-1; i++)
      {
         if (array[i] > array[i+1])
         {
            aux = array[i];
            array[i] = array[i+1];
            array[i+1] = aux;
            flag = TRUE;
         }
      }
      j--;
   } while (flag);
   return array[size/2];
}


#define MEDIAN_SIZE 7

int medianF(int newValue)
{
   static int buf[MEDIAN_SIZE], i = 0;
   int aux[MEDIAN_SIZE];
   int j;

   buf[i] = newValue;
   i = (i + 1) % MEDIAN_SIZE;

   for(j=0; j < MEDIAN_SIZE; j++)
      aux[j] = buf[j];

   return sort(aux, MEDIAN_SIZE);
}


int medianR(int newValue)
{
   static int buf[MEDIAN_SIZE], i = 0;
   int aux[MEDIAN_SIZE];
   int j;

   buf[i] = newValue;
   i = (i + 1) % MEDIAN_SIZE;

   for(j=0; j < MEDIAN_SIZE; j++)
      aux[j] = buf[j];

   return sort(aux, MEDIAN_SIZE);
}


int medianL(int newValue)
{
   static int buf[MEDIAN_SIZE], i = 0;
   int aux[MEDIAN_SIZE];
   int j;

   buf[i] = newValue;
   i = (i + 1) % MEDIAN_SIZE;

   for(j=0; j < MEDIAN_SIZE; j++)
      aux[j] = buf[j];

   return sort(aux, MEDIAN_SIZE);
}

/*
*function that walks and rotates with P (PID)
*/

void walk_rotate(double deltaAngle)
{
	double x, y, t;
	double targetAngle;
	double error;
	int cmdVel;
	
	int kp = 10;
	int ki = 0;
	int kd = 0;
	getRobotPos(&x, &y, &t);
	
	targetAngle = normalizeAngle(t + deltaAngle);
	error = normalizeAngle(targetAngle - t);
	
		
	getRobotPos(&x, &y, &t);
	error = normalizeAngle(targetAngle - t);

	cmdVel = kp * error;


	setVel2(40 - cmdVel, 40+ cmdVel); //walks 40 minimum


}

		