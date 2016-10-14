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
#define MIN_DIST 35




int Min(int x, int y);
//void walk_rotate(double);
//void rotateRel_basic(int speed, double deltaAngle);
int GetMotorSpeed(int dist_front,int dist_left,int dist_right, double * leftMotor, double * rightMotor );
//iniciate
Avoid avoid_obst;
double angle;

int C_state = 0, N_state = 0;



int Min(int x, int y)
{
	if(x<y)
		return x;
	return y;
}

int parseSensors(){
	


	if (tick20ms){
		tick20ms = 0;
		readAnalogSensors(); //update das variaveis
		                     //
		                     ////ver quanto tempo demora a ter isto????????
		//printf(" F:%d, L:%d, R:%d\n", analogSensors.obstSensFront ,analogSensors.obstSensLeft, analogSensors.obstSensRight );
		//Máquina de estados interna
		//Está à procura de obstáculos:
		if(C_state == FIND){
			printf("FIND ");
			int sum ;

			printf(" F:%d, L:%d, R:%d\n", analogSensors.obstSensFront ,analogSensors.obstSensLeft, analogSensors.obstSensRight );
			
			if (analogSensors.obstSensFront <= MIN_DIST-10 || analogSensors.obstSensRight  <= MIN_DIST-10 ||
						analogSensors.obstSensLeft  <= MIN_DIST-10 ){

				sum= bla(analogSensors.obstSensFront , analogSensors.obstSensRight,analogSensors.obstSensLeft) ;
				avoid_obst.ON = true;
		
				//setVel2(0,0);
				printf("found obstacle\n");
				

				N_state = R_LEFT; // 
				
				 
			}
			else{
				N_state = FIND;
				avoid_obst.ON = false;
			}
				
		
		}else if(C_state == R_LEFT){
			printf(" LEFT " );

			if(avoid_obst.ON == false)
				N_state = FIND;
			else{
				
				N_state = R_LEFT;

				double leftMotor, rightMotor;
				GetMotorSpeed(analogSensors.obstSensFront ,analogSensors.obstSensLeft, analogSensors.obstSensRight 
								, &leftMotor, &rightMotor);
				setVel2(leftMotor, rightMotor);
			}


		}

		C_state = N_state;
		

	}
	return 0;
}


//-------------auxiliares--------------------------
int map(int x, int in_min, int in_max, int out_min, int out_max){
    //printf("%f %f ", out_min, out_max);
    if(x < in_min )
    	return out_min;
    if(x> in_max)
    	return out_max;

  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


int bla(int front, int right, int left){

	int sum = 0;
	if(front <= MIN_DIST )
		sum+=1;

	if(left <= MIN_DIST )
		sum+=4;

	if(right <= MIN_DIST )
		sum+=2;

	return sum;
}

/*
	return error : 0 for success
	return speed of motors in arguments leftMotor and rightMotor 
*/


int GetMotorSpeed(int dist_front,int dist_left,int dist_right, double * leftMotor, double * rightMotor ){


	if (dist_right<0 || dist_left<0 || dist_front<0)
		return -1;

	int left_wall = 0;
	int right_wall = 0;
	int front_wall = 0;


	if(dist_left<= MIN_DIST)
		left_wall = 4;
	if(dist_right<= MIN_DIST)
		right_wall = 2;
	if(dist_front<= MIN_DIST)
		front_wall = 1;

	int sum = left_wall + right_wall + front_wall;
printf("sum %d  ", sum);
	if ( sum == 6 ){
		//order = STOP;
		

		*leftMotor = *rightMotor = 0.0;
		setVel2(0,0);

	}
	else if(sum == 7){
		*rightMotor = -20;
		*leftMotor = 30 ;
	}
	else if(sum == 1){
		int vel = map(dist_front,10,MIN_DIST, 25 , -15); 
		//turn right
		*rightMotor = 20;
		*leftMotor = 20 + vel;
	}
	else if(sum == 3){
		//order = TURN_LEFT; 
		int vel = map(Min(dist_right, dist_front),10,MIN_DIST, 15 , 5); 
		*rightMotor = 20 + vel;
		*leftMotor = 20; 
	}
	else if(sum == 5){
		//order = TURN_right; 
		int vel = map(Min(dist_left, dist_front),10,MIN_DIST, 35 , 15); 
		*rightMotor = -vel ;
		*leftMotor =  vel; 
	}
	else if(sum == 2){ 
		//order = TURN_LEFT; 
		int vel = map(dist_right,10,MIN_DIST, 25 , 5); 
		*rightMotor = 20 + vel;
		*leftMotor = 20; //continuam a andar ate serem corrigidos novamente
	}
	else if(sum==4){ // 4
		int vel = map(dist_left,10,MIN_DIST, 25 , 5); 
		*rightMotor = 20;
		*leftMotor = 20 + vel;
	}
	else if(dist_left>MIN_DIST && dist_left < 40){
		*rightMotor = 20;
		*leftMotor = 20;
	}
	else
	{
		//waitTick20ms() ;
		int vel = map(dist_left,80,35, 25 , 5); 
		*rightMotor = 20 + vel;
		*leftMotor = 20; //continuam a andar ate serem corrigidos novamente
	}

	return 0;
	
}

int compute_angleL(int front, int right, int left){
	
	
	int sum=0;
		//ver se o left mais pequeno deveria ser precedente
		if(front < MIN_DIST)
			sum+=1;
		if(right< MIN_DIST)
			sum+=2;
		if(left <=30 && left >=20)
			sum+= 4;
		else if(left < MIN_DIST)
			sum+=  8;
		else //left > 30
		    sum+= 16;

	
		/*if(sum==1 || sum==5 || sum==9 || sum==17){
			avoid_obst.angle = (-M_PI*45)/180;
			avoid_obst.STOP = false;
		}
		else if(sum==4){
			avoid_obst.angle = 0;
			avoid_obst.STOP = false;
		}
		else */if(sum==8){
			angle = map(left, 0 , MIN_DIST , -M_PI*30/180 ,0 );
			avoid_obst.STOP = false;
		}
		else if(sum==16){
			angle = map(left, 30 , 60 , 0 , M_PI*45 /180 );
			avoid_obst.STOP = false;
		}/*
		else if(sum== 6){
			avoid_obst.angle = M_PI*10/180;
			avoid_obst.STOP = false;
		}
		else if(sum==11 || sum== 7 || sum==10){
			avoid_obst.angle = 0;
			avoid_obst.STOP = true;
		}
		else if(sum==18 || sum==19){
			avoid_obst.angle = map(left, 30 , 60 , 10 , M_PI*55 /180 );	
			avoid_obst.STOP = false;
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
		
	printf("%f\n", avoid_obst.angle);
	return 0;


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


/*



int main(void){

avoid_obst.ON = false;
avoid_obst.STOP = false;
   

   initPIC32();


   closedLoopControl( true );
   setVel2(0, 0);
   enableObstSens();
   
   while(!startButton());
   

   int a = readCoreTimer();
    readAnalogSensors();          // Fill in "analogSensors" structure
    //printf("Battery Level: %d\n", analogSensors.array[BATTERY]);
    printf("%d\n", readCoreTimer()-a );

   
   EnableInterrupts();
   avoid_obst.angle = 0;


   while(!stopButton()){


      parseSensors();
      //printf("stop %d\n",avoid_obst.STOP );
      if(avoid_obst.STOP)
         setVel2(0,0);

      else{
         if(avoid_obst.ON){
           
           

            

         }
      }
      
      

   }
      disableObstSens();

      setVel2(0, 0);
   

   return 0;
}

void walk_rotate(double deltaAngle)
{
   double x, y, t;
   double targetAngle;
   double error;
   int cmdVel;
   
   int kp = 10;
   //int ki = 0;
   //int kd = 0;
   getRobotPos(&x, &y, &t);
   //printf(" walk_rotate--");
   targetAngle = normalizeAngle(t + deltaAngle);
   error = normalizeAngle(targetAngle - t);
   
      
   getRobotPos(&x, &y, &t);
   error = normalizeAngle(targetAngle - t);

   cmdVel = kp * error;

   setVel2(30 - cmdVel, 30 + cmdVel); //walks 40 minimum
}



void rotateRel_basic(int speed, double deltaAngle)
{
   double x, y, t;
   double targetAngle;
   double error;
   int cmdVel, errorSignOri;

   getRobotPos(&x, &y, &t);
   targetAngle = normalizeAngle(t + deltaAngle);
   error = normalizeAngle(targetAngle - t);
   errorSignOri = error < 0 ? -1 : 1;
//quando + --> cmdVel+ --> esquerda 30(as 12)
   cmdVel = error < 0 ? -speed : speed;
   //esquerda, direita
   setVel2(-cmdVel, cmdVel);

   do
   {
      getRobotPos(&x, &y, &t);
      error = normalizeAngle(targetAngle - t);
   } while (fabs(error) > 0.01 && errorSignOri * error > 0);
   setVel2(0, 0);
}

*/