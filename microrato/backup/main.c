#include "rmi-mr32.h"
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include "avoid.c"

#define WHITE              0
#define BLACK              1
#define COLOR_LINE         WHITE

enum ground_sensors {G_ML = 0, G_L = 1, G_C = 2, G_R = 3, G_MR = 4};
#define SIZE_GROUND_SENS   5
// Buffer for ground sensor (history)
#define GROUND_HISTORY     100
// Increase velocity when moving forward
#define STATIC_INCREASE    0
#define BASE_SPEED         60

// LAP Configuration
#define NUMBER_OF_LAPS     2
#define LAP_MIN_CYCLES     1000
#define LAP_DIFF_Y         150
#define LAP_DIFF_X         150
#define LAP_BUFFER_CHECK   5
// Comment next line to prevent rotate on even laps
#define LAP_INVERT_EVEN

// Feel free to comment any defines to disable debug
//#define DEBUG_VERBOSE
#ifdef DEBUG_VERBOSE
//   #define DEBUG_BLUETOOTH
   #define DEBUG_GROUND
//   #define DEBUG_PID
//   #define DEBUG_POSITION 
//   #define DEBUG_OBSTACLE
#endif

#ifdef DEBUG_BLUETOOTH
   #include "bluetooth_comm.h"
#endif

int ground_records_stored = 0;
bool ground_buffer[GROUND_HISTORY][5];

typedef enum {START, FOLLOW_LINE, OBSTACLE} State;
typedef enum {CENTERED, LOST_LEFT, LOST_RIGHT} GroundState;
State current_state;
GroundState ground_state;

typedef struct {
   double x;
   double y;
   double rot;
} Position;
Position start_position, current_position;

int line_KP = 15, line_KI = 0, line_KD = 0.4;
static double previous_error, error = 0, integral_error = 0;

void rotateRel_basic(int speed, double deltaAngle);
void rotateRel(int maxVel, double deltaAngle);
void walk_rotate(double);
void sort_array(int *array, int size);
void read_ground_sensors(int iterations);
int median_array(int sensor, int newValue);
void follow_line();

int last_ground_sensor;
int number_of_cycles;
int laps_finished;


int main(void){

avoid_obst.ON = false;
avoid_obst.STOP = false;
   

   initPIC32();

   #ifdef DEBUG_BLUETOOTH
      configBTUart(3, 115200); // Configure Bluetooth UART
      bt_on();     // enable bluetooth channel; printf
   #endif

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
   enableGroundSens();
      enableObstSens();
      current_state = START;
      ground_state = CENTERED;
      getRobotPos(&start_position.x, &start_position.y, &start_position.rot);
      number_of_cycles = 0;
      laps_finished = 0;

   while(!stopButton()){

      read_ground_sensors(1);
   
      parseSensors();
    
      

      



      switch(current_state) {
            case START:
               current_state = FOLLOW_LINE;
               break;
            case FOLLOW_LINE:
               
               if(avoid_obst.ON){
                  //printf("angle %f\n", avoid_obst.angle);
                  //setVel2(0,0);
                  current_state = OBSTACLE;
                  //  avoid_obst.ON = true;

               }else
                  follow_line();


               break;

            case OBSTACLE:
               if(ground_buffer[0][G_ML] | ground_buffer[0][G_L] |  ground_buffer[0][G_C] | 
                  ground_buffer[0][G_R] |  ground_buffer[0][G_MR]){
                  
                  current_state = FOLLOW_LINE;
                  avoid_obst.ON = false;
                  setVel2(10,10); //para ele virar para a direita

               }
                  

               break;
            default:
               break;
         }
         getRobotPos(&current_position.x, &current_position.y, &current_position.rot);

         int k;
         bool left = false, center = false, right = false;
         for (k = 0; k < 20; k++) {
            if (ground_buffer[k][G_ML])
               left = true;
            if (ground_buffer[k][G_C])
               center = true;
            if (ground_buffer[k][G_MR])
               right = true;
         }

         number_of_cycles++;
         if (left && center && right && number_of_cycles >= LAP_MIN_CYCLES &&
               abs(current_position.x - start_position.x) <= LAP_DIFF_X &&
               abs(current_position.y - start_position.y) <= LAP_DIFF_Y) {
            laps_finished++;


        setRobotPos(0, 0, 0);
            start_position.x = 0;
            start_position.y = 0;
            start_position.rot = 0;
            number_of_cycles = 0;
            if (laps_finished >= NUMBER_OF_LAPS)
               break;
         }
      }
      disableObstSens();
      disableGroundSens();
      setVel2(0, 0);
   

   return 0;
}

/*
int main(void)
{
   initPIC32();


   closedLoopControl(true);
   setVel2(0, 0);

   printf("RMI, Robot %d\n\n", ROBOT);
   

   while (true) {
      readAnalogSensors();          // Fill in "analogSensors" structure
      printf("Battery Level: %d\n", analogSensors.array[BATTERY]);

      while (!startButton());

      enableGroundSens();
      enableObstSens();
      current_state = START;
      ground_state = CENTERED;
      getRobotPos(&start_position.x, &start_position.y, &start_position.rot);
      number_of_cycles = 0;
      laps_finished = 0;

      while (!stopButton()) {
         readAnalogSensors();          // Fill in "analogSensors" structure
         read_ground_sensors(1);

#ifdef DEBUG_OBSTACLE
         printf("L = %03d, C = %03d, R = %03d\n", analogSensors.obstSensLeft, 
            analogSensors.obstSensFront, analogSensors.obstSensRight);
#endif
         switch(current_state) {
            case START:
               current_state = FOLLOW_LINE;
               break;
            case FOLLOW_LINE:
               follow_line();
               break;
            case OBSTACLE:
               break;
            default:
               break;
         }
         getRobotPos(&current_position.x, &current_position.y, &current_position.rot);

#ifdef DEBUG_POSITION
         printf("Start  Position: %6.2f, %6.2f (%6.2f)\n", 
            start_position.x, start_position.y, start_position.rot);
         printf("Actual Position: %6.2f, %6.2f (%6.2f)\n", 
            current_position.x, current_position.y, current_position.rot);
         printf("Cycles: %d\n", number_of_cycles);
#endif

         int k;
         bool left = false, center = false, right = false;
         for (k = 0; k < 20; k++) {
            if (ground_buffer[k][G_ML])
               left = true;
            if (ground_buffer[k][G_C])
               center = true;
            if (ground_buffer[k][G_MR])
               right = true;
         }

         number_of_cycles++;
         if (left && center && right && number_of_cycles >= LAP_MIN_CYCLES &&
               abs(current_position.x - start_position.x) <= LAP_DIFF_X &&
               abs(current_position.y - start_position.y) <= LAP_DIFF_Y) {
            laps_finished++;
#ifdef DEBUG_VERBOSE
            printf("\n\n> LAP FINISHED! <\n\n");
#endif
#ifdef LAP_INVERT_EVEN
            if (laps_finished % 2 != 0)
               rotateRel(100, -M_PI);
#endif
            setRobotPos(0, 0, 0);
            start_position.x = 0;
            start_position.y = 0;
            start_position.rot = 0;
            number_of_cycles = 0;
            if (laps_finished >= NUMBER_OF_LAPS)
               break;
         }
      }
      disableObstSens();
      disableGroundSens();
      setVel2(0, 0);
   }

   return 0;
}*/

void read_ground_sensors(int iterations) {
   iterations = iterations > GROUND_HISTORY ? GROUND_HISTORY : iterations;

   int i, ground_sensor;
   ground_records_stored = ground_records_stored >= GROUND_HISTORY 
      ? GROUND_HISTORY : ground_records_stored + 1;

   int k, id;
   for (k = GROUND_HISTORY - iterations - 1; k >= 0; k--) {
      for (id = 0; id < 5; id++) {
         ground_buffer[k + iterations][id] = ground_buffer[k][id];
      }
   }

   for (i = iterations - 1; i >= 0; i--) {
      ground_sensor = readLineSensors(40);

      if (COLOR_LINE == WHITE)
         ground_sensor =~ ground_sensor;


#ifdef DEBUG_GROUND
      printInt(ground_sensor & 0x0000001F, 2 | 5 << 16);   // System call
      printf("\n");
#endif

      ground_buffer[i][G_ML] = ((ground_sensor & 0x00000010) >> 4);
      ground_buffer[i][G_L] = ((ground_sensor & 0x00000008) >> 3);
      ground_buffer[i][G_C] = ((ground_sensor & 0x00000004) >> 2);
      ground_buffer[i][G_R] = ((ground_sensor & 0x00000002) >> 1);
      ground_buffer[i][G_MR] = (ground_sensor & 0x00000001);
   }
}

void follow_line() 
{
   double sum = 0;
   int nelements = 0;

   if (!ground_buffer[0][G_ML] && !ground_buffer[0][G_L] && !ground_buffer[0][G_C] && 
      !ground_buffer[0][G_R] && !ground_buffer[0][G_MR]) {
      int i;

      int left = 0, right = 0;
      if (ground_state == CENTERED) {
         for (i = 0; i < GROUND_HISTORY; i++) {
            if (ground_buffer[i][G_MR]) {
               right += (GROUND_HISTORY - i);
            }
            if (ground_buffer[i][G_ML]) {
               left += (GROUND_HISTORY - i);
            }
         }
      }
      if (left > GROUND_HISTORY * 4 || right > GROUND_HISTORY * 4) {
         if (left > right) {
            ground_state = LOST_LEFT;
         } else {
            ground_state = LOST_RIGHT;
         }
      }

      if (ground_state == LOST_RIGHT) {
         sum += 9.0;
         nelements += 2;
      } else if (ground_state == LOST_LEFT) {
         sum -= 9.0;
         nelements += 2;
      }
   } else {
      ground_state = CENTERED;
      if (ground_buffer[0][G_ML]) {
         sum -= 4.0;
         nelements+= 2;
      }

      if (ground_buffer[0][G_L]) {
         sum -= 1.0;
         nelements+= 2;
      }

      if (ground_buffer[0][G_C]) {
         nelements++;
      }

      if (ground_buffer[0][G_R]) {
         sum += 1.0;
         nelements+=2;
      }

      if (ground_buffer[0][G_MR]) {
         sum += 4.0;
         nelements+=2;
      }
   }

   previous_error = error;
   error = nelements == 0 ? 0 : (sum*2) / nelements;
   
   double propotional_error = line_KP * error;
   integral_error = (integral_error + error) * line_KI;
   integral_error = integral_error > 5 ? 5 : integral_error;
   integral_error = integral_error < -5 ? -5 : integral_error;
   double derivative_error = line_KD * (error - previous_error);


   double velocity_increment = 0;
   velocity_increment += propotional_error;
   velocity_increment += integral_error;
   velocity_increment += derivative_error;

#ifdef DEBUG_PID
   printf("Increment: %6.2f, P: %6.2f, I: %6.2f, D: %6.2f, Others: %4.2f - %4.2f (%d)\n", 
      velocity_increment, propotional_error, integral_error,
      derivative_error, error, median_sum, elements_weight);
#endif

   //setVel2(BASE_SPEED + velocity_increment + STATIC_INCREASE, 
     //      BASE_SPEED - velocity_increment + STATIC_INCREASE);
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
   printf(" walk_rotate--");
   targetAngle = normalizeAngle(t + deltaAngle);
   error = normalizeAngle(targetAngle - t);
   
      
   getRobotPos(&x, &y, &t);
   error = normalizeAngle(targetAngle - t);

   cmdVel = kp * error;

   setVel2(30 - cmdVel, 30 + cmdVel); //walks 40 minimum
}

#define KP_ROT 40
#define KI_ROT 5

// deltaAngle in radians
void rotateRel(int maxVel, double deltaAngle)
{
   double x, y, t;
   double targetAngle;
   double error;
   double integral = 0;
   int cmdVel;

   getRobotPos(&x, &y, &t);
   targetAngle = normalizeAngle(t + deltaAngle);
   do
   {
      waitTick40ms();
      getRobotPos(&x, &y, &t);
      error = normalizeAngle(targetAngle - t);

      integral += error;
      integral = integral > PI / 2 ? PI / 2: integral;
      integral = integral < -PI / 2 ? -PI / 2: integral;

      cmdVel = (int)((KP_ROT * error) + (integral * KI_ROT));
      cmdVel = cmdVel > maxVel ? maxVel : cmdVel;
      cmdVel = cmdVel < -maxVel ? -maxVel : cmdVel;

      setVel2(-cmdVel, cmdVel);
   } while (fabs(error) > 0.01);
   setVel2(0, 0);
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

