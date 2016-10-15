#include "rmi-mr32.h"
#include <math.h>
#include <string.h>
#include <stdlib.h>

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
#define NUMBER_OF_LAPS     3
#define LAP_MIN_CYCLES     1000
#define LAP_DIFF_Y         125
#define LAP_DIFF_X         125
#define LAP_BUFFER_CHECK   20
// Comment next line to prevent rotate on even laps
#define LAP_TURN           2

// Feel free to comment any defines to disable debug
#define DEBUG_VERBOSE
#ifdef DEBUG_VERBOSE
//   #define DEBUG_BLUETOOTH
//   #define DEBUG_GROUND
//   #define DEBUG_PID
//   #define DEBUG_POSITION 
//   #define DEBUG_OBSTACLE
#endif

#ifdef DEBUG_BLUETOOTH
   #include "bluetooth_comm.h"
#endif

int ground_gain = 40;
int ground_records_stored = 0;
bool ground_buffer[GROUND_HISTORY][5];

typedef enum {START, FOLLOW_LINE, OBSTACLE} State;
typedef enum {CENTERED, LOST_LEFT, LOST_RIGHT} GroundState;
typedef enum {NOT_WALL, CENTER, FOLLOWING_WALL, CORNER} ObstacleState;
ObstacleState current_obstacle_state, old, next_obstacle_state;
State current_state;
GroundState ground_state;

typedef struct {
   double x;
   double y;
   double rot;
} Position;
Position start_position, current_position;

int line_KP = 19, line_KI = 0, line_KD = 0;
static double previous_error, error = 0, integral_error = 0;

void rotateRel_basic(int speed, double deltaAngle);
void rotateRel(int maxVel, double deltaAngle);
void walk_rotate(double);
void sort_array(int *array, int size);
void read_ground_sensors(int iterations);
int median_array(int sensor, int newValue);
void follow_line();
void dodge_obstacle();
void calibrate_ground_sensors();
void update_obstacle_state();

double obstacle_found_rotation;
int last_ground_sensor;
int number_of_cycles;
int laps_finished;

int forget_obstacle_cycles;


int is_it_wall_cycles, old_is_it_wall;
int following_wall_cycles;

void follow_wall() {
   double aa = 0;
   if (analogSensors.obstSensLeft > 25) {
      aa -= 5.0;   
   } else if (analogSensors.obstSensLeft > 22) {
      aa -= 2.0;
   } else if (analogSensors.obstSensLeft < 17) {
      aa += 5.0;
   } else if (analogSensors.obstSensLeft < 20) {
      aa += 2.0;
   }
   
   double propotional_error = 3.0 * aa;


   double velocity_increment = 0;
   velocity_increment += propotional_error;

   setVel2(30 + velocity_increment, 
           30 - velocity_increment);
}

bool careful_movement = false;
int main(void)
{
   initPIC32();
#ifdef DEBUG_BLUETOOTH
   configBTUart(3, 115200); // Configure Bluetooth UART
   bt_on();     // enable bluetooth channel; printf
#endif

   closedLoopControl(true);
   setVel2(0, 0);

   printf("RMI, Robot %d\n\n", ROBOT);

   while (true) {
      readAnalogSensors();          // Fill in "analogSensors" structure
      printf("Battery Level: %d\n", analogSensors.array[BATTERY]);

      //calibrate_ground_sensors();
      while (!startButton());

      enableGroundSens();
      enableObstSens();
      current_state = START;
      ground_state = CENTERED;
      current_obstacle_state = NOT_WALL;
      next_obstacle_state = NOT_WALL;
      old = CENTER;
      getRobotPos(&start_position.x, &start_position.y, &start_position.rot);
      number_of_cycles = 0;
      laps_finished = 0;      
      careful_movement = false;

      is_it_wall_cycles = 0;
      old_is_it_wall = -1;

      while (!stopButton()) {
         readAnalogSensors();          // Fill in "analogSensors" structure
         read_ground_sensors(1);

         //follow_wall();
         //continue;

         if (old != current_obstacle_state) {
            printf("%d\n", current_obstacle_state);
            old = current_obstacle_state;   
         }

         if (old_is_it_wall != is_it_wall_cycles) {
            printf("It is wall: %d\n", is_it_wall_cycles);
            old_is_it_wall = is_it_wall_cycles;
         }

         careful_movement = (analogSensors.obstSensFront <= 25 && current_state == FOLLOW_LINE);
         if (analogSensors.obstSensFront <= 10 && current_state != OBSTACLE) {
            current_state = OBSTACLE;
            next_obstacle_state = CENTER;
            update_obstacle_state();
            is_it_wall_cycles = 0;
            following_wall_cycles = 0;
            printf("Found wall!\n");
         }

         /*if (analogSensors.obstSensFront < 20 && current_state != OBSTACLE) {
            
         }*/
         /*if (analogSensors.obstSensFront < 24 && current_state != OBSTACLE &&
               forget_obstacle_cycles >= 200) {
            current_state = OBSTACLE;
            obstacle_found_rotation = current_position.rot;
         }*/
         

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
               dodge_obstacle();
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
         for (k = 0; k < LAP_BUFFER_CHECK; k++) {
            if (ground_buffer[k][G_ML])
               left = true;
            if (ground_buffer[k][G_C])
               center = true;
            if (ground_buffer[k][G_MR])
               right = true;
         }

         forget_obstacle_cycles++;
         number_of_cycles++;
         if (left && center && right && number_of_cycles >= LAP_MIN_CYCLES &&
               abs(current_position.x - start_position.x) <= LAP_DIFF_X &&
               abs(current_position.y - start_position.y) <= LAP_DIFF_Y) {
            laps_finished++;
#ifdef DEBUG_VERBOSE
            printf("\n\n> LAP FINISHED! <\n\n");
#endif
            if (laps_finished == LAP_TURN)
               rotateRel(100, -M_PI);
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
}

void update_obstacle_state() {
   if (next_obstacle_state == NOT_WALL ||
      abs(next_obstacle_state - current_obstacle_state) == 1)
      current_obstacle_state = next_obstacle_state;
}

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
      ground_sensor = readLineSensors(ground_gain);

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

void calibrate_ground_sensors()
{
   int i;
   bool calibrated;
   printf("Started calibration\n");
   do {
      calibrated = true;
      for (i = 0; i < GROUND_HISTORY; i++) {
         read_ground_sensors(1);
      }

      for (i = 0; i < GROUND_HISTORY; i++) {
         if (ground_buffer[i][G_R] != COLOR_LINE ||
            ground_buffer[i][G_C] != COLOR_LINE) {
            calibrated = false;
            ground_gain--;
            break;
         }
      }

      //printf("ground_gain: %d\n", ground_gain);      
   } while (!calibrated);

   while (1) {}
}

int count_cycles_obstacle = 0; 

int state = 0;
double last_current = -100;

int blank_cycles;
void dodge_obstacle()
{
   if (current_obstacle_state == CENTER) { 
      setVel2(30, 30);
      if (analogSensors.obstSensFront <= 25)
         current_state = FOLLOW_LINE;
      if (analogSensors.obstSensFront <= 10)
         next_obstacle_state = FOLLOWING_WALL;
   } else if (current_obstacle_state == FOLLOWING_WALL) {
      setVel2(0, 0);
   } else if (current_obstacle_state == CORNER) {
      blank_cycles++;
      if (blank_cycles >= 200) {
         rotateRel(100, M_PI / 2);
         next_obstacle_state = FOLLOWING_WALL;
      }
   }

   /*following_wall_cycles++;
   if (analogSensors.obstSensFront > 25 && current_obstacle_state == FIRST_CENTER) {
      current_state = FOLLOW_LINE;
      next_obstacle_state = NOT_WALL;
   }

   if (analogSensors.obstSensFront <= 25) {
      is_it_wall_cycles++;
      setVel2(30, -30);

      if (is_it_wall_cycles >= 10 && current_obstacle_state == FIRST_CENTER) {
         printf("IT IS A WALL!\n");
         next_obstacle_state = CENTER;
      }
   } else if (analogSensors.obstSensLeft > 0 && analogSensors.obstSensLeft < 30) {
      next_obstacle_state = FOLLOWING_WALL;
      follow_wall();
   } else {
      next_obstacle_state = CORNER;
      setVel2(30, 30);
   }

   if (next_obstacle_state == CORNER && current_obstacle_state == FOLLOWING_WALL) {
      blank_cycles = 0;
   }

   if (following_wall_cycles > 600 && (current_obstacle_state == FOLLOWING_WALL || current_obstacle_state == CORNER)) {
      int sum = 0, i, k;
      for (i = 0; i < 5; i++) {
         for (k = 0; k < 10; k++) {
            sum = ground_buffer[k][i] == COLOR_LINE ? sum + 1 : sum;
         }
      }
      if (sum > 15)
         current_state = FOLLOW_LINE;
   }*/
   //if (current_obstacle_state == FOLLOWING_WALL || current_obstacle_state == CORNER)

   update_obstacle_state();
   /*if (last_current != current_position.rot && state >= 4) {
      last_current = current_position.rot;
      printf("%6.2f --- %6.2f\n", current_position.rot, obstacle_found_rotation);
   }

   if (state % 2 == 0) {
      
      } else {
         count_cycles_obstacle++;
         if (count_cycles_obstacle >= 400) {
            state++;
         }
         setVel2(15, 15);
      }
   } else {
      rotateRel(30, M_PI / 2);
      state++;
         count_cycles_obstacle = 0;
   }

   if (state >= 4) {
      int i, j;
      int sum = 0;
      for (i = 0; i < 5; i++) {
         for (j = 0; j < 20; j++) {
            sum = ground_buffer[j][i] == COLOR_LINE ? sum + 1 : sum;
         }
      }

      if (sum >= 7) {
         forget_obstacle_cycles = 0;
         current_state = FOLLOW_LINE;
      }
   }
   //printf("Dodging...\n");*/
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
         printf("left: %5d, right: %5d\n", left, right);
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
         sum -= 3.0;
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
         sum += 3.0;
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
   double base_speed_modifier = careful_movement ? (BASE_SPEED + STATIC_INCREASE) / 2 : BASE_SPEED + STATIC_INCREASE;
   setVel2(base_speed_modifier + velocity_increment, 
           base_speed_modifier - velocity_increment);
}

void walk_rotate(double deltaAngle)
{
   double x, y, t;
   double targetAngle;
   double error;
   int cmdVel;
   
   int kp = 15;
   //int ki = 0;
   //int kd = 0;
   getRobotPos(&x, &y, &t);
   
   targetAngle = normalizeAngle(t + deltaAngle);
   error = normalizeAngle(targetAngle - t);
   
      
   getRobotPos(&x, &y, &t);
   error = normalizeAngle(targetAngle - t);

   cmdVel = kp * error;

   setVel2(40 - cmdVel, 40 + cmdVel); //walks 40 minimum
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
