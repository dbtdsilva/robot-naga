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
#define LAP_BUFFER_CHECK   30
#define LAP_TURN           2

// Feel free to comment any defines to disable debug
//#define DEBUG_VERBOSE
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

int ground_gain = 30;
bool ground_buffer[GROUND_HISTORY][5];
int ground_weight[5] = {-3.0, -1.0, 0.0, 1.0, 3.0};

typedef enum {START, FOLLOW_LINE, OBSTACLE} State;
typedef enum {CENTERED, LOST_LEFT, LOST_RIGHT} GroundState;
typedef enum {NOT_WALL, CENTER, ROTATE_CENTER, FOLLOWING_WALL, CORNER, FINISHING_ZONE} ObstacleState;
ObstacleState current_obstacle_state, next_obstacle_state;
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
void read_ground_sensors(int iterations);
void follow_line();
void dodge_obstacle();

double obstacle_found_rotation;
int last_ground_sensor;
int number_of_cycles;
int laps_finished;

int forget_obstacle_cycles;

int following_wall_cycles;

void follow_wall() {
   double weight = 0;
   if (analogSensors.obstSensLeft > 25) {
      weight -= 3.0;   
   } else if (analogSensors.obstSensLeft > 22) {
      weight -= 1.0;
   } else if (analogSensors.obstSensLeft < 17) {
      weight += 3.0;
   } else if (analogSensors.obstSensLeft < 20) {
      weight += 1.0;
   }
   double propotional_error = 8.0 * weight;
   double velocity_increment = propotional_error;
   setVel2(50 + velocity_increment, 50 - velocity_increment);
}

bool careful_movement = false;
int confirm_wall;
int c_wall;
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

      while (!startButton());

      enableGroundSens();
      enableObstSens();
      current_state = START;
      ground_state = CENTERED;
      current_obstacle_state = NOT_WALL;
      next_obstacle_state = NOT_WALL;
      getRobotPos(&start_position.x, &start_position.y, &start_position.rot);
      number_of_cycles = 0;
      laps_finished = 0;      
      careful_movement = false;

      confirm_wall = 0;
      c_wall = 0;

      while (!stopButton()) {
         readAnalogSensors();          // Fill in "analogSensors" structure
         read_ground_sensors(1);

         careful_movement = (analogSensors.obstSensFront <= 25 && current_state == FOLLOW_LINE);
         printf("%d\n", confirm_wall);
         if (analogSensors.obstSensFront <= 11 && current_state != OBSTACLE) {
            c_wall++;
         } else {
            c_wall = 0;
         }
         if (c_wall >= 20 && current_state != OBSTACLE) {
            c_wall = 0;
            confirm_wall = 0;
            current_state = OBSTACLE;
            current_obstacle_state = CENTER;
            following_wall_cycles = 0;
            printf("Found wall!\n");
         }
         

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

         getRobotPos(&current_position.x, &current_position.y, &current_position.rot);
#ifdef DEBUG_POSITION
         printf("Start  Position: %6.2f, %6.2f (%6.2f)\n", 
            start_position.x, start_position.y, start_position.rot);
         printf("Actual Position: %6.2f, %6.2f (%6.2f)\n", 
            current_position.x, current_position.y, current_position.rot);
         printf("Cycles: %d\n", number_of_cycles);
#endif
         forget_obstacle_cycles++;
         number_of_cycles++;
         led(3, abs(current_position.x - start_position.x) <= LAP_DIFF_X);
         led(0, abs(current_position.y - start_position.y) <= LAP_DIFF_Y);

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
   int k, id, i, ground_sensor;
   iterations = iterations > GROUND_HISTORY ? GROUND_HISTORY : iterations;
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
      printInt(ground_sensor & 0x0000001F, 2 | 5 << 16);asdasd   // System call
      printf("\n");
#endif

      ground_buffer[i][G_ML] = ((ground_sensor & 0x00000010) >> 4);
      ground_buffer[i][G_L] = ((ground_sensor & 0x00000008) >> 3);
      ground_buffer[i][G_C] = ((ground_sensor & 0x00000004) >> 2);
      ground_buffer[i][G_R] = ((ground_sensor & 0x00000002) >> 1);
      ground_buffer[i][G_MR] = (ground_sensor & 0x00000001);
   }
}

int count_cycles_obstacle = 0; 

int state = 0;
double last_current = -100;

int blank_cycles;
int ncorners;
void dodge_obstacle()
{
   printf("State: %d, Confirm_Wall: %d\n", current_obstacle_state, confirm_wall);
   if (current_obstacle_state == CENTER) { 
      ncorners = 0; 
      setVel2(30, 30);
      if (analogSensors.obstSensFront > 25)
         current_state = FOLLOW_LINE;
      if (analogSensors.obstSensFront <= 11) {
         setVel2(0, 0);
         confirm_wall++;
         if (confirm_wall >= 20) {
            current_obstacle_state = ROTATE_CENTER;
            confirm_wall = 0;
         }
      }
   } else if (current_obstacle_state == ROTATE_CENTER) {
      rotateRel(40, -M_PI / 2);
      current_obstacle_state = FOLLOWING_WALL;
      confirm_wall = 0;
   } else if (current_obstacle_state == FOLLOWING_WALL) {
      if (analogSensors.obstSensLeft > 60) {
         confirm_wall++;
      }

      if (confirm_wall >= 10) {
         setVel2(0, 0);
         current_obstacle_state = CORNER;
         blank_cycles = 0;
      } else {
         follow_wall();
      }
   } else if (current_obstacle_state == CORNER) {
      setVel2(40, 40);
      blank_cycles++;
      if (blank_cycles >= 180) {
         rotateRel(100, M_PI / 2);
         current_obstacle_state = FOLLOWING_WALL;
         confirm_wall = 0;
         ncorners++;
      }
   } else if (current_obstacle_state == FINISHING_ZONE) {
      blank_cycles++;
      if (blank_cycles >= 35) {
         current_obstacle_state = NOT_WALL;
         current_state = FOLLOW_LINE;
         rotateRel(40, -M_PI / 2);
      }
   }

   if (ncorners >= 2) {
      int sum = 0, k, j;
      for (k = 0; k < 10; k++) {
         for (j = 0; j < 5; j++) {
            sum += ground_buffer[k][j];
         }
      }
      if (sum > 20) {
         current_obstacle_state = FINISHING_ZONE;
         blank_cycles = 0;
      }
   }
}

void follow_line() 
{
   double weight = 0;
   int element_influence = 0, i;
   if (!ground_buffer[0][G_ML] && !ground_buffer[0][G_L] && !ground_buffer[0][G_C] && 
      !ground_buffer[0][G_R] && !ground_buffer[0][G_MR]) {
      int left = 0, right = 0;
      if (ground_state == CENTERED) {
         for (i = 0; i < GROUND_HISTORY; i++) {
            if (ground_buffer[i][G_MR])
               right += (GROUND_HISTORY - i);
            if (ground_buffer[i][G_ML])
               left += (GROUND_HISTORY - i);
         }
      }
      if (left > GROUND_HISTORY * 4 || right > GROUND_HISTORY * 4) {
         ground_state = left > right ? LOST_LEFT : LOST_RIGHT;
      }

      if (ground_state == LOST_RIGHT) {
         weight += 9.0;
         element_influence += 2;
      } else if (ground_state == LOST_LEFT) {
         weight -= 9.0;
         element_influence += 2;
      }
   } else {
      ground_state = CENTERED;
      for (i = 0; i < SIZE_GROUND_SENS; i++) {
         if (ground_buffer[0][i]) {
            element_influence = i == G_C ? element_influence + 1 : element_influence + 2;
            weight += ground_weight[i];
         }
      }
   }

   double error = element_influence == 0 ? 0 : (weight * 2) / element_influence;
   double propotional_error = line_KP * error;
   double velocity_increment = propotional_error;

#ifdef DEBUG_PID
   printf("Increment: %6.2f, P: %6.2f, Others: %4.2f - %4.2f (%d)\n", 
      velocity_increment, propotional_error, error, median_sum, elements_weight);
#endif
   double base_speed_modifier = careful_movement ? BASE_SPEED / 2 : BASE_SPEED;
#if STATIC_INCREASE != 0
   if (error == 0)
      base_speed_modifier += STATIC_INCREASE;
#endif
   setVel2(base_speed_modifier + velocity_increment, base_speed_modifier - velocity_increment);
}

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

      cmdVel = (int)((40 * error) + (integral * 5));
      cmdVel = cmdVel > maxVel ? maxVel : cmdVel;
      cmdVel = cmdVel < -maxVel ? -maxVel : cmdVel;

      setVel2(-cmdVel, cmdVel);
   } while (fabs(error) > 0.01);
   setVel2(0, 0);
}
