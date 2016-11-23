//
// Created by myrddin on 23/11/16.
//

#ifndef RAZERNAGA_CONSTS_H
#define RAZERNAGA_CONSTS_H

// Following defines are related with the calibration of the position, maximum angle to calibrate and maximum distance
// between the difference from left and right obstacle sensors to calibrate
#define POSITION_RESET_ANGLE    2.0
#define POSITION_RESET_OBSTACLE 0.05
#define POSITION_RESET_MIN_DIFF 0.5
// Basic parameters defined for the robot
#define SQUARE_SIZE             2.0
#define BASE_SPEED              0.13
#define MAX_SPEED               0.15
// Normalization factor used for the controller that rotates, smaller is faster to rotate and big is slower
#define NORM_FACTOR             20.0
#define OBSTACLE_LIMIT          0.4
// Following parameters are related with the controller to follow a specific point
#define INTEGRAL_CLIP           MAX_SPEED - BASE_SPEED
#define KP                      0.2
#define KD                      0.2
#define KI                      0.0

#endif //RAZERNAGA_CONSTS_H
