//
// Created by Diogo Silva on 23/11/16.
//

#ifndef RAZERNAGA_CONSTS_H
#define RAZERNAGA_CONSTS_H

// Following defines are related with the calibration of the position, maximum angle to calibrate and maximum distance
// between the difference from left and right obstacle sensors to calibrate
#define POS_RESET_ANGLE         2.0
#define POS_RESET_OBSTACLE      0.1
#define POS_RESET_MIN_DIFF      0.5
#define POS_RESET_CYCLES        4
// Basic parameters defined for the robot
#define SQUARE_SIZE             2.0
#define BASE_SPEED              0.13
#define MAX_SPEED               0.15
// Normalization factor used for the controller that rotates, smaller is faster to rotate and big is slower
#define NORM_FACTOR             20.0
// This represent the maximum angle that the robot allows before stop the desired rotation
#define ROTATE_ANGLE_MAX        2.0
#define OBSTACLE_LIMIT          0.4
// Following parameters are related with the controller to follow a specific point
#define INTEGRAL_CLIP           MAX_SPEED - BASE_SPEED
#define KP                      0.2
#define KD                      0.2
#define KI                      0.0
// This define controls how many walls should appear
#define WALL_MARGIN             0.15

#endif //RAZERNAGA_CONSTS_H
