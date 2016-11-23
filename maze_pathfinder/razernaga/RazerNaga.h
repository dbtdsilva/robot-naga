//
// Created by myrddin on 28/10/16.
//

#ifndef RAZERNAGA_RAZERNAGA_H
#define RAZERNAGA_RAZERNAGA_H

#include <string>
#include <vector>
#include <QtGui/QApplication>
#include <functional>
#include <memory>
#include <ctime>
#include <tuple>
#include "Sensors.h"
#include "Position.h"
#include "Map.h"

// Easier way to access the tuple from the motors speed
#define M_MOTOR_LEFT(motors_tuple)      std::get<0>(motors_tuple)
#define M_MOTOR_RIGHT(motors_tuple)     std::get<1>(motors_tuple)

// Following defines are related with the calibration of the position, maximum angle to calibrate and maximum distance
// between the difference from left and right obstacle sensors to calibrate
#define POSITION_RESET_ANGLE    2.0
#define POSITION_RESET_OBSTACLE 0.05
// Basic parameters defined for the robot
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

class RazerNaga : public QApplication {
    Q_OBJECT
public:
    RazerNaga(int &argc, char* argv[]);
    RazerNaga(int &argc, char* argv[], int grid_position);
    RazerNaga(int &argc, char* argv[], int grid_position, std::string host);
    RazerNaga(int &argc, char* argv[], int grid_position, std::string host, std::vector<double> ir_sensor_angles);

signals:
    void cycle_ended();

public slots:
    void take_action();
    void cycle_ended_action();

private:
    void set_motors_speed(const double& motor_left, const double& motor_right);
    void apply_motors_speed();
    void follow_path();
    void retrieve_map();
    void recalibrate_position();
    bool rotate_to_point(std::tuple<double, double>&);
    static double angle_between_two_points(const std::tuple<double, double>&, const std::tuple<double, double>&);
    static double distance_between_two_points(const std::tuple<double, double>&, const std::tuple<double, double>&);
    static double normalize_angle(const double&);

    enum State { STOPPED, EXPLORING_OBJECTIVE, EXPLORING_FINAL_PATH, RETURN_TO_OBJECTIVE,
        PREPARE_TO_RETURN, RETURN_TO_START, FINISHED };

    const std::string name_;
    const int grid_position_;
    const std::string host_;
    const std::vector<double> ir_sensor_angles_;
    Map map_;
    Sensors sensors_;
    Position position_;
    State state_;
    std::tuple<double, double> motor_speed_;
    std::vector<std::tuple<double, double>>& calculated_path_reference_;
};

#endif //RAZERNAGA_RAZERNAGA_H
