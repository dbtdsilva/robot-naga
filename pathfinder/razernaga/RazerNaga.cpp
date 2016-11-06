//
// Created by myrddin on 28/10/16.
//

#include "RazerNaga.h"
#include <iostream>
#include <libRobSock/RobSock.h>
#include <cmath>

#define INTEGRAL_CLIP   0.05
#define KP              0.1    // 0.15
#define KD              0.2     // 0.2
#define KI              0.1
#define BASE_SPEED      0.1

using namespace std;

RazerNaga::RazerNaga(int &argc, char* argv[]) : RazerNaga(argc, argv, 0)
{
}

RazerNaga::RazerNaga(int &argc, char* argv[], int position) : RazerNaga(argc, argv, position, "localhost")
{
}

RazerNaga::RazerNaga(int &argc, char* argv[], int position, string host) :
        RazerNaga(argc, argv, position, host, {90.0, 30.0, -30.0, -90.0})
{
}

RazerNaga::RazerNaga(int &argc, char* argv[], int position, string host, vector<double> ir_sensor_angles) :
        QApplication(argc,argv), name_("RazerNaga"), grid_position_(position), host_(host),
        ir_sensor_angles_(ir_sensor_angles), state_(STOPPED)
{
    if (InitRobot2(const_cast<char *>(name_.c_str()), grid_position_, &ir_sensor_angles[0],
                   const_cast<char *>(host_.c_str())) == -1) {
        throw runtime_error("Failed to connect robot");
    }
    get<0>(start_position) = -1;
    get<1>(start_position) = -1;

    get<0>(motor_speed) = 0.0;
    get<1>(motor_speed) = 0.0;
    qApp->addLibraryPath("libRobSock");
    QObject::connect((QObject *)(Link()), SIGNAL(NewMessage()), this, SLOT(take_action()));

    map_.enable_debug();
}

void RazerNaga::take_action() {
    sensors_.update_values();

    if (state_ == STOPPED && GetStartButton())
        state_ = STARTED;
    if (state_ == STOPPED)
        return;

    if (get<0>(start_position) == -1) {
        get<0>(start_position) = GetX();
        get<1>(start_position) = GetY();
    }
    double x = GetX() - get<0>(start_position);
    double y = GetY() - get<1>(start_position);
    move_front();
    if (sensors_.get_obstacle_sensor(2) > 0.8) {
        get<0>(motor_speed) = 0;
        get<1>(motor_speed) = 0;
    }

    DriveMotors(get<0>(motor_speed), get<1>(motor_speed));

    retrieve_map();
    position_.update_position(sensors_.get_compass(), get<0>(motor_speed), get<1>(motor_speed));
}

void RazerNaga::retrieve_map() {
    map_.increase_ground_counter(position_.x(), position_.y());

    const double& x = position_.x();
    const double& y = position_.y();
    long double sensor_x, sensor_y, theta, distance_measured, sensor_final_x, sensor_final_y;
    long double dx, dy;
    const int N_POINTS = 10;
    vector<double> sensor_angles = {0};//{-M_PI / 6.0, 0, M_PI / 6.0};
    for (int i = 0; i < ir_sensor_angles_.size(); i++) {
        if (ir_sensor_angles_[i] != 90 && ir_sensor_angles_[i] != -90)
            continue;
        theta = normalize_angle(sensors_.get_compass() + ir_sensor_angles_[i]) * M_PI / 180.0;
        // Calculate sensor position
        sensor_x = x + cos(theta) * 0.5;
        sensor_y = y + sin(theta) * 0.5;

        for(auto angle = sensor_angles.begin(); angle != sensor_angles.end() ; ++angle)
        {
            distance_measured = sensors_.get_obstacle_sensor(i);
            if (distance_measured < 1.0) {
                sensor_final_x = x + cos(theta + *angle) * distance_measured;
                sensor_final_y = y + sin(theta + *angle) * distance_measured;
                map_.increase_wall_counter(sensor_final_x, sensor_final_y);
            } else {
                sensor_final_x = x + cos(theta + *angle) * 1.0;
                sensor_final_y = y + sin(theta + *angle) * 1.0;
                map_.increase_ground_counter(sensor_final_x, sensor_final_y);
            }

            if (sensor_final_x == sensor_x || sensor_final_y == sensor_y) {
                cout << "skipping" << endl;
                continue;
            }
            dx = (sensor_final_x - sensor_x) / N_POINTS;
            dy = (sensor_final_y - sensor_y) / N_POINTS;
            for (int points = 0; points < N_POINTS; points++) {
                cout << sensor_x + points * dx << ", " << sensor_y + points * dy << endl;
                map_.increase_ground_counter(sensor_x + points * dx, sensor_y + points * dy);
            }
        };
    }
}

void RazerNaga::move_front() {
    double error;
    static double last_error = 0, integral_error = 0;
    double right = sensors_.get_obstacle_sensor(3);
    double center_right = sensors_.get_obstacle_sensor(2);
    error = right - 0.4 + center_right - 0.65;
    integral_error += error;
    integral_error = integral_error > INTEGRAL_CLIP ? INTEGRAL_CLIP : integral_error;
    integral_error = integral_error < -INTEGRAL_CLIP ? -INTEGRAL_CLIP : integral_error;
    double correction = KP * error + KI * integral_error + KD * (error - last_error);
    last_error = error;

    get<0>(motor_speed) = BASE_SPEED + correction;
    get<1>(motor_speed) = BASE_SPEED - correction;
}

double RazerNaga::normalize_angle(double degrees_angle)
{
    while (degrees_angle <= -180.0) degrees_angle += 2.0 * 180.0;
    while (degrees_angle > 180.0) degrees_angle -= 2.0 * 180.0;
    return degrees_angle;
}

void RazerNaga::rotate(double degrees) {
    const double TARGET_ANGLE = normalize_angle(sensors_.get_compass() + degrees);
    constexpr double NORMALIZE_FACTOR = 0.15 / 90.0;
    double speed, diff;
    do {
        sensors_.update_values();
        diff = normalize_angle(TARGET_ANGLE - sensors_.get_compass());
        speed = diff * NORMALIZE_FACTOR;
        DriveMotors(-speed, speed);
    } while(fabs(diff) > 1.0);
}