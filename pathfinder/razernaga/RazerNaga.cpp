//
// Created by myrddin on 28/10/16.
//

#include "RazerNaga.h"
#include <iostream>
#include <libRobSock/RobSock.h>

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
                   const_cast<char *>(host_.c_str())) == -1)
    {
        throw runtime_error("Failed to connect robot");
    }
    qApp->addLibraryPath("libRobSock");
    QObject::connect((QObject *)(Link()), SIGNAL(NewMessage()), this, SLOT(take_action()));
}

void RazerNaga::take_action() {
    sensors_.update_values();

    if (state_ == STOPPED && GetStartButton())
        state_ = STARTED;
    if (state_ == STOPPED)
        return;

    cout << "Calcula: " << position_.x() << ", " << position_.y(); // << sensors_.get_compass() << endl;

    if (start_x == -1)
    {
        start_x = GetX();
        start_y = GetY();
    }
    double x = GetX() - start_x;
    double y = GetY() - start_y;
    cout << ", Correct: " << x << ", " << y << endl;
    //move_front();


    DriveMotors(0.1, 0.1);
    position_.update_position(sensors_.get_compass(), 0.1, 0.1);
}

void RazerNaga::move_front() {
    double KP = 0.1, KD = 0.0, KI = 0.0;
    double INTEGRAL_CLIP = 0.05;
    double error;
    static double last_error = 0, integral_error = 0;

    double left = sensors_.get_obstacle_sensor(0);
    double right = sensors_.get_obstacle_sensor(3);
    error = left - right;
    last_error = error;
    integral_error += error;
    integral_error = integral_error > INTEGRAL_CLIP ? INTEGRAL_CLIP : integral_error;
    integral_error = integral_error < -INTEGRAL_CLIP ? -INTEGRAL_CLIP : integral_error;
    double correction = KP * error + KI * integral_error + KD * (error - last_error);
    DriveMotors(0.1 + correction, 0.1 - correction);
}