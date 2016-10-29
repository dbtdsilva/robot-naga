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
        RazerNaga(argc, argv, position, host, {0.0, 60.0, -60.0, 180.0})
{
}

RazerNaga::RazerNaga(int &argc, char* argv[], int position, string host, vector<double> ir_sensor_angles) :
        QApplication(argc,argv), name_("RazerNaga"), grid_position_(position), host_(host),
        ir_sensor_angles_(ir_sensor_angles)
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

    cout << sensors_ << endl;

    DriveMotors(0.1, -0.1);
}