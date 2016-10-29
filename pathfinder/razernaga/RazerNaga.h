//
// Created by myrddin on 28/10/16.
//

#ifndef RAZERNAGA_RAZERNAGA_H
#define RAZERNAGA_RAZERNAGA_H

#include <string>
#include <vector>
#include <QtGui/QApplication>
#include <memory>
#include "Sensors.h"

class RazerNaga : public QApplication {
    Q_OBJECT
public:
    RazerNaga(int &argc, char* argv[]);
    RazerNaga(int &argc, char* argv[], int grid_position);
    RazerNaga(int &argc, char* argv[], int grid_position, std::string host);
    RazerNaga(int &argc, char* argv[], int grid_position, std::string host, std::vector<double> ir_sensor_angles);

public slots:
    void take_action();
private:
    const std::string name_;
    const int grid_position_;
    const std::string host_;
    const std::vector<double> ir_sensor_angles_;

    Sensors sensors_;
};


#endif //RAZERNAGA_RAZERNAGA_H
