//
// Created by myrddin on 28/10/16.
//

#ifndef RAZERNAGA_RAZERNAGA_H
#define RAZERNAGA_RAZERNAGA_H

#include <string>
#include <vector>
#include <QtGui/QApplication>
#include <memory>
#include <tuple>
#include "Sensors.h"
#include "Position.h"
#include "Map.h"

typedef std::tuple<double, double> TupleDouble;

class RazerNaga : public QApplication {
    Q_OBJECT
public:
    RazerNaga(int &argc, char* argv[]);
    RazerNaga(int &argc, char* argv[], int grid_position);
    RazerNaga(int &argc, char* argv[], int grid_position, std::string host);
    RazerNaga(int &argc, char* argv[], int grid_position, std::string host, std::vector<double> ir_sensor_angles);

    void move_front();
    double walk_units(double units, double degree);
    void rotate(double degress);
    void retrieve_map();
public slots:
    void take_action();
private:
    double normalize_angle(double degrees_angle);

    enum State {
        STOPPED, STARTED
    };

    const std::string name_;
    const int grid_position_;
    const std::string host_;
    const std::vector<double> ir_sensor_angles_;

    Map map_;
    Sensors sensors_;
    Position position_;
    State state_;

    TupleDouble start_position;
    TupleDouble motor_speed;
};


#endif //RAZERNAGA_RAZERNAGA_H
