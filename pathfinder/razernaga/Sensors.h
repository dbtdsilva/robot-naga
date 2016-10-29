//
// Created by myrddin on 29/10/16.
//

#ifndef RAZERNAGA_SENSORS_H
#define RAZERNAGA_SENSORS_H

#include <vector>
#include <iostream>
#include "Filter.h"

class Sensors {
public:
    Sensors();

    void update_values();
    double get_compass() const;
    double get_obstacle_sensor(const int& id) const;
private:
    std::vector<double> obstacles_;
    int ground_;
    Filter<double> compass_;
    friend std::ostream& operator<<(std::ostream& os, const Sensors& sensors);
};

#endif //RAZERNAGA_SENSORS_H
