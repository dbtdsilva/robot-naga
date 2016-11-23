//
// Created by Diogo Silva on 29/10/16.
//

#include <libRobSock/RobSock.h>
#include <libRobSock/cmeasures.h>
#include "Sensors.h"

using namespace std;

Sensors::Sensors() : obstacles_(NUM_IR_SENSORS, Filter<double>(3)), compass_(1) {
}

void Sensors::update_values() {
    ReadSensors();

    for (int obstacle_id = 0; obstacle_id < NUM_IR_SENSORS; obstacle_id++) {
        if (IsObstacleReady(obstacle_id))
            obstacles_[obstacle_id].update(1.0 / GetObstacleSensor(obstacle_id));
    }
    if (IsCompassReady())
        compass_.update(GetCompassSensor());
    if (IsGroundReady())
        ground_ = GetGroundSensor();
}

double Sensors::get_compass() const {
    return compass_.get_value();
}

double Sensors::get_obstacle_sensor(const int& id) const {
    return obstacles_[id].get_value();
}

ostream& operator<<(ostream& os, const Sensors& sensors)
{
    for (int obstacle_id = 0; obstacle_id < NUM_IR_SENSORS; obstacle_id++) {
        os << "O" << obstacle_id << ": " << sensors.obstacles_[obstacle_id].get_value() << ", ";
    }
    os << "C: " << sensors.get_compass() << ", ";
    os << "G: " << sensors.ground_;
    return os;
}