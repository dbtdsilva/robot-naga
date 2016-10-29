//
// Created by myrddin on 29/10/16.
//

#include "Position.h"

#include <cmath>

using namespace std;

Position::Position() : x_(0), y_(0){

}

double Position::x() const {
    return x_;
}

double Position::y() const {
    return y_;
}

void Position::update_position(double compass, double next_speed_left, double next_speed_right) {
    double radians = compass * M_PI / 180.0;

    // TODO: Should consider OUT speeds instead of IN speeds
    // Formula: out_speed = (0.5 * in_speed + 0.5 * prev_outspeed) * noise
    // The thing is... Motors have a constant error on motors and there is no way that I can discover that noise,
    // Robot will get lost...
    double linear = (next_speed_left + next_speed_right) / 2.0;
    double rotational = (next_speed_right - next_speed_left);

    x_ = x_ + linear * cos(radians + rotational);
    y_ = y_ + linear * sin(radians + rotational);
}