//
// Created by myrddin on 29/10/16.
//

#include "Position.h"

#include <cmath>

using namespace std;

Position::Position() : x_(0), y_(0), out_left(0), out_right(0) {

}

const double& Position::x() const {
    return x_;
}

const double& Position::y() const {
    return y_;
}

void Position::update_position(double theta, double in_left, double in_right) {
    double theta_radians = theta * M_PI / 180.0;

    double out_left_t = in_left * 0.5 + out_left * 0.5;
    double out_right_t = in_right * 0.5 + out_right * 0.5;

    double linear = (out_left_t + out_right_t) / 2.0;
    double rotational = (out_right_t - out_left_t);

    x_ = x_ + linear * cos(previous_theta);
    y_ = y_ + linear * sin(previous_theta);

    out_left = out_left_t;
    out_right = out_right_t;

    previous_theta = theta_radians + rotational;
}