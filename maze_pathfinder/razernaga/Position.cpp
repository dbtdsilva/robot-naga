//
// Created by Diogo Silva on 29/10/16.
//

#include "Position.h"

#include <math.h>
#include "Consts.h"

using namespace std;

Position::Position() : position_(0.0, 0.0), movement_output_(0.0, 0.0) {

}

const tuple<double, double>& Position::get_tuple() const {
    return position_;
}

double Position::x() const {
    return M_X(position_);
}

double Position::y() const {
    return M_Y(position_);
}

void Position::reset_y() {
    // It only resets if the position trying to reset is near the SQUARE_SIZE
    double value = M_Y(position_) / SQUARE_SIZE;
    if (fabs(value - round(value)) >= POSITION_RESET_MIN_DIFF / SQUARE_SIZE) return;

    cout << "Resetting Y from " << M_X(position_);
    M_Y(position_) = round(M_Y(position_) / SQUARE_SIZE) * SQUARE_SIZE;
    cout << " to " << M_Y(position_)  << endl;
}

void Position::reset_x() {
    // It only resets if the position trying to reset is near the SQUARE_SIZE
    double value = M_X(position_) / SQUARE_SIZE;
    if (fabs(value - round(value)) >= POSITION_RESET_MIN_DIFF / SQUARE_SIZE) return;

    cout << "Resetting X from " << M_X(position_);
    M_X(position_) = round(M_X(position_) / SQUARE_SIZE) * SQUARE_SIZE;
    cout << " to " << M_X(position_) << endl;
}

void Position::update_position(const double& theta, const double& in_left, const double& in_right) {
    double theta_radians = theta * M_PI / 180.0;

    double out_left_t = in_left * 0.5 + get<0>(movement_output_) * 0.5;
    double out_right_t = in_right * 0.5 + get<1>(movement_output_) * 0.5;

    double linear = (out_left_t + out_right_t) / 2.0;
    // Decided to use compass instead of generating a rotational
    //double rotational = (out_right_t - out_left_t);

    M_X(position_) = M_X(position_) + linear * cos(theta_radians);
    M_Y(position_) = M_Y(position_) + linear * sin(theta_radians);

    get<0>(movement_output_) = out_left_t;
    get<1>(movement_output_) = out_right_t;

    //previous_theta += rotational;
    //if (previous_theta > M_PI) previous_theta -= (2.0 * M_PI);
    //else if (previous_theta < -M_PI) previous_theta += (2.0 * M_PI);
}

ostream& operator<<(ostream& os, const Position& position) {
    os << get<0>(position.get_tuple()) << ", " << get<1>(position.get_tuple());
}