//
// Created by myrddin on 29/10/16.
//

#ifndef RAZERNAGA_POSITION_H
#define RAZERNAGA_POSITION_H

#include <iostream>
#include <tuple>

#define M_X(tuple)  std::get<0>(tuple)
#define M_Y(tuple)  std::get<1>(tuple)

class Position {
public:
    Position();

    const std::tuple<double, double>& get_tuple() const;
    double x() const;
    double y() const;

    void update_position(const double& theta, const double& in_left, const double& in_right);
    friend std::ostream& operator<<(std::ostream& os, const Position& position);
private:
    std::tuple<double, double> position_, movement_output_;
    // Decided to use compass instead of generating a rotational
    //double previous_theta;
};

#endif //RAZERNAGA_POSITION_H
