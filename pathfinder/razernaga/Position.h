//
// Created by myrddin on 29/10/16.
//

#ifndef RAZERNAGA_POSITION_H
#define RAZERNAGA_POSITION_H

class Position {
public:
    Position();

    const double& x() const;
    const double& y() const;

    void update_position(const double& theta, const double& in_left, const double& in_right);
private:
    double x_, y_, out_left_, out_right_;
    // Decided to use compass instead of generating a rotational
    //double previous_theta;
};

#endif //RAZERNAGA_POSITION_H
