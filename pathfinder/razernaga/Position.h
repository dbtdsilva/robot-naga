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

    void update_position(double theta, double in_left, double in_right);
private:
    double out_left;
    double out_right;
    double previous_theta;
    double x_;
    double y_;
};


#endif //RAZERNAGA_POSITION_H
