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
    double x_;
    double y_;
    double out_left;
    double out_right;
    // Decided to use compass instead of generating a rotational
    //double previous_theta;
};


#endif //RAZERNAGA_POSITION_H
