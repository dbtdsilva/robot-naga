//
// Created by myrddin on 29/10/16.
//

#ifndef RAZERNAGA_POSITION_H
#define RAZERNAGA_POSITION_H


class Position {
public:
    Position();

    double x() const;
    double y() const;

    void update_position(double compass, double next_speed_left, double next_speed_right);
private:
    double x_;
    double y_;
};


#endif //RAZERNAGA_POSITION_H
