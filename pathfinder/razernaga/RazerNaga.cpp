//
// Created by myrddin on 28/10/16.
//

#include "RazerNaga.h"
#include <iostream>
#include <libRobSock/RobSock.h>
#include <cmath>

#define INTEGRAL_CLIP   0.05
#define BASE_SPEED      0.1
#define MAX_SPEED       0.15

using namespace std;

RazerNaga::RazerNaga(int &argc, char* argv[]) : RazerNaga(argc, argv, 0) {
}
RazerNaga::RazerNaga(int &argc, char* argv[], int position) : RazerNaga(argc, argv, position, "localhost") {
}
RazerNaga::RazerNaga(int &argc, char* argv[], int position, string host) :
        RazerNaga(argc, argv, position, host, {60.0, 0.0, -60.0, 180.0}) {
}
RazerNaga::RazerNaga(int &argc, char* argv[], int position, string host, vector<double> ir_sensor_angles) :
        QApplication(argc,argv), name_("RazerNaga"), grid_position_(position), host_(host), start_position(0,0),
        motor_speed(0.0, 0.0), ir_sensor_angles_(ir_sensor_angles), state_(STOPPED),
        calculated_path_reference_(map_.get_calculated_path())
{
    if (InitRobot2(const_cast<char *>(name_.c_str()), grid_position_, &ir_sensor_angles[0],
                   const_cast<char *>(host_.c_str())) == -1) {
        throw runtime_error("Failed to connect robot");
    }
    qApp->addLibraryPath("libRobSock");
    QObject::connect((QObject *)(Link()), SIGNAL(NewMessage()), this, SLOT(take_action()));
    QObject::connect(this, SIGNAL(cycle_ended()), this, SLOT(cycle_ended_action()), Qt::AutoConnection);
    map_.enable_debug();
}

void RazerNaga::cycle_ended_action() {
    map_.render_map();
}

vector<tuple<double, double>> targets = {tuple<double,double>(2, 0), tuple<double, double>(2, 2),
                                         tuple<double, double>(4, 2), tuple<double, double>(4, 4),
                                         tuple<double, double>(6, 4), tuple<double, double>(6, 6),
                                         tuple<double, double>(4, 6), tuple<double, double>(6, 6),
                                         tuple<double, double>(6, 4), tuple<double, double>(12, 4),
                                         tuple<double, double>(12, 2), tuple<double, double>(14, 2),
                                         tuple<double, double>(14, 0), tuple<double, double>(10, 0),
                                         tuple<double, double>(10, -2), tuple<double, double>(2, -2),
                                         tuple<double, double>(10, -2), tuple<double, double>(10, -4),
                                         tuple<double, double>(14, -4), tuple<double, double>(14, -6),
                                         tuple<double, double>(8, -6), tuple<double, double>(8, -4),
                                         tuple<double, double>(0, -4), tuple<double, double>(0, -2),
                                         tuple<double, double>(0, -4), tuple<double, double>(-2, -4),
                                         tuple<double, double>(0, -4), tuple<double, double>(0, -6),
                                         tuple<double, double>(6, -6), tuple<double, double>(0, -6),
                                         tuple<double, double>(0, -4), tuple<double, double>(8, -4),
                                         tuple<double, double>(8, -6), tuple<double, double>(14, -6),
                                         tuple<double, double>(14, -4), tuple<double, double>(16, -4),
                                         tuple<double, double>(16, 0), tuple<double, double>(20, 0)};
void RazerNaga::take_action() {
    sensors_.update_values();
    retrieve_map();

    long double distance;
    switch (state_) {
        case STOPPED:
            if (GetStartButton()) state_ = EXPLORING;
            break;
        case EXPLORING:
            if (calculated_path_reference_.size() == 0)
                map_.set_target_nearest_exit();
            //move_right();
            move_to_the_exit();

            if (GetGroundSensor() != -1) {
                map_.set_target_starter_area();
                rotate(180);
                SetVisitingLed(true);
                SetReturningLed(true);
                state_ = RETURNING;
            }
            break;
        case RETURNING:
            move_front(true);
            distance = sqrt(pow(position_.y(), 2) + pow(position_.x(), 2));
            if (fabs(distance) < 0.3)
                state_ = FINISHED;
            break;
        case FINISHED:
            get<0>(motor_speed) = 0;
            get<1>(motor_speed) = 0;
            Finish();
            break;
    }
    //cout << position_.x() << ", " << position_.y() << ", c:, " << GetX() - get<0>(start_position) <<  ", " <<
    //     GetY() - get<1>(start_position) << ", " << GetDir() << endl;
    //printf("L: %4.2f, C: %4.2f, R: %4.2f\n", sensors_.get_obstacle_sensor(0), sensors_.get_obstacle_sensor(1),
    //       sensors_.get_obstacle_sensor(2));
    apply_motors_speed();
}

void RazerNaga::move_to_the_exit() {
    tuple<double,double> source(position_.x(), position_.y());
    tuple<double,double> dst;
    do {
        if (targets.size() == 0) {
            set_motors_speed(0, 0);
            return;
        }
        dst = targets.front();
        if (distance_between_two_points(source, dst) < 0.2) {
            targets.erase(targets.begin());
        } else
            break;
    } while (true);

    double error;
    static double last_error = 0, integral_error = 0;
    double right = sensors_.get_obstacle_sensor(2);
    double center_right = sensors_.get_obstacle_sensor(3);

    error = angle_between_two_points(source, dst) + sensors_.get_compass();
    if (error > 180.0)
        error -= 360.0;
    if (error < -180)
        error += 360.0;
    error = error / 40.0; // Normalization factor
    /*if (sensors_.get_obstacle_sensor(1) < 0.6) {
        error -= 2;
    }*/
    integral_error += error;
    integral_error = integral_error > INTEGRAL_CLIP ? INTEGRAL_CLIP : integral_error;
    integral_error = integral_error < -INTEGRAL_CLIP ? -INTEGRAL_CLIP : integral_error;
    double correction = 0.2 * error + 0.0 * integral_error + 0.2 * (error - last_error);
    last_error = error;

    set_motors_speed(BASE_SPEED + correction, BASE_SPEED - correction);
}

void RazerNaga::move_front(bool moving_back) {
    double error;
    static double last_error = 0, integral_error = 0;
    double right = sensors_.get_obstacle_sensor(2);
    double left = sensors_.get_obstacle_sensor(0);
    if (right > 0.6 && left > 0.6) {
        error = moving_back ? -1 * sensors_.get_compass() / 1000.0 : sensors_.get_compass() / 40.0;
    } else if (left > 0.7) {
        error = right - 0.46;
    } else if (right > 0.7) {
        error = -left + 0.46;
    } else {
        error = right - left;
    }

    if (sensors_.get_obstacle_sensor(1) < 0.6) {
        error -= 6.0;
    }

    //printf("L: %4.2f C: %4.2f R: %4.2f - E: %4.2f\n", sensors_.get_obstacle_sensor(0),
    //       sensors_.get_obstacle_sensor(1), sensors_.get_obstacle_sensor(2), error);
    //}
    //if (sensors_.get_obstacle_sensor(1) < 0.6) {
    //    error -= 2;
    //}
    integral_error += error;
    integral_error = integral_error > INTEGRAL_CLIP ? INTEGRAL_CLIP : integral_error;
    integral_error = integral_error < -INTEGRAL_CLIP ? -INTEGRAL_CLIP : integral_error;
    double correction = 0.05 * error + 0.0 * integral_error + 0.2 * (error - last_error);
    last_error = error;

    set_motors_speed(BASE_SPEED + correction, BASE_SPEED - correction);
}

void RazerNaga::move_right() {
    double error;
    static double last_error = 0, integral_error = 0;
    double right = sensors_.get_obstacle_sensor(2);
    double center_right = sensors_.get_obstacle_sensor(3);
    error = (right - 0.47) + (center_right - 0.65);
    if (sensors_.get_obstacle_sensor(1) < 0.6) {
        error -= 2;
    }
    integral_error += error;
    integral_error = integral_error > INTEGRAL_CLIP ? INTEGRAL_CLIP : integral_error;
    integral_error = integral_error < -INTEGRAL_CLIP ? -INTEGRAL_CLIP : integral_error;
    double correction = 0.4 * error + 0.0 * integral_error + 0.2 * (error - last_error);
    last_error = error;

    set_motors_speed(BASE_SPEED + correction, BASE_SPEED - correction);
}

void RazerNaga::rotate(double degrees) {
    const double TARGET_ANGLE = normalize_angle(sensors_.get_compass() + degrees);
    constexpr double NORMALIZE_FACTOR = 0.15 / 90.0;
    double speed, diff;
    do {
        sensors_.update_values();
        retrieve_map();
        diff = normalize_angle(TARGET_ANGLE - sensors_.get_compass());
        speed = diff * NORMALIZE_FACTOR;

        set_motors_speed(-speed, speed);
        apply_motors_speed();
    } while(fabs(diff) > 1.0);
}

void RazerNaga::retrieve_map() {
    //map_.increase_ground_counter(position_.x(), position_.y(), 0.01);
    map_.increase_visited_counter(position_.x(), position_.y());

    const double& x = position_.x();
    const double& y = position_.y();
    long double sensor_x, sensor_y, theta, distance_measured, sensor_final_x, sensor_final_y;
    long double dx, dy;
    const int N_POINTS = 5;
    vector<double> sensor_angles = {-M_PI / 6.0, 0, M_PI / 6.0};
    for (unsigned int i = 0; i < ir_sensor_angles_.size(); i++) {
        theta = normalize_angle(sensors_.get_compass() + ir_sensor_angles_[i]) * M_PI / 180.0;
        sensor_x = x + cos(theta) * 0.5;
        sensor_y = y + sin(theta) * 0.5;

        for(auto angle = sensor_angles.begin(); angle != sensor_angles.end() ; ++angle) {
            distance_measured = sensors_.get_obstacle_sensor(i);
            if (distance_measured < 1.0) {
                sensor_final_x = sensor_x + cos(theta + *angle) * distance_measured;
                sensor_final_y = sensor_y + sin(theta + *angle) * distance_measured;
                map_.increase_wall_counter(sensor_final_x, sensor_final_y);
            } else {
                sensor_final_x = sensor_x + cos(theta + *angle) * 1.0;
                sensor_final_y = sensor_y + sin(theta + *angle) * 1.0;
                map_.increase_ground_counter(sensor_final_x, sensor_final_y);
            }

            dx = (sensor_final_x - sensor_x) / N_POINTS;
            dy = (sensor_final_y - sensor_y) / N_POINTS;
            for (int points = 0; points < N_POINTS; points++) {
                map_.increase_ground_counter(sensor_x + points * dx, sensor_y + points * dy);
            }
        }
    }
}

void RazerNaga::set_motors_speed(double motor_left, double motor_right) {
    get<0>(motor_speed) = motor_left;
    if (get<0>(motor_speed) > MAX_SPEED)
        get<0>(motor_speed) = 0.15;
    if (get<0>(motor_speed) < -MAX_SPEED)
        get<0>(motor_speed) = -0.15;

    get<1>(motor_speed) = motor_right;
    if (get<1>(motor_speed) > MAX_SPEED)
        get<1>(motor_speed) = 0.15;
    if (get<1>(motor_speed) < -MAX_SPEED)
        get<1>(motor_speed) = -0.15;
}
void RazerNaga::apply_motors_speed() {
    DriveMotors(get<0>(motor_speed), get<1>(motor_speed));
    if (!GetBumperSensor())
        position_.update_position(sensors_.get_compass(), get<0>(motor_speed), get<1>(motor_speed));
    cycle_ended();
}

double RazerNaga::angle_between_two_points(std::tuple<double, double>& source, std::tuple<double, double>& target) {
    double opposite = -(get<1>(target) - get<1>(source));
    double adjacent = get<0>(target) - get<0>(source);

    double value;
    if (adjacent == 0) {
        value = opposite > 0 ? M_PI / 2.0 : -M_PI / 2.0;
    } else {
        // tan(theta) = theta + K * M_PI
        value = atan(opposite / adjacent);
        if (adjacent < 0)
            value += M_PI;
    }
    return normalize_angle((value * 180.0) / M_PI);
}

double RazerNaga::distance_between_two_points(std::tuple<double, double>& source, std::tuple<double, double>& target) {
    return sqrt(pow(get<1>(target) - get<1>(source), 2) + pow(get<0>(target) - get<0>(source), 2));
}

double RazerNaga::normalize_angle(double degrees_angle) {
    while (degrees_angle <= -180.0) degrees_angle += 2.0 * 180.0;
    while (degrees_angle > 180.0) degrees_angle -= 2.0 * 180.0;
    return degrees_angle;
}