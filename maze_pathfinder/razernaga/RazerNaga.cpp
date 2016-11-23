//
// Created by myrddin on 28/10/16.
//

#include "RazerNaga.h"
#include <iostream>
#include <libRobSock/RobSock.h>
#include <cmath>

using namespace std;

RazerNaga::RazerNaga(int &argc, char* argv[]) : RazerNaga(argc, argv, 0) {
}
RazerNaga::RazerNaga(int &argc, char* argv[], int position) : RazerNaga(argc, argv, position, "localhost") {
}
RazerNaga::RazerNaga(int &argc, char* argv[], int position, string host) :
        RazerNaga(argc, argv, position, host, {60.0, 0.0, -60.0, 180.0}) {
}

RazerNaga::RazerNaga(int &argc, char* argv[], int position, string host, vector<double> ir_sensor_angles) :
        QApplication(argc,argv), name_("RazerNaga"), grid_position_(position), host_(host),
        motor_speed_(0.0, 0.0), ir_sensor_angles_(ir_sensor_angles), state_(STOPPED),
        calculated_path_reference_(map_.get_calculated_path()), position_cycles_reset(0)
{
    if (InitRobot2(const_cast<char *>(name_.c_str()), grid_position_, &ir_sensor_angles[0],
                   const_cast<char *>(host_.c_str())) == -1) {
        throw runtime_error("Failed to connect robot");
    }
    qApp->addLibraryPath("libRobSock");
    QObject::connect((QObject *)(Link()), SIGNAL(NewMessage()), this, SLOT(take_action()));
    QObject::connect(this, SIGNAL(cycle_ended()), this, SLOT(cycle_ended_action()), Qt::AutoConnection);
#ifdef DEBUG
    map_.enable_debug();
#endif

    SetReturningLed(false);
    SetVisitingLed(false);
}

void RazerNaga::cycle_ended_action() {
    map_.render_map();
}

void RazerNaga::take_action() {
    sensors_.update_values();
    retrieve_map();

    switch (state_) {
        case STOPPED:
            if (GetStartButton()) state_ = EXPLORING_OBJECTIVE;
            break;
        case EXPLORING_OBJECTIVE:
            if (GetGroundSensor() != -1) {
                state_ = EXPLORING_FINAL_PATH;
                map_.set_objective(position_.get_tuple());
                calculated_path_reference_.clear();
                SetVisitingLed(true);
            } else {
                if (calculated_path_reference_.size() == 0 ||
                        sensors_.get_obstacle_sensor(1) < OBSTACLE_LIMIT || GetBumperSensor())
                    map_.set_target_nearest_exit();
                follow_path();
            }
            break;
        case EXPLORING_FINAL_PATH:
            SetVisitingLed(false);
            if (map_.is_best_path_discovered()) {
                map_.set_target_objective_area();
                state_ = RETURN_TO_OBJECTIVE;
            } else {
                if (calculated_path_reference_.size() == 0 ||
                        sensors_.get_obstacle_sensor(1) < OBSTACLE_LIMIT || GetBumperSensor()) {
                    map_.set_target_unknown_path();
                }
                follow_path();
            }
            break;
        case RETURN_TO_OBJECTIVE:
            if (GetGroundSensor() != -1) {
                state_ = PREPARE_TO_RETURN;
                map_.set_target_starter_area();
            } else { // Path is complete at this point, it shouldn't require recalculations
                follow_path();
            }
            break;
        case PREPARE_TO_RETURN:
            if (rotate_to_point(calculated_path_reference_.back())) {
                state_ = RETURN_TO_START;
                SetReturningLed(true);
            }
            break;
        case RETURN_TO_START:
            if (distance_between_two_points(tuple<double, double>(0,0), position_.get_tuple()) < 0.1) {
                state_ = FINISHED;
            } else {
                follow_path();
            }
            break;
        case FINISHED:
            M_MOTOR_LEFT(motor_speed_) = 0;
            M_MOTOR_RIGHT(motor_speed_) = 0;
            Finish();
            break;
    }

    if (sensors_.get_obstacle_sensor(1) <= 0.2 || GetBumperSensor())
        set_motors_speed(-0.1, -0.1);

    recalibrate_position();
    apply_motors_speed();
}

void RazerNaga::recalibrate_position() {
    // Resetting X if possible
    if ((fabs(sensors_.get_compass()) < POSITION_RESET_ANGLE ||
            fabs(normalize_angle(sensors_.get_compass() - 180.0)) < POSITION_RESET_ANGLE) &&
            (fabs(sensors_.get_obstacle_sensor(0) - sensors_.get_obstacle_sensor(2)) < POSITION_RESET_OBSTACLE)) {
        if (++position_cycles_reset == POSITION_RESET_CYCLES)
            position_.reset_y();
    // Resetting Y if possible
    } else if ((fabs(sensors_.get_compass() - 90) < POSITION_RESET_ANGLE ||
            fabs(sensors_.get_compass() + 90) < POSITION_RESET_ANGLE) &&
            (fabs(sensors_.get_obstacle_sensor(0) - sensors_.get_obstacle_sensor(2)) < POSITION_RESET_OBSTACLE)) {
        if (++position_cycles_reset == POSITION_RESET_CYCLES)
            position_.reset_x();
    } else {
        position_cycles_reset = 0;
    }
}

void RazerNaga::follow_path() {
    if (calculated_path_reference_.size() == 0) return;

    tuple<double,double>& dst = calculated_path_reference_.back();
    while (distance_between_two_points(position_.get_tuple(), dst) < 0.2) {
        calculated_path_reference_.pop_back();
        if (calculated_path_reference_.size() == 0) return;
        dst = calculated_path_reference_.back();
    }

    double error;
    static double last_error = 0, integral_error = 0;
    error = normalize_angle(angle_between_two_points(position_.get_tuple(), dst) + sensors_.get_compass());
    error = error / NORM_FACTOR; // Normalization factor

    integral_error += error;
    integral_error = integral_error > INTEGRAL_CLIP ? INTEGRAL_CLIP : integral_error;
    integral_error = integral_error < -INTEGRAL_CLIP ? -INTEGRAL_CLIP : integral_error;
    double correction = KP * error + KI * integral_error + KD * (error - last_error);
    last_error = error;

    set_motors_speed(BASE_SPEED + correction, BASE_SPEED - correction);
}

bool RazerNaga::rotate_to_point(std::tuple<double, double>& point) {
    constexpr double NORMALIZE_FACTOR = 0.15 / 90.0;
    double speed, diff;
    diff = normalize_angle(angle_between_two_points(position_.get_tuple(), point) - sensors_.get_compass());
    speed = diff * NORMALIZE_FACTOR;

    set_motors_speed(-speed, speed);
    return fabs(diff) <= 2.0;
}

void RazerNaga::retrieve_map() {
    map_.increase_visited_counter(position_.x(), position_.y());

    const double& x = position_.x();
    const double& y = position_.y();
    long double sensor_x, sensor_y, theta, distance_measured, sensor_final_x, sensor_final_y;
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
            map_.increase_ground_counter_range(sensor_x, sensor_y, sensor_final_x, sensor_final_y);
        }
    }
}

void RazerNaga::set_motors_speed(const double& motor_left, const double& motor_right) {
    M_MOTOR_LEFT(motor_speed_) = motor_left;
    if (M_MOTOR_LEFT(motor_speed_) > MAX_SPEED)
        M_MOTOR_LEFT(motor_speed_) = MAX_SPEED;
    if (M_MOTOR_LEFT(motor_speed_) < -MAX_SPEED)
        M_MOTOR_LEFT(motor_speed_) = -MAX_SPEED;

    M_MOTOR_RIGHT(motor_speed_) = motor_right;
    if (M_MOTOR_RIGHT(motor_speed_) > MAX_SPEED)
        M_MOTOR_RIGHT(motor_speed_) = MAX_SPEED;
    if (M_MOTOR_RIGHT(motor_speed_) < -MAX_SPEED)
        M_MOTOR_RIGHT(motor_speed_) = -MAX_SPEED;
}
void RazerNaga::apply_motors_speed() {
    DriveMotors(M_MOTOR_LEFT(motor_speed_), M_MOTOR_RIGHT(motor_speed_));
    if (!GetBumperSensor())
        position_.update_position(sensors_.get_compass(), M_MOTOR_LEFT(motor_speed_), M_MOTOR_RIGHT(motor_speed_));
    cycle_ended();
}

double RazerNaga::angle_between_two_points(const std::tuple<double, double>& source,
                                           const std::tuple<double, double>& target) {
    double opposite = -(M_Y(target) - M_Y(source));
    double adjacent = M_X(target) - M_X(source);

    double value;
    if (adjacent == 0) {
        value = opposite > 0 ? M_PI / 2.0 : -M_PI / 2.0;
    } else { // tan(theta) = theta + K * M_PI
        value = atan(opposite / adjacent);
        if (adjacent < 0)
            value += M_PI;
    }
    return normalize_angle((value * 180.0) / M_PI);
}

double RazerNaga::distance_between_two_points(const std::tuple<double, double>& source,
                                              const std::tuple<double, double>& target) {
    return sqrt(pow(M_Y(target) - M_Y(source), 2) + pow(M_X(target) - M_X(source), 2));
}

double RazerNaga::normalize_angle(const double& degrees_angle) {
    double value = degrees_angle;
    while (value <= -180.0) value += 360.0;
    while (value > 180.0) value -= 360.0;
    return value;
}