#pragma once
#ifndef MOTOR_HPP
#define MOTOR_HPP

#include <units.hpp>
#include <pid.hpp>
#include <vex.h>
#include <cmath>

class Motor {
    friend int main();
    vex::motor* m_device;
    vex::gearSetting m_cartridge_gears;
    double m_gear_ratio;
    double m_cartridge_ratio;
    double m_net_gear_ratio;
    double m_reciprocal_net_gear_ratio;
    Quantity m_wheel_radius;
    bool m_suction_reset = false;
    bool m_run_thread = false;
    bool m_spin_velocity = false;
    bool m_pid_position = false;
    Quantity target_position;
    Quantity target_velocity;

    // PID state variables for position control
    double m_pid_last_error = 0.0;
    double m_pid_integral = 0.0;

public:
    Motor(int32_t index, vex::gearSetting gears, double gear_ratio, bool reverse, Quantity wheel_radius);
    ~Motor();

    Quantity max_tangential_velocity();
    Quantity max_angular_velocity();
    void reset();
    void spin(Quantity velocity);
    void spin(double velocity, Unit unit);
    void spin_angular(Quantity angular_velocity);
    void spin_angular(double angular_velocity, Unit unit);
    void stop();
    void spin_distance(Quantity distance, bool reset = false);
    void spin_distance(double distance, Unit unit);
};

#endif // MOTOR_HPP