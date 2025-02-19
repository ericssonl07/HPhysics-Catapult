// #pragma once
// #ifndef MOTOR_HPP
// #define MOTOR_HPP

// #include <units.hpp>
// #include <vex.h>
// #include <cmath>

// class Motor {
//     vex::motor* m_device;
//     vex::gearSetting m_cartridge_gears;
//     double m_gear_ratio;
//     double m_cartridge_ratio;
//     double m_net_gear_ratio;
//     double m_reciprocal_net_gear_ratio;
//     Quantity m_wheel_radius;
//     bool m_suction_reset = false;
//     bool m_run_thread = false;
//     bool m_spin_velocity = false;
//     bool m_pid_position = false;
//     Quantity target_position;
//     Quantity target_velocity;
//     vex::thread m_thread;

//     // PID state variables for position control
//     double m_pid_last_error = 0.0;
//     double m_pid_integral = 0.0;

//     void thread()
//     {
//         // PID gains for position control (tune as needed)
//         const double kP_p = 0.1, kI_p = 0.1, kD_p = 0.1;

//         if (m_suction_reset)
//         {
//             // Reset motor: spin slowly until a hard limit is reached
//             m_device->spin(vex::directionType::fwd, 1.2, vex::voltageUnits::volt);
//         }
//         else if (m_spin_velocity)
//         {
//             // Velocity control: convert the target linear (tangential) velocity
//             // to a motor voltage output (based on the maximum tangential velocity)
//             double t_vel = target_velocity.as(Unit::meter() / Unit::second()).value();
//             double m_vel = max_tangential_velocity().as(Unit::meter() / Unit::second()).value();
//             double ratio = t_vel / m_vel;
//             m_device->spin(vex::directionType::fwd, ratio * 12.0, vex::voltageUnits::volt);
//         }
//         else if (m_pid_position)
//         {
//             // PID control to move the wheel a desired linear distance.
//             // 1. Get the target distance (in meters)
//             double t_pos = target_position.as(Unit::meter()).value();

//             // 2. Get current motor position (in revolutions)
//             double motor_rev = m_device->position(vex::rotationUnits::rev);

//             // 3. Convert motor revolutions to wheel linear displacement.
//             //    Wheel distance = (motor_rev * net_gear_ratio) * (wheel circumference)
//             double wheel_distance = motor_rev * m_net_gear_ratio * (2.0 * M_PI * m_wheel_radius.as(Unit::meter()).value());

//             // 4. Calculate the error (in meters)
//             double error = t_pos - wheel_distance;

//             // dt is set by the sleep duration in the thread (20ms)
//             double dt = 0.02; // seconds

//             // 5. Compute the derivative and update the integral.
//             double derivative = (error - m_pid_last_error) / dt;
//             m_pid_integral += error * dt;
//             m_pid_last_error = error;

//             // 6. Compute the PID controller output (voltage)
//             double output_voltage = kP_p * error + kI_p * m_pid_integral + kD_p * derivative;

//             // 7. Clamp the output voltage to within the motor's allowable range.
//             if (output_voltage > 12.0)
//                 output_voltage = 12.0;
//             else if (output_voltage < -12.0)
//                 output_voltage = -12.0;

//             // 8. Check if we are close enough to the target (within 5mm tolerance)
//             const double threshold = 0.005; // 5 mm
//             if (std::fabs(error) < threshold)
//             {
//                 m_device->stop();
//                 m_pid_position = false;
//                 // Reset PID state variables so a new command starts fresh.
//                 m_pid_last_error = 0.0;
//                 m_pid_integral = 0.0;
//             }
//             else
//             {
//                 // 9. Spin the motor with the calculated voltage.
//                 if (output_voltage >= 0)
//                     m_device->spin(vex::directionType::fwd, std::fabs(output_voltage), vex::voltageUnits::volt);
//                 else
//                     m_device->spin(vex::directionType::rev, std::fabs(output_voltage), vex::voltageUnits::volt);
//             }
//         }
//     }

//     static void call_thread(void* instance)
//     {
//         Motor* obj = static_cast<Motor*>(instance);
//         while (true)
//         {
//             if (obj->m_run_thread)
//             {
//                 obj->thread();
//                 vex::this_thread::sleep_for(20);
//             }
//         }
//         return;
//     }

// public:
//     Motor(int32_t index, vex::gearSetting gears, double gear_ratio, bool reverse, Quantity wheel_radius)
//         : m_device(new vex::motor(index, reverse)),
//           m_cartridge_gears(gears),
//           m_gear_ratio(gear_ratio),
//           m_wheel_radius(wheel_radius),
//           target_position(Quantity(0.0, Unit::meter())),
//           target_velocity(Quantity(0.0, Unit::meter() / Unit::second())),
//           m_thread(&call_thread, this)
//     {
//         // Set the cartridge ratio based on the gear setting.
//         switch (gears) {
//             case vex::gearSetting::ratio6_1: // blue
//                 m_cartridge_ratio = 1.66666666666666666666e-1;
//                 break;
//             case vex::gearSetting::ratio18_1: // green
//                 m_cartridge_ratio = 5.55555555555555555555e-2;
//                 break;
//             case vex::gearSetting::ratio36_1: // red
//                 m_cartridge_ratio = 2.77777777777777777777e-2;
//                 break;
//         }
//         m_net_gear_ratio = m_gear_ratio * m_cartridge_ratio;
//         m_reciprocal_net_gear_ratio = 1.0 / m_net_gear_ratio;
//     }

//     ~Motor()
//     {
//         delete m_device;
//     }

//     Quantity max_tangential_velocity()
//     {
//         double max_rps = max_angular_velocity().value();
//         double wheel_circumference = 2.0 * M_PI * m_wheel_radius.as(Unit::meter()).value();
//         return Quantity(max_rps * wheel_circumference, Unit::meter() / Unit::second());
//     }

//     Quantity max_angular_velocity()
//     {
//         double rps = 60.0 * m_reciprocal_net_gear_ratio;
//         return Quantity(rps, Unit::revolution() / Unit::second());
//     }

//     void reset()
//     {
//         m_run_thread = true;
//         m_suction_reset = true;
//         // Allow time for the motor to reach the hard limit before resetting
//         vex::this_thread::sleep_for(500);
//         m_device->resetPosition();
//         m_suction_reset = false;
//         m_run_thread = false;
//     }

//     void spin(Quantity velocity)
//     {
//         m_run_thread = true;
//         m_spin_velocity = true;
//         target_velocity = velocity;
//         // Give the thread a chance to process the command
//         vex::this_thread::sleep_for(20);
//         m_spin_velocity = false;
//         m_run_thread = false;
//     }

//     void spin(double velocity, Unit unit)
//     {
//         spin(Quantity(velocity, unit));
//     }

//     void spin_angular(Quantity angular_velocity)
//     {
//         auto velocity = m_wheel_radius * angular_velocity;
//         spin(velocity);
//     }

//     void spin_angular(double angular_velocity, Unit unit)
//     {
//         spin_angular(Quantity(angular_velocity, unit));
//     }

//     void stop()
//     {
//         m_run_thread = true;
//         m_spin_velocity = true;
//         target_velocity = Quantity(0.0, Unit::meter() / Unit::second());
//         vex::this_thread::sleep_for(20);
//         m_spin_velocity = false;
//         m_run_thread = false;
//     }

//     // New method: spin the motor so that the wheel travels a given linear distance.
//     void spin_distance(Quantity distance, bool reset = false)
//     {
//         // Optionally reset the motor's position before starting.
//         if (reset)
//         {   
//             Motor::reset();
//         }

//         m_run_thread = true;
//         m_pid_position = true;
//         target_position = distance;
//         // Reset the PID state so previous commands do not interfere.
//         m_pid_last_error = 0.0;
//         m_pid_integral = 0.0;

//         // Block until the PID loop (in the thread) completes reaching the target.
//         while (m_pid_position)
//         {
//             vex::this_thread::sleep_for(5);
//         }
//         m_run_thread = false;
//     }

//     // Overload for spinForDistance with a raw value and a Unit.
//     void spin_distance(double distance, Unit unit)
//     {
//         spin_distance(Quantity(distance, unit));
//     }
// };

// #endif // MOTOR_HPP


#pragma once
#ifndef MOTOR_HPP
#define MOTOR_HPP

#include <units.hpp>
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