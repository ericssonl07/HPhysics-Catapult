#include <motor.hpp>

Motor::Motor(int32_t index, vex::gearSetting gears, double gear_ratio, bool reverse, Quantity wheel_radius)
    : m_device(new vex::motor(index, reverse)),
      m_cartridge_gears(gears),
      m_gear_ratio(gear_ratio),
      m_wheel_radius(wheel_radius),
      target_position(Quantity(0.0, Unit::meter())),
      target_velocity(Quantity(0.0, Unit::meter() / Unit::second()))
{
    // Set the cartridge ratio based on the gear setting.
    switch (gears) {
        case vex::gearSetting::ratio6_1: // blue
            m_cartridge_ratio = 1.66666666666666666666e-1;
            break;
        case vex::gearSetting::ratio18_1: // green
            m_cartridge_ratio = 5.55555555555555555555e-2;
            break;
        case vex::gearSetting::ratio36_1: // red
            m_cartridge_ratio = 2.77777777777777777777e-2;
            break;
    }
    m_net_gear_ratio = m_gear_ratio * m_cartridge_ratio;
    m_reciprocal_net_gear_ratio = 1.0 / m_net_gear_ratio;
}

Motor::~Motor()
{
    delete m_device;
}

Quantity Motor::max_tangential_velocity()
{
    const double total_ratio = m_cartridge_ratio * m_gear_ratio;
    const double max_wheel_rpm = 3600.0 * total_ratio;
    const double wheel_radius_meter = m_wheel_radius.convert_to(Unit::meter()).value();
    const double wheel_circumference = 2 * M_PI * wheel_radius_meter;
    const double max_velocity_meters_per_minute = max_wheel_rpm * wheel_circumference;
    const double max_velocity_meters_per_second = max_velocity_meters_per_minute / 60.0;
    return Quantity(max_velocity_meters_per_second, Unit::meters_per_second());
}

Quantity Motor::max_angular_velocity()
{
    return max_tangential_velocity() / m_wheel_radius;
}

void Motor::reset()
{
    m_device -> resetPosition();
}

void Motor::spin(Quantity velocity)
{
    double t_vel = velocity.as(Unit::meter() / Unit::second()).value();
    double m_vel = max_tangential_velocity().as(Unit::meter() / Unit::second()).value();
    double ratio = t_vel / m_vel;
    printf("Spin: %.2f\n", ratio * 12.0);
    m_device -> spin(vex::directionType::fwd, ratio * 12.0, vex::voltageUnits::volt);
}

void Motor::spin(double velocity, Unit unit)
{
    spin(Quantity(velocity, unit));
}

void Motor::spin_angular(Quantity angular_velocity)
{
    auto velocity = m_wheel_radius * angular_velocity;
    spin(velocity);
}

void Motor::spin_angular(double angular_velocity, Unit unit)
{
    spin_angular(Quantity(angular_velocity, unit));
}

void Motor::stop()
{
    m_device -> stop(vex::brakeType::brake);
}

void Motor::spin_distance(Quantity distance, bool reset)
{
    // (Optionally) reset the motor’s position.
    if (reset)
    {
        m_device -> resetPosition();
    }

    // Convert the target distance to meters.
    double distance_m = distance.convert_to(Unit::meter()).value();
    
    // Get the wheel radius in meters.
    double wheel_radius_m = m_wheel_radius.convert_to(Unit::meter()).value();

    double displacement_rad = distance_m / wheel_radius_m;

    double displacement_deg = displacement_rad * 180 / M_PI;

    double net_displacement_deg = displacement_deg / m_gear_ratio;

    double initial_pos = m_device -> position(vex::rotationUnits::deg);
    double target_pos = initial_pos + net_displacement_deg;
    
    // --- PID Controller Setup ---
    // These constants are provided as an example. You may need to tune them for your system.
    // const double Kp = 0.025;
    const double Kp = 0.15;
    const double Ki = 0.015;
    const double I_threshold = 1000; // when error is greater than this, don't accumulate
    const double Kd = 0.025;
    // const double Kd = 0.001;
    
    double integral = 0.0;
    double previous_error = target_pos - initial_pos;
    
    // Tolerance (in motor degrees) at which we consider the move “complete”
    const double tolerance = 0.5;
    
    // How long to wait between iterations (in milliseconds)
    const int loop_delay_ms = 2;
    
    // Use a steady clock to compute dt between iterations.
    auto last_time = std::chrono::high_resolution_clock::now();
    
    // --- Control Loop ---
    long long iter = 0;
    while (true)
    {
        // Get the current motor position (in degrees).
        double current_pos = m_device -> position(vex::rotationUnits::deg);
        double error = target_pos - current_pos;

        if (!(++iter % 100))
            printf("Current: %.2f, Target: %.2f, Error: %.2f\n", current_pos, target_pos, error);
        
        // If we’re within tolerance, exit the loop.
        if (std::fabs(error) < tolerance)
        {
            printf("Final error: %.e deg < %e deg tolerance\n", error, tolerance);
            break;
        }
        
        // Compute time elapsed (in seconds) since the last loop.
        auto current_time = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> elapsed = current_time - last_time;
        double dt = elapsed.count();
        last_time = current_time;
        
        // Update the integral and derivative terms.
        if (std::fabs(error) < I_threshold)
            integral += error * dt;
        double derivative = (error - previous_error) / dt;
        previous_error = error;
        
        // Compute the PID output.
        double output = Kp * error + Ki * integral + Kd * derivative;
        
        // Saturate the output to ±12 volts.
        if (output > 9.0)
            output = 9.0;
        else if (output < -9.0)
            output = -9.0;
        
        // Command the motor.
        // Note: We choose the motor’s spin direction based on the sign of the PID output.
        if (output >= 0)
            m_device -> spin(vex::directionType::fwd, output, vex::voltageUnits::volt);
        else
            m_device -> spin(vex::directionType::rev, -output, vex::voltageUnits::volt);
        
        // Wait a short while before the next control update.
        vex::this_thread::sleep_for(loop_delay_ms);
    }
    
    // When the loop ends, stop the motor (using a braking mode).
    m_device -> stop(vex::brakeType::brake);
}

// void Motor::spin_distance(Quantity distance, bool reset)
// {
//     // (Optionally) reset the motor’s position.
//     if (reset)
//     {
//         m_device->resetPosition();
//     }

//     // Convert the target distance to meters.
//     double distance_m = distance.convert_to(Unit::meter()).value();
    
//     // Get the wheel radius in meters.
//     double wheel_radius_m = m_wheel_radius.convert_to(Unit::meter()).value();
//     double displacement_rad = distance_m / wheel_radius_m;
//     double displacement_deg = displacement_rad * 180 / M_PI;
//     double net_displacement_deg = displacement_deg / m_gear_ratio;

//     double initial_pos = m_device->position(vex::rotationUnits::deg);
//     double target_pos = initial_pos + net_displacement_deg;

//     // --- PID Controller Setup ---
//     const double Kp = 0.15;
//     const double Ki = 0.015;
//     const double Kd = 0.025;
//     const double I_threshold = 1000; // When error is greater than this, don't accumulate
//     const double output_min = 0.3;
//     const double output_max = 9.0;
//     const double p_threshold = 1000.0;
//     const double gamma = 1.0;
//     const double i_max = 1.0e30;

//     // Create a PID instance
//     PID pidController(Kp, Ki, Kd, output_min, output_max, I_threshold, gamma, i_max, p_threshold);
    
//     // Set the target position in the PID controller
//     pidController.set_target(target_pos);

//     // Tolerance (in motor degrees) at which we consider the move “complete”
//     const double tolerance = 0.5;

//     // How long to wait between iterations (in milliseconds)
//     const int loop_delay_ms = 2;
    
//     // --- Control Loop ---
//     long long iter = 0;
//     while (true)
//     {
//         // Get the current motor position (in degrees).
//         double current_pos = m_device->position(vex::rotationUnits::deg);
//         double error = target_pos - current_pos;

//         if (!(++iter % 100))
//             printf("Current: %.2f, Target: %.2f, Error: %.2f\n", current_pos, target_pos, error);
        
//         // If we’re within tolerance, exit the loop.
//         if (std::fabs(error) < tolerance)
//         {
//             printf("Final error: %.e deg < %e deg tolerance\n", error, tolerance);
//             break;
//         }

//         // Calculate the PID output
//         double output = pidController.output(current_pos);

//         // Command the motor.
//         m_device->spin(vex::directionType::fwd, output, vex::voltageUnits::volt);
        
//         // Wait a short while before the next control update.
//         vex::this_thread::sleep_for(loop_delay_ms);
//     }
    
//     // When the loop ends, stop the motor (using a braking mode).
//     m_device->stop(vex::brakeType::brake);
// }


void Motor::spin_distance(double distance, Unit unit)
{
    spin_distance(Quantity(distance, unit));
}