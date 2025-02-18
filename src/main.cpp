/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       ericssonlin                                               */
/*    Created:      2/18/2025, 8:40:17 AM                                     */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/
#include <vex.h>
#include <motor.hpp>
#include <selection.hpp>

vex::brain brain;
vex::controller controller;
vex::pneumatics pneumatics(brain.ThreeWirePort.A);
Motor motor(vex::PORT14, vex::gearSetting::ratio36_1, 1, false, Quantity(0.051, Unit::meter()));

int main() {
    const double max_meter_displacement = 0.3;
    pneumatics.open();
    double distance = selector(max_meter_displacement);
    motor.spin_distance(distance, Unit::meter());
    while (true)
    {
        if (controller.ButtonA.pressing())
        {
            pneumatics.close();
            break;
        }
        vex::this_thread::sleep_for(20);
    }
    // motor.spin_distance(Quantity(0.32044245, Unit::meter())); // 2 inches * 2PI
    while (true)
    {
        vexDelay(1000);
    }
}