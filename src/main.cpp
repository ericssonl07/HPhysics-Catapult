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

vex::brain brain;

int main() {
    Motor motor(vex::PORT14, vex::gearSetting::ratio36_1, 1, false, Quantity(0.051, Unit::meter()));
    // motor.spin_angular(Quantity(2*M_PI, Unit::radian() / Unit::second()));
    // motor.spin(Quantity(0.32044245, Unit::meter() / Unit::second()));
    motor.spin_distance(Quantity(0.32044245, Unit::meter()));
    while(true)vexDelay(1000);
}
