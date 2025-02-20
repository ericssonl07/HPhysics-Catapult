#include <selection.hpp>

double selector(double max)
{
    double value = 0;
    int power = -1;
    bool lastL, lastR, lastU, lastD;
    lastL = lastR = lastU = lastD = false;
    while (!controller.ButtonA.pressing())
    {
        controller.Screen.clearScreen();
        controller.Screen.setCursor(1, 1);
        controller.Screen.print("%.6f m", value);
        controller.Screen.setCursor(2, 1);
        controller.Screen.print("Precision: 10^%d", power);
        // selection mode
        if (controller.ButtonLeft.pressing() && !lastL)
        {
            // left was pressed
            double new_value = value - pow(10, power);
            if (new_value >= 0 and new_value <= max)
            {
                value = new_value;
            } else
            {
                controller.rumble("..");
            }
        }
        else if (controller.ButtonRight.pressing() && !lastR)
        {
            // right was pressed
            double new_value = value + pow(10, power);
            if (new_value >= 0 and new_value <= max)
            {
                value = new_value;
            } else
            {
                controller.rumble("..");
            }
        }
        else if (controller.ButtonUp.pressing() && !lastU)
        {
            // up was pressed
            ++power;
        }
        else if (controller.ButtonDown.pressing() && !lastD)
        {
            // down was pressed
            --power;
        }
        lastL = controller.ButtonLeft.pressing();
        lastR = controller.ButtonRight.pressing();
        lastU = controller.ButtonUp.pressing();
        lastD = controller.ButtonDown.pressing();
    }
    return value;
}