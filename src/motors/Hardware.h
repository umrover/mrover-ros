#ifndef HARDWARE_H
#define HARDWARE_H

#include <algorithm>

//Helper enum representing the valid types of real motor controllers
enum HardwareType
{
    Motor6V,
    Motor9V,
    Motor12V,
    Motor24V,
    Motor22Percent,
    Motor29Percent,
    HBridge,
    None
};

//Helper class to abstract away Motor Controller details 
class Hardware
{
public:
    uint16_t speed_max; //out of 100 to avoid sending floats
    HardwareType type;

    HardwareType getType(std::string input) 
    {
        if (input == "Motor6V") {
            return Motor6V;
        }
        else if (input == "Motor9V") {
            return Motor9V;
        }
        else if (input == "Motor12V") {
            return Motor12V;
        }
        else if (input == "Motor24V") {
            return Motor24V;
        }
        else if (input == "Motor22Percent") {
            return Motor22Percent;
        }
        else if(input == "Motor29Percent") {
            return Motor29Percent;
        }
        else if (input == "HBridge") {
            return HBridge;
        }
        else 
        {
            return None;
        }
    }

    Hardware() : type(None) {}

    Hardware(std::string input) : type(getType(input))
    {
        switch (type)
        {
        case Motor6V:
            speed_max = 16;  // 5.76 V        
            break;
        case Motor9V:
            speed_max = 25;  // 9 V
            break;
        case Motor12V:
            speed_max = 33;  // 11.88 V
            break;
        case Motor24V:
            speed_max = 60;  // 21.6 V
	        break;
        case Motor22Percent:
            speed_max = 22;  // 7.92 V, or 24V * 0.33 / 36 %
	        break;
        case Motor29Percent:
            speed_max = 29;  // 10.44 V, or 24V * 0.44 / 36 %
	        break;
        case HBridge:
            speed_max = 100;
        case None:
            break;
        }
    }

    //Helper function that takes a [-1.0, 1.0] (reranged between min and max) input and converts it into a 16-bit pwm output
    // float rerange(float input, float min, float max)
    // {
    //     return (((pwm_max) / (max - min)) * (input - min));
    // }

    //limits throttle
    float throttle(float input)
    {
        if (input > 1.0) {
            input = 1.0;
        }
        else if (input < -1.0) {
            input = -1.0;
        }
        return input;
    }
};

#endif
