#ifndef ESP_WHEEL_H
#define ESP_WHEEL_H

#include <string>
#include <cmath>

class Wheel
{
    public:

    std::string name = "";
    std::string name_void = "";
    int enc = 0;
    double cmd = 0;
    double cmd_void = 0;
    double pos = 0;
    double vel = 0;
    double eff = 0;
    double velSetPt = 0;
    double rads_per_count = 0;

    Wheel() = default;

    Wheel(const std::string &wheel_name, const std::string &wheel_name_void , int counts_per_rev)
    {
        setup(wheel_name, wheel_name_void, counts_per_rev);
    }
    
    void setup(const std::string &wheel_name, const std::string &wheel_name_void, int counts_per_rev)
    {
        name = wheel_name;
        name_void = wheel_name_void;
        rads_per_count = (2*M_PI)/counts_per_rev;
    }

    double calcEncAngle()
    {
        return enc * rads_per_count;
    }
};


#endif // ESP_WHEEL_H