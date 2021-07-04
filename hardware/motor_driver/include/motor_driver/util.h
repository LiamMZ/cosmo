#ifndef UTIL_H
#define UTIL_H

#include<vector>


namespace motor_driver
{
    class Util
    {
        public:
        static double clamp(double value, const double low, const double high)
        {
            if (value < low) return low;
            else if (value>high) return high;
            return value;
        };

        template<class T>
        static double clipped_first_order_filter(T input, T target, T max_rate, T tau)
        {
            T rate = (target - input)/tau;
            return clamp(rate, -max_rate, max_rate);
        };
    };
}

#endif