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
    }
}