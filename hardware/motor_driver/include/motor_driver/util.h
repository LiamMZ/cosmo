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
    };

    struct EnumClassHash
    {
        template <typename T>
        std::size_t operator()(T t) const
        {
            return static_cast<std::size_t>(t);
        }
    };
}