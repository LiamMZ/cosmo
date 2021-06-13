#include <motor_driver/gait_controller.h>

motor_driver::GaitController::GaitController(const std::vector< std::vector<bool> > contact_phases, const unsigned int num_phases, const std::vector<unsigned int> phase_ticks, const unsigned int phase_length)
 : contact_phases_(contact_phases), num_phases_(num_phases), phase_ticks_(phase_ticks), phase_length_(phase_length)
{}

unsigned int motor_driver::GaitController::phase_index(const unsigned long ticks)
{
    unsigned int phase_time = ticks%phase_length_;
    unsigned int phase_sum = 0;
    for(int i = 0; i<num_phases_; i++)
    {
        phase_sum+=phase_ticks_[i];
        if (phase_time < phase_sum)
        {
            return i;
        }
    }
    throw false;
}

unsigned int motor_driver::GaitController::subphase_ticks(const unsigned long ticks)
{
    unsigned int phase_time = ticks % phase_length_;
    unsigned int phase_sum = 0;
    for( int i = 0; i < phase_length_; i++)
    {
        phase_sum+=phase_ticks_[i];
        if (phase_time < phase_sum)
        {
            return phase_time - phase_sum + phase_ticks_[i];
        }
    }
    throw false;
}

std::vector<bool> motor_driver::GaitController::contacts(const unsigned long ticks)
{
    return contact_phases_[phase_index(ticks)];
}

