#ifndef GAITCONTROLLER_H
#define GAITCONTROLLER_H

#include <vector>

namespace motor_driver
{
    class GaitController
    {
        public:
            GaitController();

            /**
            * Constructor initializes member variables
            *
            * @param[in] contact_phases - 2d vector of bools for each foot in each phase 1 indicates contact 0 indicates in motion
            * @param[in] num_phases - number of phases in gait
            * @param[in] phase_ticks - vector of number of ticks in each phase of gait
            * @param[in] phase_length - the length of each phase
            */
            GaitController(const std::vector< std::vector<bool> > contact_phases, const int num_phases, const std::vector<int> phase_tics, const int phase_length);

            /**
            * Function to indicate which feet should be in contact with the ground
            *
            * @param[in] ticks - the number of time steps since the program started
            * 
            * @returns boolean vector of length 4 - 1 indicates foot in contact with ground 0 indicates in air
            */
            std::vector<bool> contacts(const long ticks);

            /**
            * Function to return the number of ticks into the current phase
            * 
            * @param[in] ticks - the number of timesteps since program started
            * @returns subphase_ticks - the number of timesteps since the beginning of the current phase
            */
            int subphase_ticks(const long ticks);

            GaitController& operator =(const GaitController& a)
            {
                num_phases_ = a.num_phases_;
                contact_phases_ = a.contact_phases_;
                phase_ticks_ = a.phase_ticks_;
                phase_length_ = a.phase_length_;

                return *this;
            }
        private:
            /**
            * Function to return the index of the given phase based on the number of timesteps that have passed
            *
            * @param[in] ticks - the number of time steps since the program started
            *
            * @returns index of current phase
            */
            int phase_index(const long ticks);

            // vector of contact configurations for each foot for each phase
            std::vector< std::vector<bool> > contact_phases_;

            // number of phases
            int num_phases_;

            // number of ticks for each phase
            std::vector<int> phase_ticks_;

            // the length of each phase
            int phase_length_;
    };
}

#endif