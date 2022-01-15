#pragma once
/**
 * @file timer.hpp 
 * @brief file used to keep track of the time performance. 
 * This file contains a simple timer used to chronometrize the run time of the various
 * functions and operations done in the final implementation in student_interface.cpp
 * 
 */
#include <chrono>
#include <vector>
#include <string>

namespace utils
{
    class Timer
    {
    private:
     /**
     * @brief The timer prints the amount of taken by a certain operation
     * the final format is "DONE [n ms]" where n is the amount of milliseconds 
     */

        std::vector<std::chrono::_V2::system_clock::time_point> _tic;
        std::chrono::duration<float, std::milli> _duration;

        void indent();

    public:
        void tic(std::string msg = "");
        void toc(std::string msg = "DONE");
        float millis();
    };
}
