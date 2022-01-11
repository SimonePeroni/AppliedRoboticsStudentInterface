#include "utils/timer.hpp"

#include <chrono>
#include <iostream>
#include <string>

namespace utils
{
    void Timer::tic(std::string msg)
    {
        _tic = std::chrono::high_resolution_clock::now();
        if (!msg.empty())
            std::cout << msg << std::endl;
    }

    void Timer::toc(std::string msg)
    {
        auto toc = std::chrono::high_resolution_clock::now();
        _duration = toc - _tic;
        if (!msg.empty())
            std::cout << msg << " [" << _duration.count() << " ms]" << std::endl;
    }

    float Timer::millis()
    {
        return _duration.count();
    }
}