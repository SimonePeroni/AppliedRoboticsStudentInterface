#include "utils/timer.hpp"

#include <chrono>
#include <iostream>
#include <string>

namespace utils
{
    void Timer::tic(std::string msg)
    {
        _tic.push_back(std::chrono::_V2::system_clock::time_point());
        if (!msg.empty())
        {
            indent();
            std::cout << msg << std::endl;
        }
        _tic.back() = std::chrono::high_resolution_clock::now();
    }

    void Timer::toc(std::string msg)
    {
        auto toc = std::chrono::high_resolution_clock::now();
        _duration = toc - _tic.back();
        if (!msg.empty())
        {
            indent();
            std::cout << msg << " [" << millis() << " ms]" << std::endl;
        }
        _tic.pop_back();
    }

    void Timer::indent()
    {
        for (size_t i = 0; i < _tic.size(); i++)
        {
            std::cout << " ";
        }
    }

    float Timer::millis()
    {
        return _duration.count();
    }
}