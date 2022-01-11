#pragma once

#include <chrono>
#include <string>

namespace utils
{
    class Timer
    {
    private:
        std::chrono::_V2::system_clock::time_point _tic;
        std::chrono::duration<float, std::milli> _duration;
    public:
        void tic(std::string msg = "");
        void toc(std::string msg = " DONE");
        float millis();
    };
}