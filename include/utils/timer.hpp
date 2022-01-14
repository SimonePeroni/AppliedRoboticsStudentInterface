#pragma once

#include <chrono>
#include <vector>
#include <string>

namespace utils
{
    class Timer
    {
    private:
        std::vector<std::chrono::_V2::system_clock::time_point> _tic;
        std::chrono::duration<float, std::milli> _duration;

        void indent();

    public:
        void tic(std::string msg = "");
        void toc(std::string msg = "DONE");
        float millis();
    };
}