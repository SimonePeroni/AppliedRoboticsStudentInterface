#pragma once

#include <chrono>
#include <vector>
#include <string>

/**
 * @file timer.hpp 
 * @brief File dedicated to the Timer class.
 * 
 */

namespace utils
{
    /**
     * @brief Simple multi-scope timer for execution timing.
     * 
     */
    class Timer
    {
    private:
        std::vector<std::chrono::_V2::system_clock::time_point> _tic;
        std::chrono::duration<float, std::milli> _duration;

        void indent();

    public:
        /**
         * @brief Create a new scope, start a timer and print an optional message.
         * 
         * @param msg Optional: Message to display
         */
        void tic(std::string msg = "");

        /**
         * @brief Stop the timer of the inner-most scope and display an optional message along with the duration in milliseconds.
         * 
         * @param msg Optional: Message to display before duration. If empty, the duration is not displayed
         */
        void toc(std::string msg = "DONE");

        /**
         * @brief Duration of the last stopped timer.
         * 
         * @return Duration in milliseconds
         */
        float millis();
    };
}
