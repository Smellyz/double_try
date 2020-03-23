#ifndef SIMULATE_H
#define SIMULATE_H

#include "robot.h"
#include "referee.h"

namespace robot_decision_simulate
{
    class Simulate
    {
        public:
        Simulate();
        ~Simulate();
        private:
        std::vector<Robot*> robots_;
        Referee referee_;
    };
}

#endif