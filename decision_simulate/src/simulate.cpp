#include "simulate.h"

namespace robot_decision_simulate
{
    Simulate::Simulate()
    {
        // Robot *robot = new Robot(0);
        // robots_.push_back(robot);
        // robot = new Robot(2);
        // robots_.push_back(robot);
        // robot = new Robot(4);
        // robots_.push_back(robot);
        // robot = new Robot(6);
        // robots_.push_back(robot);
    }

    Simulate::~Simulate(){}
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "decision_simulation_node");
    robot_decision_simulate::Simulate robot_simulate;
    ros::AsyncSpinner async_spinner(4);
    async_spinner.start();
    ros::waitForShutdown();
    return 0;
}