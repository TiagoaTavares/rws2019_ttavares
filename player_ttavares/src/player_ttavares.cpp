#include <iostream>
#include <ros/ros.h>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "player_ttavres");

    // std::cout << "Hello World" << std::endl;

    ros::Rate r(1); // p fazermos sleep a 1hz

    for (int i = 0; i < 10; i++)
    {
        std::cout << i << std::endl;
        r.sleep();
    }
    return 1;
}