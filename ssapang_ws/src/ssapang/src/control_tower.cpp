#include <ros/ros.h>
#include <random>
#include <ssapang/Locations.h>
#include <ssapang/Coordinate.h>
#include <ssapang/Move.h>
#include <ssapang/RobotWait.h>
#include <ssapang/RobotPos.h>
#include <ssapang/RobotStatus.h>
#include <ssapang/Task.h>
#include <ssapang/TaskList.h>
#include "robot.h"
#include <time.h>
#include <unistd.h>
#include <iostream>
#include <stdlib.h>
#include <vector>
#include <string>
#include <unordered_map>
#include <queue>

std::unordered_map<std::string, std::queue<info>> node;

class ControlTower
{
public:
    ControlTower(ros::NodeHandle *nh){
        for(int i = 0; i < 10; i++){
            robots.push_back(Robot(i, nh));
            robotStatus.push_back(0);
        }
    }
private:
    void updateNode(){
        
        std::cout << "update" << std::endl;
    }
    std::vector<Robot> robots;
    std::vector<int> robotStatus;
    ssapang::TaskList taskList;
    ros::Rate rate = 30;

    std::string robotName, shelfNode;
    
};

int main(int argc, char **argv)
{
    
    ros::init(argc, argv, "tower");
    ros::NodeHandle nh;
    ControlTower ControlTower(&nh);
    ros::spin();

    return 0;
}