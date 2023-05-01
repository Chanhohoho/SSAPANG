#include <ros/ros.h>
#include <random>
#include <study_lhj/Locations.h>
#include <study_lhj/Coordinate.h>
#include <study_lhj/Move.h>
#include <study_lhj/RobotWait.h>
#include <study_lhj/RobotPos.h>
#include <study_lhj/RobotStatus.h>
#include <study_lhj/Task.h>
#include <study_lhj/TaskList.h>
#include <time.h>
#include <unistd.h>
#include <iostream>
#include <stdlib.h>
#include <vector>
#include <string>
// using namespace std;



class Robot
{
public:
    Robot(int num, ros::NodeHandle *nh){
        std::string name = "robot" + std::to_string(num);
        waitPub = nh->advertise<study_lhj::RobotWait>(name+"/wait", 1);
        RobotStatusSub = nh->subscribe<study_lhj::RobotStatus>(name+"/status", 1, boost::bind(&Robot::status, this, _1, num));
        RobotPosSub = nh->subscribe<study_lhj::RobotPos>(name+"/pos", 1, boost::bind(&Robot::pos, this, _1, num));
    }

private:
    ros::Publisher waitPub;
    ros::Subscriber RobotStatusSub, RobotPosSub;
    ros::Rate rate = 30;

    void status(const study_lhj::RobotStatus::ConstPtr &msg, int num){
        // printf("&s:status\n", robotName);
        // printf("&d\n", msg->status);
        std::cout <<num << "-status\n" << (msg->status) << "\n";
    }

    void pos(const study_lhj::RobotPos::ConstPtr &msg, int num){
        std::cout <<num << "-pos\n";
        std::cout << "now : " << msg->fromNode << ", next: " << msg->toNode << "\n";
        std::cout << "bottery : " <<msg->battery << "\n";
    }
    
};

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
    ros::Publisher waitPub;
    ros::Subscriber RobotStatusSub, RobotPosSub;
    std::vector<Robot> robots;
    std::vector<int> robotStatus;
    study_lhj::TaskList taskList;
    ros::Rate rate = 30;

    std::string robotName, shelfNode;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "tower");
    ros::NodeHandle nh;
    ControlTower ControlTower(&nh);
    // ros::MultiThreadedSpinner s(10);
    ros::spin();

    return 0;
}