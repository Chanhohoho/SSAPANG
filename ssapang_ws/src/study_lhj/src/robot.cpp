#include <ros/ros.h>
#include <study_lhj/RobotPos.h>
#include <study_lhj/RobotStatus.h>
#include <study_lhj/RobotWait.h>
#include <string>
using namespace std;

class Robot
{
public:
    Robot(std::string name){
        robotName = name;
        waitPub = nh.advertise<study_lhj::RobotWait>(name+"/wait", 1);
        RobotStatusSub = nh.subscribe(name+"/status", 10, &Robot::status, this);
        RobotPosSub = nh.subscribe(name+"/pos", 10, &Robot::pos, this);
    }

private:
    ros::Publisher waitPub;
    ros::Subscriber RobotStatusSub, RobotPosSub;
    ros::NodeHandle nh;
    ros::Rate rate = 30;
    string robotName;

    void status(const study_lhj::RobotStatus::ConstPtr &msg){
        printf("&s:status\n", robotName);
        printf("&d\n", msg->status);
    }

    void pos(const study_lhj::RobotPos::ConstPtr &msg){
        printf("&s:pos\n", robotName);
        printf("now : &s, next : &s\n", msg->fromNode, msg->toNode);
        printf("bottery : &f\n", msg->battery);
    }
};
