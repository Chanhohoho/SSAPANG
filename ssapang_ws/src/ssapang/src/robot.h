#include <ros/ros.h>
#include <ssapang/RobotPos.h>
#include <ssapang/RobotStatus.h>
#include <ssapang/RobotWait.h>
#include <string>
#include <unordered_map>
#include <queue>


struct info{
    int robotNum;
    std::string beforeNode;
};

extern std::unordered_map<std::string, std::queue<info>> node;

class Robot
{
public:
    Robot(int num, ros::NodeHandle *nh){
        std::string name = "burger" + std::to_string(num);
        waitPub = nh->advertise<ssapang::RobotWait>(name+"/wait", 1);
        RobotStatusSub = nh->subscribe<ssapang::RobotStatus>(name+"/status", 1, boost::bind(&Robot::status, this, _1, num));
        RobotPosSub = nh->subscribe<ssapang::RobotPos>(name+"/pos", 1, boost::bind(&Robot::pos, this, _1, num));
    }

    ros::Publisher waitPub;
    ros::Subscriber RobotStatusSub, RobotPosSub; 
    ros::Rate rate = 30;

    void status(const ssapang::RobotStatus::ConstPtr &msg, int num){
        // printf("&s:status\n", robotName);
        // printf("&d\n", msg->status);
        std::cout <<num << "-status\n" << (msg->status) << "\n";
    }

    void pos(const ssapang::RobotPos::ConstPtr &msg, int num){
        

        std::cout <<num << "-pos\n";
        std::cout << "now : " << msg->fromNode << ", next: " << msg->toNode << "\n";
        std::cout << "bottery : " <<msg->battery << "\n";
    }
    
};
