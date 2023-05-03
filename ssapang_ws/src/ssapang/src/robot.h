#include <ros/ros.h>
#include <ssapang/RobotPos.h>
#include <ssapang/RobotStatus.h>
#include <ssapang/RobotWait.h>
#include <ssapang/Task.h>
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
        std::string name = "robot" + std::to_string(num);
        waitPub = nh->advertise<ssapang::RobotWait>(name+"/wait", 1);
        taskPub = nh->advertise<ssapang::Task>(name+"/task", 1);
        RobotStatusSub = nh->subscribe<ssapang::RobotStatus>(name+"/status", 1, boost::bind(&Robot::status, this, _1, num));
        RobotPosSub = nh->subscribe<ssapang::RobotPos>(name+"/pos", 1, boost::bind(&Robot::pos, this, _1, num));
    }

private:
    ros::Publisher waitPub, taskPub;
    ros::Subscriber RobotStatusSub, RobotPosSub; 
    ros::Rate rate = 30;

    void status(const ssapang::RobotStatus::ConstPtr &msg, int num){
        // printf("&s:status\n", robotName);
        // printf("&d\n", msg->status);
        std::cout <<num << "-status\n" << (msg->status) << "\n";
    }

    void pos(const ssapang::RobotPos::ConstPtr &msg, int num){
        std::cout <<num << "-pos\n";
        std::cout << "now : " << msg->fromNode << ", next : " << msg->toNode << "\n";
        std::cout << "bottery : " << msg->battery << "\n";
        if(msg->idx == 0){
            if(node[msg->fromNode].size() > 0 && node[msg->fromNode].front().robotNum == num) node[msg->fromNode].pop();
            node[msg->fromNode].push({num, msg->fromNode});
        }

        if(node[msg->toNode].size() > 0) node[msg->toNode].push({num, msg->fromNode});
        else node[msg->toNode].push({num, msg->toNode});
    }
    
};
