#include <ros/ros.h>
#include <ssapang/RobotPos.h>
#include <ssapang/RobotStatus.h>
#include <ssapang/RobotWait.h>
#include <ssapang/Task.h>
#include <ssapang/str.h>
#include <string>
#include <unordered_map>
#include <queue>
#include <unistd.h>

struct info{
    std::string robotName;
    std::string beforeNode;
};
std::unordered_map<std::string, std::queue<info>> node;

class Robot
{
public:
    ssapang::RobotWait Wait;
    Robot(std::string robot,int num, ros::NodeHandle *nh){
        std::string name = robot + std::to_string(num);
        this->waitPub = nh->advertise<ssapang::RobotWait>(name+"/wait", 10);
        this->taskPub = nh->advertise<ssapang::Task>(name+"/task", 10);
        this->RobotStatusSub = nh->subscribe<ssapang::RobotStatus>(name+"/status", 10, boost::bind(&Robot::status, this, _1, name));
        this->RobotPosSub = nh->subscribe<ssapang::RobotPos>(name+"/pos", 10, boost::bind(&Robot::pos, this, _1, name));
        this->checkGoSub = nh->subscribe<ssapang::str>(name+"/checkGo", 10, boost::bind(&Robot::checkGo, this, _1, name, num));
        this->goSub = nh->subscribe<ssapang::str>(name+"/Go", 10, boost::bind(&Robot::go, this, _1, name));
    }

private:
    int cnt = 0;
    ros::Publisher waitPub, taskPub;
    ros::Subscriber RobotStatusSub, RobotPosSub, checkGoSub, goSub; 
    ros::Rate rate = 30;
    void status(const ssapang::RobotStatus::ConstPtr &msg, std::string name){
        // printf("&s:status\n", robotName);
        // printf("&d\n", msg->status);
        std::cout << name << "-status\n" << (msg->status) << "\n";
    }

    void pos(const ssapang::RobotPos::ConstPtr &msg, std::string name){
        if(msg->idx == 1){
            if(node[msg->fromNode].size() > 0 && node[msg->fromNode].front().robotName == name) node[msg->fromNode].pop();
            node[msg->fromNode].push({name, msg->fromNode});
        }
        if(node[msg->toNode].size() > 0) node[msg->toNode].push({name, msg->fromNode});
        else node[msg->toNode].push({name, msg->toNode});
        std::cout << name << "-pos "<< msg->idx<<"\n";
        std::cout << "now : " << msg->fromNode << ", size : " << node[msg->fromNode].size() << ", front : " << node[msg->fromNode].front().robotName  << "\n";
        std::cout << "next : " << msg->toNode << ", size : " << node[msg->toNode].size() << ", front : " << node[msg->toNode].front().robotName << "\n";
    }

    void checkGo(const ssapang::str::ConstPtr &msg, std::string name, int num){
        // std::cout << name <<" go?\n"; 
        msg->data;
        if(node[msg->data].size() == 0 || node[msg->data].front().robotName != name || cnt)
            return;
        this->Wait.wait = 0;
        std::cout <<node[msg->data].front().robotName << ", " << name << ", " << msg->data <<" wait - " << Wait.wait << "\n";
        this->waitPub.publish(Wait);
        rate.sleep();
        cnt++;
    }
    void go(const ssapang::str::ConstPtr &msg, std::string name){
        node[msg->data].pop();
        cnt = 0;
        std::cout << "\n" << name <<" now : " << msg->data << ", size : " << node[msg->data].size() << "---------------------------------------------------------------------------\n";
    }
    
};
