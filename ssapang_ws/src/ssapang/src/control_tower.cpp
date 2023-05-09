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
#include <ssapang/str.h>
#include <time.h>
#include <unistd.h>
#include <iostream>
#include <stdlib.h>
#include <vector>
#include <string>
#include <unordered_map>
#include <queue>

struct info{
    std::string robotName;
    std::string beforeNode;
};
std::unordered_map<std::string, std::queue<info>> node;
class Robot;
std::vector<Robot> robots;

class Robot
{
public:
    ssapang::RobotWait Wait;
    Robot(std::string robot,int num, ros::NodeHandle *nh){
        
        std::string name = robot + std::to_string(num);
        robot_nh = new ros::NodeHandle(*nh, name);
        this->waitPub = robot_nh->advertise<ssapang::RobotWait>("wait", 10);
        this->taskPub = robot_nh->advertise<ssapang::Task>("task", 10);
        this->RobotStatusSub = robot_nh->subscribe<ssapang::RobotStatus>("status", 10, boost::bind(&Robot::status, this, _1, name));
        this->RobotPosSub = robot_nh->subscribe<ssapang::RobotPos>("pos", 10, boost::bind(&Robot::pos, this, _1, name));
        this->checkGoSub = robot_nh->subscribe<ssapang::str>("checkGo", 10, boost::bind(&Robot::checkGo, this, _1, name, num));
        this->goSub = robot_nh->subscribe<ssapang::str>("Go", 10, boost::bind(&Robot::go, this, _1, name));
    }

private:
    ros::NodeHandle* robot_nh;
    bool sw = false;
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
        std::cout << "now : " << msg->fromNode << ", size : " << node[msg->fromNode].size() << ", front : " << (node[msg->fromNode].size() ? node[msg->fromNode].front().robotName : "None") << "\n";
        std::cout << "next : " << msg->toNode << ", size : " << node[msg->toNode].size() << ", front : " << (node[msg->toNode].size() ? node[msg->toNode].front().robotName : "None") << "\n";
    }

    void checkGo(const ssapang::str::ConstPtr &msg, std::string name, int num){
        // std::cout << name <<" go?\n"; 
        if(node[msg->data].size() == 0 || node[msg->data].front().robotName != name || sw)
            return;
        std::cout <<node[msg->data].front().robotName << ", " << name << ", " << msg->data <<" wait - " << Wait.wait << "\n";
        Wait.wait = 0;
        robots[num-1].waitPub.publish(Wait);
        // rate.sleep();
        sw = true;
    }
    void go(const ssapang::str::ConstPtr &msg, std::string name){
        node[msg->data].pop();
        sw = false;
        std::cout << "\n" << name <<" now : " << msg->data << ", size : " << node[msg->data].size() << "---------------------------------------------------------------------------\n";
    }
    
    
};

class ControlTower
{
public:
    ControlTower(ros::NodeHandle *nh){
        for(int i = 1; i <= 9; i++){
            Robot robot = Robot("burger",i, nh);
            robots.push_back(robot);
            robotStatus.push_back(0);
        }
    }
private:
    ros::Subscriber taskListSub, taskSub;

    std::vector<int> robotStatus;
    ssapang::TaskList taskList;
    
    ros::Rate rate = 30;
    std::string robotName, shelfNode;

};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "tower");
    ros::NodeHandle nh;
    ControlTower tower = ControlTower(&nh);
    ros::spin();

    return 0;
}