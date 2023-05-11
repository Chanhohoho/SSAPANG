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
#include <ssapang/PathLen.h>
#include <ssapang/Station.h>
#include <time.h>
#include <unistd.h>
#include <iostream>
#include <stdlib.h>
#include <vector>
#include <string>
#include <unordered_map>
#include <queue>
#include <list>
#include <boost/thread.hpp>

struct status{
    int status;
    std::string nowIdx;
    double battery;
};
std::list <std::string> batteryStation1;
std::list <std::string> batteryStation2;
std::unordered_map<std::string, std::queue<std::string>> node;
std::unordered_map<std::string, status> robotStatus;
int robotCnt = 10;
struct Station{
    std::string node;
    bool empty;
};

Station station[21] = {{"",0},
        {"LB1112",0},{"LB1122",0},{"LB1132",0},{"LB1142",0},{"LB1152",0},
        {"LB2112",0},{"LB2122",0},{"LB2132",0},{"LB2142",0},{"LB2152",0},
        {"RB1112",0},{"RB1112",0},{"RB1112",0},{"RB1112",0},{"RB1112",0}, 
        {"RB1112",0},{"RB1112",0},{"RB1112",0},{"RB1112",0},{"RB1112",0}
        };

class Robot;
std::vector<Robot> robots;

class Robot
{
public:
    ros::Publisher waitPub, taskPub;
    ssapang::RobotWait Wait;
    Robot(std::string robot,int num, ros::NodeHandle *nh){
        
        std::string name = robot + std::to_string(num);
        robotStatus[name] = {0,station[num].node,100};
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
    ros::Subscriber RobotStatusSub, RobotPosSub, checkGoSub, goSub; 
    ros::Rate rate = 30;

    void status(const ssapang::RobotStatus::ConstPtr &msg, std::string name){
        // printf("&s:status\n", robotName);
        // printf("&d\n", msg->status);
        std::cout << name << "-status\n";
    }

    void pos(const ssapang::RobotPos::ConstPtr &msg, std::string name){
        if(msg->idx == 1){
            //경로의 첫번째일 때 
            // if(node[msg->fromNode].size() > 0 && node[msg->fromNode].front() == name) {
            //     //이전 움직일 때 현재 노드의 큐 안에 대기가 자신일 때 삭제 후 이동하도록 
            //     node[msg->fromNode].pop();
            // }
            //현재 노드안에 과거 이동 기록 
            if(node[msg->fromNode].size() == 0 || node[msg->fromNode].front() != name)
                node[msg->fromNode].push(name);
        }
        node[msg->toNode].push(name);
        
        // std::cout << name << "-pos "<< msg->idx<<"\n";
        // std::cout << "now : " << msg->fromNode << ", size : " << node[msg->fromNode].size() << ", front : " << (node[msg->fromNode].size() ? node[msg->fromNode].front() : "None") << "\n";
        // std::cout << "next : " << msg->toNode << ", size : " << node[msg->toNode].size() << ", front : " << (node[msg->toNode].size() ? node[msg->toNode].front() : "None") << "\n";
    }

    void checkGo(const ssapang::str::ConstPtr &msg, std::string name, int num){
        // std::cout << name <<" go?\n"; 
        if(node[msg->data].size() == 0 || node[msg->data].front() != name){
            // 다음 노드에 대기 로봇이 있고 대기 순번이 가장 앞이 아닐 때 대기
            return;
        }
        std::cout << "너 감 - " << name << " - "<<msg->data << "\n";
        // std::cout <<node[msg->data].front() << ", " << name << ", " << msg->data <<" wait - " << Wait.wait << "\n";
        Wait.wait = 0;
        robots[num-1].waitPub.publish(Wait);
        // rate.sleep();
        sw = true;
    }
    void go(const ssapang::str::ConstPtr &msg, std::string name){
        //로봇이 움직인 후 뺀다.
        node[msg->data].pop();
        sw = false;
        // std::cout << "\n" << name <<" now : " << msg->data << ", size : " << node[msg->data].size() << "---------------------------------------------------------------------------\n";
    }
};

class ControlTower
{
public:
    ControlTower(ros::NodeHandle *nh){
        taskListSub = nh->subscribe<ssapang::TaskList>("/task_list", 10, &ControlTower::taskListCallback, this);
        reqMinDist = nh->serviceClient<ssapang::PathLen>("/min_len");
        stationSrv = nh->advertiseService("/station",  &ControlTower::findStationNode,this);
        for(int i = 1; i <= robotCnt; i++){
            Robot robot = Robot("burger",i, nh);
            robots.push_back(robot);
        }

    }
private:
    ros::Subscriber taskListSub, taskSub;
    ros::ServiceClient reqMinDist;
    ros::ServiceServer stationSrv;
    std::vector<ssapang::Task> taskList;
    ros::Rate rate = 30;
    std::string robotName, shelfNode;

    void taskListCallback(const ssapang::TaskList::ConstPtr &msg){
        taskList = msg->list;
        std::string name = "burger";

        distributeTask();
    }
    bool findStationNode(ssapang::Station::Request &req, ssapang::Station::Response &res){
        ssapang::PathLen pathLen;
        // 로봇의 현재 위치
        pathLen.request.startNode = req.nowNode;

        int add = (req.num-1)/10*10;
        int minLen = 1000;
        int minIdx = -1;

        for(int i = 1 + add; i <= 10 + add; i++){
                // 각 충전소의 노드 
                if(station[i].empty == false)continue;
                pathLen.request.endNode = station[i].node;

                if(ControlTower::reqMinDist.call(pathLen)){  
                    int len = pathLen.response.len;
                    if(len < minLen){
                        minLen = len;   
                        minIdx = i;
                    }              
                }
                else{
                    ROS_ERROR("find StationNode fail");
                    return false;
                }
        }
        if(minIdx == -1)return false;
        res.stationNode = station[minIdx].node;
        station[minIdx].empty = false;
        return true;
    }
    void distributeTask(){
        std::string name = "burger";
        ssapang::PathLen pathLen;
        int minLen = 100000;
        int robotNum;

        for(auto task:taskList){
            minLen = 0x7fff0000;
            robotNum = -1;
            for(int i = 1; i <= robotCnt; i++){
                if(robotStatus[name +std::to_string(i)].status != 0 && robotStatus[name +std::to_string(i)].status != 3) continue;
                pathLen.request.startNode = robotStatus[name +std::to_string(i)].nowIdx;
                pathLen.request.endNode = task.product;
                if(reqMinDist.call(pathLen)){  
                    int len = pathLen.response.len;
                    if(len < minLen){
                        minLen = len;   
                        robotNum = i;
                    }              
                }
                else{
                    ROS_ERROR("fail");
                }
                rate.sleep();
            }
            if(robotNum == -1) continue;
            // std::cout << "select robot - " << robotNum << "\n";
            robots[robotNum-1].taskPub.publish(task);
            if(!station[robotNum].empty){
                station[robotNum].empty = true;
            }
            robotStatus[name +std::to_string(robotNum)].status = 1;
            
        }
    }

};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "tower");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(0);
    spinner.start();
    ControlTower tower = ControlTower(&nh);
    ros::waitForShutdown();
    // ros::spin();

    return 0;
}

