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
#include <ssapang/End.h>
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
struct status{
    int status;
    std::string nowIdx;
    double battery;
};
std::list <std::string> batteryStation1;
std::list <std::string> batteryStation2;
std::unordered_map<std::string, std::queue<std::string>> node;
std::unordered_map<std::string, status> robotStatus;
std::queue<ssapang::Task> taskList;
int robotCnt = 3;

std::string startNode[31] = {"",
    "RW21","RW22","RW23","RW24","RW25",
    "LB2112","LB2122","LB2132","LB2142","LB2152",
    "LB3112","LB3122","LB3132","LB3142","LB3152",
    "RB1112","RB1122","RB1132","RB1142","RB1152",
    "RB2112","RB2122","RB2132","RB2142","RB2152",
    "RB3112","RB3122","RB3132","RB3142","RB3152",
};

std::unordered_map<std::string, bool> station;

class Robot;
std::vector<Robot> robots;

class Robot
{
public:
    ros::Publisher waitPub, taskPub;
    ssapang::RobotWait Wait;
    Robot(std::string robot,int num, ros::NodeHandle *nh){
        
        std::string name = robot + std::to_string(num);
        robotStatus[name] = {0, startNode[num],100};

        robot_nh = new ros::NodeHandle(*nh, name);
        this->waitPub = robot_nh->advertise<ssapang::RobotWait>("wait", 10);
        this->taskPub = robot_nh->advertise<ssapang::Task>("task", 10);
        this->RobotStatusSub = robot_nh->subscribe<ssapang::RobotStatus>("status", 10, boost::bind(&Robot::status, this, _1, name, num));
        this->RobotPosSub = robot_nh->subscribe<ssapang::RobotPos>("pos", 10, boost::bind(&Robot::pos, this, _1, name));
        this->checkGoSub = robot_nh->subscribe<ssapang::str>("checkGo", 10, boost::bind(&Robot::checkGo, this, _1, name, num));
        this->goSub = robot_nh->subscribe<ssapang::str>("Go", 10, boost::bind(&Robot::go, this, _1, name));
        this->endSrv = robot_nh->advertiseService("end", &Robot::task, this);
    }

private:
    ros::NodeHandle* robot_nh;
    ros::Subscriber RobotStatusSub, RobotPosSub, checkGoSub, goSub;
    ros::ServiceServer endSrv;
    ros::Rate rate = 30;

    void status(const ssapang::RobotStatus::ConstPtr &msg, std::string name, int num){
        // printf("&s:status\n", robotName);
        // printf("&d\n", msg->status);
        std::cout << name << "-status: " << msg->status <<"\n";
        robotStatus[name].status = msg->status;
        if(msg->status == -1){
            std::cout << name << "- 충전완료\n";
            robotStatus[name].status = 0;
        }else{
            robotStatus[name].status = msg->status;
        }
        if(msg->status || taskList.size() == 0) return;
        ssapang::Task task = taskList.front();
        robots[num-1].taskPub.publish(task);
        taskList.pop();
        robotStatus[name].status = 1;
    }

    void pos(const ssapang::RobotPos::ConstPtr &msg, std::string name){
        if(msg->idx == 1){
            //경로의 첫번째일 때 
            if(node[msg->fromNode].size() == 0 || node[msg->fromNode].front() != name) {
                //이전 움직일 때 현재 노드의 큐 안에 대기가 자신일 때 삭제 후 이동하도록 
                node[msg->fromNode].push(name);
            }
            //현재 노드안에 과거 이동 기록 
        }
        node[msg->toNode].push(name);
        robotStatus[name].nowIdx = msg->fromNode;
        robotStatus[name].battery = msg->battery;
        // std::cout << name << "-pos "<< msg->idx<<"\n";
        // std::cout << "now : " << msg->fromNode << ", size : " << node[msg->fromNode].size() << ", front : " << (node[msg->fromNode].size() ? node[msg->fromNode].front() : "None") << "\n";
        // std::cout << "next : " << msg->toNode << ", size : " << node[msg->toNode].size() << ", front : " << (node[msg->toNode].size() ? node[msg->toNode].front() : "None") << "\n";
    }

    void checkGo(const ssapang::str::ConstPtr &msg, std::string name, int num){
        // std::cout << name <<" go?\n"; 
        if(node[msg->data].size() == 0 || node[msg->data].front() != name){
                // std::cout << "너 못감 - " << name << " - "<<msg->data << "\n";
            // Wait.wait = 2;
            return;
        }else{
            // std::cout << "너 감 - " << name << " - "<<msg->data << "\n";
            // std::cout <<node[msg->data].front() << ", " << name << ", " << msg->data <<" wait - " << Wait.wait << "\n";
            Wait.wait = 0;

        }
        robots[num-1].waitPub.publish(Wait);
    }
    void go(const ssapang::str::ConstPtr &msg, std::string name){
        //로봇이 움직인 후 뺀다.
        node[msg->data].pop();
        // std::cout << name <<" now : " << msg->data << ", size : " << node[msg->data].size() << "---------------------------------------------------------------------------\n";
    }

    bool task(ssapang::End::Request &req, ssapang::End::Response &res)
    {
        if(taskList.size() == 0) return false;
        std::string name = req.name;
        res.task = taskList.front();
        taskList.pop();
        return true ;
    }
};

class ControlTower
{
public:
    ControlTower(ros::NodeHandle *nh){
        tower_nh = new ros::NodeHandle(*nh, "burger" );
        taskListSub = tower_nh->subscribe<ssapang::TaskList>("task_list", 10, &ControlTower::taskListCallback, this);
        reqMinDist = nh->serviceClient<ssapang::PathLen>("/min_len");
        stationSrv = tower_nh->advertiseService("station",  &ControlTower::findStationNode,this);

        for(int i = robotCnt+1; i <= 30; i++)
            station[startNode[i]] = 1;
        
        for(int i = 1; i <= robotCnt; i++){
            station[startNode[i]] = 0;
            Robot robot = Robot("waffle",i, nh);
            robots.push_back(robot);
        }

    }
private:
    ros::NodeHandle* tower_nh;
    ros::Subscriber taskListSub, taskSub;
    ros::ServiceClient reqMinDist;
    ros::ServiceServer stationSrv;
    ros::Rate rate = 30;

    void taskListCallback(const ssapang::TaskList::ConstPtr &msg){
        for(auto task: msg->list)
            taskList.push(task);
        distributeTask("waffle");
        // distributeTask("waffle");
    }
    bool findStationNode(ssapang::Station::Request &req, ssapang::Station::Response &res){
        ssapang::PathLen pathLen;
        // 로봇의 현재 위치
        pathLen.request.startNode = req.nowNode;
        pathLen.request.type = "waffle";

        int st = req.num <= 15 ? 1 : 16;
        int en = req.num <= 15 ? 15 : 30;
        int minLen = 1000;
        int minIdx = -1;

        for(int i = st; i <= en; i++){
                // 각 충전소의 노드 
                pathLen.request.endNode = startNode[i];
                if(station[pathLen.request.endNode] == false)continue;

                if(reqMinDist.call(pathLen)){  
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
        res.stationNode = startNode[minIdx];
        // robotStatus["burger"+std::to_string(r)]
        station[res.stationNode] = false;
        return true;
    }

    void distributeTask(std:: string name){
        ssapang::PathLen pathLen;
        int minLen = 100000;
        int robotNum;

        int len = std::min(robotCnt, int(taskList.size()));
        pathLen.request.type = "waffle";

        for(int i = 0; i < len; i++){
            auto task = taskList.front();
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
            std::cout << "select robot - " << robotNum << "\n";
            taskList.pop();
            robots[robotNum-1].taskPub.publish(task);
            std::string node = startNode[robotNum];
            if(!station[node]){
                station[node] = true;
            }

            robotStatus[name +std::to_string(robotNum)].status = 1;

        }
    }

};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "waffle_tower");
    ros::NodeHandle nh1, nh2;
    // ros::AsyncSpinner spinner(0);
    // spinner.start();
    ControlTower ControlTower(&nh2);
    
    // ros::waitForShutdown();
    ros::spin();

    return 0;
}

