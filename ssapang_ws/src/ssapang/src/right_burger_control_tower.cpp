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
std::unordered_map<std::string, std::queue<std::string>> node;
std::unordered_map<std::string, status> robotStatus;
std::queue<ssapang::Task> taskList;
int robotCnt = 3;

std::string startNode[13] = {"",
    "RB1112","RB1122",
    "RB1132","RB2112",
    "RB2122","RB2132",

    "RB3112","RB3122",
    "RB3132","RB4112",
    "RB4122","RB4132",
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
        // std::cout << name << "-status: " << msg->status <<"\n";
        robotStatus[name].status = msg->status;
        if(msg->status == -1){
            // std::cout << name << "- 충전완료, " << taskList.size() << "\n";
            robotStatus[name].status = 0;
            robotStatus[name].battery = 100;
            if(robotStatus[name].status || taskList.size() == 0) return;
            std::cout << name << "일 재할당\n";
            ssapang::Task task = taskList.front();
            robots[num-1].taskPub.publish(task);
            taskList.pop();
            robotStatus[name].status = 1;
        }
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
    }

    void checkGo(const ssapang::str::ConstPtr &msg, std::string name, int num){
        // std::cout << name <<" go?\n"; 
        if(node[msg->data].size() == 0 || node[msg->data].front() != name)
            return;
        Wait.wait = 0;
        robots[num-1].waitPub.publish(Wait);
    }
    void go(const ssapang::str::ConstPtr &msg, std::string name){
        //로봇이 움직인 후 뺀다.
        node[msg->data].pop();
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
        taskListSub = tower_nh->subscribe<ssapang::TaskList>("right_task_list", 10, &ControlTower::taskListCallback, this);
        reqMinDist = nh->serviceClient<ssapang::PathLen>("/min_len");
        stationSrv = tower_nh->advertiseService("right_station",  &ControlTower::findStationNode,this);

        for(int i = robotCnt+1; i < 13; i++)
            station[startNode[i]] = 1;
        
        for(int i = 1; i <= robotCnt; i++){
            station[startNode[i]] = 0;
            Robot robot = Robot("rburger",i, nh);
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
        distributeTask("rburger");
    }
    bool findStationNode(ssapang::Station::Request &req, ssapang::Station::Response &res){
        ssapang::PathLen pathLen;
        // 로봇의 현재 위치
        pathLen.request.startNode = req.nowNode;
        pathLen.request.type = "burger";

        int minLen = 1000;
        int minIdx = -1;

        for(int i = 1; i < 13; i++){
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
        station[res.stationNode] = false;
        return true;
    }

    void distributeTask(std:: string name){
        ssapang::PathLen pathLen;
        int minLen = 0x7fff0000;
        int robotNum;

        int len = std::min(robotCnt, int(taskList.size()));
        pathLen.request.type = "burger";

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
            // std::cout << "select robot - " << robotNum << "\n";
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
    ros::init(argc, argv, "right_burger_tower");
    ros::NodeHandle nh;
    // ros::AsyncSpinner spinner(0);
    // spinner.start();
    ControlTower ControlTower(&nh);
    
    // ros::waitForShutdown();
    ros::spin();

    return 0;
}

