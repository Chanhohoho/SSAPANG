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
std::queue<ssapang::Task> taskList[2];
int burgerRobotCnt = 3;
int waffleRobotCnt = 2;

std::string burgerStartNode[31] = {"",
    "LB1112","LB1122","LB1132","LB1142","LB1152",
    "LB2112","LB2122","LB2132","LB2142","LB2152",
    "LB3112","LB3122","LB3132","LB3142","LB3152",
    "RB1112","RB1122","RB1132","RB1142","RB1152",
    "RB2112","RB2122","RB2132","RB2142","RB2152",
    "RB3112","RB3122","RB3132","RB3142","RB3152",
};
std::string waffleStartNode[31] = {"",
    
    "RB1112","RB1122","RB1132","RB1142","RB1152",
    "RB2112","RB2122","RB2132","RB2142","RB2152",
    "RB3112","RB3122","RB3132","RB3142","RB3152",
    "LB1112","LB1122","LB1132","LB1142","LB1152",
    "LB2112","LB2122","LB2132","LB2142","LB2152",
    "LB3112","LB3122","LB3132","LB3142","LB3152",
};

std::unordered_map<std::string, bool> station[2];

class ControlTower;
class Robot;
std::vector<Robot> robots[2];
ControlTower *burgerTower;
ControlTower *waffleTower;

class Robot
{
public:
    ros::Publisher waitPub, taskPub;
    ssapang::RobotWait Wait;
    Robot(std::string robot,int num, ros::NodeHandle *nh){
        
        std::string name = robot + std::to_string(num);
        type = robot == "burger" ? 0 : 1;
        robotStatus[name] = {0,type ? burgerStartNode[num] : waffleStartNode[num],100};

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
    int type;

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
        if(msg->status || taskList[type].size() == 0) return;
        ssapang::Task task = taskList[type].front();
        robots[type][num-1].taskPub.publish(task);
        taskList[type].pop();
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
        robots[type][num-1].waitPub.publish(Wait);
    }
    void go(const ssapang::str::ConstPtr &msg, std::string name){
        //로봇이 움직인 후 뺀다.
        node[msg->data].pop();
        // std::cout << name <<" now : " << msg->data << ", size : " << node[msg->data].size() << "---------------------------------------------------------------------------\n";
    }

    bool task(ssapang::End::Request &req, ssapang::End::Response &res)
    {
        if(taskList[type].size() == 0) return false;
        std::string name = req.name;
        res.task = taskList[type].front();
        taskList[type].pop();
        return true ;
    }
};

class ControlTower
{
public:
    ControlTower(ros::NodeHandle *nh, int type){
        robotType = type;
        tower_nh = new ros::NodeHandle(*nh, robotType == 0 ? "burger" : "waffle");
        taskListSub = tower_nh->subscribe<ssapang::TaskList>("task_list", 10, &ControlTower::taskListCallback, this);
        reqMinDist = nh->serviceClient<ssapang::PathLen>("/min_len");
        stationSrv = tower_nh->advertiseService("station",  &ControlTower::findStationNode,this);
        cnt = type ? burgerRobotCnt : waffleRobotCnt;
        if(type){
            for(int i = 1; i <= cnt; i++)
                station[robotType][burgerStartNode[i]] = 0;
            for(int i = cnt+1; i <= 30; i++)
                station[robotType][burgerStartNode[i]] = 1;
        }else{
            for(int i = 1; i <= cnt; i++)
                station[robotType][waffleStartNode[i]] = 0;
            for(int i = cnt+1; i <= 30; i++)
                station[robotType][waffleStartNode[i]] = 1;
        }

        for(int i = 1; i <= cnt; i++){
            Robot robot = Robot(robotType == 0 ? "burger" : "waffle",i, nh);
            robots[robotType].push_back(robot);
        }

    }
private:
    ros::NodeHandle* tower_nh;
    ros::Subscriber taskListSub, taskSub;
    ros::ServiceClient reqMinDist;
    ros::ServiceServer stationSrv;
    ros::Rate rate = 30;
    int robotType, cnt;

    void taskListCallback(const ssapang::TaskList::ConstPtr &msg){
        for(auto task: msg->list)
            taskList[robotType].push(task);
        distributeTask(robotType == 0 ? "burger" : "waffle");
        // distributeTask("waffle");
    }
    bool findStationNode(ssapang::Station::Request &req, ssapang::Station::Response &res){
        ssapang::PathLen pathLen;
        // 로봇의 현재 위치
        pathLen.request.startNode = req.nowNode;

        int st = req.num <= 15 ? 1 : 16;
        int en = req.num <= 15 ? 15 : 30;
        int minLen = 1000;
        int minIdx = -1;

        for(int i = st; i <= en; i++){
                // 각 충전소의 노드 
                pathLen.request.endNode = robotType == 0 ? burgerStartNode[i] : waffleStartNode[i];
                if(station[robotType][pathLen.request.endNode] == false)continue;

                if(robotType == 0 ? burgerTower->reqMinDist.call(pathLen) : waffleTower->reqMinDist.call(pathLen)){  
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
        res.stationNode = robotType == 0 ? burgerStartNode[minIdx] : waffleStartNode[minIdx];
        // robotStatus["burger"+std::to_string(r)]
        station[robotType][res.stationNode] = false;
        return true;
    }

    void distributeTask(std:: string name){
        ssapang::PathLen pathLen;
        int minLen = 100000;
        int robotNum;

        int len = std::min(cnt, int(taskList[robotType].size()));

        for(int i = 0; i < len; i++){
            auto task = taskList[robotType].front();
            minLen = 0x7fff0000;
            robotNum = -1;
            for(int i = 1; i <= cnt; i++){
                if(robotStatus[name +std::to_string(i)].status != 0 && robotStatus[name +std::to_string(i)].status != 3) continue;
                pathLen.request.startNode = robotStatus[name +std::to_string(i)].nowIdx;
                pathLen.request.endNode = task.product;
                if(robotType == 0 ? burgerTower->reqMinDist.call(pathLen) : waffleTower->reqMinDist.call(pathLen)){  
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
            taskList[robotType].pop();
            robots[robotType][robotNum-1].taskPub.publish(task);
            std::string node = robotType == 0 ? burgerStartNode[robotNum] : waffleStartNode[robotNum];
            if(!station[robotType][node]){
                station[robotType][node] = true;
            }

            robotStatus[name +std::to_string(robotNum)].status = 1;

        }
    }

};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "tower");
    ros::NodeHandle nh1, nh2;
    // ros::AsyncSpinner spinner(0);
    // spinner.start();
    // for(int i = 0; i <= 0; i++){
    //     ControlTower tower = ControlTower(&nh,0);
    //     towers.push_back(tower);
    // }
    ControlTower tower0 = ControlTower(&nh2,0);
    ControlTower tower1 = ControlTower(&nh1,1);
    burgerTower = &tower0;
    waffleTower = &tower1;
    
    // ros::waitForShutdown();
    ros::spin();

    return 0;
}

