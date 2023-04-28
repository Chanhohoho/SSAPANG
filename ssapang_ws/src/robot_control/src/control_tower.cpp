#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <iostream>
#include <vector>
#include <robot_control/TaskList.h>
#include <string>

class controlTower
{
public:
    controlTower(int argc, char **argv);
    void init(){
        for(int i = 1; i <= robotCnt; i++){
            taskPub.push_back(nh.advertise<robot_control::TaskList>("robot" + std::to_string(i), 10));
        }
    }
    void run(){
        
    }
private:
    int robotCnt = 0;
    ros::NodeHandle nh;
    std::vector <ros::Publisher> taskPub;
    ros::Subscriber

};
controlTower::controlTower(int argc, char **argv){

}
int main(int argc, char **argv){
    ros::init(argc,argv,"control_tower");
    controlTower controlTower(argc, argv);
    return 0;
}