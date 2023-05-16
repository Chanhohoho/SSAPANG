#include <ros/ros.h>
#include <random>
#include <ssapang/Task.h>
#include <ssapang/TaskList.h>
#include <time.h>
#include <unistd.h>
#include <iostream>
#include <stdlib.h>
#include <unistd.h>

using namespace std;

class Task
{
public:
    Task(ros::NodeHandle *nh){
        taskPub = nh->advertise<ssapang::TaskList>( "/waffle/task_list", 1);

        ros::Rate loop_rate(10);
        sleep(3);
        for(int i = 0; i < 15; i++){
            task.product = sectTaskList[rand()%8];
            task.destination = destination[rand()%9][rand()%4];
            taskList.list.push_back(task);
        }

        taskPub.publish(taskList);
    }

private:
    ros::Publisher taskPub;
    std::string startNode, endNode;
    ssapang::Task task;
    ssapang::TaskList taskList;

    std::string sectTaskList[8] = {
        "LWO13",//"LWO14",
        "LWO23",//"LWO24",
        "LWO33",//"LWO34",
        "LWO43",//"LWO44",

        "RWO13",//"RWO14",
        "RWO23",//"RWO24",
        "RWO33",//"RWO34",
        "RWO43",//"RWO44",
    };

    std::string destination[9][4] = {
        {"WBP0107", "WBP0108", "WBP0207", "WBP0208"},
        {"WBP0109","WBP0110","WBP0209","WBP0210"},
        {"WBP0111","WBP0112","WBP0211","WBP0212"},
        {"WBP0307", "WBP0308", "WBP0407", "WBP0408"},
        {"WBP0309","WBP0310","WBP0409","WBP0410"},
        {"WBP0311","WBP0312","WBP0411","WBP0412"},
        {"WBP0507", "WBP0508", "WBP0607","WBP0608"},
        {"WBP0509","WBP0510","WBP0609","WBP0610"},
        {"WBP0511","WBP0512","WBP0611","WBP0612"}
    };

    
};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "waffle_task_pub");
    ros::NodeHandle nh;
    Task burger = Task(&nh);
    ros::spin();

    return 0;
}