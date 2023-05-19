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

        for(int i = 0; i < 16; i++){
            task.destination = destination[i][rand()%4];
            task.product = sectTaskList[i];
            
            taskList.list.push_back(task);
        }

        taskPub.publish(taskList);
    }

private:
    ros::Publisher taskPub;
    std::string startNode, endNode;
    ssapang::Task task;
    ssapang::TaskList taskList;
    int shelfStatus[16]={0,};
    std::string sectTaskList[16] = {
        "LWO14",
        "LWO24",
        "LWO34",
        "LWO44",
        "RWO14",
        "RWO24",
        "RWO34",
        "RWO44",

        "LWO13",
        "LWO23",
        "LWO33",
        "LWO43",
        "RWO13",
        "RWO23",
        "RWO33",
        "RWO43",
    };

    std::string destination[18][4] = {
        {"WBP0101", "WBP0102", "WBP0201", "WBP0202"},

        {"WBP0103","WBP0104", "WBP0203","WBP0204"},

        {"WBP0105","WBP0106", "WBP0205","WBP0206"},

        {"WBP0301", "WBP0302", "WBP0401", "WBP0402"},

        {"WBP0107", "WBP0108", "WBP0207", "WBP0208"},
        {"WBP0109","WBP0110","WBP0209","WBP0210"},
        {"WBP0111","WBP0112","WBP0211","WBP0212"},
        {"WBP0307", "WBP0308", "WBP0407", "WBP0408"},




        {"WBP0303","WBP0304", "WBP0403","WBP0404"},

        {"WBP0305","WBP0306", "WBP0405","WBP0406"},

        {"WBP0501", "WBP0502", "WBP0601", "WBP0602"},
        
        {"WBP0503","WBP0504", "WBP0603","WBP0604"},

         {"WBP0309","WBP0310","WBP0409","WBP0410"},
        {"WBP0311","WBP0312","WBP0411","WBP0412"},
        {"WBP0507", "WBP0508", "WBP0607","WBP0608"},
        {"WBP0509","WBP0510","WBP0609","WBP0610"},


        {"WBP0505","WBP0506", "WBP0605","WBP0606"},

        
       
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