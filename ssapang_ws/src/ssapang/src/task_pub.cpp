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
    Task(int argc, char **argv, ros::NodeHandle *nh){
        taskPub = nh->advertise<ssapang::TaskList>("/task_list", 1);

        ros::Rate loop_rate(10);
        sleep(3);
        // while (ros::ok()){
            for(int i = 0; i < 5; i++){
                task.product = rightSectTaskList[rand()%9][rand()%4];
                task.destination = destination[rand()%7];
                taskList.list.push_back(task);
            }

            taskPub.publish(taskList);
            // loop_rate.sleep();
        // }
    }

private:
    ros::Publisher taskPub;
    std::string startNode, endNode;
    ssapang::Task task;
    ssapang::TaskList taskList;
    std::string rightSectTaskList[9][4] = {
        {"BP0101", "BP0102", "BP0201", "BP0202"},
        {"BP0103","BP0104", "BP0203","BP0204"},
        {"BP0105","BP0106", "BP0205","BP0206"},
        {"BP0301", "BP0302", "BP0401", "BP0402"},
        {"BP0303","BP0304", "BP0403","BP0404"},
        {"BP0305","BP0306", "BP0405","BP0406"},
        {"BP0501", "BP0502", "BP0601", "BP0602"},
        {"BP0503","BP0504", "BP0603","BP0604"},
        {"BP0505","BP0506", "BP0605","BP0606"}
    };
    std::string leftSectTaskList[9][4] = {
        {"BP0107", "BP0108", "BP0207", "BP0208"},
        {"BP0109","BP0110","BP0209","BP0210"},
        {"BP0111","BP0112","BP0211","BP0212"},
        {"BP0307", "BP0308", "BP0407", "BP0408"},
        {"BP0309","BP0310","BP0409","BP0410"},
        {"BP0311","BP0312","BP0411","BP0412"},
        {"BP0507", "BP0508", "BP0607","BP0608"},
        {"BP0509","BP0510","BP0609","BP0610"},
        {"BP0511","BP0512","BP0611","BP0612"}
    };

    // std::string destination[15] = {
    //     "BO0101","BO0102","BO0103","BO0104","BO0105",
    //     "BO0106","BO0107","BO0108","BO0109","BO0110",
    //     "BO0111","BO0112","BO0113","BO0114","BO0115",
    // };
    std::string destination[7] = {
        "BO0101","BO0102","BO0103","BO0104","BO0105",
        "BO0106","BO0107",
    };
};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "task_pub");
    ros::NodeHandle nh;
    Task task(argc, argv, &nh);
    ros::spin();

    return 0;
}