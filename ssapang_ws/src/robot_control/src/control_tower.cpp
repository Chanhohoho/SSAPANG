#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <iostream>
#include <vector>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <robot_control/TaskList.h>
#include <robot_control/RobotPos.h>
#include <turtlesim/Pose.h>
#include <string>
using namespace std;
class controlTower
{
public:
    controlTower(int argc, char **argv){
        run();
    };

    void run(){
        

        for(int i=1; i<=3; i++){
            std::stringstream ss;
            ss << "robot" << i << "/odom";
            std::string topic_name = ss.str();
            tb_subs[i] = nh.subscribe<nav_msgs::Odometry>(topic_name, 10, boost::bind(&controlTower::callback, this, _1, i, &nowPosition[i]));
        }
        ros::spin();
    }

    
    void callback(const nav_msgs::Odometry::ConstPtr &msg, int idx, turtlesim::Pose *tb_pose)
    {
        turtlesim::Pose pose;
        pose.x =  msg->pose.pose.position.x;
        pose.y = msg->pose.pose.position.y;
        tf::Quaternion quat;
        tf::quaternionMsgToTF(msg->pose.pose.orientation, quat);
        double roll, pitch, yaw;
        tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
        pose.theta = yaw;
        *tb_pose = pose;
        std::cout << "robot " << idx << " position: (" << pose.x << ", " << pose.y << ")" << std::endl;
    }

private:

    ros::NodeHandle nh;

    std::vector <ros::Publisher> taskPub;
    std::vector <ros::Subscriber> robotSub;
    
    ros::Rate rate = 10;
    turtlesim::Pose nowPosition[4];  
    ros::Subscriber tb_subs[10];

};
int main(int argc, char **argv){
    ros::init(argc,argv,"control_tower");
    controlTower controlTower(argc, argv);
    return 0;
}