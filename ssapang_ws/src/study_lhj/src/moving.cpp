 
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_datatypes.h>
#include <turtlesim/Pose.h>
#include <nav_msgs/Odometry.h>
#include <iostream>
#include <study_lhj/Locations.h>
#include <study_lhj/Coordinate.h>
#include <study_lhj/Move.h>
#include <study_lhj/str.h>
#include <study_lhj/RobotWait.h>
#include <study_lhj/RobotPos.h>
#include <study_lhj/RobotStatus.h>
#include <study_lhj/Task.h>
#include <vector>
#include <string>

using std::sqrt;
using std::pow;
using std::atan2;
using std::abs;
using std::max;
using std::min;

const double PI = std::acos(-1);

class Robot
{
public:
    Robot(int argc, char **argv, ros::NodeHandle *nh){
        cmdPub = nh->advertise<geometry_msgs::Twist>("cmd_vel", 10);
        movePub = nh->advertise<study_lhj::Move>("move", 10);
        posPub = nh->advertise<study_lhj::RobotPos>("pos", 10);
        statusPub = nh->advertise<study_lhj::RobotStatus>("status", 10);


        odomSub = nh->subscribe("odom", 10, &Robot::odomCallback, this);
        shelfSub = nh->subscribe("shelf", 10, &Robot::shelfCallback, this);
        pathSub = nh->subscribe("path", 10, &Robot::pathCallback, this);
        waitSub = nh->subscribe("wait", 10, &Robot::waitCallback, this);


        stop.angular.z = 0.0;
        stop.linear.x = 0.0;

        linearSpeed = 0.22;
        angularSpeed = 1.0;
        idx = 0;
        status = 0;
        battery = 100.0;

        nh->getParam("robotName", robotName);

        try
        { 
            while (ros::ok())
            {
                ros::spinOnce();
                // std::cout << path.size() << std::endl;
                if(wait ||path.size() == 0 ) continue;
                std::cout << idx << '/' << path.size() << "\n";
                nextIdx();
                std::cout << nh->getNamespace() << " - " << nextPos[0] << ", " << nextPos[1] << ", " << nextPos[2] << std::endl;
                turn();
                go();
                std::cout << robotName<< "arrival idx..." << std::endl;
                study_lhj::RobotPos pos;
                pos.fromNode = std::to_string(nextPos[0]) +','+std::to_string(nextPos[1]);
                idx++;
                pos.toNode = std::to_string(path[idx].x) +','+std::to_string(path[idx].y);
                pos.battery = --battery;
                posPub.publish(pos);
                if(idx >= path.size()) {
                    path.clear();
                    idx = 0;
                    status++;
                    // statusPub.publish(status);
                }
            }
        }
        catch (const std::exception &e)
        {
            std::cerr << "Exception occurred: " << e.what() << std::endl;
        }
        catch (...)
        {
            std::cerr << "Unknown exception occurred." << std::endl;
        }
        cmdPub.publish(stop);
    }
      

private:
    ros::Publisher cmdPub, movePub, posPub, statusPub;
    ros::Subscriber odomSub, pathSub, shelfSub, waitSub;
    ros::Rate rate = 30;

    geometry_msgs::Twist moveCmd;
    geometry_msgs::Twist stop;
    turtlesim::Pose nowPosition;  
    std::string robotName, shelfNode;
    std::vector<study_lhj::Coordinate> path;
    int idx;
    double nextPos[3];
    double d, linearSpeed, angularSpeed;
    double lastDeg;
    bool wait = false;
    int status;
    double battery;

    void odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
    {
        turtlesim::Pose pose; 
        pose.x = msg->pose.pose.position.x;
        pose.y = msg->pose.pose.position.y;
        tf::Quaternion quat;
        tf::quaternionMsgToTF(msg->pose.pose.orientation, quat);
        double roll, pitch, yaw;
        tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
        pose.theta = yaw;
        nowPosition = pose;
    }

    void shelfCallback(const study_lhj::str::ConstPtr &msg)
    {
        shelfNode = msg->data;
        study_lhj::Move move;
        move.startNode = "nowidx";
        move.endNode = shelfNode;
        movePub.publish(move);
    }

    void pathCallback(const study_lhj::Locations::ConstPtr &msg)
    {
        // std::cout << "asdgasgs\n";
        for(auto c:msg->location ){
            std::cout << c.y << ' ' << c.x << ' ' << c.deg<< std::endl;
        }
        path = msg->location;
    }
    void waitCallback(const study_lhj::RobotWait::ConstPtr &msg)
    {
        wait = msg->wait;
    }

    void nextIdx()
    {
        nextPos[0] = path[idx].x;
        nextPos[1] = path[idx].y;
        nextPos[2] = path[idx].deg * PI / 180.0;
    }

    void turn()
    {
        try{
            moveCmd.linear.x = 0.00;
            float speed = 0.0;
            double deg=0.0;
            while (1){
                ros::spinOnce();
                if (abs(nowPosition.theta - nextPos[2]) <= 0.01)
                    return;
                deg = abs(nowPosition.theta - nextPos[2]);
                speed = max(2*min(deg, 1.0), 0.1);
                
                
                if (nextPos[2] >= 0){
                    if (nowPosition.theta <= nextPos[2] and nowPosition.theta >= nextPos[2] - PI)
                        moveCmd.angular.z = speed;
                    else
                        moveCmd.angular.z = -speed;
                }
                else{
                    if (nowPosition.theta <= nextPos[2] + PI and nowPosition.theta > nextPos[2])
                        moveCmd.angular.z = -speed;
                    else
                        moveCmd.angular.z = speed;
                }
                cmdPub.publish(moveCmd);
                rate.sleep();
            }
        }catch(...){
            std::cout << robotName <<  " - turn error" << std::endl;
        }
        cmdPub.publish(stop);
        rate.sleep();
    }

     
    void go()
    {
        try
        {
            double dX, dY, pathAng, distance;
            moveCmd.linear.x = linearSpeed;
            while(1){
                ros::spinOnce();
                dX = nextPos[0] - nowPosition.x;
                dY = nextPos[1] - nowPosition.y;
                distance = sqrt(pow(dY,2) + pow(dX,2));
                if(distance <= 0.02) return;

                pathAng = atan2(dY, dX);
                moveCmd.linear.x = max(min(distance,0.3), 0.1);
                
                if(pathAng >= 0){
                    if(nowPosition.theta <= pathAng && nowPosition.theta >= pathAng - PI)
                        moveCmd.angular.z = 0.2;
                    else
                        moveCmd.angular.z = -0.2;
                }
                else{
                    if(nowPosition.theta <= pathAng + PI && nowPosition.theta > pathAng)
                        moveCmd.angular.z = -0.2;
                    else
                        moveCmd.angular.z = 0.2;
                }
                cmdPub.publish(moveCmd);
                rate.sleep();
            }
        }
        catch(...)
        {
            std::cout << robotName <<  " - go error" << std::endl;
        }
        cmdPub.publish(stop);
        rate.sleep();
    } 

};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "robot");
    ros::NodeHandle nh;
    Robot robot(argc, argv, &nh);

    return 0;
}