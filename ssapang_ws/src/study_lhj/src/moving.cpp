 
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_datatypes.h>
#include <turtlesim/Pose.h>
#include <nav_msgs/Odometry.h>
#include <iostream>
#include "study_lhj/Locations.h"
#include <study_lhj/Coordinate.h>
#include <vector>

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
        odomSub = nh->subscribe("odom", 10, &Robot::callback, this);
        pathSub = nh->subscribe("/path", 10, &Robot::pathCallback, this);
        stop.angular.z = 0.0;
        stop.linear.x = 0.0;

        linearSpeed = 0.22;
        idx = 0;

        nh->getParam("robotName", robotName);

        try
        { 
            while (ros::ok())
            {
                ros::spinOnce();
                // std::cout << path.size() << std::endl;
                if(path.size() == 0 || idx >= path.size() ) continue;
                nextIdx();
                std::cout << nh->getNamespace() << " - " << nextPos[0] << ", " << nextPos[1] << ", " << nextPos[2] << std::endl;
                turn();

                go();
                std::cout << robotName<< "arrival idx..." << std::endl;
                idx++;
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
    void callback(const nav_msgs::Odometry::ConstPtr &msg)
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

    void pathCallback(const study_lhj::Locations::ConstPtr &msg)
    {
        // std::cout << "asdgasgs\n";
        for(auto c:msg->location ){
            std::cout << c.y << ' ' << c.x << ' ' << c.deg<< std::endl;
        }
        path = msg->location;
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
                if (abs(nowPosition.theta - nextPos[2]) <= 0.02)
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
            double nowX, nowY, pathAng;
            double error = 0;
            double prev_error = 0;
            double integral = 0;
            double derivative = 0;
            double Kp = 1.0;
            double Ki = 0.01;
            double Kd = 0.1;
            moveCmd.linear.x = linearSpeed;
            ros::spinOnce();
            std::cout << nowPosition.theta << "\n";
            while (1)
            {
                ros::spinOnce();
                nowX = nowPosition.x;
                nowY = nowPosition.y;
                pathAng = atan2(nextPos[1] - nowY, nextPos[0] - nowX);

                error = 0 - (pathAng - nowPosition.theta);
                std::cout << "error : " << error << std::endl;
                // if (error > 0)
                //     error -= 2 * PI;
                // else if (error < 0)
                //     error += 2 * PI;

                integral += error;
                derivative = error - prev_error;
                prev_error = error;

                double pid_output = Kp * error + Ki * integral + Kd * derivative;


                std::cout << "pid_output : " << pid_output << std::endl;
                moveCmd.angular.z = std::min(std::max(-0.2, pid_output), 0.2);

                if (sqrt(pow(nextPos[1] - nowY, 2) + pow(nextPos[0] - nowX, 2)) <= 0.1)
                    return;

                cmdPub.publish(moveCmd);
                rate.sleep();
            }
        }
        catch (...)
        {
            std::cout << robotName << " - go error" << std::endl;
        }
        cmdPub.publish(stop);
        rate.sleep();
    }
    // void go()
    // {
    //     try
    //     {
    //         double nowX, nowY, pathAng;
    //         moveCmd.linear.x = linearSpeed;
    //         double error = nextPos[2] - nowPosition.theta;
    //         double prev_error = 0;
    //         double integral = 0;
    //         double derivative = 0;
    //         double Kp = 1.5;
    //         double Ki = 0.02;
    //         double Kd = 0.3;
            
    //         ros::spinOnce();
    //         std::cout << nowPosition.theta << "\n";
    //         while(1){
    //             ros::spinOnce();
    //             nowX = nowPosition.x;
    //             nowY = nowPosition.y;
    //             pathAng = atan2(nextPos[1] - nowY, nextPos[0] - nowX);

    //             std::cout <<"nowPosition.theta : "<<nowPosition.theta << ", pathAng : "<<  pathAng  << "\n";
    //             if(pathAng >= 0){
    //                 if(nowPosition.theta <= pathAng && nowPosition.theta >= pathAng - PI)
    //                     moveCmd.angular.z = -0.2;
    //                 else
    //                     moveCmd.angular.z = 0.2;
    //             }
    //             else{
    //                 if(nowPosition.theta <= pathAng + PI && nowPosition.theta > pathAng)
    //                     moveCmd.angular.z = 0.2;
    //                 else
    //                     moveCmd.angular.z = -0.2;
    //             }

    //             if(sqrt(pow(nextPos[1] - nowY,2) + pow(nextPos[0] - nowX,2)) <= 0.05) return;
                
    //             cmdPub.publish(moveCmd);
    //             rate.sleep();
    //         }
    //     }
    //     catch(...)
    //     {
    //         std::cout << robotName <<  " - go error" << std::endl;
    //     }
    //     cmdPub.publish(stop);
    //     rate.sleep();
    // }

private:
    ros::Publisher cmdPub;
    ros::Subscriber odomSub, pathSub;
    ros::Rate rate = 10;

    geometry_msgs::Twist moveCmd;
    geometry_msgs::Twist stop;
    turtlesim::Pose nowPosition;  
    std::string robotName;
    std::vector<study_lhj::Coordinate> path;
    int idx;
    double nextPos[3];
    double d;
    double linearSpeed;


};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "robot");
    ros::NodeHandle nh;
    Robot robot(argc, argv, &nh);

    return 0;
}