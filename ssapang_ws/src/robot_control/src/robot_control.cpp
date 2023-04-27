 
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_datatypes.h>
#include <turtlesim/Pose.h>
#include <nav_msgs/Odometry.h>
#include <iostream>
#include <robot_control/Coordinate.h>
#include <robot_control/Location.h>
#include <vector>

using std::sqrt;
using std::pow;
using std::atan2;
using std::abs;

const double PI = std::acos(-1);

class Robot
{
public:
    Robot(int argc, char **argv, ros::NodeHandle *nh){
        cmdPub = nh->advertise<geometry_msgs::Twist>("cmd_vel", 10);
        odomSub = nh->subscribe("odom", 10, &Robot::callback, this);
        pathSub = nh->subscribe("/path", 10, &Robot::pathCallback, this);
        stop = geometry_msgs::Twist();

        linearSpeed = 0.3;
        idx = 0;

        nh->getParam("robotName", robotName);

        try
        { 
            while (ros::ok())
            {
                ros::spinOnce();
                // std::cout << path.size() << std::endl;
                if(path.size() == 0) continue;
                nextIdx();
                std::cout << nh->getNamespace() << " - " << nextPos[0] << ", " << nextPos[1] << ", " << nextPos[2] << std::endl;
                turn();

                go();
                std::cout << robotName<< "arrival idx..." << std::endl;
                idx++;
                if(path.size() <= idx) {
                    std::cout << robotName<< " FINISH" << std::endl;
                    break;
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

    void pathCallback(const robot_control::Location::ConstPtr &msg)
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
            double error = nextPos[2] - nowPosition.theta;
            double prev_error = 0;
            double integral = 0;
            double derivative = 0;
            double Kp = 1.5;
            double Ki = 0.02;
            double Kd = 0.18;

            moveCmd.linear.x = 0.00;
            while (1){
                ros::spinOnce();
                std::cout << "nP : " << nextPos[2] <<" nowP : " << nowPosition.theta << " error : " << error << std::endl;
                
                
                if(PI - 0.1 < nextPos[2] && PI + 0.1 > nextPos[2]){
                    if(nowPosition.theta > 0){
                        error = nextPos[2] - nowPosition.theta;
                        integral += error;
                        derivative = error - prev_error;
                        moveCmd.angular.z = std::max(Kp * error + Ki * integral + Kd * derivative, 0.2);
                    }else{
                        error = -1*(nextPos[2] + nowPosition.theta);
                        integral += error;
                        derivative = error - prev_error;
                        moveCmd.angular.z = std::min(Kp * error + Ki * integral + Kd * derivative, -0.2);
                    }
                }
                else if (PI/2 - 0.1 < nextPos[2] && PI/2 + 0.1 > nextPos[2]){
                    if(nowPosition.theta > 0){
                        if (nowPosition.theta <= nextPos[2]){
                            error = nextPos[2] - nowPosition.theta;
                            integral += error;
                            derivative = error - prev_error;
                            moveCmd.angular.z = std::max(Kp * error + Ki * integral + Kd * derivative, 0.2);
                        }else{
                            error = nextPos[2] - nowPosition.theta;
                            integral += error;
                            derivative = error - prev_error;
                            moveCmd.angular.z = std::min(Kp * error + Ki * integral + Kd * derivative, -0.2);
                        }
                    }
                    else {
                        if (nowPosition.theta + PI >= nextPos[2]){
                            error = nextPos[2] - nowPosition.theta;
                            integral += error;
                            derivative = error - prev_error;
                            moveCmd.angular.z = std::max(Kp * error + Ki * integral + Kd * derivative, 0.2);
                        }
                        else{
                            error = -1*(nextPos[2] + (PI + nowPosition.theta));
                            integral += error;
                            derivative = error - prev_error;
                            moveCmd.angular.z = std::min(Kp * error + Ki * integral + Kd * derivative, -0.2);
                        }
                    }
                }
                else if (-PI/2 - 0.1 < nextPos[2] && -PI/2 + 0.1 > nextPos[2]){
                    if(nowPosition.theta < 0){
                        if (nowPosition.theta <= nextPos[2]){
                            error = nextPos[2] - nowPosition.theta;
                            integral += error;
                            derivative = error - prev_error;
                            moveCmd.angular.z = std::max(Kp * error + Ki * integral + Kd * derivative, 0.2);
                        }else{
                            error = nextPos[2] - nowPosition.theta;
                            integral += error;
                            derivative = error - prev_error;
                            moveCmd.angular.z = std::min(Kp * error + Ki * integral + Kd * derivative, -0.2);
                        }
                    }
                    else {
                        if (nowPosition.theta - PI >= nextPos[2]){
                            error = -1*(nextPos[2] - (PI - nowPosition.theta));
                            integral += error;
                            derivative = error - prev_error;
                            moveCmd.angular.z = std::max(Kp * error + Ki * integral + Kd * derivative, 0.2);
                        }
                        else{
                            error = nextPos[2] - nowPosition.theta;
                            integral += error;
                            derivative = error - prev_error;
                            moveCmd.angular.z = std::min(Kp * error + Ki * integral + Kd * derivative, -0.2);
                        }
                    }
                }
                else if( -0.1 < nextPos[2] && 0.1 > nextPos[2]){
                    if(nowPosition.theta > 0){
                        error = nextPos[2] - nowPosition.theta;
                        integral += error;
                        derivative = error - prev_error;
                        moveCmd.angular.z = std::min(Kp * error + Ki * integral + Kd * derivative, -0.2);
                    }else{
                        error = nextPos[2] - nowPosition.theta;
                        integral += error;
                        derivative = error - prev_error;
                        moveCmd.angular.z = std::max(Kp * error + Ki * integral + Kd * derivative, 0.2);
                    }
                }

                std::cout << "prev : " << Kp * error + Ki * integral + Kd * derivative <<" now : " << moveCmd.angular.z << std::endl;
                prev_error = error;

                if (abs(error) <= 0.02)
                    return;
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
            moveCmd.linear.x = linearSpeed;
            ros::spinOnce();
            std::cout << nowPosition.theta << "\n";
            while(1){
                ros::spinOnce();
                nowX = nowPosition.x;
                nowY = nowPosition.y;
                pathAng = atan2(nextPos[1] - nowY, nextPos[0] - nowX);
                if(pathAng >= 0){
                    if(nowPosition.theta <= pathAng && nowPosition.theta >= pathAng - PI)
                        moveCmd.angular.z = 0.23;
                    else
                        moveCmd.angular.z = -0.23;
                }
                else{
                    if(nowPosition.theta <= pathAng + PI && nowPosition.theta > pathAng)
                        moveCmd.angular.z = -0.23;
                    else
                        moveCmd.angular.z = 0.23;
                }

                if(sqrt(pow(nextPos[1] - nowY,2) + pow(nextPos[0] - nowX,2)) <= 0.1) return;
                
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

private:
    ros::Publisher cmdPub;
    ros::Subscriber odomSub, pathSub;
    ros::Rate rate = 10;

    geometry_msgs::Twist moveCmd;
    geometry_msgs::Twist stop;
    turtlesim::Pose nowPosition;  
    std::string robotName;
    std::vector<robot_control::Coordinate> path;
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