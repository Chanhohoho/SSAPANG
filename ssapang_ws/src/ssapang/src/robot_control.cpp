 
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_datatypes.h>
#include <turtlesim/Pose.h>
#include <nav_msgs/Odometry.h>
#include <iostream>
#include <ssapang/Locations.h>
#include <ssapang/Coordinate.h>
#include <ssapang/Move.h>
#include <ssapang/str.h>
#include <ssapang/RobotWait.h>
#include <ssapang/RobotPos.h>
#include <ssapang/RobotStatus.h>
#include <ssapang/Task.h>
#include <vector>
#include <string>


const double PI = std::acos(-1);

class RobotControl
{
public:
    RobotControl(int argc, char **argv, ros::NodeHandle *nh){
        cmdPub = nh->advertise<geometry_msgs::Twist>("cmd_vel", 10);
        movePub = nh->advertise<ssapang::Move>("move", 10);
        posPub = nh->advertise<ssapang::RobotPos>("pos", 10);
        statusPub = nh->advertise<ssapang::RobotStatus>("status", 10);


        odomSub = nh->subscribe("odom", 10, &RobotControl::odomCallback, this);
        shelfSub = nh->subscribe("shelf", 10, &RobotControl::shelfCallback, this);
        pathSub = nh->subscribe("path", 10, &RobotControl::pathCallback, this);
        waitSub = nh->subscribe("wait", 10, &RobotControl::waitCallback, this);


        stop.angular.z = 0.0;
        stop.linear.x = 0.0;

        linearSpeed = 0.22;
        angularSpeed = 1.0;
        idx = -1;
        status.status = 0;
        battery = 100.0;
        nextPos.QR = "BS0102";
        wait = true;


        nh->getParam("robotName", robotName);

        try
        { 
            while (ros::ok())
            {
                ros::spinOnce();
                if(wait || idx < 0 ) continue;
                nextIdx();
                if(nextPos.y == 100) {
                    path.clear();
                    idx = -1;
                    status.status++;
                    cmdPub.publish(stop);
                    statusPub.publish(status);
                    rate.sleep();
                }else{
                    std::cout << nh->getNamespace() << " - " << nextPos.x << ", " << nextPos.y << ", " << nextPos.deg << std::endl;
                    turn();
                    go();
                    ssapang::RobotPos pos;
                    pos.fromNode = nextPos.QR;
                    pos.toNode = path[idx+1].QR;
                    pos.idx = idx++;
                    pos.battery = --battery;
                    posPub.publish(pos);
                    std::cout << idx << '/' << path.size() << "\n";
                    wait = true;
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
    ros::Rate rate = 60;

    geometry_msgs::Twist moveCmd;
    geometry_msgs::Twist stop;
    turtlesim::Pose nowPosition;  
    std::string robotName, shelfNode;
    std::vector<ssapang::Coordinate> path;
    ssapang::Coordinate nextPos;
    int idx;
    double d, linearSpeed, angularSpeed;
    double lastDeg;
    bool wait;
    ssapang::RobotStatus status;
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

    void shelfCallback(const ssapang::str::ConstPtr &msg)
    {
        shelfNode = msg->data;
        ssapang::Move move;
        move.startNode = nextPos.QR;
        move.endNode = shelfNode;
        movePub.publish(move);
    }

    void pathCallback(const ssapang::Locations::ConstPtr &msg)
    {
        // std::cout << "asdgasgs\n";
        for(auto c:msg->location ){
            std::cout << c.QR << ' ' << c.x << ' ' << c.y << ' ' << c.deg<< std::endl;
        }
        path = msg->location;


        idx = 0;

        ssapang::RobotPos pos;
        pos.fromNode = nextPos.QR;
        pos.toNode = path[idx].QR;
        pos.idx = idx;
        pos.battery = battery;
        posPub.publish(pos);

    }
    void waitCallback(const ssapang::RobotWait::ConstPtr &msg)
    {
        wait = msg->wait;
    }

    void nextIdx()
    {
        nextPos = path[idx];
        nextPos.deg = path[idx].deg * PI / 180.0;
    }

    void turn()
    {
        try{
            moveCmd.linear.x = 0.00;
            float speed = 0.0;
            double deg=0.0;
            while (1){
                ros::spinOnce();
                if (std::abs(nowPosition.theta - nextPos.deg) <= 0.01)
                    return;
                deg = std::abs(nowPosition.theta - nextPos.deg);
                speed = std::max(2*std::min(deg, 1.0), 0.1);
                
                
                if (nextPos.deg >= 0){
                    if (nowPosition.theta <= nextPos.deg and nowPosition.theta >= nextPos.deg - PI)
                        moveCmd.angular.z = speed;
                    else
                        moveCmd.angular.z = -speed;
                }
                else{
                    if (nowPosition.theta <= nextPos.deg + PI and nowPosition.theta > nextPos.deg)
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
                dX = nextPos.x - nowPosition.x;
                dY = nextPos.y - nowPosition.y;
                distance = std::sqrt(std::pow(dY,2) + std::pow(dX,2));
                if(distance <= 0.02) return;
                std::cout << nowPosition.x << ", " << nowPosition.y << "\n";

                pathAng = std::atan2(dY, dX);
                moveCmd.linear.x = std::max(std::min(distance,0.1), 0.1);
                
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
    ros::init(argc, argv, "robot_control");
    ros::NodeHandle nh;
    RobotControl RobotControl(argc, argv, &nh);

    return 0;
}