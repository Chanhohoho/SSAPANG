 
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
#include <unordered_map>
#include <queue>
#include <unistd.h>

const double PI = std::acos(-1);

class RobotControl
{
public:
    std::string robotName;
    ssapang::Coordinate nextPos;
    RobotControl(int argc, char **argv, ros::NodeHandle *nh){
        cmdPub = nh->advertise<geometry_msgs::Twist>("cmd_vel", 10);
        movePub = nh->advertise<ssapang::Move>("move", 10);
        posPub = nh->advertise<ssapang::RobotPos>("pos", 10);
        statusPub = nh->advertise<ssapang::RobotStatus>("status", 10);
        checkGoPub = nh->advertise<ssapang::str>("checkGo", 1);
        goPub = nh->advertise<ssapang::str>("Go", 1);


        odomSub = nh->subscribe("odom", 10, &RobotControl::odomCallback, this);
        pathSub = nh->subscribe("path", 10, &RobotControl::pathCallback, this);
        waitSub = nh->subscribe("wait", 10, &RobotControl::waitCallback, this);
        shelfSub = nh->subscribe("shelf", 10, &RobotControl::shelfCallback, this);
        taskSub = nh->subscribe("task", 10, &RobotControl::taskCallback, this);


        stop.angular.z = 0.0;
        stop.linear.x = 0.0;

        linearSpeed = 0.22;
        angularSpeed = 1.0;
        idx = -1;
        status.status = 0;
        battery = 100.0;
        nextPos.QR = argv[2];
        GO.data = "GO";
        wait = 1;

        nh->getParam("robotName", robotName);

        try
        {
            sleep(3);
            while (ros::ok())
            {
                // 판단
                ros::spinOnce();
                if(idx < 0) continue;
                // std::cout << NOW.data << "\n";
                checkGoPub.publish(NEXT); // 다음위치
                rate.sleep();
                // sleep(1);
                
                ros::spinOnce();
                // std::cout << "wait " << wait << "\n";
                if(wait) continue;
                // std::cout << nextPos.QR << ": " << node[nextPos.QR].size() << std::endl;
                NOW.data = path[idx-1].QR;
                
                nextIdx();

                if(idx+1 >= path.size()) {
                    path.clear();
                    idx = -1;
                    status.status++;
                    statusPub.publish(status);
                    
                }else{
                    std::cout << nh->getNamespace() << " - " << nextPos.x << ", " << nextPos.y << ", " << nextPos.deg << std::endl;
                    turn();
                    cmdPub.publish(stop);
                    rate.sleep();
                }
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
                    goPub.publish(NOW);
                    rate.sleep();
                    go();
                    if(idx < path.size()){
                        ssapang::RobotPos pos;
                        pos.fromNode = nextPos.QR;
                        pos.toNode = path[idx+1].QR;
                        NOW.data = path[idx].QR;
                        NEXT.data = path[idx+1].QR;
                        pos.battery = --battery;
                        pos.idx = ++idx;
                        posPub.publish(pos);
                    }
                    std::cout << idx << '/' << path.size() << "\n";
                    wait = 1;
                }
                cmdPub.publish(stop);
                rate.sleep();
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
    ros::Publisher cmdPub, movePub, posPub, statusPub, checkGoPub, goPub;
    ros::Subscriber odomSub, pathSub, waitSub, shelfSub, taskSub;
    ros::Rate rate = 60;

    geometry_msgs::Twist moveCmd;
    geometry_msgs::Twist stop;
    turtlesim::Pose nowPosition;  
    std::string shelfNode;
    ssapang::Task task;
    std::vector<ssapang::Coordinate> path;
    // ssapang::Coordinate nextPos;
    ssapang::str NOW, NEXT, GO;
    int idx;
    double d, linearSpeed, angularSpeed;
    double lastDeg;
    int wait;
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
        rate.sleep();
    }
    void taskCallback(const ssapang::Task::ConstPtr &msg)
    {
        // task받아서 처리 할 수있게 코드 수정 필요 //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        task = *msg;
    }

    void pathCallback(const ssapang::Locations::ConstPtr &msg)
    {
        // std::cout << "asdgasgs\n";
        // for(auto c:msg->location ){
        //     std::cout << c.QR << ' ' << c.x << ' ' << c.y << ' ' << c.deg<< std::endl;
        // }
        path = msg->location;
        idx = 1;
        NEXT.data = path[1].QR;
        NOW.data = path[0].QR;
        ssapang::RobotPos pos;
        pos.fromNode = path[0].QR;
        pos.toNode = path[1].QR;
        pos.battery = battery;
        pos.idx = idx;
        posPub.publish(pos);
        rate.sleep();
    }
    void waitCallback(const ssapang::RobotWait::ConstPtr &msg)
    {
        std::cout <<robotName << " " << msg->wait << "\n";
        wait = msg->wait;
    }

    void nextIdx()
    {
        nextPos = path[idx-1];
        nextPos.deg = path[idx-1].deg * PI / 180.0;
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
                // std::cout << nowPosition.x << ", " << nowPosition.y << "\n";

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