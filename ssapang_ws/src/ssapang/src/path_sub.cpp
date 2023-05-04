#include "ros/ros.h"
#include "ssapang/Locations.h"
#include "ssapang/Coordinate.h"
#include <iostream>
using namespace std;

void callback(const ssapang::Locations::ConstPtr& msg)
{
    
    for(auto c:msg->location ){
        cout << c.y << ' ' << c.x << ' ' << c.deg<< endl;
    }
    // std::vector<ssapang::Coordinate> path = msg->location;
    // for (auto p : path)
    // {
    //     ROS_INFO_STREAM("x: " << p.x << ", y: " << p.y << ", deg: " << p.deg);
    // }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "path_sub_node");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("/path", 1, callback);
    ros::spin();
    return 0;
}
