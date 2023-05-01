#include <ros/ros.h>
#include <random>
#include <robot_control/Coordinate.h>
#include <robot_control/Location.h>
#include <time.h>
#include <unistd.h>
#include <iostream>
#include <stdlib.h>
using namespace std;

double now[3] = {0, 0, 0};
double dir[4][3]= {{-1, 0, 180}, {1, 0, 0}, {0, -1, -90}, {0, 1, 90}};

void nextIdx() {
    try {
        int num = rand() % 4;
        for (int i = 0; i < 2; i++) {
            now[i] += static_cast<double>(dir[num][i]);
        }
        now[2] = dir[num][2];
    } catch (exception &e) {
        cout << "find next idx error: " << e.what() << endl;
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "path_pub");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<robot_control::Location>("path", 1);
    ros::Rate rate(20);

    bool sw = true;
    usleep(10000000); 
    while (ros::ok()) {
        if(sw){
            robot_control::Location loc;
            for (int i = 0; i < 100; i++) {
                nextIdx();
                robot_control::Coordinate coord;
                coord.x = now[0];
                coord.y = now[1];
                coord.deg = now[2];
                loc.location.push_back(coord);
            }
            pub.publish(loc);
            sw = false;
        }
        rate.sleep();
    }

    return 0;
}