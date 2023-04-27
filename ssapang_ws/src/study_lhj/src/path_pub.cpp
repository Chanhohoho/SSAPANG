#include <ros/ros.h>
#include <random>
#include <study_lhj/Locations.h>
#include <study_lhj/Coordinate.h>
#include <time.h>
#include <unistd.h>
#include <iostream>
#include <stdlib.h>
using namespace std;

double now[3] = {0, 0, 0};
double dir[4][3]= {{-0.5, 0, 180}, {0.5, 0, 0}, {0, -0.5, -90}, {0, 0.5, 90}};

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
    ros::init(argc, argv, "path_pub_node");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<study_lhj::Locations>("/path", 1);
    ros::Rate rate(20);

    bool sw = true;
    usleep(10000000); 
    while (ros::ok()) {
        if(sw){
            study_lhj::Locations loc;
            for (int i = 0; i < 30; i++) {
                nextIdx();
                study_lhj::Coordinate coord;
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
