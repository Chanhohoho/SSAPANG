#include <ros/ros.h>
#include <random>
#include <ssapang/Locations.h>
#include <ssapang/Coordinate.h>
#include <ssapang/Move.h>
#include <time.h>
#include <unistd.h>
#include <iostream>
#include <stdlib.h>
using namespace std;

class Path
{
public:
    Path(int argc, char **argv, ros::NodeHandle *nh){
        pathPub = nh->advertise<ssapang::Locations>("path", 1);
        // cmdPub = nh->advertise<geometry_msgs::Twist>("cmd_vel", 10);
        Sub = nh->subscribe("move", 10, &Path::callback, this);
    }

private:
    ros::Publisher pathPub;
    ros::Subscriber Sub;
    ros::Rate rate = 10;
    std::string startNode, endNode;

    double now[3] = {0, 0, 0};
    // double dir[4][3]= {{-0.5, 0, 180}, {0.5, 0, 0}, {0, -0.5, -90}, {0, 0.5, 90}};
    double dir[4][3]= {{-1.0, 0, 180}, {1.0, 0, 0}, {0, -1.0, -90}, {0, 1.0, 90}};
    // double dir[4][3]= {{-2.0, 0, 180}, {2.0, 0, 0}, {0, -2.0, -90}, {0, 2.0, 90}};


    void callback(const ssapang::Move::ConstPtr &msg)
    {
        startNode = msg->startNode;
        endNode = msg->endNode;
        std::cout << startNode << ", " << endNode<< "\n";
        ssapang::Locations loc;
        for (int i = 0; i < 100; i++) {
            nextIdx();
            ssapang::Coordinate coord;
            coord.x = now[0];
            coord.y = now[1];
            coord.deg = now[2];
            loc.location.push_back(coord);
        }
        pathPub.publish(loc);
    }

    void nextIdx() {
        try {
            int num = rand() % 4;
            for (int i = 0; i < 2; i++) 
                now[i] += static_cast<double>(dir[num][i]);
            now[2] = dir[num][2];
        } catch (exception &e) {
            cout << "find next idx error: " << e.what() << endl;
        }
    }

};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "path_pub");
    ros::NodeHandle nh;
    Path path(argc, argv, &nh);
    ros::spin();

    return 0;
}