//
// Created by kai on 8/30/17.
//

#include <fstream>
#include "ros/ros.h"
#include "gio_path.h"


std::vector<std::pair<double, double> > simulatedPath{};

CGioController controller;
const double epsilon = 0.000001;
const double rad_abstand = 0.485;
const double timeStep = 0.01;

void writeToCsv() {
    std::cout << "Write simulatedPath to csv" << std::endl;
    std::ofstream outFile("/home/kai/Documents/simulated_path.csv", std::ofstream::out);

    for (std::pair<double, double> pt : simulatedPath) {
        outFile << pt.first << " " << pt.second << std::endl;
    }

    //Close files
    outFile.close();
}

int predictRobotBehaviour(const double leftspeed, const double rightspeed,
                          const double theta,
                          const double elapsed_time,
                          double &dtheta, double &dx, double &dy) {
    double v = 0.0;

    if (fabs(rightspeed) < epsilon) {
        if (fabs(leftspeed) < epsilon) {
            dtheta = 0.0;
            dx = 0.0;
            dy = 0.0;
            return (2);
        }
    }

    v = 0.5 * (leftspeed + rightspeed);
    dtheta = elapsed_time * (rightspeed - leftspeed) / rad_abstand;

    dx = v * cos(theta + dtheta * 0.5) * elapsed_time;
    dy = v * sin(theta + dtheta * 0.5) * elapsed_time;

    return (0);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "robot_control");
    std::cerr << "Try to load file" << std::endl;
    controller.getPathFromFile("/home/kai/catkin_ws/util/paths/acht.dat");
    std::cerr << "done Load file" << std::endl;


    int loop = 1;
    double theta = 0.0;
    double x = 3.0;
    double y = 0.0;

    controller.setPose(x,y,theta);
    while (loop) {
        double leftSpeed = 0.0;
        double rightSpeed = 0.0;
        double dtheta = 0.0;
        double dx = 0.0;
        double dy = 0.0;

        // not important
        double w = 0.5;
        double u = 0.1;

        loop = controller.getNextState(u, w, leftSpeed, rightSpeed);
        predictRobotBehaviour(leftSpeed,rightSpeed,theta,timeStep,dtheta,dx,dy);
        theta += dtheta;
        x += dx;
        y += dy;
        std::pair <double, double > pt = std::make_pair(x,y);
        simulatedPath.push_back(pt);
        controller.setPose(x,y,theta);
    }

    std::cout << "DONE simulation" << std::endl;
    writeToCsv();
    ros::spin();
    return 0;
}

