//
// Created by kai on 8/30/17.
//

#include <fstream>
#include "ros/ros.h"
#include "gio_path.h"
#include "nav_msgs/Odometry.h"
#include "volksbot/vels.h"
#include "tf/transform_broadcaster.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"


std::vector<std::pair<double,double> > simulatedPath {};

const double rad_abstand = 0.485;
const double SPEED_INCREASE = 5.0;
const bool useOdo = false;
const double xOffset = 10.0;
const double yOffset = -0.3;
const double yawOffset = 0.0;
geometry_msgs::Pose pose;
CGioController controller;

void odometryCallback(const nav_msgs::Odometry::ConstPtr &msg) {
    if(useOdo){
        pose = msg->pose.pose;
    }
}

void amclCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg){
        if(!useOdo){
        pose = msg->pose.pose;
    }
}

void writeToCsv() {
    std::cout << "Write simulatedPath to csv" << std::endl;
    std::ofstream outFile ("/home/kai/Documents/simulated_path.csv", std::ofstream::out);

    for(std::pair<double,double> pt : simulatedPath){
        outFile << pt.first << " " << pt.second << std::endl;
    }

    //Close files
    outFile.close();
}

volksbot::vels control(){
    double leftSpeed=0.0;
    double rightSpeed=0.0;

    int loop;

    // not important
    double w = 0.0;
    double u = 0.0;

    //std::cerr << " x:" << pose.position.x << " y:" << pose.position.y << " yaw: " << tf::getYaw(pose.orientation) <<std::endl;
    controller.setPose(pose.position.x+xOffset,pose.position.y+yOffset,tf::getYaw(pose.orientation)+yawOffset);

    std::cerr << " Pose: " << pose.position.x+xOffset << " | " << pose.position.y+yOffset << "yaw :" << tf::getYaw(pose.orientation)-yawOffset << std::endl;

    loop = controller.getNextState(u,w,leftSpeed,rightSpeed);

    volksbot::vels velocity;
    if(loop) {
        velocity.right = -leftSpeed * SPEED_INCREASE;
        velocity.left = -rightSpeed * SPEED_INCREASE;
        //std::cerr << "Left : " << leftSpeed << " Right: " << rightSpeed << " u: "<< u << " w: " << w << std::endl;
        return velocity;
    }
    std::cerr << "Path DONE !!!" << std::endl;
    velocity.left = 0;
    velocity.right = 0;
    return velocity;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "robot_control");
    std::cerr << "Try to load file" << std::endl;
    controller.getPathFromFile("/home/kai/catkin_ws/util/paths/acht.dat");
    controller.setAxisLength(rad_abstand);
    std::cerr << "done Load file" << std::endl;

    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("odom", 1000, odometryCallback);
    ros::Subscriber sub2 = n.subscribe("amcl_pose",1000,amclCallback);

    ros::Publisher publisher = n.advertise<volksbot::vels>("Vel",100);
    ros::Rate loop_rate(100);

    while(ros::ok()){
        publisher.publish(control());
        //std::cerr << pose.position.x << "|" << pose.position.y << " yaw: " << tf::getYaw(pose.orientation) <<std::endl;
        ros::spinOnce();
        loop_rate.sleep();
    }

    ros::spin();

    return 0;
}

