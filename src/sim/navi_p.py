#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <cmath>
#include <iostream>
#include <vector>

// Predefined goals
std::vector<std::tuple<double, double, double>> goals = {
    std::make_tuple(0.298, 0.778, 1.607),
    std::make_tuple(0.752, 2.090, 1.626),
    std::make_tuple(0.814, 7.766, 1.446),
    std::make_tuple(2.228, 8.376, 2.289),
    std::make_tuple(5.969, 4.759, -2.978),
    std::make_tuple(10.619, 5.756, -2.606),
    std::make_tuple(8.190, 4.163, 1.617)
};

void setPose(double x, double y, double theta, ros::Publisher& pub) {
    geometry_msgs::PoseWithCovarianceStamped pose;
    pose.header.frame_id = "map";
    pose.header.stamp = ros::Time::now();

    // set x,y coord
    pose.pose.pose.position.x = x;
    pose.pose.pose.position.y = y;
    pose.pose.pose.position.z = 0.0;

    // set theta
    tf::Quaternion quat = tf::createQuaternionFromYaw(theta);
    pose.pose.pose.orientation.x = quat.x();
    pose.pose.pose.orientation.y = quat.y();
    pose.pose.pose.orientation.z = quat.z();
    pose.pose.pose.orientation.w = quat.w();
    pose.pose.covariance[6 * 0 + 0] = 0.5 * 0.5;
    pose.pose.covariance[6 * 1 + 1] = 0.5 * 0.5;
    pose.pose.covariance[6 * 5 + 5] = (M_PI / 12.0) * (M_PI / 12.0);

    // publish
    ROS_INFO("Initial pose : x: %f, y: %f, z: 0.0, theta: %f", x, y, theta);
    pub.publish(pose);
}

void sendGoal(double x, double y, double theta, actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>& client) {
    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();

    // set x,y coord
    goal.target_pose.pose.position.x = x;
    goal.target_pose.pose.position.y = y;
    goal.target_pose.pose.position.z = 0.0;

    // set theta
    tf::Quaternion quat = tf::createQuaternionFromYaw(theta);
    goal.target_pose.pose.orientation.x = quat.x();
    goal.target_pose.pose.orientation.y = quat.y();
    goal.target_pose.pose.orientation.z = quat.z();
    goal.target_pose.pose.orientation.w = quat.w();

    // send goal
    client.sendGoal(goal);
    client.waitForResult();
    ROS_INFO("Reached Goal : x: %f, y: %f, z: 0.0, theta: %f", x, y, theta);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "send_position");
    ros::NodeHandle nh;
    ros::Publisher pubInitialPose = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 1);

    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> client("move_base", true);
    client.waitForServer();

    ros::Rate rate(15);

    double x, y, theta;
    std::tie(x, y, theta) = goals[0];
    sendGoal(x, y, theta, client);
    ros::Duration(2.0).sleep();  // Give some time to set initial pose

    while (ros::ok()) {
        std::string mode;
        std::cout << "Enter mode ('auto' or 'manual'): ";
        std::cin >> mode;

        if (mode == "auto" || mode == "a") {
            for (const auto& goal : goals) {
                std::tie(x, y, theta) = goal;
                sendGoal(x, y, theta, client);
                ros::Duration(10.0).sleep();  // Adjust this to a dynamic wait based on feedback
            }
            // Return to start point
            std::tie(x, y, theta) = goals[0];
            sendGoal(x, y, theta, client);
        } else if (mode == "manual" || mode == "m") {
            while (true) {
                std::string command;
                std::cout << "Enter command ('0'-'6') or 'exit' to return to mode selection: ";
                std::cin >> command;
                if (command == "exit" || command == "e") {
                    break;
                } else if (isdigit(command[0]) && std::stoi(command) >= 0 && std::stoi(command) <= 6) {
                    int idx = std::stoi(command);
                    std::tie(x, y, theta) = goals[idx];
                    sendGoal(x, y, theta, client);
                } else {
                    ROS_INFO("Sorry, command not recognized");
                }
            }
        } else {
            ROS_INFO("Sorry, mode not recognized");
        }

        rate.sleep();
    }

    return 0;
}

