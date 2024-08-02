#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <fstream>
#include <nlohmann/json.hpp>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <sstream>

using namespace std;
using json = nlohmann::json;
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

void setPose(ros::NodeHandle& nh) {
    ros::Publisher initial_pose_pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("initialpose", 10);
    ros::Duration(2.0).sleep();

    geometry_msgs::PoseWithCovarianceStamped initial_pose_msg;
    initial_pose_msg.header.stamp = ros::Time::now();
    initial_pose_msg.header.frame_id = "map";
    initial_pose_msg.pose.pose.position.x = 0.298;
    initial_pose_msg.pose.pose.position.y = 0.778;
    initial_pose_msg.pose.pose.position.z = 0;
    initial_pose_msg.pose.pose.orientation.x = 0;
    initial_pose_msg.pose.pose.orientation.y = 0;
    initial_pose_msg.pose.pose.orientation.z = 0.7192;
    initial_pose_msg.pose.pose.orientation.w = 0.6948;
    initial_pose_msg.pose.covariance[0] = 0.25;
    initial_pose_msg.pose.covariance[7] = 0.25;
    initial_pose_msg.pose.covariance[35] = 0.06853892326654787;

    initial_pose_pub.publish(initial_pose_msg);
    ros::Duration(1.0).sleep();
}

vector<move_base_msgs::MoveBaseGoal> parseGoals(const json& data) {
    vector<move_base_msgs::MoveBaseGoal> goals;

    for (const auto& item : data.items()) {
        json text = item.value();

        move_base_msgs::MoveBaseGoal goal;
        goal.target_pose.header.frame_id = "map";
        goal.target_pose.header.stamp = ros::Time::now();

        int number = -1;
        if (text.contains("number")) {
            if (text["number"].is_number_integer()) {
                number = text["number"];
            } else if (!text["number"].is_null()) {
                cerr << "Error: 'number' is not an integer or null in JSON data" << endl;
                continue;
            }
        } else {
            cerr << "Error: 'number' is missing in JSON data" << endl;
            continue;
        }

        switch (number) {
            case 0:
                goal.target_pose.pose.position.x = 0.298;
                goal.target_pose.pose.position.y = 0.778;
                goal.target_pose.pose.orientation.z = 0.7192;
                goal.target_pose.pose.orientation.w = 0.6948;
                break;
            case 1:
                goal.target_pose.pose.position.x = 0.7526435852050781;
                goal.target_pose.pose.position.y = 2.0905284881591797;
                goal.target_pose.pose.orientation.z = 0.7264136260662274;
                goal.target_pose.pose.orientation.w = 0.6872577710475999;
                break;
            case 2:
                goal.target_pose.pose.position.x = 0.8147426843643188;
                goal.target_pose.pose.position.y = 7.766988277435303;
                goal.target_pose.pose.orientation.z = 0.6615929085068617;
                goal.target_pose.pose.orientation.w = 0.7498632031333657;
                break;
            case 3:
                goal.target_pose.pose.position.x = 2.228806734085083;
                goal.target_pose.pose.position.y = 8.37690258026123;
                goal.target_pose.pose.orientation.z = 0.9104235509471554;
                goal.target_pose.pose.orientation.w = 0.41367735964247826;
                break;
            case 4:
                goal.target_pose.pose.position.x = 5.969287295477295;
                goal.target_pose.pose.position.y = 4.759875297546387;
                goal.target_pose.pose.orientation.z = -0.9966697172001557;
                goal.target_pose.pose.orientation.w = 0.08154431197920281;
                break;
            case 5:
                goal.target_pose.pose.position.x = 10.619894981384277;
                goal.target_pose.pose.position.y = 5.756357192993164;
                goal.target_pose.pose.orientation.z = -0.9644117701141499;
                goal.target_pose.pose.orientation.w = 0.26440487451121636;
                break;
            case 6:
                goal.target_pose.pose.position.x = 8.190423965454102;
                goal.target_pose.pose.position.y = 4.163373947143555;
                goal.target_pose.pose.orientation.z = 0.7230919258198363;
                goal.target_pose.pose.orientation.w = 0.690781812747647;
                break;
            case -1:
                goal.target_pose.pose.position.x = 0;
                goal.target_pose.pose.position.y = 0;
                goal.target_pose.pose.orientation.z = 0;
                goal.target_pose.pose.orientation.w = 0;
                break;
            default:
                cerr << "Error: Invalid 'number' value in JSON data" << endl;
                continue;
        }

        goals.push_back(goal);
    }

    return goals;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "json_goal_node");
    ros::NodeHandle nh;

    MoveBaseClient ac("move_base", true);

    while (!ac.waitForServer(ros::Duration(5.0))) {
        ROS_INFO("Waiting for the move_base action server to come up");
    }

    setPose(nh);

    string file_path = "/home/aew/sandee/src/jsonfile/control.json";
    while (ros::ok()) {
        ifstream file(file_path);
        if (!file.is_open()) {
            cerr << "Error opening file!" << endl;
            ros::Duration(5.0).sleep();
            continue;
        }

        json data;
        try {
            file >> data;
        } catch (json::parse_error& e) {
            cerr << "JSON parse error: " << e.what() << endl;
            ros::Duration(5.0).sleep();
            continue;
        }

        vector<move_base_msgs::MoveBaseGoal> goals = parseGoals(data);

        for (size_t i = 0; i < goals.size(); ++i) {
            ROS_INFO("Sending goal to waypoint %zu", i + 1);
            ac.sendGoal(goals[i]);

            if (!ac.waitForResult(ros::Duration(20.0))) {
                ROS_WARN("Timeout reached while waiting for the robot to reach waypoint %zu", i + 1);
            }

            if (ac.getState() == actionlib::SimpleClientGoalState::ACTIVE) {
                ROS_INFO("Robot is still moving to waypoint %zu", i + 1);
                ac.waitForResult();
            } else if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
                ROS_INFO("Robot has arrived at waypoint %zu", i + 1);
            } else {
                ROS_INFO("Robot failed to reach waypoint %zu", i + 1);
            }

            ros::Duration(5.0).sleep(); // Delay before sending the next goal
        }

        file.close();

        // Sleep before checking the file again
        ros::Duration(10.0).sleep(); // Adjust the sleep duration as needed
    }

    return 0;
}

