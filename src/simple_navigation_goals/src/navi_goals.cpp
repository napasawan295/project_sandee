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
    initial_pose_msg.pose.pose.position.x = 0.9767409563064575;
    initial_pose_msg.pose.pose.position.y = 0.7730389833450317;
    initial_pose_msg.pose.pose.position.z = 0;
    initial_pose_msg.pose.pose.orientation.x = 0;
    initial_pose_msg.pose.pose.orientation.y = 0;
    initial_pose_msg.pose.pose.orientation.z = 0.7061899307755225;
    initial_pose_msg.pose.pose.orientation.w = 0.708022444327341;
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
                goal.target_pose.pose.position.x = 0.9767409563064575;
                goal.target_pose.pose.position.y = 0.7730389833450317;
                goal.target_pose.pose.orientation.z = 0.7061899307755225;
                goal.target_pose.pose.orientation.w = 0.708022444327341;
                break;
            case 1:
                goal.target_pose.pose.position.x = 0.9879441261291504;
                goal.target_pose.pose.position.y = 2.0192620754241943;
                goal.target_pose.pose.orientation.z = 0.7068718310911537;
                goal.target_pose.pose.orientation.w = 0.7073416532410908;
                break;
            case 2:
                goal.target_pose.pose.position.x = 1.1633069515228271;
                goal.target_pose.pose.position.y = 7.598809719085693;
                goal.target_pose.pose.orientation.z = 0.7176336499479044;
                goal.target_pose.pose.orientation.w = 0.6964208099004859;
                break;
            case 3:
                goal.target_pose.pose.position.x = 2.324207067489624;
                goal.target_pose.pose.position.y = 8.647153854370117;
                goal.target_pose.pose.orientation.z = -0.6964207984949514;
                goal.target_pose.pose.orientation.w = 0.7176336610162976;
                break;
            case 4:
                goal.target_pose.pose.position.x = 5.8508052730560303;
                goal.target_pose.pose.position.y = 4.6246562004089355;
                goal.target_pose.pose.orientation.z = 0.01909737974351948;
                goal.target_pose.pose.orientation.w = 0.999817284137682;
                break;
            case 5:
                goal.target_pose.pose.position.x = 9.370543479919434;
                goal.target_pose.pose.position.y = 5.275940418243408;
                goal.target_pose.pose.orientation.z = -0.7051188962416745;
                goal.target_pose.pose.orientation.w = 0.7090890932477545;
                break;
            case 6:
                goal.target_pose.pose.position.x = 8.475961685180664;
                goal.target_pose.pose.position.y = 4.351451396942139;
                goal.target_pose.pose.orientation.z = -0.999579629699633;
                goal.target_pose.pose.orientation.w = 0.028992479879177344;
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

