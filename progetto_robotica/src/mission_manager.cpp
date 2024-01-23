#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include <ros/console.h>
#include <eigen3/Eigen/Dense>                // for eigen matrix
#include "progetto_robotica/Floats.h"        // for accessing -- progetto_robotica Floats()
#include "progetto_robotica/Floats_String.h" // for accessing -- progetto_robotica Floats_String()
#include <vector>
#include "yaml-cpp/yaml.h" // for yaml

std::string status_true = "PAUSED";
std::string status_req = "";

// Subscriber callback function
void statusReqCallback(const std_msgs::String::ConstPtr &msg)
{
    status_req = msg->data.c_str();
}

int main(int argc, char **argv)
{
    // Initialize the ROS node
    ros::init(argc, argv, "mission_manager");
    ros::NodeHandle nh;

    // Create a publisher
    ros::Publisher pub = nh.advertise<std_msgs::String>("manager/status_topic", 10);

    // Create a subscriber
    ros::Subscriber sub = nh.subscribe("manager/status_requested_topic", 10, statusReqCallback);

    // Loop at 10Hz
    ros::Rate rate(20);
    bool first_time = true;

    while (ros::ok())
    {
        if (status_true == "PAUSED")
        {
            if (status_req == "RUNNING")
            {
                status_true = "RUNNING";
                ROS_WARN("YOU WENT FROM PAUSE -> RUNNING");
                first_time = true;
            }
            else if (status_req == "COMPLETED")
            {
                ROS_WARN("You can't go from PAUSED to COMPLETED");
            }
            else if (status_req == "PAUSED")
            {
                if (first_time)
                {
                    ROS_WARN("You are already PAUSED");
                    first_time = false;
                }
            }
        }
        else if (status_true == "RUNNING")
        {
            if (status_req == "RUNNING")
            {
                if (first_time)
                {
                    ROS_WARN("You are already RUNNING");
                    first_time = false;
                }
            }
            else if (status_req == "COMPLETED")
            {
                status_true = "COMPLETED";
                ROS_WARN("MISSION COMPLETED");
                first_time = true;
            }
            else if (status_req == "PAUSED")
            {
                status_true = "PAUSED";
                ROS_WARN("YOU WENT FROM RUNNING -> PAUSED");
                first_time = true;
            }
        }
        else if (status_true == "COMPLETED")
        {
            if (status_req == "RUNNING")
            {
                ROS_WARN("Mission already COMPLETED");
            }
            else if (status_req == "COMPLETED")
            {
                ROS_WARN("Mission already COMPLETED");
            }
            else if (status_req == "PAUSED")
            {
                ROS_WARN("Mission already COMPLETED");
            }
        }

        // Create a message
        std_msgs::String msg;

        msg.data = status_true;

        // Publish the message
        pub.publish(msg);

        // Spin once to process callbacks
        ros::spinOnce();

        // Sleep to maintain the loop rate
        rate.sleep();
    }

    return 0;
}
