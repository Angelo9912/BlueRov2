#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include <ros/console.h>
#include <vector>
#include "yaml-cpp/yaml.h" // for yaml

std::string status_true = "IDLE";
std::string status_req = "";
bool first_time = true;

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

    while (ros::ok())
    {
        if (status_true == "IDLE")
        {
            if (first_time)
            {
                ROS_WARN("IDLE");
                first_time = false;
            }

            if (status_req == "READY")
            {
                status_true = "READY";
                ROS_WARN("YOU WENT FROM IDLE -> READY");
            }
        }
        else if (status_true == "READY")
        {
            if (status_req == "RUNNING")
            {
                status_true = "RUNNING";
                ROS_WARN("YOU WENT FROM READY -> RUNNING");
            }
        }
        else if (status_true == "RUNNING")
        {

            if (status_req == "COMPLETED")
            {
                status_true = "COMPLETED";
                ROS_WARN("YOU WENT FROM RUNNING -> COMPLETED");
            }
            else if (status_req == "PAUSED")
            {
                status_true = "PAUSED";
                ROS_WARN("YOU WENT FROM RUNNING -> PAUSED");
            }
        }
        else if (status_true == "PAUSED")
        {
            if (status_req == "RUNNING")
            {
                status_true = "RUNNING";
                ROS_WARN("YOU WENT FROM PAUSE -> RUNNING");
            }
            else if (status_req == "READY")
            {
                status_true = "READY";
                ROS_WARN("YOU WENT FROM PAUSE -> READY");
            }
            else if (status_req == "COMPLETED")
            {
                ROS_WARN("You can't go from PAUSED to COMPLETED");
            }
        }

        else if (status_true == "COMPLETED")
        {
            ROS_WARN("MISSION COMPLETED \a");
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
