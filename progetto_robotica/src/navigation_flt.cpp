#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include <ros/console.h>
#include <eigen3/Eigen/Dense>         // for eigen matrix
#include "progetto_robotica/Floats.h" // for accessing -- progetto_robotica Floats()
#include <vector>
#include "yaml-cpp/yaml.h" // for yaml

// Callback function for the subscriber
void GPSCallback(const progetto_robotica::Floats::ConstPtr& msg)
{
    ROS_INFO("I heard: [%s]", msg->data);
}

void IMUCallback(const progetto_robotica::Floats::ConstPtr& msg)
{
    ROS_INFO("I heard: [%s]", msg->data);
}

void depthSensorCallback(const progetto_robotica::Floats::ConstPtr& msg)
{
    ROS_INFO("I heard: [%s]", msg->data);
}



int main(int argc, char **argv)
{
    // Initialize the ROS system and become a node.
    ros::init(argc, argv, "navigation_flt");

    // Create a ROS node handle
    ros::NodeHandle n;

    double freq = 1000;
    double dt = 1/freq;
    
    // Set the loop rate
    ros::Rate loop_rate(freq);

    // Create a publisher object
    ros::Publisher est_state_pub = n.advertise<progetto_robotica::Floats>("est_state_topic", 1000);

    // Create subscriber objects
    ros::Subscriber GPS_sub = n.subscribe("GPS_topic", 1000, GPSCallback);

    ros::Subscriber IMU_sub = n.subscribe("IMU_topic", 1000, IMUCallback);

    ros::Subscriber depth_sub = n.subscribe("depth_sensor_topic", 1000, depthSensorCallback);



    while (ros::ok())
    {
        // Create a message to send
        std_msgs::String msg;
        msg.data = "Hello, ROS!";

        // Publish the message
        est_state_pub.publish(msg);

        // Let ROS handle all incoming messages in a callback function
        ros::spinOnce();

        // Sleep for the remaining time to hit our 10Hz target
        loop_rate.sleep();
    }

    return 0;
}