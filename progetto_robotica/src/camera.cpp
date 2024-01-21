#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include <ros/console.h>
#include <eigen3/Eigen/Dense>         // for eigen matrix
#include "progetto_robotica/Floats.h" // for accessing -- progetto_robotica Floats()
#include "progetto_robotica/Floats_String.h"   // for accessing -- progetto_robotica buoy()
#include <vector>
#include "yaml-cpp/yaml.h" // for yaml

double x_hat = 0.0;
double y_hat = 0.0;
double z_hat = 0.0;

// Callback function for the state_topic subscriber
void estStateCallback(const progetto_robotica::Floats::ConstPtr &msg)
{
    if (msg->data[0] != msg->data[0])
    {
        x_hat = 0.0;
        y_hat = 0.0;
        z_hat = 0.0;
    }
    else
    {
        x_hat = msg->data[0];
        y_hat = msg->data[1];
        z_hat = msg->data[2];
    }
}

int main(int argc, char **argv)
{
    // Initialize the ROS node
    ros::init(argc, argv, "camera_node");

    // Create a ROS node handle
    ros::NodeHandle nh;

    // Create a publisher that publishes std_msgs::String messages on the "buoy_topic" topic
    ros::Publisher buoy_pub = nh.advertise<progetto_robotica::Floats_String>("buoy_topic", 10);

    // Create a subscriber that subscribes to messages on the "state_topic" topic
    ros::Subscriber state_sub = nh.subscribe("state_topic", 10, estStateCallback);

    // Set the publishing rate (e.g., 10 Hz)
    ros::Rate rate(10);
    double sphere[3];
    sphere = [ 3.0, 5.0, 3.0 ];
    double box[3];
    box = [ -3.0, 5.5, 5.0 ];

    while (ros::ok())
    {
        // Compute distance from nearest buoy
        dist_sphere = sqrt(pow(x_hat - sphere[0], 2) + pow(y_hat - sphere[1], 2) + pow(z_hat - sphere[2], 2));
        dist_box = sqrt(pow(x_hat - box[0], 2) + pow(y_hat - box[1], 2) + pow(z_hat - box[2], 2));

        if (dist_sphere < dist_box)
        {
            x_b = sphere[0];
            y_b = sphere[1];
            z_b = sphere[2];
            std::vector<double> buoy_pos = {x_b, y_b, z_b};
            progetto_robotica::Floats_String buoy_msg;
            buoy_msg.data = buoy_pos;
            buoy_msg.strategy = "Circumference";
            buoy_pub.publish(buoy_msg);
        }
        else
        {
            x_b = box[0];
            y_b = box[1];
            z_b = box[2];
            std::vector<double> buoy_pos = {x_b, y_b, z_b};
            progetto_robotica::Floats_String buoy_msg;
            buoy_msg.data = buoy_pos;
            buoy_msg.strategy = "UP_DOWN";
            buoy_pub.publish(buoy_msg);
        }

        // Spin once to let the ROS node handle callbacks
        ros::spinOnce();

        // Sleep for the remaining time to achieve the desired publishing rate
        rate.sleep();
    }

    return 0;
}
