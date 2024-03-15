#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include <ros/console.h>
#include <eigen3/Eigen/Dense>                // for eigen matrix
#include "progetto_robotica/Floats.h"        // for accessing -- progetto_robotica Floats()
#include "progetto_robotica/Floats_String.h" // for accessing -- progetto_robotica buoy()
#include <vector>
#include "yaml-cpp/yaml.h" // for yaml

double x_hat = 0.0;
double y_hat = 0.0;
double z_hat = 0.0;
double x_b = 0.0;
double y_b = 0.0;
double z_b = 0.0;

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
    ros::Rate rate(5);

    // Eigen::Matrix<double, 1, 3> sphere1;
    // sphere1 << 3.0, 5.0, 2.5;
    // Eigen::Matrix<double, 1, 3> sphere2;
    // sphere2 << -8.0, -12.0, 2.0;
    // Eigen::Matrix<double, 1, 3> box1;
    // box1 << -3.0, 5.5, 1.0;
    // Eigen::Matrix<double, 1, 3> box2;
    // box2 << 10.0, -10, 1.5;
    // Eigen::Matrix<double, 4, 3> buoy_positions;
    // buoy_positions << sphere1, box1, sphere2, box2;

    Eigen::Matrix<double, 1, 3> sphere1;
    sphere1 << 10.0, 10.0, 3.0;
    Eigen::Matrix<double, 1, 3> sphere2;
    sphere2 << -8.0, -12.0, 2.0;
    Eigen::Matrix<double, 1, 3> box1;
    box1 << 14.0, 10.0, 1.0;
    Eigen::Matrix<double, 1, 3> box2;
    box2 << 10.0, -10.0, 1.5;
    Eigen::Matrix<double, 4, 3> buoy_positions;
    buoy_positions << sphere1, box1, sphere2, box2;

    while (ros::ok())
    {
        // Compute distance from nearest buoy
        Eigen::Matrix<double, 4, 1> distances;
        double dist_sphere1 = sqrt(pow(x_hat - sphere1(0), 2) + pow(y_hat - sphere1(1), 2) + pow(z_hat - sphere1(2), 2));
        double dist_box1 = sqrt(pow(x_hat - box1(0), 2) + pow(y_hat - box1(1), 2) + pow(z_hat - box1(2), 2));
        double dist_sphere2 = sqrt(pow(x_hat - sphere2(0), 2) + pow(y_hat - sphere2(1), 2) + pow(z_hat - sphere2(2), 2));
        double dist_box2 = sqrt(pow(x_hat - box2(0), 2) + pow(y_hat - box2(1), 2) + pow(z_hat - box2(2), 2));
        distances << dist_sphere1, dist_box1, dist_sphere2, dist_box2;
        int i_min = 0;
        for (int i = 0; i < 4; i++)
        {
            if (distances[i] <= distances[i_min])
            {
                i_min = i;
            }
        }

        if ((i_min % 2) == 0)
        {
            x_b = buoy_positions(i_min, 0);
            y_b = buoy_positions(i_min, 1);
            z_b = buoy_positions(i_min, 2);
            std::vector<double> buoy_pos = {x_b, y_b, z_b};
            progetto_robotica::Floats_String buoy_msg;
            buoy_msg.data = buoy_pos;
            buoy_msg.strategy = "Circumference";
            buoy_pub.publish(buoy_msg);
        }
        else if ((i_min % 2) != 0)
        {
            x_b = buoy_positions(i_min, 0);
            y_b = buoy_positions(i_min, 1);
            z_b = buoy_positions(i_min, 2);
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
