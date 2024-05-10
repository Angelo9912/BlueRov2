#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include <ros/console.h>
#include <eigen3/Eigen/Dense>            // for eigen matrix
#include "tesi_bluerov2/Floats.h"        // for accessing -- tesi_bluerov2 Floats()
#include "tesi_bluerov2/Floats_String.h" // for accessing -- tesi_bluerov2 buoy()
#include "tesi_bluerov2/buoy.h"      // for accessing -- tesi_bluerov2
#include <vector>
#include "yaml-cpp/yaml.h" // for yaml

double x_hat = 0.0;
double y_hat = 0.0;
double z_hat = 0.0;
double x_b = 0.0;
double y_b = 0.0;
double z_b = 0.0;
int clockwise;
int up;

// Callback function for the state_topic subscriber
void estStateCallback(const tesi_bluerov2::Floats::ConstPtr &msg)
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
    ros::Publisher buoy_pub = nh.advertise<tesi_bluerov2::buoy>("sensors/buoy_topic", 10);

    // Create a subscriber that subscribes to messages on the "state_topic" topic
    ros::Subscriber state_sub = nh.subscribe("state/state_topic", 10, estStateCallback);

    // Set the publishing rate (e.g., 10 Hz)
    ros::Rate rate(5);

    Eigen::Matrix<double, 1, 3> sphere1;
    sphere1 << 0.0, 4.0, 0.4;
    Eigen::Matrix<double, 1, 3> sphere2;
    sphere2 << 0.0, 4.0, 4.0;
    Eigen::Matrix<double, 1, 3> box1;
    box1 << 8.0, 6.0, 0.4;
    Eigen::Matrix<double, 1, 3> box2;
    box2 << 8.0, 6.0, 4.0;
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
            tesi_bluerov2::buoy buoy_msg;
            buoy_msg.buoy_pos = buoy_pos;
            buoy_msg.strategy = "Circumference";
            if (distances[i_min] <= 1.5)
            {
                if (i_min == 0)
                    buoy_msg.clockwise = 1;
                else 
                    buoy_msg.clockwise = 0;
                buoy_pub.publish(buoy_msg);
            }
        }
        else if ((i_min % 2) != 0)
        {
            x_b = buoy_positions(i_min, 0);
            y_b = buoy_positions(i_min, 1);
            z_b = buoy_positions(i_min, 2);
            std::vector<double> buoy_pos = {x_b, y_b, z_b};
            tesi_bluerov2::buoy buoy_msg;
            buoy_msg.buoy_pos = buoy_pos;
            buoy_msg.strategy = "UP_DOWN";
            if (distances[i_min] <= 1.5)
            {
                if (i_min == 1)
                    buoy_msg.up = 0;
                else
                    buoy_msg.up = 1;
                buoy_pub.publish(buoy_msg);
            }
        }

        // Spin once to let the ROS node handle callbacks
        ros::spinOnce();

        // Sleep for the remaining time to achieve the desired publishing rate
        rate.sleep();
    }

    return 0;
}
