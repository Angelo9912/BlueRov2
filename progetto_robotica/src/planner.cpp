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
double psi_hat = 0.0;
double u_hat = 0.0;
double v_hat = 0.0;
double w_hat = 0.0;
double r_hat = 0.0;


double x_b = 0.0;
double y_b = 0.0;
double z_b = 0.0;
std::string strategy = "";


// Subscriber callback function

void estStateCallback(const progetto_robotica::Floats::ConstPtr &msg)
{
    if (msg->data[0] != msg->data[0])
    {
        x_hat = 0.0;
        y_hat = 0.0;
        z_hat = 0.0;
        psi_hat = 0.0;
        u_hat = 0.0;
        v_hat = 0.0;
        w_hat = 0.0;
        r_hat = 0.0;
    }
    else
    {
        x_hat = msg->data[0];
        y_hat = msg->data[1];
        z_hat = msg->data[2];
        psi_hat = msg->data[3];
        u_hat = msg->data[4];
        v_hat = msg->data[5];
        w_hat = msg->data[6];
        r_hat = msg->data[7];
    }
}

void buoyCallback(const progetto_robotica::Floats_String::ConstPtr &msg)
{
    if (msg->data[0] != msg->data[0])
    {
        x_b = 0.0;
        y_b = 0.0;
        z_b = 0.0;
        strategy = "";
        
    }
    else
    {
        x_b = msg->data[0];
        y_b = msg->data[1];
        z_b = msg->data[2];
        strategy = msg->strategy;
    }
}

int main(int argc, char **argv)
{
    // Initialize the ROS node
    ros::init(argc, argv, "publisher_subscriber_node");

    // Create a ROS node handle
    ros::NodeHandle nh;

    // Create a publisher object
    ros::Publisher publisher = nh.advertise<progetto_robotica::Floats_String>("waypoints_topic", 10);

    // Create a subscriber object
    ros::Subscriber subscriber = nh.subscribe("buoy_topic", 10, buoyCallback);
    ros::Subscriber subscriber_state = nh.subscribe("state_topic", 10, estStateCallback);
    // Set the loop rate (in Hz)
    ros::Rate loop_rate(2);

    // Main loop
    while (ros::ok())
    {
        // Compute distance from nearest buoy
        double dist2buoy = sqrt(pow(x_hat - x_b, 2) + pow(y_hat - y_b, 2));
        double dist2point = sqrt(dist2buoy*dist2buoy + 1.0);
        double alpha = atan2(1,dist2buoy);
        double beta = atan2(y_b - y_hat, x_b - x_hat) - alpha;
        double x_p = x_hat + dist2point*cos(beta);
        double y_p = y_hat + dist2point*sin(beta);
        double z_p = z_b;
        progetto_robotica::Floats_String waypoint_msg;
        if(strategy == "Circumference"){
            waypoint_msg.strategy = "Circumference";
            std::vector<double> waypoint_pos = {x_b, y_b, z_b, x_p, y_p, z_p};
            waypoint_msg.data = waypoint_pos;
        }
        else if(strategy == "UP_DOWN"){
            waypoint_msg.strategy = "UP_DOWN";
            std::vector<double> waypoint_pos = {x_b, y_b, z_b};
        }

        // Publish the message
        publisher.publish(msg);

        // Process any incoming messages
        ros::spinOnce();

        // Sleep to maintain the loop rate
        loop_rate.sleep();
    }

    return 0;
}
