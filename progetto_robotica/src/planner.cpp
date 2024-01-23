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
double psi_hat = 0.0;
double u_hat = 0.0;
double v_hat = 0.0;
double w_hat = 0.0;
double r_hat = 0.0;

double x_b = 0.0;
double y_b = 0.0;
double z_b = 0.0;
std::string strategy = "";
std::string mission_status = "";
std::string status_req = "";
bool buoy_seen = false;

double target_x = 0.0;
double target_y = 0.0;
double target_z = 0.0;

Eigen::Matrix<double, 1, 3> buoys_pos;
int n_buoys = 0;

// Subscriber callback function

void statusCallback(const std_msgs::String::ConstPtr &msg)
{
    mission_status = msg->data.c_str();
}

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

        

        if(n_buoys == 0){
            buoys_pos(0) = x_b;
            buoys_pos(1) = y_b;
            buoys_pos(2) = z_b;
            n_buoys++;
        }
        else if(n_buoys > 0){
            buoys_pos.conservativeResize(n_buoys+1, 3);
            buoys_pos(n_buoys, 0) = x_b;
            buoys_pos(n_buoys, 1) = y_b;
            buoys_pos(n_buoys, 2) = z_b;
            n_buoys++;
        }
    }
}

int main(int argc, char **argv)
{
    // Initialize the ROS node
    ros::init(argc, argv, "planner_node");

    // Create a ROS node handle
    ros::NodeHandle nh;

    // Create a publisher object
    ros::Publisher publisher = nh.advertise<progetto_robotica::Floats_String>("waypoints_topic", 10);
    ros::Publisher publisher_status = nh.advertise<std_msgs::String>("manager/status_requested_topic", 10);

    // Create a subscriber object
    ros::Subscriber subscriber = nh.subscribe("buoy_topic", 10, buoyCallback);
    ros::Subscriber subscriber_state = nh.subscribe("state_topic", 10, estStateCallback);
    ros::Subscriber subscriber_status = nh.subscribe("manager/status_topic", 10, statusCallback);
    nh.getParam("target_x", target_x);
    nh.getParam("target_y", target_y);
    nh.getParam("target_z", target_z);
    double target[3] = {target_x, target_y, target_z};

    // Set the loop rate (in Hz)
    ros::Rate loop_rate(2);
    // Main loop
    while (ros::ok())
    {
        // Compute distance from nearest buoy
        double alpha = atan2(y_b - y_hat, x_b - x_hat);
        double x_p = x_b + cos(alpha - M_PI / 2);
        double y_p = y_b + sin(alpha - M_PI / 2);
        double z_p = z_b;
        double dist2buoy = sqrt(pow(x_b - x_hat, 2) + pow(y_b - y_hat, 2) + pow(z_b - z_hat, 2));
        buoy_seen = dist2buoy < 100.0;
        progetto_robotica::Floats_String waypoint_msg;
        if (mission_status == "PAUSED")
        {
            if (strategy == "Circumference" && buoy_seen)
            {
                waypoint_msg.strategy = "Circumference";
                std::vector<double> waypoint_pos = {x_b, y_b, z_b, x_p, y_p, z_p};
                waypoint_msg.data = waypoint_pos;
                status_req = "RUNNING";
                // Publish the message
                std_msgs::String status_req_msg;
                status_req_msg.data = status_req;
                // Publish the message
                publisher_status.publish(status_req_msg);
                publisher.publish(waypoint_msg);
            }
            else if (strategy == "UP_DOWN" && buoy_seen)
            {
                waypoint_msg.strategy = "UP_DOWN";
                std::vector<double> waypoint_pos = {x_b, y_b, z_b};
                waypoint_msg.data = waypoint_pos;
                status_req = "RUNNING";
                // Publish the message
                std_msgs::String status_req_msg;
                status_req_msg.data = status_req;
                // Publish the message
                publisher_status.publish(status_req_msg);
                publisher.publish(waypoint_msg);
            }
            else if (strategy == "Spline")
            {
            }
        }
        else if (mission_status == "RUNNING" && buoy_seen && strategy == "Spline")
        {
            status_req = "PAUSED";
            // Publish the message
            std_msgs::String status_req_msg;
            status_req_msg.data = status_req;
            // Publish the message
            publisher_status.publish(status_req_msg);
        }

        // Process any incoming messages
        ros::spinOnce();

        // Sleep to maintain the loop rate
        loop_rate.sleep();
    }

    return 0;
}
