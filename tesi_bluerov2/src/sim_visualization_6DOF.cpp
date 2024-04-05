#include "ros/ros.h"
#include <sstream>
#include <ros/console.h>
#include <eigen3/Eigen/Dense>         // for eigen matrix
#include "tesi_bluerov2/Floats.h" // for accessing -- tesi_bluerov2 Floats()
#include "gazebo_msgs/ModelState.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Quaternion.h"
#include <tf2/LinearMath/Quaternion.h> // for tf2 quaternion
#include <vector>
#include "yaml-cpp/yaml.h" // for yaml


// Global variables
double x = 0.0;
double y = 0.0;
double z = 0.0;
double phi = 0.0;
double theta = 0.0;
double psi = 0.0;
double u = 0.0;
double v = 0.0;
double w = 0.0;
double p = 0.0;
double q = 0.0;
double r = 0.0;

geometry_msgs::Pose pose_msg;
geometry_msgs::Twist twist_msg;
gazebo_msgs::ModelState state_msg;
tf2::Quaternion quat;

void stateCallback(const tesi_bluerov2::Floats::ConstPtr &msg)
{
    x = msg->data[0];
    y = msg->data[1];
    z = msg->data[2];
    phi = msg->data[3];
    theta = msg->data[4];
    psi = msg->data[5];
    u = msg->data[6];
    v = msg->data[7];
    w = msg->data[8];
    p = msg->data[9];
    q = msg->data[10];
    r = msg->data[11];
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "sim_visualization_6DOF");
    ros::NodeHandle n;
    ros::Publisher chatter_pub = n.advertise<gazebo_msgs::ModelState>("gazebo/set_model_state", 1);
    ros::Subscriber sub = n.subscribe("state_topic", 1, stateCallback);

    double freq = 100.0;
    double dt = 1 / freq;
    ros::Rate loop_rate(freq);

    while (ros::ok())
    {
        // Converting the state to quaternion

        quat.setRPY(phi, theta, psi - M_PI_2);
        pose_msg.position.x = y;
        pose_msg.position.y = x;
        pose_msg.position.z = -z + 10;
        pose_msg.orientation.x = quat.y();
        pose_msg.orientation.y = quat.x();
        pose_msg.orientation.z = -quat.z();
        pose_msg.orientation.w = quat.w();

        twist_msg.linear.x = u;
        twist_msg.linear.y = v;
        twist_msg.linear.z = w;
        twist_msg.angular.x = p;
        twist_msg.angular.y = q;
        twist_msg.angular.z = r;

        // Publishing the state
        state_msg.model_name = "bluerov2";
        state_msg.pose = pose_msg;
        state_msg.twist = twist_msg;
        state_msg.reference_frame = "world";

        chatter_pub.publish(state_msg);

        ros::spinOnce();

        loop_rate.sleep();
    }

    return 0;
}