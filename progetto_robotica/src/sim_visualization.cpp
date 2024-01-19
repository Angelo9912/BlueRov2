#include "ros/ros.h"
#include <sstream>
#include <ros/console.h>
#include <eigen3/Eigen/Dense>         // for eigen matrix
#include "progetto_robotica/Floats.h" // for accessing -- progetto_robotica Floats()
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
double psi = 0.0;
double u = 0.0;
double v = 0.0;
double w = 0.0;
double r = 0.0;

geometry_msgs::Pose pose_msg;
geometry_msgs::Twist twist_msg;
gazebo_msgs::ModelState state_msg;
tf2::Quaternion q;

void stateCallback(const progetto_robotica::Floats::ConstPtr &msg)
{
    x = msg->data[0];
    y = msg->data[1];
    z = msg->data[2];
    psi = msg->data[3];
    u = msg->data[4];
    v = msg->data[5];
    w = msg->data[6];
    r = msg->data[7];
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "sim_visualization");
    ros::NodeHandle n;
    ros::Publisher chatter_pub = n.advertise<gazebo_msgs::ModelState>("gazebo/set_model_state", 1);
    ros::Subscriber sub = n.subscribe("state_topic", 1, stateCallback);

    double freq = 100.0;
    double dt = 1 / freq;
    ros::Rate loop_rate(freq);
    while (ros::ok())
    {
        // Converting the state to quaternion

        q.setRPY(0.0, 0.0, psi);
        pose_msg.position.x = x;
        pose_msg.position.y = y;
        pose_msg.position.z = z;
        pose_msg.orientation.x = q.x();
        pose_msg.orientation.y = q.y();
        pose_msg.orientation.z = q.z();
        pose_msg.orientation.w = q.w();

        twist_msg.linear.x = u;
        twist_msg.linear.y = v;
        twist_msg.linear.z = w;
        twist_msg.angular.x = 0.0;
        twist_msg.angular.y = 0.0;
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