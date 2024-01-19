#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include <ros/console.h>
#include <eigen3/Eigen/Dense>         // for eigen matrix
#include "progetto_robotica/Floats.h" // for accessing -- progetto_robotica Floats()
#include <vector>
#include "yaml-cpp/yaml.h" // for yaml

int main(int argc, char **argv)
{
    ros::init(argc, argv, "waypoint_publish");
    ros::NodeHandle n;
    ros::Publisher chatter_pub = n.advertise<progetto_robotica::Floats>("desired_state_topic", 1);
    double freq = 100;
    double dt = 1 / freq;
    ros::Rate loop_rate(freq);
    while (ros::ok())
    {
        double x_d = 5.0;
        double y_d = 5.0;
        double z_d = 1.0;
        double psi_d = 0.0;
        double x_dot_d = 0.0;//0.5*cos(psi_d);
        double y_dot_d = 0.0;//0.5*sin(psi_d);
        double z_dot_d = 0.0;
        double psi_dot_d = 0.0;

        // Publishing the state
        std::vector<double> state = {x_d, y_d, z_d, psi_d, x_dot_d, y_dot_d, z_dot_d, psi_dot_d};
        progetto_robotica::Floats state_msg;
        state_msg.data = state;

        chatter_pub.publish(state_msg);

        ros::spinOnce();

        loop_rate.sleep();
    }

    return 0;
}