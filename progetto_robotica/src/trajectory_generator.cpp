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
    ros::init(argc, argv, "trajectory_generator");
    ros::NodeHandle n;
    ros::Publisher chatter_pub = n.advertise<progetto_robotica::Floats>("desired_state_topic", 1);
    double freq = 100;
    double dt = 1 / freq;
    ros::Rate loop_rate(freq);

    //Punti di interpolazione

    double x1 = 0.0;
    double y1 = 0.0;
    double z1 = 0.0;

    double x2 = 5.0;
    double y2 = 5.0;
    double z2 = 2.0;

    double x3 = 10.0;
    double y3 = 10.0;
    double z3 = 4.0;


    double coeff_acc = 5.0;
    double coeff_vel = 5.0;
    double coeff_temp = 1.0;
    Eigen::Vector<double,1000> J_eval;
    Eigen::Index i_min;
    while (ros::ok())
    {
        for(int i=1, i<=1000, i++){

            double h = i*0.1;

            double J = 2*coeff_temp*h + (3*coeff_acc*(x1*x1-4*x1*x2+2*x1*x3+4*x2*x2-4*x2*x3+x3*x3+y1*y1-4*y1*y2+2*y1*y3+4*y2*y2-4*y2*y3+y3*y3+z1*z1-4*z1*z2+2*z1*z3+4*z2*z2-4*z2*z3+z3*z3))/(2*h*h*h)+
            (coeff_vel*(11*x1*x1-24*x1*x2+2*x1*x3+24*x2*x2-24*x2*x3+11*x3*x3+11*y1*y1-24*y1*y2+2*y1*y3+24*y2*y2-24*y2*y3+11*y3*y3+11*z1*z1-24*z1*z2+2*z1*z3+24*z2*z2-24*z2*z3+11*z3*z3))/(10*h)+
            (coeff_vel*(13*x1*x1-36*x1*x2+10*x1*x3+36*x2*x2-36*x2*x3+13*x3*x3+13*y1*y1-36*y1*y2+10*y1*y3+36*y2*y2-36*y2*y3+13*y3*y3+13*z1*z1-36*z1*z2+10*z1*z3+36*z2*z2-36*z2*z3+13*z3*z3))/(8*h*h);
            
            J_eval(i) = J;
        }

        double J_min = J_eval.minCoeff(&i_min);

        






































        double x_d = 5.0;
        double y_d = 5.0;
        double z_d = 2.0;
        double psi_d = 0.0;
        double x_dot_d = 0.01;
        double y_dot_d = 0.01;
        double z_dot_d = 0.01;
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