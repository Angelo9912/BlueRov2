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
        double h_ott = i_min*0.1;

        // spline sulle x

        double Mx = 3/(2*h_ott*h_ott)*(x1-2*x2+x3);
        double bx1 = 0;
        double bx2 = Mx/2;
        double dx1 = x1;
        double dx2 = x2;
        double ax1 = Mx/(6*h_ott);
        double ax2 = -Mx/(6*h_ott);
        double cx1 = (x2-x1)/h_ott - Mx*h_ott/6;
        double cx2 = (x3-x2)/h_ott - Mx*h_ott/3;

        // spline sulle y   

        double My = 3/(2*h_ott*h_ott)*(y1-2*y2+y3);
        double by1 = 0;
        double by2 = My/2;
        double dy1 = y1;
        double dy2 = y2;
        double ay1 = My/(6*h_ott);
        double ay2 = -My/(6*h_ott);
        double cy1 = (y2-y1)/h_ott - My*h_ott/6;
        double cy2 = (y3-y2)/h_ott - My*h_ott/3;

        // spline sulle z

        double Mz = 3/(2*h_ott*h_ott)*(z1-2*z2+z3);
        double bz1 = 0;
        double bz2 = Mz/2;
        double dz1 = z1;
        double dz2 = z2;
        double az1 = Mz/(6*h_ott);
        double az2 = -Mz/(6*h_ott);
        double cz1 = (z2-z1)/h_ott - Mz*h_ott/6;
        double cz2 = (z3-z2)/h_ott - Mz*h_ott/3;
        int t0 = 0;
        for (t = t0; t < t0+h_ott; t+=h_ott/149){
            
        }
        






































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