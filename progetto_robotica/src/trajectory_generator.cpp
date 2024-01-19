#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include <ros/console.h>
#include <eigen3/Eigen/Dense>         // for eigen matrix
#include "progetto_robotica/Floats.h" // for accessing -- progetto_robotica Floats()
#include <vector>
#include "yaml-cpp/yaml.h" // for yaml

// Declare callback variables
double x_hat = 0.0;
double y_hat = 0.0;
double z_hat = 0.0;
double psi_hat = 0.0;
double u_hat = 0.0;
double v_hat = 0.0;
double w_hat = 0.0;
double r_hat = 0.0;

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

int main(int argc, char **argv)
{
    ros::init(argc, argv, "trajectory_generator");
    ros::NodeHandle n;
    ros::Publisher chatter_pub = n.advertise<progetto_robotica::Floats>("desired_state_topic", 1);
    ros::Subscriber sub_est_state = n.subscribe("state_topic", 1, estStateCallback);
    double freq = 100.0;
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
    double y3 = 0.0;
    double z3 = 4.0;


    // double coeff_acc = 5.0;
    // double coeff_vel = 5.0;
    // double coeff_temp = 1.0;
    // Eigen::Vector<double,1000> J_eval;
    Eigen::Vector<double,300> dist;
    // double J_min;
    double dist_min;

    // int i_min = 0;
    int i_dist_min = 0;
    while (ros::ok())
    {
        // for(int i=0; i<1000; i++){

        //     double h = i*0.1;

        //     double J = 2.0*coeff_temp*h + (3.0*coeff_acc*(x1*x1-4.0*x1*x2+2.0*x1*x3+4.0*x2*x2-4.0*x2*x3+x3*x3+y1*y1-4.0*y1*y2+2.0*y1*y3+4.0*y2*y2-4.0*y2*y3+y3*y3+z1*z1-4.0*z1*z2+2.0*z1*z3+4.0*z2*z2-4.0*z2*z3+z3*z3))/(2.0*h*h*h)+
        //     (coeff_vel*(11.0*x1*x1-24.0*x1*x2+2.0*x1*x3+24.0*x2*x2-24.0*x2*x3+11.0*x3*x3+11.0*y1*y1-24.0*y1*y2+2.0*y1*y3+24.0*y2*y2-24.0*y2*y3+11.0*y3*y3+11.0*z1*z1-24.0*z1*z2+2.0*z1*z3+24.0*z2*z2-24.0*z2*z3+11.0*z3*z3))/(10.0*h)+
        //     (coeff_vel*(13.0*x1*x1-36.0*x1*x2+10.0*x1*x3+36.0*x2*x2-36.0*x2*x3+13.0*x3*x3+13.0*y1*y1-36.0*y1*y2+10.0*y1*y3+36.0*y2*y2-36.0*y2*y3+13.0*y3*y3+13.0*z1*z1-36.0*z1*z2+10.0*z1*z3+36.0*z2*z2-36.0*z2*z3+13.0*z3*z3))/(8.0*h*h);
            
        //     if(i==0){
        //         J_min = J;
        //     }
        //     else{
        //         if(J<J_min){
        //             J_min = J;
        //             i_min = i;
        //         }
        //     }

        //     J_eval(i) = J;
        // }

        double u_d = 0.5;
        double v_d = 0.0;
        double w_d = 0.0;
        double r_d = 0.0;
        double vel = sqrt(u_d*u_d+v_d*v_d+w_d*w_d);
        double waypoint_distance = (sqrt(pow(x2-x1,2) + pow(y2-y1,2) + pow(z2-z1,2)) + sqrt(pow(x3-x2,2) + pow(y3-y2,2) + pow(z3-z2,2)))/2;

        double h_ott = waypoint_distance/vel;
        //(i_min)*0.1;

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

        double x[300];
        double y[300];
        double z[300];
        double psi[300];

    

        for (int i=0; i<150; i++){

            // Troviamo il tempo corrispondente all'indice i
            double t = (i+1)*h_ott/150;

            x[i] = ax1*pow(t-t0,3.0) + bx1*pow(t-t0,2.0) + cx1*(t-t0) + dx1;
            y[i] = ay1*pow(t-t0,3.0) + by1*pow(t-t0,2.0) + cy1*(t-t0) + dy1;
            z[i] = az1*pow(t-t0,3.0) + bz1*pow(t-t0,2.0) + cz1*(t-t0) + dz1;
            psi[i] = atan2(3*ay1*pow(t-t0,2.0) + 2*by1*(t-t0) + cy1, 3*ax1*pow(t-t0,2.0) + 2*bx1*(t-t0) + cx1);
        }

        for (int i=150; i<=299; i++){

            // Troviamo il tempo corrispondente all'indice i
            double t = (i+1)*h_ott/149;

            x[i] = ax2*pow(t-t0-h_ott,3.0) + bx2*pow(t-t0-h_ott,2.0) + cx2*(t-t0-h_ott) + dx2;
            y[i] = ay2*pow(t-t0-h_ott,3.0) + by2*pow(t-t0-h_ott,2.0) + cy2*(t-t0-h_ott) + dy2;
            z[i] = az2*pow(t-t0-h_ott,3.0) + bz2*pow(t-t0-h_ott,2.0) + cz2*(t-t0-h_ott) + dz2;
            psi[i] = atan2(3*ay2*pow(t-t0-h_ott,2.0) + 2*by2*(t-t0-h_ott) + cy2, 3*ax2*pow(t-t0-h_ott,2.0) + 2*bx2*(t-t0-h_ott) + cx2);


        }

        for (int i=0; i<=299; i++){
            dist(i) = sqrt(pow(x_hat -x[i],2.0) + pow(y_hat-y[i],2.0) + pow(z_hat-z[i],2.0));
            if(i==0){
                dist_min = dist(i);
            }
            else{
                if(dist(i)<dist_min){
                    dist_min = dist(i);
                    i_dist_min = i;
                }
            }
        }

        double x_d = x[i_dist_min];
        double y_d = y[i_dist_min];
        double z_d = z[i_dist_min];
        double psi_d = psi[i_dist_min];

        if(i_dist_min>280){
            u_d = 0.0;
            v_d = 0.0;
            w_d = 0.0;
            r_d = 0.0;
        }

        double x_dot_d = u_d*cos(psi_d)-v_d*sin(psi_d);
        double y_dot_d = u_d*sin(psi_d)+v_d*cos(psi_d);
        double z_dot_d = w_d;
        double psi_dot_d = r_d;

        // Publishing the state
        std::vector<double> waypoint = {x_d, y_d, z_d, psi_d, x_dot_d, y_dot_d, z_dot_d, psi_dot_d};
        progetto_robotica::Floats waypoint_msg;
        waypoint_msg.data = waypoint;

        ROS_WARN("u: %f", u_hat);

        chatter_pub.publish(waypoint_msg);

        ros::spinOnce();

        loop_rate.sleep();
    }

    return 0;
}