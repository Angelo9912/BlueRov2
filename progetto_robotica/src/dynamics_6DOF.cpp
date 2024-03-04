#include "ros/ros.h"
#include <rosbag/bag.h>
#include "std_msgs/String.h"
#include <sstream>
#include <ros/console.h>
#include <eigen3/Eigen/Dense>         // for eigen matrix
#include "progetto_robotica/Floats.h" // for accessing -- progetto_robotica Floats()
#include <vector>
#include "yaml-cpp/yaml.h" // for yaml

double tau_u = 0.0;
double tau_v = 0.0;
double tau_w = 0.0;
double tau_p = 0.0;
double tau_r = 0.0;

double x = +16.5;
double y = +17.0;
double z = 5.0;
double phi = 0;
double theta = 0;
double psi = 0.0;
double u = 0.0;
double v = 0.0;
double w = 0.0;
double p = 0.0;
double q = 0.0;
double r = 0.0;

Eigen::Matrix<double, 6, 1> eta(x, y, z, phi, theta, psi);

Eigen::Matrix<double, 6, 1> nu(u, v, w, p, q, r);

void tauCallback(const progetto_robotica::Floats::ConstPtr &msg)
{
    if (msg->data[0] != msg->data[0])
    {
        tau_u = 0.0;
        tau_v = 0.0;
        tau_w = 0.0;
        tau_p = 0.0;
        tau_r = 0.0;
    }
    else
    {
        tau_u = msg->data[0];
        tau_v = msg->data[1];
        tau_w = msg->data[2];
        tau_p = msg->data[3];
        tau_r = msg->data[4];
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "dynamics");
    ros::NodeHandle n;

    rosbag::Bag state_bag;

    state_bag.open("/home/angelo/catkin_ws/src/progetto_robotica/bag/state.bag", rosbag::bagmode::Write);

    // Import parameters from YAML file
    double m = 0.0;
    double d = 0.0;
    double l = 0.0;
    double I = 0.0;
    double X_u_dot = 0.0;
    double Y_v_dot = 0.0;
    double X_u = 0.0;
    double Y_r = 0.0;
    double Y_v = 0.0;
    double N_r = 0.0;
    double N_r_r = 0.0;
    double Z_w = 0.0;
    double N_v = 0.0;
    double x_g = 0.0;
    double y_g = 0.0;
    double z_g = 0.0;
    double I_x = 0.0;   
    double I_y = 0.0;
    double I_z = 0.0;
    double I_xy = 0.0;
    double I_xz = 0.0;
    double I_yz = 0.0;
    double I_yx = 0.0;
    double I_zx = 0.0;

    n.getParam("m", m);
    n.getParam("d", d);
    n.getParam("l", l);
    I = 1.0 / 12.0 * m * (pow(d, 2) + pow(l, 2));
    n.getParam("X_u_dot", X_u_dot);
    n.getParam("Y_v_dot", Y_v_dot);
    n.getParam("X_u", X_u);
    n.getParam("Y_r", Y_r);
    n.getParam("Y_v", Y_v);
    n.getParam("N_r", N_r);
    n.getParam("N_r_r", N_r_r);
    n.getParam("Z_w", Z_w);
    n.getParam("N_v", N_v);

    Eigen::Matrix<double, 6, 6> M_rb;
    M_rb << m, 0.0, 0.0, 0.0, m * z_g, -m *y_g,
        0.0, m, 0.0, -m * z_g, 0.0, m * x_g,
        0.0, 0.0, m, m * y_g, -m * x_g, 0.0,
        0.0, -m * z_g, m * y_g, I_x, -I_xy, -I_xz,
        m * z_g, 0.0, -m * x_g, -I_xy, I_y, -I_yz,
        -m * y_g, m * x_g, 0.0, -I_xz, -I_yz, I_z;

    ros::Publisher chatter_pub = n.advertise<progetto_robotica::Floats>("state_topic", 1);
    ros::Subscriber sub = n.subscribe("tau_topic", 1000, tauCallback);
    double freq = 1000;
    double dt = 1 / freq;
    ros::Rate loop_rate(freq);

    while (ros::ok())
    {
        // DEFINIAMO LA DINAMICA DEL SISTEMA

        // MATRICE DI CORIOLIS
        Eigen::Matrix<double, 6, 6> C;
        // C << 0.0, 0.0, 0.0, Y_v_dot * nu(1) + Y_r * nu(3),
        //     0.0, 0.0, 0.0, -X_u_dot * nu(0),
        //     0.0, 0.0, 0.0, 0.0,
        //     -Y_v_dot * nu(1) - Y_r * nu(3), X_u_dot * nu(0), 0.0, 0.0;
        u = nu(0);
        v = nu(1);
        w = nu(2);
        p = nu(3);
        q = nu(4);
        r = nu(5);

        C << 0.0, 0.0, 0.0, m*(y_g*q + z_g*r), -m*(x_g*q - w), -m*(x_g*r+v),
            0.0, 0.0, 0.0, -m*(y_g*p + w), m*(z_g*r + x_g*p), -m*(y_g * r - u),
            0.0, 0.0, 0.0, -m*(z_g*p - v), -m*(z_g*q + u), m*(x_g*p + y_g*q),
            -m*(y_g*q + z_g*r), m*(y_g*p + w), m*(z_g*p - v), 0.0, I_z*r - I_yz*q- I_xz*p, I_xy*p - I_yz*r - I_y*q;
            m*(x_g*q - w), -m*(z_g*r + x_g*p), m*(z_g*q + u), I_yz*q + I_xz * p - I_z, 0.0, -I_xz*r - I_xy*q + I_x*p;
            m*(x_g*r + v), -m*(y_g*r - u), -m*(x_g*p + y_g*q), -I_yz*r - I_xy*p + I_y*q, I_xy*q + I_xz*r - I_x*p, 0.0;
        // MATRICE DI DAMPING
        Eigen::Matrix<double, 4, 4> D;
        // D.diagonal() << -X_u, -Y_v, -Z_w, -N_r - N_r_r * nu(3);
        D << -X_u, 0.0, 0.0, 0.0,
            0.0, -Y_v, 0.0, -Y_r,
            0.0, 0.0, -Z_w, 0.0,
            0.0, -N_v, 0.0, -N_r;

        // VETTORE DI FORZE E MOMENTI
        Eigen::Matrix<double, 4, 1> tau;
        tau << tau_u, tau_v, tau_w, tau_r;

        // VETTORE DELLE VELOCITA'
        Eigen::Matrix<double, 4, 1> nu_k1;
        nu_k1 = (dt * M.inverse() * (tau - C * nu - D * nu)) + nu;

        // VETTORE DELLE POSIZIONI
        Eigen::Matrix<double, 4, 4> Jacobian;
        Jacobian << cos(eta(3)), -sin(eta(3)), 0.0, 0.0,
            sin(eta(3)), cos(eta(3)), 0.0, 0.0,
            0.0, 0.0, 1.0, 0.0,
            0.0, 0.0, 0.0, 1.0;

        Eigen::Matrix<double, 4, 1> eta_dot;
        eta_dot = Jacobian * nu;
        Eigen::Matrix<double, 4, 1> eta_k1;
        eta_k1 = dt * eta_dot + eta;

        // wrapToPi(eta_k1(3));
        double rem = std::fmod(eta_k1(3) + M_PI, 2 * M_PI);
        if (rem < 0)
        {
            rem += 2 * M_PI;
        }
        eta_k1(3) = rem - M_PI;

        // SETTARE LA POSIZIONE DEL MODELLO
        std::vector<double> state = {eta(0), eta(1), eta(2), eta(3), nu(0), nu(1), nu(2), nu(3)};

        progetto_robotica::Floats state_msg;
        state_msg.data = state;

        chatter_pub.publish(state_msg);
        // double stampa = eta(3);
        // //ROS_INFO("%f", stampa);
        // // ROS_WARN("%f",stampa);
        eta = eta_k1;
        nu = nu_k1;

        if (ros::Time::now().toSec() > ros::TIME_MIN.toSec())
        {
            state_bag.write("state_topic", ros::Time::now(), state_msg);
        }

        ros::spinOnce();

        loop_rate.sleep();
    }

    state_bag.close();
    return 0;
}