#include "ros/ros.h"
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
double tau_r = 0.0;

double x = 0.0;
double y = 0.0;
double z = 0.0;
double psi = 0.0;
double u = 0.0;
double v = 0.0;
double w = 0.0;
double r = 0.0;


Eigen::Matrix<double, 4, 1> eta(x, y, z, psi);

Eigen::Matrix<double, 4, 1> nu(u, v, w, r);

void tauCallback(const progetto_robotica::Floats::ConstPtr &msg)
{
    if (msg->data[0] != msg->data[0])
    {
        tau_u = 0.0;
        tau_v = 0.0;
        tau_w = 0.0;
        tau_r = 0.0;
    }
    else
    {
        tau_u = msg->data[0];
        tau_v = msg->data[1];
        tau_w = msg->data[2];
        tau_r = msg->data[3];
    }
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "dynamics");
    ros::NodeHandle n;

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

    Eigen::Matrix<double, 4, 4> M;
    M << m, 0.0, 0.0, 0.0,
        0.0, m, 0.0, 0.0,
        0.0, 0.0, m, 0.0,
        0.0, 0.0, 0.0, I;

    ros::Publisher chatter_pub = n.advertise<progetto_robotica::Floats>("state_topic", 1);
    ros::Subscriber sub = n.subscribe("tau_topic", 1000, tauCallback);
    double freq = 100;
    double dt = 1 / freq;
    ros::Rate loop_rate(freq);

    while (ros::ok())
    {
        // DEFINIAMO LA DINAMICA DEL SISTEMA

        // MATRICE DI CORIOLIS
        Eigen::Matrix<double, 4, 4> C;
        C << 0.0, 0.0, 0.0, Y_v_dot * nu(1) + Y_r * nu(3),
            0.0, 0.0, 0.0, -X_u_dot * nu(0),
            0.0, 0.0, 0.0, 0.0,
            -Y_v_dot * nu(1) - Y_r * nu(3), X_u_dot * nu(0), 0.0, 0.0;

        // MATRICE DI DAMPING
        Eigen::Matrix<double, 4, 4> D;
        D.diagonal() << -X_u, -Y_v, -Z_w, -N_r - N_r_r * nu(3);

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
            0.0, 0.0, 1, 0.0,
            0.0, 0.0, 0.0, 1;

        Eigen::Matrix<double, 4, 1> eta_dot;
        eta_dot = Jacobian * nu;
        Eigen::Matrix<double, 4, 1> eta_k1;
        eta_k1 = dt * eta_dot + eta;


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

        ros::spinOnce();

        loop_rate.sleep();
    }

    return 0;
}