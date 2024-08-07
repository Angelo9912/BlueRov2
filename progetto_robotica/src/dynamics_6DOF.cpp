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
double tau_q = 0.0;
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
        tau_q = 0.0;
        tau_r = 0.0;
    }
    else
    {
        tau_u = msg->data[0];
        tau_v = msg->data[1];
        tau_w = msg->data[2];
        tau_p = msg->data[3];
        tau_q = msg->data[4];
        tau_r = msg->data[4];
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "dynamics");
    ros::NodeHandle n;

    rosbag::Bag state_bag;

    state_bag.open("/home/antonio/catkin_ws/src/progetto_robotica/bag/state.bag", rosbag::bagmode::Write);

    // Import parameters from YAML file
    double m = 0.0;
    double d = 0.0;
    double l = 0.0;
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
    double I_zy = 0.0;
    double A_x = 0.0;
    double A_y = 0.0;
    double A_z = 0.0;
    double A_p = 0.0;
    double A_q = 0.0;
    double A_r = 0.0;
    double X_u_dot = 0.0;
    double X_v_dot = 0.0;
    double X_w_dot = 0.0;
    double X_p_dot = 0.0;
    double X_q_dot = 0.0;
    double X_r_dot = 0.0;
    double Y_u_dot = 0.0;
    double Y_v_dot = 0.0;
    double Y_w_dot = 0.0;
    double Y_p_dot = 0.0;
    double Y_q_dot = 0.0;
    double Y_r_dot = 0.0;
    double Z_u_dot = 0.0;
    double Z_v_dot = 0.0;
    double Z_w_dot = 0.0;
    double Z_p_dot = 0.0;
    double Z_q_dot = 0.0;
    double Z_r_dot = 0.0;
    double K_u_dot = 0.0;
    double K_v_dot = 0.0;
    double K_w_dot = 0.0;
    double K_p_dot = 0.0;
    double K_q_dot = 0.0;
    double K_r_dot = 0.0;
    double M_u_dot = 0.0;
    double M_v_dot = 0.0;
    double M_w_dot = 0.0;
    double M_p_dot = 0.0;
    double M_q_dot = 0.0;
    double M_r_dot = 0.0;
    double N_u_dot = 0.0;
    double N_v_dot = 0.0;
    double N_w_dot = 0.0;
    double N_p_dot = 0.0;
    double N_q_dot = 0.0;
    double N_r_dot = 0.0;

    n.getParam("m", m);
    n.getParam("d", d);
    n.getParam("l", l);
    // I = 1.0 / 12.0 * m * (pow(d, 2) + pow(l, 2));
    n.getParam("X_u_dot", X_u_dot);
    n.getParam("Y_v_dot", Y_v_dot);
    n.getParam("X_u", X_u);
    n.getParam("Y_r", Y_r);
    n.getParam("Y_v", Y_v);
    n.getParam("N_r", N_r);
    n.getParam("N_r_r", N_r_r);
    n.getParam("Z_w", Z_w);
    n.getParam("N_v", N_v);
    n.getParam("x_g", x_g);
    n.getParam("y_g", y_g);
    n.getParam("z_g", z_g);
    n.getParam("I_x", I_x);
    n.getParam("I_y", I_y);
    n.getParam("I_z", I_z);
    n.getParam("I_xy", I_xy);
    n.getParam("I_xz", I_xz);
    n.getParam("I_yz", I_yz);
    n.getParam("I_yx", I_yx);
    n.getParam("I_zx", I_zx);
    n.getParam("I_zy", I_zy);
    n.getParam("A_x", A_x);
    n.getParam("A_y", A_y);
    n.getParam("A_z", A_z);
    n.getParam("A_p", A_p);
    n.getParam("A_q", A_q);
    n.getParam("A_r", A_r);
    n.getParam("X_u_dot", X_u_dot);
    n.getParam("X_v_dot", X_v_dot);
    n.getParam("X_w_dot", X_w_dot);
    n.getParam("X_p_dot", X_p_dot);
    n.getParam("X_q_dot", X_q_dot);
    n.getParam("X_r_dot", X_r_dot);
    n.getParam("Y_u_dot", Y_u_dot);
    n.getParam("Y_v_dot", Y_v_dot);
    n.getParam("Y_w_dot", Y_w_dot);
    n.getParam("Y_p_dot", Y_p_dot);
    n.getParam("Y_q_dot", Y_q_dot);
    n.getParam("Y_r_dot", Y_r_dot);
    n.getParam("Z_u_dot", Z_u_dot);
    n.getParam("Z_v_dot", Z_v_dot);
    n.getParam("Z_w_dot", Z_w_dot);
    n.getParam("Z_p_dot", Z_p_dot);
    n.getParam("Z_q_dot", Z_q_dot);
    n.getParam("Z_r_dot", Z_r_dot);
    n.getParam("K_u_dot", K_u_dot);
    n.getParam("K_v_dot", K_v_dot);
    n.getParam("K_w_dot", K_w_dot);
    n.getParam("K_p_dot", K_p_dot);
    n.getParam("K_q_dot", K_q_dot);
    n.getParam("K_r_dot", K_r_dot);
    n.getParam("M_u_dot", M_u_dot);
    n.getParam("M_v_dot", M_v_dot);
    n.getParam("M_w_dot", M_w_dot);
    n.getParam("M_p_dot", M_p_dot);
    n.getParam("M_q_dot", M_q_dot);
    n.getParam("M_r_dot", M_r_dot);
    n.getParam("N_u_dot", N_u_dot);
    n.getParam("N_v_dot", N_v_dot);
    n.getParam("N_w_dot", N_w_dot);
    n.getParam("N_p_dot", N_p_dot);
    n.getParam("N_q_dot", N_q_dot);
    n.getParam("N_r_dot", N_r_dot);

    // MATRICE DI MASSA

    Eigen::Matrix<double, 6, 6> M_rb;
    M_rb << m, 0.0, 0.0, 0.0, m * z_g, -m * y_g,
        0.0, m, 0.0, -m * z_g, 0.0, m * x_g,
        0.0, 0.0, m, m * y_g, -m * x_g, 0.0,
        0.0, -m * z_g, m * y_g, I_x, -I_xy, -I_xz,
        m * z_g, 0.0, -m * x_g, -I_xy, I_y, -I_yz,
        -m * y_g, m * x_g, 0.0, -I_xz, -I_yz, I_z;

    Eigen::Matrix<double, 6, 6> M_a;
    M_a << X_u_dot, X_v_dot, X_w_dot, X_p_dot, X_q_dot, X_r_dot,
        Y_u_dot, Y_v_dot, Y_w_dot, Y_p_dot, Y_q_dot, Y_r_dot,
        Z_u_dot, Z_v_dot, Z_w_dot, Z_p_dot, Z_q_dot, Z_r_dot,
        K_u_dot, K_v_dot, K_w_dot, K_p_dot, K_q_dot, K_r_dot,
        M_u_dot, M_v_dot, M_w_dot, M_p_dot, M_q_dot, M_r_dot,
        N_u_dot, N_v_dot, N_w_dot, N_p_dot, N_q_dot, N_r_dot;

    Eigen::Matrix<double, 6, 6> M;
    M = M_rb + M_a;

    ros::Publisher chatter_pub = n.advertise<progetto_robotica::Floats>("state_topic", 1);
    ros::Subscriber sub = n.subscribe("tau_topic", 1000, tauCallback);
    double freq = 1000;
    double dt = 1 / freq;
    ros::Rate loop_rate(freq);

    while (ros::ok())
    {
        // DEFINIAMO LA DINAMICA DEL SISTEMA

        u = nu(0);
        v = nu(1);
        w = nu(2);
        p = nu(3);
        q = nu(4);
        r = nu(5);

        // MATRICE DI CORIOLIS
        Eigen::Matrix<double, 6, 6> C_rb;

        C_rb << 0.0, 0.0, 0.0, m * (y_g * q + z_g * r), -m * (x_g * q - w), -m * (x_g * r + v),
            0.0, 0.0, 0.0, -m * (y_g * p + w), m * (z_g * r + x_g * p), -m * (y_g * r - u),
            0.0, 0.0, 0.0, -m * (z_g * p - v), -m * (z_g * q + u), m * (x_g * p + y_g * q),
            -m * (y_g * q + z_g * r), m * (y_g * p + w), m * (z_g * p - v), 0.0, I_z * r - I_yz * q - I_xz * p, I_xy * p - I_yz * r - I_y * q,
            m *(x_g * q - w), -m *(z_g * r + x_g * p), m *(z_g * q + u), I_yz *q + I_xz *p - I_z, 0.0, -I_xz *r - I_xy *q + I_x *p,
            m *(x_g * r + v), -m *(y_g * r - u), -m *(x_g * p + y_g * q), -I_yz *r - I_xy *p + I_y *q, I_xy *q + I_xz *r - I_x *p, 0.0;

        Eigen::Matrix<double, 6, 6> C_a;

        double a1 = X_u_dot * u + X_v_dot * v + X_w_dot * w + X_p_dot * p + X_q_dot * q + X_r_dot * r;
        double a2 = X_v_dot * u + Y_v_dot * v + Y_w_dot * w + Y_p_dot * p + Y_q_dot * q + Y_r_dot * r;
        double a3 = X_w_dot * u + Y_w_dot * v + Z_w_dot * w + Z_p_dot * p + Z_q_dot * q + Z_r_dot * r;
        double b1 = X_p_dot * u + Y_p_dot * v + Z_p_dot * w + K_p_dot * p + K_q_dot * q + K_r_dot * r;
        double b2 = X_q_dot * u + Y_q_dot * v + Z_q_dot * w + K_p_dot * p + M_q_dot * q + M_r_dot * r;
        double b3 = X_r_dot * u + Y_r_dot * v + Z_r_dot * w + K_r_dot * p + M_r_dot * q + N_r_dot * r;

        C_a << 0.0, 0.0, 0.0, 0.0, -a3, a2,
            0.0, 0.0, 0.0, a3, 0.0, -a1,
            0.0, 0.0, 0.0, -a2, a1, 0.0,
            0.0, -a3, a2, 0.0, -b3, b2,
            a3, 0.0, -a1, b3, 0.0, -b1,
            -a2, a1, 0.0, -b2, b1, 0.0;
        
        Eigen::Matrix<double, 6, 6> C;
        C = C_rb + C_a;

        // MATRICE DI DAMPING
        Eigen::Matrix<double, 6, 6> D;

        D << A_x * abs(u) * u, A_x * abs(v) * v, A_x * abs(w) * w, A_x * abs(p) * p, A_x * abs(q) * q, A_x * abs(r) * r;
        A_y *abs(u) * u, A_y *abs(v) * v, A_y *abs(w) * w, A_y *abs(p) * p, A_y *abs(q) * q, A_y *abs(r) * r;
        A_z *abs(u) * u, A_z *abs(v) * v, A_z *abs(w) * w, A_z *abs(p) * p, A_z *abs(q) * q, A_z *abs(r) * r;
        A_p *abs(u) * u, A_p *abs(v) * v, A_p *abs(w) * w, A_p *abs(p) * p, A_p *abs(q) * q, A_p *abs(r) * r;
        A_q *abs(u) * u, A_q *abs(v) * v, A_q *abs(w) * w, A_q *abs(p) * p, A_q *abs(q) * q, A_q *abs(r) * r;
        A_r *abs(u) * u, A_r *abs(v) * v, A_r *abs(w) * w, A_r *abs(p) * p, A_r *abs(q) * q, A_r *abs(r) * r;

        // VETTORE DI FORZE E MOMENTI
        Eigen::Matrix<double, 6, 1> tau;
        tau << tau_u, tau_v, tau_w, tau_p, tau_q, tau_r;

        // VETTORE DELLE VELOCITA'
        Eigen::Matrix<double, 6, 1> nu_k1;
        nu_k1 = (dt * M.inverse() * (tau - C * nu - D * nu)) + nu;

        // VETTORE DELLE POSIZIONI
        Eigen::Matrix<double, 6, 6> Jacobian;
        Jacobian << cos(psi) * cos(theta), cos(psi) * sin(phi) * sin(theta) - cos(phi) * sin(psi), sin(phi) * sin(psi) + cos(phi) * cos(psi) * sin(theta), 0, 0, 0,
            cos(theta) * sin(psi), cos(phi) * cos(psi) + sin(phi) * sin(psi) * sin(theta), cos(phi) * sin(psi) * sin(theta) - cos(psi) * sin(phi), 0, 0, 0,
            -sin(theta), cos(theta) * sin(phi), cos(phi) * cos(theta), 0, 0, 0,
            0, 0, 0, 1, sin(phi) * tan(theta), cos(phi) * tan(theta),
            0, 0, 0, 0, cos(phi), -sin(phi),
            0, 0, 0, 0, sin(phi) / cos(theta), cos(phi) / cos(theta);

        Eigen::Matrix<double, 6, 1> eta_dot;
        eta_dot = Jacobian * nu;
        Eigen::Matrix<double, 6, 1> eta_k1;
        eta_k1 = dt * eta_dot + eta;
        
        double rem;

        // wrapToPi(eta_k1(phi));
        rem = std::fmod(eta_k1(3) + M_PI, 2 * M_PI);
        if (rem < 0)
        {
            rem += 2 * M_PI;
        }
        eta_k1(3) = rem - M_PI;

        // wrapToPi(eta_k1(theta));
        rem = std::fmod(eta_k1(4) + M_PI, 2 * M_PI);
        if (rem < 0)
        {
            rem += 2 * M_PI;
        }
        eta_k1(4) = rem - M_PI;

        // wrapToPi(eta_k1(psi));
        rem = std::fmod(eta_k1(5) + M_PI, 2 * M_PI);
        if (rem < 0)
        {
            rem += 2 * M_PI;
        }
        eta_k1(5) = rem - M_PI;

        // SETTARE LA POSIZIONE DEL MODELLO
        std::vector<double> state = {eta(0), eta(1), eta(2), eta(3), eta(4), eta(5), nu(0), nu(1), nu(2), nu(3), nu(4), nu(5)};

        progetto_robotica::Floats state_msg;
        state_msg.data = state;

        chatter_pub.publish(state_msg);

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