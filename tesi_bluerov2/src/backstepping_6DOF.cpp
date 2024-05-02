#include "ros/ros.h"
#include <rosbag/bag.h>
#include "std_msgs/String.h"
#include <sstream>
#include <ros/console.h>
#include <eigen3/Eigen/Dense>     // for eigen matrix
#include "tesi_bluerov2/Floats.h" // for accessing -- tesi_bluerov2 Floats()
#include <vector>
#include "yaml-cpp/yaml.h" // for yaml
#include <math.h>

// Declare callback variables
double x_d = 0.0;
double y_d = 0.0;
double z_d = 0.0;
double phi_d = 0.0;
double theta_d = 0.0;
double psi_d = 0.0;
double x_dot_d = 0.0;
double y_dot_d = 0.0;
double z_dot_d = 0.0;
double phi_dot_d = 0.0;
double theta_dot_d = 0.0;
double psi_dot_d = 0.0;

double x_hat = 0.0;
double y_hat = 0.0;
double z_hat = 0.0;
double phi_hat = 0.0;
double theta_hat = 0.0;
double psi_hat = 0.0;
double u_hat = 0.0;
double v_hat = 0.0;
double w_hat = 0.0;
double p_hat = 0.0;
double q_hat = 0.0;
double r_hat = 0.0;

// Velocità angolari stimate
double phi_hat_dot;
double theta_hat_dot;
double psi_hat_dot;
std::string GNC_status = "NOT_READY";
double wait_for_controller = 0.0;
double angleDifference(double e)
{
    if (e > 0)
    {
        if (e > 2 * M_PI - e)
        {
            e = -(2 * M_PI - e);
        }
        else
        {
            e = e;
        }
    }
    else
    {
        e = -e;
        if (e > 2 * M_PI - e)
        {
            e = 2 * M_PI - e;
        }
        else
        {
            e = -e;
        }
    }
    return e;
}

// Define callback functions
void desStateCallback(const tesi_bluerov2::Floats::ConstPtr &msg)
{
    if (msg->data[0] != msg->data[0])
    {
        x_d = x_hat;
        y_d = y_hat;
        z_d = z_hat;
        phi_d = phi_hat;
        theta_d = theta_hat;
        psi_d = psi_hat;
        x_dot_d = 0;
        y_dot_d = 0;
        z_dot_d = 0;
        phi_dot_d = 0;
        theta_dot_d = 0;
        psi_dot_d = 0;
    }
    else
    {
        x_d = msg->data[0];
        y_d = msg->data[1];
        z_d = msg->data[2];
        phi_d = msg->data[3];
        theta_d = msg->data[4];
        psi_d = msg->data[5];
        x_dot_d = msg->data[6];
        y_dot_d = msg->data[7];
        z_dot_d = msg->data[8];
        phi_dot_d = msg->data[9];
        theta_dot_d = msg->data[10];
        psi_dot_d = msg->data[11];
    }
}
void estStateCallback(const tesi_bluerov2::Floats::ConstPtr &msg)
{
    if (msg->data[0] != msg->data[0])
    {
        x_hat = 0.0;
        y_hat = 0.0;
        z_hat = 0.0;
        phi_hat = 0.0;
        theta_hat = 0.0;
        psi_hat = 0.0;
        u_hat = 0.0;
        v_hat = 0.0;
        w_hat = 0.0;
        p_hat = 0.0;
        q_hat = 0.0;
        r_hat = 0.0;
    }
    else
    {
        x_hat = msg->data[0];
        y_hat = msg->data[1];
        z_hat = msg->data[2];
        phi_hat = msg->data[3];
        theta_hat = msg->data[4];
        psi_hat = msg->data[5];
        u_hat = msg->data[6];
        v_hat = msg->data[7];
        w_hat = msg->data[8];
        p_hat = msg->data[9];
        q_hat = msg->data[10];
        r_hat = msg->data[11];
    }
}
void GNCstatusCallback(const std_msgs::String::ConstPtr &msg) // CALLBACK che riceve lo stato del GNC
{
    GNC_status = msg->data;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "backstepping_6DOF");
    ros::NodeHandle n;
    rosbag::Bag tau_bag;
    std::string path = ros::package::getPath("tesi_bluerov2");
    tau_bag.open(path + "/bag/tau.bag", rosbag::bagmode::Write);

    ros::Publisher chatter_pub = n.advertise<tesi_bluerov2::Floats>("tau_topic", 1);
    ros::Publisher publisher_gnc_status = n.advertise<std_msgs::String>("manager/GNC_status_requested_topic", 10); // publisher stato richiesto al GNC

    ros::Subscriber sub_gnc_status = n.subscribe("manager/GNC_status_topic", 1, GNCstatusCallback); // sottoscrizione alla topic di stato del GNC
    ros::Subscriber sub_des_state = n.subscribe("state/desired_state_topic", 1, desStateCallback);
    ros::Subscriber sub_est_state = n.subscribe("state/est_state_topic_no_dyn", 1, estStateCallback);
    //ros::Subscriber sub_est_state = n.subscribe("state/state_topic", 1, estStateCallback);

    double freq = 200;
    double dt = 1 / freq;
    ros::Rate loop_rate(freq);

    // Import parameters from YAML file
    double m = 0.0;
    double d = 0.0;
    double l = 0.0;
    double x_g = 0.0;
    double y_g = 0.0;
    double z_g = 0.0;
    double x_b = 0.0;
    double y_b = 0.0;
    double z_b = 0.0;
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
    double var_tau_u = 0.0;
    double var_tau_v = 0.0;
    double var_tau_w = 0.0;
    double var_tau_p = 0.0;
    double var_tau_q = 0.0;
    double var_tau_r = 0.0;

    n.getParam("m", m);
    n.getParam("x_g", x_g);
    n.getParam("y_g", y_g);
    n.getParam("z_g", z_g);
    n.getParam("x_b", x_b);
    n.getParam("y_b", y_b);
    n.getParam("z_b", z_b);
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
    n.getParam("var_tau_u", var_tau_u);
    n.getParam("var_tau_v", var_tau_v);
    n.getParam("var_tau_w", var_tau_w);
    n.getParam("var_tau_p", var_tau_p);
    n.getParam("var_tau_q", var_tau_q);
    n.getParam("var_tau_r", var_tau_r);
    n.getParam("wait_for_controller", wait_for_controller);

    bool is_init = true;
    double init_time;

    // Define the desired state and the estimated state vectors
    Eigen::Matrix<double, 6, 1> des_pose;
    Eigen::Matrix<double, 6, 1> des_pos_dot;
    Eigen::Matrix<double, 6, 1> des_pos_2dot;
    des_pos_2dot.setZero();
    Eigen::Matrix<double, 6, 1> est_pose;

    // Define error vector
    Eigen::Matrix<double, 6, 1> error;

    Eigen::Matrix<double, 6, 1> nu;

    Eigen::Matrix<double, 6, 1> est_pose_dot;

    Eigen::Matrix<double, 6, 1> error_dot;

    Eigen::Matrix<double, 6, 1> q_r_2dot;

    Eigen::Matrix<double, 6, 1> s;

    Eigen::Matrix<double, 6, 6> J;

    Eigen::Matrix<double, 6, 6> LAMBDA;

    Eigen::Matrix<double, 6, 1> q_r_dot;

    Eigen::Matrix<double, 6, 6> J_inv_dot;

    while (ros::ok())
    {
        if (GNC_status == "NAVIGATION_READY")
        {
            std_msgs::String msg;
            msg.data = "CONTROLLER_READY";
            ros::Duration(wait_for_controller).sleep();
            publisher_gnc_status.publish(msg);
        }
        else if (GNC_status == "CONTROLLER_READY" || GNC_status == "GNC_READY")
        {

            des_pose << x_d, y_d, z_d, phi_d, theta_d, psi_d;
            des_pos_dot << x_dot_d, y_dot_d, z_dot_d, phi_dot_d, theta_dot_d, psi_dot_d;
            est_pose << x_hat, y_hat, z_hat, phi_hat, theta_hat, psi_hat;

            error = des_pose - est_pose;

            error(3) = angleDifference(error(3));
            error(4) = angleDifference(error(4));
            error(5) = angleDifference(error(5));

            // Define Jacobian Matrix
            J << cos(psi_hat) * cos(theta_hat), cos(psi_hat) * sin(phi_hat) * sin(theta_hat) - cos(phi_hat) * sin(psi_hat), sin(phi_hat) * sin(psi_hat) + cos(phi_hat) * cos(psi_hat) * sin(theta_hat), 0, 0, 0,
                cos(theta_hat) * sin(psi_hat), cos(phi_hat) * cos(psi_hat) + sin(phi_hat) * sin(psi_hat) * sin(theta_hat), cos(phi_hat) * sin(psi_hat) * sin(theta_hat) - cos(psi_hat) * sin(phi_hat), 0, 0, 0,
                -sin(theta_hat), cos(theta_hat) * sin(phi_hat), cos(phi_hat) * cos(theta_hat), 0, 0, 0,
                0, 0, 0, 1, sin(phi_hat) * tan(theta_hat), cos(phi_hat) * tan(theta_hat),
                0, 0, 0, 0, cos(phi_hat), -sin(phi_hat),
                0, 0, 0, 0, sin(phi_hat) / cos(theta_hat), cos(phi_hat) / cos(theta_hat);

            nu << u_hat, v_hat, w_hat, p_hat, q_hat, r_hat;

            est_pose_dot = J * nu;

            phi_hat_dot = est_pose_dot(3);
            theta_hat_dot = est_pose_dot(4);
            psi_hat_dot = est_pose_dot(5);

            LAMBDA << Eigen::Matrix<double, 6, 6>::Identity();
            LAMBDA(0, 0) = 10.0;
            LAMBDA(1, 1) = 10.0;
            LAMBDA(2, 2) = 10.0;
            LAMBDA(5, 5) = 5.0;

            q_r_dot = J.inverse() * (des_pos_dot + LAMBDA * error);

            J_inv_dot << -psi_hat_dot * cos(theta_hat) * sin(psi_hat) - theta_hat_dot * cos(psi_hat) * sin(theta_hat), psi_hat_dot * cos(psi_hat) * cos(theta_hat) - theta_hat_dot * sin(psi_hat) * sin(theta_hat), -theta_hat_dot * cos(theta_hat), 0, 0, 0,
                phi_hat_dot * (sin(phi_hat) * sin(psi_hat) + cos(phi_hat) * cos(psi_hat) * sin(theta_hat)) - psi_hat_dot * (cos(phi_hat) * cos(psi_hat) + sin(phi_hat) * sin(psi_hat) * sin(theta_hat)) + theta_hat_dot * cos(psi_hat) * cos(theta_hat) * sin(phi_hat), theta_hat_dot * cos(theta_hat) * sin(phi_hat) * sin(psi_hat) - psi_hat_dot * (cos(phi_hat) * sin(psi_hat) - cos(psi_hat) * sin(phi_hat) * sin(theta_hat)) - phi_hat_dot * (cos(psi_hat) * sin(phi_hat) - cos(phi_hat) * sin(psi_hat) * sin(theta_hat)), phi_hat_dot * cos(phi_hat) * cos(theta_hat) - theta_hat_dot * sin(phi_hat) * sin(theta_hat), 0, 0, 0,
                phi_hat_dot * (cos(phi_hat) * sin(psi_hat) - cos(psi_hat) * sin(phi_hat) * sin(theta_hat)) + psi_hat_dot * (cos(psi_hat) * sin(phi_hat) - cos(phi_hat) * sin(psi_hat) * sin(theta_hat)) + theta_hat_dot * cos(phi_hat) * cos(psi_hat) * cos(theta_hat), psi_hat_dot * (sin(phi_hat) * sin(psi_hat) + cos(phi_hat) * cos(psi_hat) * sin(theta_hat)) - phi_hat_dot * (cos(phi_hat) * cos(psi_hat) + sin(phi_hat) * sin(psi_hat) * sin(theta_hat)) + theta_hat_dot * cos(phi_hat) * cos(theta_hat) * sin(psi_hat), -phi_hat_dot * cos(theta_hat) * sin(phi_hat) - theta_hat_dot * cos(phi_hat) * sin(theta_hat), 0, 0, 0,
                0, 0, 0, 0, 0, -theta_hat_dot * cos(theta_hat),
                0, 0, 0, 0, -phi_hat_dot * sin(phi_hat), phi_hat_dot * cos(phi_hat) * cos(theta_hat) - theta_hat_dot * sin(phi_hat) * sin(theta_hat),
                0, 0, 0, 0, -phi_hat_dot * cos(phi_hat), -phi_hat_dot * cos(theta_hat) * sin(phi_hat) - theta_hat_dot * cos(phi_hat) * sin(theta_hat);

            error_dot = des_pos_dot - est_pose_dot;

            q_r_2dot = J.inverse() * (des_pos_2dot + LAMBDA * error_dot) + J_inv_dot * (des_pos_dot + LAMBDA * error);

            s = J.inverse() * (error_dot + LAMBDA * error);

            // MATRICE DI CORIOLIS
            Eigen::Matrix<double, 6, 6> C_rb;

            C_rb << 0.0, 0.0, 0.0, m * (y_g * q_hat + z_g * r_hat), -m * (x_g * q_hat - w_hat), -m * (x_g * r_hat + v_hat),
                0.0, 0.0, 0.0, -m * (y_g * p_hat + w_hat), m * (z_g * r_hat + x_g * p_hat), -m * (y_g * r_hat - u_hat),
                0.0, 0.0, 0.0, -m * (z_g * p_hat - v_hat), -m * (z_g * q_hat + u_hat), m * (x_g * p_hat + y_g * q_hat),
                -m * (y_g * q_hat + z_g * r_hat), m * (y_g * p_hat + w_hat), m * (z_g * p_hat - v_hat), 0.0, I_z * r_hat - I_yz * q_hat - I_xz * p_hat, I_xy * p_hat + I_yz * r_hat - I_y * q_hat,
                m * (x_g * q_hat - w_hat), -m * (z_g * r_hat + x_g * p_hat), m * (z_g * q_hat + u_hat), I_yz * q_hat + I_xz * p_hat - I_z * r_hat, 0.0, -I_xz * r_hat - I_xy * q_hat + I_x * p_hat,
                m * (x_g * r_hat + v_hat), m * (y_g * r_hat - u_hat), -m * (x_g * p_hat + y_g * q_hat), -I_yz * r_hat - I_xy * p_hat + I_y * q_hat, I_xy * q_hat + I_xz * r_hat - I_x * p_hat, 0.0;

            Eigen::Matrix<double, 6, 6> C_a;

            double a1 = X_u_dot * u_hat + X_v_dot * v_hat + X_w_dot * w_hat + X_p_dot * p_hat + X_q_dot * q_hat + X_r_dot * r_hat;
            double a2 = X_v_dot * u_hat + Y_v_dot * v_hat + Y_w_dot * w_hat + Y_p_dot * p_hat + Y_q_dot * q_hat + Y_r_dot * r_hat;
            double a3 = X_w_dot * u_hat + Y_w_dot * v_hat + Z_w_dot * w_hat + Z_p_dot * p_hat + Z_q_dot * q_hat + Z_r_dot * r_hat;
            double b1 = X_p_dot * u_hat + Y_p_dot * v_hat + Z_p_dot * w_hat + K_p_dot * p_hat + K_q_dot * q_hat + K_r_dot * r_hat;
            double b2 = X_q_dot * u_hat + Y_q_dot * v_hat + Z_q_dot * w_hat + K_p_dot * p_hat + M_q_dot * q_hat + M_r_dot * r_hat;
            double b3 = X_r_dot * u_hat + Y_r_dot * v_hat + Z_r_dot * w_hat + K_r_dot * p_hat + M_r_dot * q_hat + N_r_dot * r_hat;

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

            D << A_x * abs(u_hat), 0, 0, 0, 0, 0,
                0, A_y * abs(v_hat), 0, 0, 0, 0,
                0, 0, A_z * abs(w_hat), 0, 0, 0,
                0, 0, 0, A_p * abs(p_hat), 0, 0,
                0, 0, 0, 0, A_q * abs(q_hat), 0,
                0, 0, 0, 0, 0, A_r * abs(r_hat);

            D = 0.5 * 1000 * D;

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

            // VETTORE DEI TERMINI GRAVITAZIONALI
            Eigen::Matrix<double, 6, 1> G;
            double W = m * 9.81;

            G << 0,
                0,
                0,
                -(y_g - y_b) * W * cos(phi_hat) * cos(theta_hat) + (z_g - z_b) * W * sin(phi_hat) * cos(theta_hat),
                (z_g - z_b) * W * sin(theta_hat) + (x_g - x_b) * W * cos(theta_hat) * cos(phi_hat),
                -(x_g - x_b) * W * cos(theta_hat) * sin(phi_hat) - (y_g - y_b) * W * sin(theta_hat);

            if (z_hat < 0.0)
            {
                G(2) = -W;
            }

            Eigen::Matrix<double, 6, 6> K_d;
            K_d << Eigen::Matrix<double, 6, 6>::Identity();

            K_d(0, 0) = 5.0;
            K_d(1, 1) = 5.0;
            K_d(2, 2) = 5.0;
            K_d(5, 5) = 5.0;

            Eigen::Matrix<double, 6, 4> B;

            Eigen::Matrix<double, 4, 6> B_pinv;

            B_pinv << 1, 0, 0, 0, 0, 0,
                0, 1, 0, 0, 0, 0,
                0, 0, 1, 0, 0, 0,
                0, 0, 0, 0, 0, 1;

            // Define the torques vector
            Eigen::Matrix<double, 4, 1> torques_vec;
            torques_vec = B_pinv * (M * q_r_2dot + K_d * s + C * q_r_dot + G + J.transpose() * error + D * nu);

            if (torques_vec(3) > 37.471)
            {
                torques_vec(3) = 37.471;
            }
            else if (torques_vec(3) < -37.471)
            {
                torques_vec(3) = -37.471;
            }

            if (torques_vec(0) > 141.42)
            {
                torques_vec(0) = 141.42;
            }
            else if (torques_vec(0) < -141.42)
            {
                torques_vec(0) = -141.42;
            }

            if (torques_vec(1) > 141.42)
            {
                torques_vec(1) = 141.42;
            }
            else if (torques_vec(1) < -141.42)
            {
                torques_vec(1) = -141.42;
            }

            if (torques_vec(2) > 70.71)
            {
                torques_vec(2) = 70.71;
            }
            else if (torques_vec(2) < -70.71)
            {
                torques_vec(2) = -70.71;
            }

            std::vector<double> torques = {torques_vec(0), torques_vec(1), torques_vec(2), 0.0, 0.0, torques_vec(3)};

            // Publishing the torques
            tesi_bluerov2::Floats torques_msg;
            torques_msg.data = torques;

            chatter_pub.publish(torques_msg);

            if (ros::Time::now().toSec() > ros::TIME_MIN.toSec())
            {
                tau_bag.write("tau_topic", ros::Time::now(), torques_msg);
            }
        }

        ros::spinOnce();

        loop_rate.sleep();
    }
    tau_bag.close();
    return 0;
}
