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
    ros::init(argc, argv, "adaptive_backstepping_6DOF");
    ros::NodeHandle n;
    rosbag::Bag tau_bag;
    std::string path = ros::package::getPath("tesi_bluerov2");
    tau_bag.open(path + "/bag/tau.bag", rosbag::bagmode::Write);

    ros::Publisher chatter_pub = n.advertise<tesi_bluerov2::Floats>("tau_topic", 1);
    ros::Publisher publisher_gnc_status = n.advertise<std_msgs::String>("manager/GNC_status_requested_topic", 10); // publisher stato richiesto al GNC

    ros::Subscriber sub_des_state = n.subscribe("state/desired_state_topic", 1, desStateCallback);
    ros::Subscriber sub_est_state = n.subscribe("state/est_state_topic_no_dyn_imu", 1, estStateCallback);
    ros::Subscriber sub_gnc_status = n.subscribe("manager/GNC_status_topic", 1, GNCstatusCallback); // sottoscrizione alla topic di stato del GNC

    double freq = 60;
    double dt = 1 / freq;
    ros::Rate loop_rate(freq);

    ROS_WARN("ADAPTIVE BACKSTEPPING INITIALIZED");
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

    // Define the desired state and the estimated state vectors
    Eigen::Matrix<double, 6, 1> des_pose;
    Eigen::Matrix<double, 6, 1> des_pos_dot;
    Eigen::Matrix<double, 6, 1> des_pos_2dot;
    des_pos_2dot.setZero();
    Eigen::Matrix<double, 6, 1> est_pose;

    Eigen::Vector<double, 25> pi_d;

    pi_d << m, I_x, I_y, I_z, I_xy, I_xz, I_yz, x_g, y_g, z_g, x_b, y_b, z_b, X_u_dot, Y_v_dot, Z_w_dot, K_p_dot, M_q_dot, N_r_dot, A_x, A_y, A_z, A_p, A_q, A_r;
    pi_d = pi_d + pi_d * (-0.10);

    double u_r;
    double v_r;
    double w_r;
    double p_r;
    double q_r;
    double r_r;

    double u_r_dot;
    double v_r_dot;
    double w_r_dot;
    double p_r_dot;
    double q_r_dot;
    double r_r_dot;

    double m_hat = pi_d(0);
    double I_x_hat = pi_d(1);
    double I_y_hat = pi_d(2);
    double I_z_hat = pi_d(3);
    double I_xy_hat = pi_d(4);
    double I_xz_hat = pi_d(5);
    double I_yz_hat = pi_d(6);
    double x_g_hat = pi_d(7);
    double y_g_hat = pi_d(8);
    double z_g_hat = pi_d(9);
    double x_b_hat = pi_d(10);
    double y_b_hat = pi_d(11);
    double z_b_hat = pi_d(12);
    double X_u_dot_hat = pi_d(13);
    double Y_v_dot_hat = pi_d(14);
    double Z_w_dot_hat = pi_d(15);
    double K_p_dot_hat = pi_d(16);
    double M_q_dot_hat = pi_d(17);
    double N_r_dot_hat = pi_d(18);

    // Define error vector
    Eigen::Matrix<double, 6, 1> error;

    Eigen::Matrix<double, 6, 1> nu;

    Eigen::Matrix<double, 6, 1> est_pose_dot;

    Eigen::Matrix<double, 6, 1> error_dot;

    Eigen::Matrix<double, 6, 1> nu_r_dot;

    Eigen::Matrix<double, 6, 1> s;

    Eigen::Matrix<double, 6, 6> J;

    Eigen::Matrix<double, 6, 6> LAMBDA;
    LAMBDA << Eigen::Matrix<double, 6, 6>::Identity();
    LAMBDA(0, 0) = 3.0;
    LAMBDA(1, 1) = 3.0;
    LAMBDA(2, 2) = 3.0;
    LAMBDA(5, 5) = 2.0;

    Eigen::Matrix<double, 6, 6> K_d;
    K_d << Eigen::Matrix<double, 6, 6>::Identity();

    K_d(0, 0) = 1.0;
    K_d(1, 1) = 1.0;
    K_d(2, 2) = 2.0;
    K_d(5, 5) = 1.0;

    Eigen::Matrix<double, 25, 25> R;

    R << 10.0 * Eigen::Matrix<double, 25, 25>::Identity();

    Eigen::Matrix<double, 4, 6> B_pinv;

    B_pinv << 1, 0, 0, 0, 0, 0,
        0, 1, 0, 0, 0, 0,
        0, 0, 1, 0, 0, 0,
        0, 0, 0, 0, 0, 1;

    Eigen::Matrix<double, 4, 1> torques_vec;

    Eigen::Matrix<double, 6, 1> nu_r;

    Eigen::Matrix<double, 6, 6> J_inv_dot;

    double init_time;
    while (ros::ok())
    {

        if (GNC_status == "NAVIGATION_READY")
        {
            std_msgs::String msg;
            msg.data = "CONTROLLER_READY";
            if (is_init)
            {
                init_time = ros::Time::now().toSec();
                is_init = false;
            }
            if (ros::Time::now().toSec() - init_time >= wait_for_controller)
            {
                publisher_gnc_status.publish(msg);
            }
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

            // ROS_WARN_STREAM("error" << error);

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

            nu_r = J.inverse() * (des_pos_dot + LAMBDA * error);

            J_inv_dot << -psi_hat_dot * cos(theta_hat) * sin(psi_hat) - theta_hat_dot * cos(psi_hat) * sin(theta_hat), psi_hat_dot * cos(psi_hat) * cos(theta_hat) - theta_hat_dot * sin(psi_hat) * sin(theta_hat), -theta_hat_dot * cos(theta_hat), 0, 0, 0,
                phi_hat_dot * (sin(phi_hat) * sin(psi_hat) + cos(phi_hat) * cos(psi_hat) * sin(theta_hat)) - psi_hat_dot * (cos(phi_hat) * cos(psi_hat) + sin(phi_hat) * sin(psi_hat) * sin(theta_hat)) + theta_hat_dot * cos(psi_hat) * cos(theta_hat) * sin(phi_hat), theta_hat_dot * cos(theta_hat) * sin(phi_hat) * sin(psi_hat) - psi_hat_dot * (cos(phi_hat) * sin(psi_hat) - cos(psi_hat) * sin(phi_hat) * sin(theta_hat)) - phi_hat_dot * (cos(psi_hat) * sin(phi_hat) - cos(phi_hat) * sin(psi_hat) * sin(theta_hat)), phi_hat_dot * cos(phi_hat) * cos(theta_hat) - theta_hat_dot * sin(phi_hat) * sin(theta_hat), 0, 0, 0,
                phi_hat_dot * (cos(phi_hat) * sin(psi_hat) - cos(psi_hat) * sin(phi_hat) * sin(theta_hat)) + psi_hat_dot * (cos(psi_hat) * sin(phi_hat) - cos(phi_hat) * sin(psi_hat) * sin(theta_hat)) + theta_hat_dot * cos(phi_hat) * cos(psi_hat) * cos(theta_hat), psi_hat_dot * (sin(phi_hat) * sin(psi_hat) + cos(phi_hat) * cos(psi_hat) * sin(theta_hat)) - phi_hat_dot * (cos(phi_hat) * cos(psi_hat) + sin(phi_hat) * sin(psi_hat) * sin(theta_hat)) + theta_hat_dot * cos(phi_hat) * cos(theta_hat) * sin(psi_hat), -phi_hat_dot * cos(theta_hat) * sin(phi_hat) - theta_hat_dot * cos(phi_hat) * sin(theta_hat), 0, 0, 0,
                0, 0, 0, 0, 0, -theta_hat_dot * cos(theta_hat),
                0, 0, 0, 0, -phi_hat_dot * sin(phi_hat), phi_hat_dot * cos(phi_hat) * cos(theta_hat) - theta_hat_dot * sin(phi_hat) * sin(theta_hat),
                0, 0, 0, 0, -phi_hat_dot * cos(phi_hat), -phi_hat_dot * cos(theta_hat) * sin(phi_hat) - theta_hat_dot * cos(phi_hat) * sin(theta_hat);

            error_dot = des_pos_dot - est_pose_dot;

            nu_r_dot = J.inverse() * (des_pos_2dot + LAMBDA * error_dot) + J_inv_dot * (des_pos_dot + LAMBDA * error);

            s = J.inverse() * (error_dot + LAMBDA * error);

            u_r = nu_r(0);
            v_r = nu_r(1);
            w_r = nu_r(2);
            p_r = nu_r(3);
            q_r = nu_r(4);
            r_r = nu_r(5);

            u_r_dot = nu_r_dot(0);
            v_r_dot = nu_r_dot(1);
            w_r_dot = nu_r_dot(2);
            p_r_dot = nu_r_dot(3);
            q_r_dot = nu_r_dot(4);
            r_r_dot = nu_r_dot(5);

            m_hat = pi_d(0);
            I_x_hat = pi_d(1);
            I_y_hat = pi_d(2);
            I_z_hat = pi_d(3);
            I_xy_hat = pi_d(4);
            I_xz_hat = pi_d(5);
            I_yz_hat = pi_d(6);
            x_g_hat = pi_d(7);
            y_g_hat = pi_d(8);
            z_g_hat = pi_d(9);
            x_b_hat = pi_d(10);
            y_b_hat = pi_d(11);
            z_b_hat = pi_d(12);
            X_u_dot_hat = pi_d(13);
            Y_v_dot_hat = pi_d(14);
            Z_w_dot_hat = pi_d(15);
            K_p_dot_hat = pi_d(16);
            M_q_dot_hat = pi_d(17);
            N_r_dot_hat = pi_d(18);

            // Calcolo del Regressore sui parametri incogniti
            Eigen::Matrix<double, 6, 25> Y;

            Y << u_r_dot + q_r_dot * z_g - r_r_dot * y_g + p_r * (q_hat * y_g + r_hat * z_g) + q_r * (w_hat - q_hat * x_g) - r_r * (v_hat + r_hat * x_g), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, u_r_dot, r_r * v_hat, -q_r * w_hat, 0.0, 0.0, 0.0, -m * (q_hat * q_r + r_hat * r_r), -m * (r_r_dot - p_r * q_hat), m * (q_r_dot + p_r * r_hat), 0.0, 0.0, 0.0, 500 * u_hat * abs(u_hat), 0.0, 0.0, 0.0, 0.0, 0.0,
                v_r_dot - p_r_dot * z_g + r_r_dot * x_g + q_r * (p_hat * x_g + r_hat * z_g) - p_r * (w_hat + p_hat * y_g) + r_r * (u_hat - r_hat * y_g), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -r_r * u_hat, v_r_dot, p_r * w_hat, 0.0, 0.0, 0.0, m * (r_r_dot + p_hat * q_r), -m * (p_hat * p_r + r_hat * r_r), -m * (p_r_dot - q_r * r_hat), 0.0, 0.0, 0.0, 0.0, 500 * v_hat * abs(v_hat), 0.0, 0.0, 0.0, 0.0,
                w_r_dot + p_r_dot * y_g - q_r_dot * x_g + r_r * (p_hat * x_g + q_hat * y_g) + p_r * (v_hat - p_hat * z_g) - q_r * (u_hat + q_hat * z_g), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, q_r * u_hat, -p_r * v_hat, w_r_dot, 0.0, 0.0, 0.0, -m * (q_r_dot - p_hat * r_r), m * (p_r_dot + q_hat * r_r), -m * (p_hat * p_r + q_hat * q_r), 0.0, 0.0, 0.0, 0.0, 0.0, 500 * w_hat * abs(w_hat), 0.0, 0.0, 0.0,
                w_r_dot * y_g - v_r_dot * z_g - u_r * (q_hat * y_g + r_hat * z_g) + v_r * (w_hat + p_hat * y_g) - w_r * (v_hat - p_hat * z_g) + (981 * cos(phi_hat) * cos(theta_hat) * (y_b - y_g)) / 100 - (981 * cos(theta_hat) * sin(phi_hat) * (z_b - z_g)) / 100, p_r_dot, -q_hat * r_r, q_r * r_hat, p_hat * r_r - q_r_dot, -r_r_dot - p_hat * q_r, r_hat * r_r - q_hat * q_r, 0.0, v_hat * w_r, -v_r * w_hat, p_r_dot + p_hat * r_r, q_hat * r_r, -q_r * r_hat, 0.0, (m * (100 * w_r_dot - 981 * cos(phi_hat) * cos(theta_hat) + 100 * p_hat * v_r - 100 * q_hat * u_r)) / 100, -(m * (100 * v_r_dot - 981 * cos(theta_hat) * sin(phi_hat) - 100 * p_hat * w_r + 100 * r_hat * u_r)) / 100, 0.0, (981 * m * cos(phi_hat) * cos(theta_hat)) / 100, -(981 * m * cos(theta_hat) * sin(phi_hat)) / 100, 0.0, 0.0, 0.0, 500 * p_hat * abs(p_hat), 0.0, 0.0,
                u_r_dot * z_g - w_r_dot * x_g - v_r * (p_hat * x_g + r_hat * z_g) - u_r * (w_hat - q_hat * x_g) + w_r * (u_hat + q_hat * z_g) - (981 * sin(theta_hat) * (z_b - z_g)) / 100 - (981 * cos(phi_hat) * cos(theta_hat) * (x_b - x_g)) / 100, p_hat * r_r, q_r_dot, -p_r * r_hat, -p_r_dot - q_hat * r_r, p_hat * p_r - r_hat * r_r, p_r * q_hat - r_r_dot, -u_hat * w_r, 0.0, u_r * w_hat, -p_hat * r_r, q_r_dot, p_r * r_hat, -(m * (100 * w_r_dot - 981 * cos(phi_hat) * cos(theta_hat) + 100 * p_hat * v_r - 100 * q_hat * u_r)) / 100, 0.0, (m * (100 * u_r_dot + 981 * sin(theta_hat) + 100 * q_hat * w_r - 100 * r_hat * v_r)) / 100, -(981 * m * cos(phi_hat) * cos(theta_hat)) / 100, 0.0, -(981 * m * sin(theta_hat)) / 100, 0.0, 0.0, 0.0, 0.0, 500 * q_hat * abs(q_hat), 0.0,
                v_r_dot * x_g - u_r_dot * y_g - w_r * (p_hat * x_g + q_hat * y_g) + u_r * (v_hat + r_hat * x_g) - v_r * (u_hat - r_hat * y_g) + (981 * sin(theta_hat) * (y_b - y_g)) / 100 + (981 * cos(theta_hat) * sin(phi_hat) * (x_b - x_g)) / 100, -p_hat * q_r, p_r * q_hat, r_r_dot, q_hat * q_r - p_hat * p_r, q_r * r_hat - p_r_dot, -q_r_dot - p_r * r_hat, u_hat * v_r, -u_r * v_hat, 0.0, -p_hat * (p_r - q_r), -p_r * q_hat, r_r_dot, (m * (100 * v_r_dot - 981 * cos(theta_hat) * sin(phi_hat) - 100 * p_hat * w_r + 100 * r_hat * u_r)) / 100, -(m * (100 * u_r_dot + 981 * sin(theta_hat) + 100 * q_hat * w_r - 100 * r_hat * v_r)) / 100, 0.0, (981 * m * cos(theta_hat) * sin(phi_hat)) / 100, (981 * m * sin(theta_hat)) / 100, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 500 * r_hat * abs(r_hat);

            torques_vec = B_pinv * (Y * pi_d + J.transpose() * error + K_d * s);

            pi_d = dt * R.inverse() * Y.transpose() * s + pi_d;
            
            if (torques_vec(3) > 5.471)
            {
                torques_vec(3) = 5.471;
            }
            else if (torques_vec(3) < -5.471)
            {
                torques_vec(3) = -5.471;
            }

            if (torques_vec(0) > 141.42)
            {
                torques_vec(0) = 141.42;
            }
            else if (torques_vec(0) < -141.42)
            {
                torques_vec(0) = -141.42;
            }

            if (torques_vec(1) > 14.42)
            {
                torques_vec(1) = 14.42;
            }
            else if (torques_vec(1) < -14.42)
            {
                torques_vec(1) = -14.42;
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
