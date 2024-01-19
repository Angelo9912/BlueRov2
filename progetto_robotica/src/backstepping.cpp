#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include <ros/console.h>
#include <eigen3/Eigen/Dense>         // for eigen matrix
#include "progetto_robotica/Floats.h" // for accessing -- progetto_robotica Floats()
#include <vector>
#include "yaml-cpp/yaml.h" // for yaml

// Declare callback variables
double x_d = 0.0;
double y_d = 0.0;
double z_d = 0.0;
double psi_d = 0.0;
double x_dot_d = 0.0;
double y_dot_d = 0.0;
double z_dot_d = 0.0;
double psi_dot_d = 0.0;
double x_hat = 0.0;
double y_hat = 0.0;
double z_hat = 0.0;
double psi_hat = 0.0;
double u_hat = 0.0;
double v_hat = 0.0;
double w_hat = 0.0;
double r_hat = 0.0;

// Define callback functions
void desStateCallback(const progetto_robotica::Floats::ConstPtr &msg)
{
    if (msg->data[0] != msg->data[0])
    {
        x_d = 0.0;
        y_d = 0.0;
        z_d = 0.0;
        psi_d = 0.0;
        x_dot_d = 0.0;
        y_dot_d = 0.0;
        z_dot_d = 0.0;
        psi_dot_d = 0.0;
    }
    else
    {
        x_d = msg->data[0];
        y_d = msg->data[1];
        z_d = msg->data[2];
        psi_d = msg->data[3];
        x_dot_d = msg->data[4];
        y_dot_d = msg->data[5];
        z_dot_d = msg->data[6];
        psi_dot_d = msg->data[7];
    }
}
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
    ros::init(argc, argv, "backstepping");
    ros::NodeHandle n;
    ros::Publisher chatter_pub = n.advertise<progetto_robotica::Floats>("tau_topic", 1);
    ros::Subscriber sub_des_state = n.subscribe("desired_state_topic", 1000, desStateCallback);
    ros::Subscriber sub_est_state = n.subscribe("state_topic", 1000, estStateCallback);
    double freq = 100;
    double dt = 1 / freq;
    ros::Rate loop_rate(freq);
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

    while (ros::ok())
    {
        // Define the desired state and the estimated state vectors
        Eigen::Matrix<double, 4, 1> des_pose;
        des_pose << x_d, y_d, z_d, psi_d;
        Eigen::Matrix<double, 4, 1> des_pos_dot;
        des_pos_dot << x_dot_d, y_dot_d, z_dot_d, psi_dot_d;
        Eigen::Matrix<double, 4, 1> des_pos_2dot;
        des_pos_dot.setZero();
        Eigen::Matrix<double, 4, 1> est_pose;
        est_pose << x_hat, y_hat, z_hat, psi_hat;
        Eigen::Matrix<double, 4, 1> est_vel;
        est_vel << u_hat, v_hat, w_hat, r_hat;

        // Define error vector
        Eigen::Matrix<double, 4, 1> error;
        error = des_pose - est_pose;

        // Define Jacobian Matrix
        Eigen::Matrix<double, 4, 4> J;
        J << cos(est_pose(3)), -sin(est_pose(3)), 0.0, 0.0,
            sin(est_pose(3)), cos(est_pose(3)), 0.0, 0.0,
            0.0, 0.0, 1.0, 0.0,
            0.0, 0.0, 0.0, 1.0;
        Eigen::Matrix<double, 4, 1> nu(u_hat, v_hat, w_hat, r_hat);
        Eigen::Matrix<double, 4, 1> est_pose_dot;
        est_pose_dot = J * nu;
        Eigen::Matrix<double, 4, 4> LAMBDA;
        LAMBDA << 1.0, 0.0, 0.0, 0.0,
            0.0, 1.0, 0.0, 0.0,
            0.0, 0.0, 1.0, 0.0,
            0.0, 0.0, 0.0, 1.0;
        Eigen::Matrix<double, 4, 1> q_r_dot;
        q_r_dot = J.inverse() * (des_pos_dot + LAMBDA * error);
        Eigen::Matrix<double, 4, 4> J_inv_dot;
        J_inv_dot << -sin(est_pose(3)) * est_pose_dot(3), -cos(est_pose(3) * est_pose_dot(3)), 0.0, 0.0,
            -cos(est_pose(3)) * est_pose_dot(3), -sin(est_pose(3) * est_pose_dot(3)), 0.0, 0.0,
            0.0, 0.0, 1.0, 0.0,
            0.0, 0.0, 0.0, 1.0;
        Eigen::Matrix<double, 4, 1> error_dot;
        error_dot = des_pos_dot - est_pose_dot;
        Eigen::Matrix<double, 4, 1> q_r_2dot;
        q_r_2dot = J.inverse() * (des_pos_2dot + LAMBDA * error_dot) + J_inv_dot * (des_pos_dot + LAMBDA * error);
        Eigen::Matrix<double, 4, 1> s;
        s = J.inverse() * (error_dot + LAMBDA * error);

        // CORIOLIS MATRIX
        Eigen::Matrix<double, 4, 4> C;
        C << 0.0, 0.0, 0.0, Y_v_dot * nu(1) + Y_r * nu(3),
            0.0, 0.0, 0.0, -X_u_dot * nu(0),
            0.0, 0.0, 0.0, 0.0,
            -Y_v_dot * nu(1) - Y_r * nu(3), X_u_dot * nu(0), 0.0, 0.0;

        // DAMPING MATRIX
        Eigen::Matrix<double, 4, 4> D;
        D.diagonal() << -X_u, -Y_v, -Z_w, -N_r - N_r_r * r_hat;
        Eigen::Matrix<double, 4, 4> M;
        M << m, 0.0, 0.0, 0.0,
            0.0, m, 0.0, 0.0,
            0.0, 0.0, m, 0.0,
            0.0, 0.0, 0.0, I;

        Eigen::Matrix<double, 4, 4> K_d;
        K_d << 1.0, 0.0, 0.0, 0.0,
            0.0, 1.0, 0.0, 0.0,
            0.0, 0.0, 1.0, 0.0,
            0.0, 0.0, 0.0, 1.0;

        // Define the torques vector
        Eigen::Matrix<double, 4, 1> torques_vec;
        torques_vec = M * q_r_2dot + C * q_r_dot + K_d * s + J.transpose() * error;

        std::vector<double> torques = {torques_vec(0), torques_vec(1), torques_vec(2), torques_vec(3)};

        // Publishing the torques
        progetto_robotica::Floats torques_msg;
        torques_msg.data = torques;

        chatter_pub.publish(torques_msg);

        ros::spinOnce();

        loop_rate.sleep();
    }

    return 0;
}
