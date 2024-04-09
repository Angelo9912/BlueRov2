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
void desStateCallback(const tesi_bluerov2::Floats::ConstPtr &msg)
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
void estStateCallback(const tesi_bluerov2::Floats::ConstPtr &msg)
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
    rosbag::Bag tau_bag;
    tau_bag.open("/home/antonio/catkin_ws/src/tesi_bluerov2/bag/backstepping/tau.bag", rosbag::bagmode::Write);

    ros::Publisher chatter_pub = n.advertise<tesi_bluerov2::Floats>("tau_topic", 1);
    ros::Subscriber sub_des_state = n.subscribe("desired_state_topic", 1, desStateCallback);
    ros::Subscriber sub_est_state = n.subscribe("state_topic", 1, estStateCallback);
    double freq = 100;
    double dt = 1 / freq;
    ros::Rate loop_rate(freq);
    // Import parameters from YAML file
    double m = 0.0;
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
    bool is_init = true;
    double init_time;

    while (ros::ok())
    {
        // Calcolo il tempo iniziale in maniera periodica fino a che non diventa maggiore di 0 in modo tale da non avere falsi positivi nella
        // condizione sulla differenza temporale di 5 secondi
        if (is_init)
        {
            init_time = ros::Time::now().toSec();
            if (init_time > 0.0)
            {
                is_init = false;
            }
        }

        // Define the desired state and the estimated state vectors
        Eigen::Matrix<double, 4, 1> des_pose;
        des_pose << x_d, y_d, z_d, psi_d;
        Eigen::Matrix<double, 4, 1> des_pos_dot;
        des_pos_dot << x_dot_d, y_dot_d, z_dot_d, psi_dot_d;
        Eigen::Matrix<double, 4, 1> des_pos_2dot;
        des_pos_2dot.setZero();
        Eigen::Matrix<double, 4, 1> est_pose;
        est_pose << x_hat, y_hat, z_hat, psi_hat;

        // Define error vector
        Eigen::Matrix<double, 4, 1> error;
        error = des_pose - est_pose;
        double angle_tmp = error(3);
        if (angle_tmp > 0)
        {
            if (angle_tmp > 2 * M_PI - angle_tmp)
            {
                error(3) = angle_tmp - 2 * M_PI;
            }
            else
            {
                error(3) = angle_tmp;
            }
        }
        else
        {
            angle_tmp = -angle_tmp;
            if (angle_tmp > 2 * M_PI - angle_tmp)
            {
                error(3) = 2 * M_PI - angle_tmp;
            }
            else
            {
                error(3) = -angle_tmp;
            }
        }

        // Define Jacobian Matrix
        Eigen::Matrix<double, 4, 4> J;
        J << cos(est_pose(3)), -sin(est_pose(3)), 0.0, 0.0,
            sin(est_pose(3)), cos(est_pose(3)), 0.0, 0.0,
            0.0, 0.0, 1.0, 0.0,
            0.0, 0.0, 0.0, 1.0;

        Eigen::Matrix<double, 4, 1> nu;
        nu << u_hat, v_hat, w_hat, r_hat;

        Eigen::Matrix<double, 4, 1> est_pose_dot;
        est_pose_dot = J * nu;

        Eigen::Matrix<double, 4, 4> LAMBDA;
        LAMBDA << 3 * Eigen::Matrix<double, 4, 4>::Identity();

        LAMBDA(2, 2) = 1.0;
        LAMBDA(3, 3) = 10.0;

        Eigen::Matrix<double, 4, 1> q_r_dot;
        q_r_dot = J.inverse() * (des_pos_dot + LAMBDA * error);

        Eigen::Matrix<double, 4, 4> J_inv_dot;
        J_inv_dot << -sin(est_pose(3)) * nu(3), cos(est_pose(3)) * nu(3), 0.0, 0.0,
            -cos(est_pose(3)) * nu(3), -sin(est_pose(3)) * nu(3), 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0;

        Eigen::Matrix<double, 4, 1> error_dot;
        error_dot = des_pos_dot - est_pose_dot;

        Eigen::Matrix<double, 4, 1> q_r_2dot;
        q_r_2dot = J.inverse() * (des_pos_2dot + LAMBDA * error_dot) + J_inv_dot * (des_pos_dot + LAMBDA * error);

        Eigen::Matrix<double, 4, 1> s;
        s = J.inverse() * (error_dot + LAMBDA * error);

        // CORIOLIS MATRIX
        Eigen::Matrix<double, 4, 4> C;

        C << 0.0, 0.0, 0.0, -m * nu(1),
            0.0, 0.0, 0.0, m * nu(0),
            0.0, 0.0, 0.0, 0.0,
            m * nu(1), -m * nu(0), 0.0, 0.0;

        // DAMPING MATRIX
        Eigen::Matrix<double, 4, 4> D;
        // D.diagonal() << -X_u, -Y_v, -Z_w, -N_r - N_r_r * r_hat;

        D << -X_u, 0.0, 0.0, 0.0,
            0.0, -Y_v, 0.0, -Y_r,
            0.0, 0.0, -Z_w, 0.0,
            0.0, -N_v, 0.0, -N_r;

        Eigen::Matrix<double, 4, 4> M;
        M << m, 0.0, 0.0, 0.0,
            0.0, m, 0.0, 0.0,
            0.0, 0.0, m, 0.0,
            0.0, 0.0, 0.0, I;

        Eigen::Matrix<double, 4, 4> K_d;
        K_d << Eigen::Matrix<double, 4, 4>::Identity();

        // Define the torques vector
        Eigen::Matrix<double, 4, 1> torques_vec;
        torques_vec = M * q_r_2dot + K_d * s + C * q_r_dot + J.transpose() * error + D * nu;

        std::vector<double> torques = {torques_vec(0), torques_vec(1), torques_vec(2), torques_vec(3)};

        // Publishing the torques
        tesi_bluerov2::Floats torques_msg;
        torques_msg.data = torques;

        // Pubblico i dati solo dopo 5 secondi e se il tempo iniziale è stato calcolato correttamente
        if (ros::Time::now().toSec() - init_time > 5.0 && !is_init)
        {
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
