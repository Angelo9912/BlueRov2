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

int main(int argc, char **argv)
{
    ros::init(argc, argv, "MPC_6DOF");
    ros::NodeHandle n;
    rosbag::Bag tau_bag;
    std::string path = ros::package::getPath("tesi_bluerov2");
    tau_bag.open(path + "/bag/tau.bag", rosbag::bagmode::Write);

    ros::Publisher chatter_pub = n.advertise<tesi_bluerov2::Floats>("tau_topic", 1);
    ros::Subscriber sub_des_state = n.subscribe("state/desired_state_topic", 1, desStateCallback);
    ros::Subscriber sub_est_state = n.subscribe("state/est_state_UKF_topic", 1, estStateCallback);

    double freq = 100;
    double dt = 1 / freq;
    ros::Rate loop_rate(freq);

    ros::Duration(10).sleep();

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

    bool is_init = true;
    double init_time;

    // Define the desired state and the estimated state vectors
    Eigen::Matrix<double, 8, 1> est_state;

    Eigen::Matrix<double, 6, 1> eta_dot;

    // Define error vector
    Eigen::Matrix<double, 24, 1> error;

    Eigen::Matrix<double, 6, 1> nu;

    Eigen::Matrix<double, 6, 6> J;

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

        // Define Jacobian Matrix
        J << cos(psi_hat) * cos(theta_hat), cos(psi_hat) * sin(phi_hat) * sin(theta_hat) - cos(phi_hat) * sin(psi_hat), sin(phi_hat) * sin(psi_hat) + cos(phi_hat) * cos(psi_hat) * sin(theta_hat), 0, 0, 0,
            cos(theta_hat) * sin(psi_hat), cos(phi_hat) * cos(psi_hat) + sin(phi_hat) * sin(psi_hat) * sin(theta_hat), cos(phi_hat) * sin(psi_hat) * sin(theta_hat) - cos(psi_hat) * sin(phi_hat), 0, 0, 0,
            -sin(theta_hat), cos(theta_hat) * sin(phi_hat), cos(phi_hat) * cos(theta_hat), 0, 0, 0,
            0, 0, 0, 1, sin(phi_hat) * tan(theta_hat), cos(phi_hat) * tan(theta_hat),
            0, 0, 0, 0, cos(phi_hat), -sin(phi_hat),
            0, 0, 0, 0, sin(phi_hat) / cos(theta_hat), cos(phi_hat) / cos(theta_hat);

        nu << u_hat, v_hat, w_hat, p_hat, q_hat, r_hat;

        eta_dot = J * nu;

        est_state << x_hat, eta_dot(0), y_hat, eta_dot(1), z_hat, eta_dot(2), psi_hat, eta_dot(5);

        double W = m * 9.81;

        // Computing the control law
        Eigen::Matrix<double, 8, 8> A_cont;
        A_cont << 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
        Eigen::Matrix<double, 8, 8> A = dt * A_cont + Eigen::Matrix<double, 8, 8>::Identity();

        Eigen::Matrix<double, 8, 4> B_cont;
        B_cont << 0.0, 0.0, 0.0, 0.0,
            1.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0,
            0.0, 1.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 1.0, 0.0,
            0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 1.0;

        Eigen::Matrix<double, 8, 4> B = dt * B_cont;

        Eigen::Matrix<double, 8, 8> C = Eigen::Matrix<double, 8, 8>::Identity();

        Eigen::Matrix<double, 8, 4> Zero;
        Zero.setZero();

        Eigen::Matrix<double, 24, 12> PHI;
        PHI << C * B, Zero, Zero,
            C * A * B, C * B, Zero,
            C * A * A * B, C * A * B, C * B;

        Eigen::Matrix<double, 24, 8> F;
        F << C * A,
            C * A * A,
            C * A * A * A;

        Eigen::Matrix<double, 4, 4> Zero_4;
        Eigen::Matrix<double, 12, 12> Rc = 0.01 * Eigen::Matrix<double, 12, 12>::Identity();

        Eigen::Matrix<double, 24, 24> Qc = 10.0 * Eigen::Matrix<double, 24, 24>::Identity();

        // Coefficienti per la x
        Qc(0, 0) = 100.0;
        Qc(8, 8) = 100.0;
        Qc(16, 16) = 100.0;

        // Coefficienti per la y
        Qc(2, 2) = 100.0;
        Qc(10, 10) = 100.0;
        Qc(18, 18) = 100.0;

        // Coefficienti per la z
        Qc(4, 4) = 500.0;
        Qc(12, 12) = 500.0;
        Qc(20, 20) = 500.0;

        // Coefficienti per la psi
        Qc(6, 6) = 1000.0;
        Qc(14, 14) = 1000.0;
        Qc(22, 22) = 1000.0;

        // Estrazione valori desired state

        Eigen::Matrix<double, 8, 1> des_state;
        des_state << x_d, x_dot_d, y_d, y_dot_d, z_d, z_dot_d, psi_d, psi_dot_d;

        Eigen::Matrix<double, 24, 1> r;
        r << des_state, des_state, des_state;

        Eigen::Matrix<double, 24, 1> prediction;
        prediction = F * est_state;
        for (int i = 6; i < 24; i += 8)
        {
            prediction(i) = atan2(sin(prediction(i)), cos(prediction(i)));
        }

        error = r - prediction;

        for (int i = 6; i < 24; i += 8)
        {
            error(i) = angleDifference(error(i));
        }

        Eigen::Matrix<double, 12, 1> DeltaU;
        /*DeltaU = (PHI.transpose() * PHI + Rc).inverse() * PHI.transpose() * (error);*/
        DeltaU = (PHI.transpose() * Qc * PHI + Rc).inverse() * (PHI.transpose() * Qc * (error));
        Eigen::Matrix<double, 4, 1> u;
        u << DeltaU(0), DeltaU(1), DeltaU(2), DeltaU(3);

        // Define the torques vector
        Eigen::Matrix<double, 4, 1> torques_vec;

        torques_vec << X_u_dot * r_hat * v_hat - m * u(2) * sin(theta_hat) - X_u_dot * q_hat * w_hat - X_u_dot * u(2) * sin(theta_hat) + Y_v_dot * r_hat * v_hat - Z_w_dot * q_hat * w_hat + 500 * A_x * u_hat * abs(u_hat) + X_u_dot * u(0) * cos(psi_hat) * cos(theta_hat) + X_u_dot * u(1) * cos(theta_hat) * sin(psi_hat) + m * u(0) * cos(psi_hat) * cos(theta_hat) + m * u(1) * cos(theta_hat) * sin(psi_hat),
            Y_v_dot * u(1) + m * u(1) - X_u_dot * r_hat * u_hat + Y_v_dot * p_hat * w_hat - Y_v_dot * r_hat * u_hat + Z_w_dot * p_hat * w_hat - 2 * Y_v_dot * u(1) * cos(phi_hat / 2) * cos(phi_hat / 2) - 2 * Y_v_dot * u(1) * cos(psi_hat / 2) * cos(psi_hat / 2) - 2 * m * u(1) * cos(phi_hat / 2) * cos(phi_hat / 2) - 2 * m * u(1) * cos(psi_hat / 2) * cos(psi_hat / 2) + 500 * A_y * v_hat * abs(v_hat) + 4 * Y_v_dot * u(1) * cos(phi_hat / 2) * cos(phi_hat / 2) * cos(psi_hat / 2) * cos(psi_hat / 2) + 4 * m * u(1) * cos(phi_hat / 2) * cos(phi_hat / 2) * cos(psi_hat / 2) * cos(psi_hat / 2) - 2 * Y_v_dot * u(2) * cos(phi_hat / 2) * sin(phi_hat / 2) + 2 * Y_v_dot * u(0) * cos(psi_hat / 2) * sin(psi_hat / 2) - 2 * m * u(2) * cos(phi_hat / 2) * sin(phi_hat / 2) + 2 * m * u(0) * cos(psi_hat / 2) * sin(psi_hat / 2) - 4 * Y_v_dot * u(0) * cos(phi_hat / 2) * cos(phi_hat / 2) * cos(psi_hat / 2) * sin(psi_hat / 2) + 4 * Y_v_dot * u(2) * cos(phi_hat / 2) * cos(theta_hat / 2) * cos(theta_hat / 2) * sin(phi_hat / 2) - 4 * m * u(0) * cos(phi_hat / 2) * cos(phi_hat / 2) * cos(psi_hat / 2) * sin(psi_hat / 2) + 4 * m * u(2) * cos(phi_hat / 2) * cos(theta_hat / 2) * cos(theta_hat / 2) * sin(phi_hat / 2) - 4 * Y_v_dot * u(0) * cos(phi_hat / 2) * cos(theta_hat / 2) * sin(phi_hat / 2) * sin(theta_hat / 2) - 4 * m * u(0) * cos(phi_hat / 2) * cos(theta_hat / 2) * sin(phi_hat / 2) * sin(theta_hat / 2) + 8 * Y_v_dot * u(0) * cos(phi_hat / 2) * cos(psi_hat / 2) * cos(psi_hat / 2) * cos(theta_hat / 2) * sin(phi_hat / 2) * sin(theta_hat / 2) + 8 * m * u(0) * cos(phi_hat / 2) * cos(psi_hat / 2) * cos(psi_hat / 2) * cos(theta_hat / 2) * sin(phi_hat / 2) * sin(theta_hat / 2) + 8 * Y_v_dot * u(1) * cos(phi_hat / 2) * cos(psi_hat / 2) * cos(theta_hat / 2) * sin(phi_hat / 2) * sin(psi_hat / 2) * sin(theta_hat / 2) + 8 * m * u(1) * cos(phi_hat / 2) * cos(psi_hat / 2) * cos(theta_hat / 2) * sin(phi_hat / 2) * sin(psi_hat / 2) * sin(theta_hat / 2),
            X_u_dot * q_hat * u_hat - Y_v_dot * p_hat * v_hat - Z_w_dot * p_hat * v_hat + Z_w_dot * q_hat * u_hat + 500 * A_z * w_hat * abs(w_hat) + m * u(0) * sin(phi_hat) * sin(psi_hat) + Z_w_dot * u(2) * cos(phi_hat) * cos(theta_hat) - Z_w_dot * u(1) * cos(psi_hat) * sin(phi_hat) + m * u(2) * cos(phi_hat) * cos(theta_hat) + Z_w_dot * u(0) * sin(phi_hat) * sin(psi_hat) - m * u(1) * cos(psi_hat) * sin(phi_hat) + Z_w_dot * u(0) * cos(phi_hat) * cos(psi_hat) * sin(theta_hat) + Z_w_dot * u(1) * cos(phi_hat) * sin(psi_hat) * sin(theta_hat) + m * u(0) * cos(phi_hat) * cos(psi_hat) * sin(theta_hat) + m * u(1) * cos(phi_hat) * sin(psi_hat) * sin(theta_hat),
            (cos(theta_hat) * (I_z + N_r_dot) * (u(3) + (r_hat * (-2 * q_hat * sin(theta_hat) * cos(phi_hat) * cos(phi_hat) + 2 * r_hat * sin(phi_hat) * sin(theta_hat) * cos(phi_hat) + q_hat * sin(theta_hat) + p_hat * cos(theta_hat) * sin(phi_hat))) / (cos(theta_hat) * cos(theta_hat)) - (q_hat * (p_hat * cos(phi_hat) * cos(theta_hat) - r_hat * sin(theta_hat) + 2 * r_hat * cos(phi_hat) * cos(phi_hat) * sin(theta_hat) + 2 * q_hat * cos(phi_hat) * sin(phi_hat) * sin(theta_hat))) / (cos(theta_hat) * cos(theta_hat)) + (cos(phi_hat) * (W * y_b * sin(theta_hat) - I_x * p_hat * q_hat + I_y * p_hat * q_hat + K_p_dot * p_hat * q_hat - M_q_dot * p_hat * q_hat + X_u_dot * u_hat * v_hat - Y_v_dot * u_hat * v_hat + 500 * A_r * r_hat * abs(r_hat) + W * x_b * cos(theta_hat) * sin(phi_hat))) / (cos(theta_hat) * (I_z + N_r_dot)) - (sin(phi_hat) * (W * z_b * sin(theta_hat) - I_x * p_hat * r_hat + I_z * p_hat * r_hat + K_p_dot * p_hat * r_hat - N_r_dot * p_hat * r_hat + X_u_dot * u_hat * w_hat - Z_w_dot * u_hat * w_hat - 500 * A_q * q_hat * abs(q_hat) + W * x_b * cos(phi_hat) * cos(theta_hat))) / (cos(theta_hat) * (I_y + M_q_dot)))) / cos(phi_hat);

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