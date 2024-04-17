#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include <ros/console.h>
#include <eigen3/Eigen/Dense>     // for eigen matrix
#include "tesi_bluerov2/Floats.h" // for accessing -- tesi_bluerov2 Floats()
#include <vector>
#include "yaml-cpp/yaml.h" // for yaml
#include "UnscentedOutput.h"
#include <random>

double tau_u = 0.0;
double tau_v = 0.0;
double tau_w = 0.0;
double tau_p = 0.0;
double tau_q = 0.0;
double tau_r = 0.0;

double x = 0.0;
double y = 0.0;
double z = 0.0;
double phi = 0.0;
double theta = 0.0;
double psi = 0.0;
double u = 0.0;
double v = 0.0;
double w = 0.0;
double p = 0.0;
double q = 0.0;
double r = 0.0;

double x_GPS = 0.0;
double y_GPS = 0.0;
double x_scanner = 0.0;
double y_scanner = 0.0;
double phi_IMU = 0.0;
double theta_IMU = 0.0;
double psi_IMU = 0.0;
double u_DVL = 0.0;
double v_DVL = 0.0;
double w_DVL = 0.0;
double p_IMU = 0.0;
double q_IMU = 0.0;
double r_IMU = 0.0;
double z_depth_sensor = 0.0;

double valid_GPS = 0;
double valid_scanner = 0;
double valid_IMU = 0;
double valid_DVL = 0;
double valid_depth_sensor = 0;

// Covariance values (process noise)
double var_tau_u = 0.0;
double var_tau_v = 0.0;
double var_tau_w = 0.0;
double var_tau_p = 0.0;
double var_tau_q = 0.0;
double var_tau_r = 0.0;

// Covariance values (sensor noise)

double var_x_GPS = 0.0;
double var_y_GPS = 0.0;
double var_x_scanner = 0.0;
double var_y_scanner = 0.0;
double var_z_depth_sensor = 0.0;
double var_phi_IMU = 0.0;
double var_theta_IMU = 0.0;
double var_psi_IMU = 0.0;
double var_u_DVL = 0.0;
double var_v_DVL = 0.0;
double var_w_DVL = 0.0;
double var_p_IMU = 0.0;
double var_q_IMU = 0.0;
double var_r_IMU = 0.0;

// Parameters for the Unscented Kalman Filter

// Import parameters from YAML file
double m = 0.0;
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

// Distanza di Mahalanobis
double mahalanobis_distance = 0.0;

bool is_prima_volta = true;

int sign(double x)
{
    if (x > 0)
    {
        return 1;
    }
    else if (x < 0)
    {
        return -1;
    }
    else
    {
        return 0;
    }
}

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

// Function to generate Gaussian random number
double gaussianNoise(double mean, double var)
{
    double stddev = sqrt(var);
    std::random_device rd;
    std::mt19937 gen(rd());
    std::normal_distribution<> d(mean, stddev);
    return d(gen);
}

// Callback function for the subscriber
void GPSCallback(const tesi_bluerov2::Floats::ConstPtr &msg)
{
    if (msg->data[0] != msg->data[0])
    {
        x_GPS = 0.0;
        y_GPS = 0.0;
        valid_GPS = 0;
    }
    else
    {
        x_GPS = msg->data[0];
        y_GPS = msg->data[1];
        valid_GPS = msg->data[2];
    }
}

void ScannerCallback(const tesi_bluerov2::Floats::ConstPtr &msg)
{
    if (msg->data[0] != msg->data[0])
    {
        x_scanner = 0.0;
        y_scanner = 0.0;
        valid_scanner = 0;
    }
    else
    {
        x_scanner = msg->data[0];
        y_scanner = msg->data[1];
        valid_scanner = msg->data[2];
    }
}

void IMUCallback(const tesi_bluerov2::Floats::ConstPtr &msg)
{
    if (msg->data[0] != msg->data[0])
    {
        phi_IMU = 0.0;
        theta_IMU = 0.0;
        psi_IMU = 0.0;
        p_IMU = 0.0;
        q_IMU = 0.0;
        r_IMU = 0.0;
        valid_IMU = 0;
    }
    else
    {
        phi_IMU = msg->data[0];
        theta_IMU = msg->data[1];
        psi_IMU = msg->data[2];
        p_IMU = msg->data[3];
        q_IMU = msg->data[4];
        r_IMU = msg->data[5];
        valid_IMU = msg->data[6];
    }
}

void DVLCallback(const tesi_bluerov2::Floats::ConstPtr &msg)
{
    if (msg->data[0] != msg->data[0])
    {
        u_DVL = 0.0;
        v_DVL = 0.0;
        w_DVL = 0.0;
        valid_DVL = 0;
    }
    else
    {
        u_DVL = msg->data[0];
        v_DVL = msg->data[1];
        w_DVL = msg->data[2];
        valid_DVL = msg->data[3];
    }
}

void depthSensorCallback(const tesi_bluerov2::Floats::ConstPtr &msg)
{
    if (msg->data[0] != msg->data[0])
    {
        z_depth_sensor = 0.0;
        valid_depth_sensor = 0;
    }
    else
    {
        z_depth_sensor = msg->data[0];
        valid_depth_sensor = msg->data[1];
    }
}

void tau_callback(const tesi_bluerov2::Floats::ConstPtr &msg)
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
        tau_r = msg->data[5];
    }
}

int main(int argc, char **argv)
{
    // Initialize the ROS system and become a node.
    ros::init(argc, argv, "EKF");

    // Create a ROS node handle
    ros::NodeHandle n;
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
    n.getParam("var_x_GPS", var_x_GPS);
    n.getParam("var_y_GPS", var_y_GPS);
    n.getParam("var_x_scanner", var_x_scanner);
    n.getParam("var_y_scanner", var_y_scanner);
    n.getParam("var_z_depth_sensor", var_z_depth_sensor);
    n.getParam("var_phi_IMU", var_phi_IMU);
    n.getParam("var_theta_IMU", var_theta_IMU);
    n.getParam("var_psi_IMU", var_psi_IMU);
    n.getParam("var_u_DVL", var_u_DVL);
    n.getParam("var_v_DVL", var_v_DVL);
    n.getParam("var_w_DVL", var_w_DVL);
    n.getParam("var_p_IMU", var_p_IMU);
    n.getParam("var_q_IMU", var_q_IMU);
    n.getParam("var_r_IMU", var_r_IMU);

    n.getParam("var_tau_u", var_tau_u);
    n.getParam("var_tau_v", var_tau_v);
    n.getParam("var_tau_w", var_tau_w);
    n.getParam("var_tau_p", var_tau_p);
    n.getParam("var_tau_q", var_tau_q);
    n.getParam("var_tau_r", var_tau_r);

    // Current corrected state vector (12x1)
    Eigen::VectorXd xi_curr(12);
    xi_curr.setZero();

    // Matrice MMSE
    Eigen::Matrix<double, 12, 12> P_curr;
    P_curr.setZero();

    // Predicted state vector
    Eigen::VectorXd xi_pred(12);
    xi_pred.setZero();

    // Matrice di covarianza di predizione
    Eigen::MatrixXd P_pred(12, 12);
    P_pred.setZero();

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

    // Time step
    double freq = 50;
    double dt = 1 / freq;

    bool is_init = false;

    bool is_GPS_init = false;
    bool is_scanner_init = false;
    bool is_depth_init = false;
    bool is_IMU_init = false;

    Eigen::VectorXd var_sensors(14);
    var_sensors << var_x_GPS, var_y_GPS, var_x_scanner, var_y_scanner, var_z_depth_sensor, var_phi_IMU, var_theta_IMU, var_psi_IMU, var_u_DVL, var_v_DVL, var_w_DVL, var_p_IMU, var_q_IMU, var_r_IMU;

    Eigen::MatrixXd Q(6, 6);
    Q << var_tau_u, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, var_tau_v, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, var_tau_w, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, var_tau_p, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, var_tau_q, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, var_tau_r;

    // Set the loop rate
    ros::Rate loop_rate(freq);

    // Create a publisher object
    ros::Publisher est_state_pub = n.advertise<tesi_bluerov2::Floats>("est_state_topic", 1000);

    // Create subscriber objects

    ros::Subscriber tau_sub = n.subscribe("tau_topic", 1000, tau_callback);

    ros::Subscriber GPS_sub = n.subscribe("GPS_topic", 1000, GPSCallback);

    ros::Subscriber scanner_sub = n.subscribe("scanner_topic", 1000, ScannerCallback);

    ros::Subscriber IMU_sub = n.subscribe("IMU_topic", 1000, IMUCallback);

    ros::Subscriber DVL_sub = n.subscribe("DVL_topic", 1000, DVLCallback);

    ros::Subscriber depth_sub = n.subscribe("depth_sensor_topic", 1000, depthSensorCallback);

    tesi_bluerov2::Floats msg;

    while (ros::ok())
    {

        //////Kalman Filter///////

        ///////////////////////////////////////////////////////////////////////
        ///////////////// INITIALIZATION OF THE FILTER ////////////////////////
        ///////////////////////////////////////////////////////////////////////

        if (!is_init)
        {
            double var_x;
            double var_y;
            if (valid_scanner == 1.0)
            {
                xi_curr(0) = x_scanner;
                xi_curr(1) = y_scanner;
                var_x = var_x_scanner;
                var_y = var_y_scanner;
                is_scanner_init = true;
            }
            else if (valid_GPS == 1.0)
            {
                xi_curr(0) = x_GPS;
                xi_curr(1) = y_GPS;
                var_x = var_x_GPS;
                var_y = var_y_GPS;
                is_GPS_init = true;
            }

            if (valid_depth_sensor == 1.0)
            {
                xi_curr(2) = z_depth_sensor;
                is_depth_init = true;
            }

            if (valid_IMU == 1.0)
            {
                xi_curr(3) = phi_IMU;
                xi_curr(4) = theta_IMU;
                xi_curr(5) = psi_IMU;

                xi_curr(9) = p_IMU;
                xi_curr(10) = q_IMU;
                xi_curr(11) = r_IMU;

                is_IMU_init = true;
            }

            xi_curr(6) = 0.0 + gaussianNoise(0, var_u_DVL);
            xi_curr(7) = 0.0 + gaussianNoise(0, var_v_DVL);
            xi_curr(8) = 0.0 + gaussianNoise(0, var_w_DVL);

            P_curr << var_x, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                0, var_y, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                0, 0, var_z_depth_sensor, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                0, 0, 0, var_phi_IMU, 0, 0, 0, 0, 0, 0, 0, 0,
                0, 0, 0, 0, var_theta_IMU, 0, 0, 0, 0, 0, 0, 0,
                0, 0, 0, 0, 0, var_psi_IMU, 0, 0, 0, 0, 0, 0,
                0, 0, 0, 0, 0, 0, var_u_DVL, 0, 0, 0, 0, 0,
                0, 0, 0, 0, 0, 0, 0, var_v_DVL, 0, 0, 0, 0,
                0, 0, 0, 0, 0, 0, 0, 0, var_w_DVL, 0, 0, 0,
                0, 0, 0, 0, 0, 0, 0, 0, 0, var_p_IMU, 0, 0,
                0, 0, 0, 0, 0, 0, 0, 0, 0, 0, var_q_IMU, 0,
                0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, var_r_IMU;

            is_init = ((is_GPS_init || is_scanner_init) && is_depth_init && is_IMU_init);

            if (is_init) // Se l'inizializzazione è completata pubblica lo stato stimato iniziale e passa alla predizione
            {
                ROS_WARN_STREAM("INITIALIZATION COMPLETED\n");

                // ROS_WARN_STREAM("eigs: \n"
                //               << P_curr.eigenvalues().real().minCoeff() << " , " << P_curr.eigenvalues().real().maxCoeff());

                msg.data = {xi_curr(0), xi_curr(1), xi_curr(2), xi_curr(3), xi_curr(4), xi_curr(5), xi_curr(6), xi_curr(7), xi_curr(8), xi_curr(9), xi_curr(10), xi_curr(11), mahalanobis_distance};
                est_state_pub.publish(msg);

                ////////////////////////////////////////////////////////////////////
                /////////////////////////PREDICTION/////////////////////////////////
                ////////////////////////////////////////////////////////////////////

                // VETTORE DI FORZE E MOMENTI
                Eigen::VectorXd tau(6);
                tau << tau_u, tau_v, tau_w, tau_p, tau_q, tau_r;

                phi = xi_curr(3);
                theta = xi_curr(4);
                psi = xi_curr(5);
                u = xi_curr(6);
                v = xi_curr(7);
                w = xi_curr(8);
                p = xi_curr(9);
                q = xi_curr(10);
                r = xi_curr(11);

                double W = m * 9.81;

                Eigen::MatrixXd F(12, 12);

                F << 0.0, 0.0, 0.0, v * (sin(phi) * sin(psi) + cos(phi) * cos(psi) * sin(theta)) + w * (cos(phi) * sin(psi) - cos(psi) * sin(phi) * sin(theta)), cos(psi) * (w * cos(phi) * cos(theta) - u * sin(theta) + v * cos(theta) * sin(phi)), w * (cos(psi) * sin(phi) - cos(phi) * sin(psi) * sin(theta)) - v * (cos(phi) * cos(psi) + sin(phi) * sin(psi) * sin(theta)) - u * cos(theta) * sin(psi), cos(psi) * cos(theta), cos(psi) * sin(phi) * sin(theta) - cos(phi) * sin(psi), (sin(phi) * sin(psi) + cos(phi) * cos(psi) * sin(theta)), 0.0, 0.0, 0.0,
                    0.0, 0.0, 0.0, -v * (cos(psi) * sin(phi) - cos(phi) * sin(psi) * sin(theta)) - w * (cos(phi) * cos(psi) + sin(phi) * sin(psi) * sin(theta)), sin(psi) * (w * cos(phi) * cos(theta) - u * sin(theta) + v * cos(theta) * sin(phi)), w * (sin(phi) * sin(psi) + cos(phi) * cos(psi) * sin(theta)) - v * (cos(phi) * sin(psi) - cos(psi) * sin(phi) * sin(theta)) + u * cos(psi) * cos(theta), cos(theta) * sin(psi), (cos(phi) * cos(psi) + sin(phi) * sin(psi) * sin(theta)), cos(phi) * sin(psi) * sin(theta) - cos(psi) * sin(phi), 0.0, 0.0, 0.0,
                    0.0, 0.0, 0.0, cos(theta) * (v * cos(phi) - w * sin(phi)), -u * cos(theta) - w * cos(phi) * sin(theta) - v * sin(phi) * sin(theta), 0.0, -sin(theta), cos(theta) * sin(phi), cos(phi) * cos(theta), 0.0, 0.0, 0.0,
                    0.0, 0.0, 0.0, (2 * tan(theta) * (q - r * tan(phi / 2))) / (tan(phi / 2) * tan(phi/2) + 1) - q * tan(theta), (r * cos(phi) + q * sin(phi)) / (cos(theta) *cos(theta)), 0.0, 0.0, 0.0, 0.0, 1, sin(phi) * tan(theta), cos(phi) * tan(theta),
                    0.0, 0.0, 0.0, -r * cos(phi) - q * sin(phi), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, cos(phi), -sin(phi),
                    0.0, 0.0, 0.0, (q * cos(phi) - r * sin(phi)) / cos(theta), -(sin(theta) * (r * cos(phi) + q * sin(phi))) / (sin(theta) * sin(theta) - 1), 0.0, 0.0, 0.0, 0.0, 0.0, sin(phi) / cos(theta), cos(phi) / cos(theta),
                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -(1000 * A_x * abs(u)) / (X_u_dot + m), -(r * (Y_v_dot - m)) / (X_u_dot + m), (q * (Z_w_dot - m)) / (X_u_dot + m), 0.0, (w * (Z_w_dot - m)) / (X_u_dot + m), -(v * (Y_v_dot - m)) / (X_u_dot + m),
                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, (r * (X_u_dot - m)) / (Y_v_dot + m), -(1000 * A_y * abs(v)) / (Y_v_dot + m), -(p * (Z_w_dot - m)) / (Y_v_dot + m), -(w * (Z_w_dot - m)) / (Y_v_dot + m), 0.0, (u * (X_u_dot - m)) / (Y_v_dot + m),
                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -(q * (X_u_dot - m)) / (Z_w_dot + m), (p * (Y_v_dot - m)) / (Z_w_dot + m), -(1000 * A_z * abs(w)) / (Z_w_dot + m), (v * (Y_v_dot - m)) / (Z_w_dot + m), -(u * (X_u_dot - m)) / (Z_w_dot + m), 0.0,
                    0.0, 0.0, 0.0, (W * cos(theta) * (z_b * cos(phi) + y_b * sin(phi))) / (I_x + K_p_dot), (W * sin(theta) * (y_b * cos(phi) - z_b * sin(phi))) / (I_x + K_p_dot), 0.0, 0.0, -(w * (Y_v_dot - Z_w_dot)) / (I_x + K_p_dot), -(v * (Y_v_dot - Z_w_dot)) / (I_x + K_p_dot), -(1000 * A_p * abs(p)) / (I_x + K_p_dot), (r * (I_y - I_z - M_q_dot + N_r_dot)) / (I_x + K_p_dot), (q * (I_y - I_z - M_q_dot + N_r_dot)) / (I_x + K_p_dot),
                    0.0, 0.0, 0.0, -(W * x_b * cos(theta) * sin(phi)) / (I_y + M_q_dot), (W * (z_b * cos(theta) - x_b * cos(phi) * sin(theta))) / (I_y + M_q_dot), 0.0, (w * (X_u_dot - Z_w_dot)) / (I_y + M_q_dot), 0.0, (u * (X_u_dot - Z_w_dot)) / (I_y + M_q_dot), -(r * (I_x - I_z - K_p_dot + N_r_dot)) / (I_y + M_q_dot), -(1000 * A_q * abs(q)) / (I_y + M_q_dot), -(p * (I_x - I_z - K_p_dot + N_r_dot)) / (I_y + M_q_dot),
                    0.0, 0.0, 0.0, -(W * x_b * cos(phi) * cos(theta)) / (I_z + N_r_dot), -(W * (y_b * cos(theta) - x_b * sin(phi) * sin(theta))) / (I_z + N_r_dot), 0.0, -(v * (X_u_dot - Y_v_dot)) / (I_z + N_r_dot), -(u * (X_u_dot - Y_v_dot)) / (I_z + N_r_dot), 0.0, (q * (I_x - I_y - K_p_dot + M_q_dot)) / (I_z + N_r_dot), (p * (I_x - I_y - K_p_dot + M_q_dot)) / (I_z + N_r_dot), -(1000 * A_r * abs(r)) / (I_z + N_r_dot);

                F = dt * F + Eigen::MatrixXd::Identity(12, 12);

                Eigen::MatrixXd D(12, 6);

                D << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                    1 / (X_u_dot + m), 0.0, 0.0, 0.0, 0.0, 0.0,
                    0.0, 1 / (Y_v_dot + m), 0.0, 0.0, 0.0, 0.0,
                    0.0, 0.0, 1 / (Z_w_dot + m), 0.0, 0.0, 0.0,
                    0.0, 0.0, 0.0, 1 / (I_x + K_p_dot), 0.0, 0.0,
                    0.0, 0.0, 0.0, 0.0, 1 / (I_y + M_q_dot), 0.0,
                    0.0, 0.0, 0.0, 0.0, 0.0, 1 / (I_z + N_r_dot);

                D = dt * D;

                xi_pred = F * xi_curr + D * tau;

                // wrapToPi
                xi_pred(3) = atan2(sin(xi_pred(3)), cos(xi_pred(3)));
                xi_pred(4) = atan2(sin(xi_pred(4)), cos(xi_pred(4)));
                xi_pred(5) = atan2(sin(xi_pred(5)), cos(xi_pred(5)));

                P_pred = F * P_curr * F.transpose() + D * Q * D.transpose();
            }
        }
        else
        {
            ///////////////////////////////////////////////////////////////////////
            ///////////////////////////// CORRECTION //////////////////////////////
            ///////////////////////////////////////////////////////////////////////

            // VETTORE DI MISURA
            Eigen::VectorXd z(14);
            z << x_GPS, y_GPS, x_scanner, y_scanner, z_depth_sensor, phi_IMU, theta_IMU, psi_IMU, u_DVL, v_DVL, w_DVL, p_IMU, q_IMU, r_IMU;

            Eigen::VectorXd valid(14);
            valid << valid_GPS, valid_GPS, valid_scanner, valid_scanner, valid_depth_sensor, valid_IMU, valid_IMU, valid_IMU, valid_DVL, valid_DVL, valid_DVL, valid_IMU, valid_IMU, valid_IMU;

            Eigen::MatrixXd H(1, 12);
            H.setZero();

            Eigen::VectorXd var_valid(1);
            var_valid.setZero();

            // Pesco le misure valide
            Eigen::VectorXd z_valid(1);
            z_valid.setZero();

            bool is_first_element = true;
            int n_valid = 0;

            // Indice della misura di phi
            int id_phi = -1;

            for (int i = 0; i < valid.size(); i++)
            {
                if (valid(i) == 1.0)
                {
                    if (z_valid.size() == 1 && is_first_element)
                    {
                        z_valid(0) = z(i);
                        if (i < 2)
                        {
                            H(0, i) = 1.0;
                        }
                        else
                        {
                            H(0, i - 2) = 1.0;
                        }
                        var_valid(0) = var_sensors(i);

                        if (i == 5)
                        {
                            id_phi = 0;
                        }

                        is_first_element = false;
                    }
                    else
                    {
                        n_valid = z_valid.size();
                        z_valid.conservativeResize(n_valid + 1);
                        H.conservativeResize(n_valid + 1, 12);
                        var_valid.conservativeResize(n_valid + 1);
                        n_valid++;
                        z_valid(n_valid - 1) = z(i);
                        H.row(n_valid - 1) << Eigen::MatrixXd::Zero(1, 12);

                        // Gestione della rindondanza delle misure GPS/scanner (x,y)
                        if (i < 2)
                        {
                            H(n_valid - 1, i) = 1.0;
                        }
                        else
                        {
                            H(n_valid - 1, i - 2) = 1.0;
                        }

                        // Vettore delle varianze delle misure valide (per la creazione a dimensione variabile di R)
                        var_valid(n_valid - 1) = var_sensors(i);

                        // Indice della misura di phi
                        if (i == 5)
                        {
                            id_phi = n_valid - 1;
                        }
                    }
                }
            }


            if (z_valid.size() == 1 && z_valid(0) == 0.0)
            {
                ROS_WARN("NO VALID MEASURES");
                xi_curr = xi_pred;
                P_curr = P_pred;
                mahalanobis_distance = 0.0;
            }
            else
            {
                // Correzione

                int n_z = z_valid.size();

                Eigen::MatrixXd R(n_z, n_z);
                R = var_valid.asDiagonal();

                Eigen::MatrixXd S_k(n_z, n_z);
                S_k = H * P_pred * H.transpose() + R;

                // Calcolo il guadagno di Kalman
                Eigen::MatrixXd K(12, n_z);
                K = P_pred * H.transpose() * S_k.inverse();

                // Calcolo la predizione corretta
                Eigen::VectorXd e_k(n_z);
                e_k = z_valid - H * xi_pred;

                if (id_phi >= 0)
                {
                    for (int i = id_phi; i < id_phi + 3; i++)
                    {
                        e_k(i) = angleDifference(e_k(i));
                    }
                }

                // Calcolo la stima corretta

                Eigen::VectorXd xi_corr(xi_pred.size());

                xi_corr = xi_pred + K * e_k;

                xi_corr(3) = atan2(sin(xi_corr(3)), cos(xi_corr(3)));
                xi_corr(4) = atan2(sin(xi_corr(4)), cos(xi_corr(4)));
                xi_corr(5) = atan2(sin(xi_corr(5)), cos(xi_corr(5)));

                // Calcolo la matrice di covarianza corretta
                Eigen::MatrixXd P_corr(12, 12);
                P_corr = (Eigen::MatrixXd::Identity(12, 12) - K * H) * P_pred * (Eigen::MatrixXd::Identity(12, 12) - K * H).transpose() + K * R * K.transpose();

                xi_curr = xi_corr;
                P_curr = P_corr;

                mahalanobis_distance = sqrt((e_k.transpose() * S_k.inverse() * e_k));
            }

            ////////////////////////////////////////////////////////////////////
            /////////////////////////PREDICTION/////////////////////////////////
            ////////////////////////////////////////////////////////////////////

            // VETTORE DI FORZE E MOMENTI
            Eigen::VectorXd tau(6);
            tau << tau_u, tau_v, tau_w, tau_p, tau_q, tau_r;

            phi = xi_curr(3);
            theta = xi_curr(4);
            psi = xi_curr(5);
            u = xi_curr(6);
            v = xi_curr(7);
            w = xi_curr(8);
            p = xi_curr(9);
            q = xi_curr(10);
            r = xi_curr(11);

            Eigen::MatrixXd F(12, 12);

            F << 0.0, 0.0, 0.0, v * (sin(phi) * sin(psi) + cos(phi) * cos(psi) * sin(theta)) + w * (cos(phi) * sin(psi) - cos(psi) * sin(phi) * sin(theta)), w * cos(phi) * cos(psi) * cos(theta) - u * cos(psi) * sin(theta) + v * cos(psi) * cos(theta) * sin(phi), w * (cos(psi) * sin(phi) - cos(phi) * sin(psi) * sin(theta)) - v * (cos(phi) * cos(psi) + sin(phi) * sin(psi) * sin(theta)) - u * cos(theta) * sin(psi), cos(psi) * cos(theta), cos(psi) * sin(phi) * sin(theta) - cos(phi) * sin(psi), (sin(phi) * sin(psi) + cos(phi) * cos(psi) * sin(theta)), 0.0, 0.0, 0.0,
                0.0, 0.0, 0.0, -v * (cos(psi) * sin(phi) - cos(phi) * sin(psi) * sin(theta)) - w * (cos(phi) * cos(psi) + sin(phi) * sin(psi) * sin(theta)), w * cos(phi) * cos(theta) * sin(psi) - u * sin(psi) * sin(theta) + v * cos(theta) * sin(phi) * sin(psi), w * (sin(phi) * sin(psi) + cos(phi) * cos(psi) * sin(theta)) - v * (cos(phi) * sin(psi) - cos(psi) * sin(phi) * sin(theta)) + u * cos(psi) * cos(theta), cos(theta) * sin(psi), (cos(phi) * cos(psi) + sin(phi) * sin(psi) * sin(theta)), cos(phi) * sin(psi) * sin(theta) - cos(psi) * sin(phi), 0.0, 0.0, 0.0,
                0.0, 0.0, 0.0, v * cos(phi) * cos(theta) - w * cos(theta) * sin(phi), -u * cos(theta) - w * cos(phi) * sin(theta) - v * sin(phi) * sin(theta), 0.0, -sin(theta), cos(theta) * sin(phi), cos(phi) * cos(theta), 0.0, 0.0, 0.0,
                0.0, 0.0, 0.0, q * cos(phi) * tan(theta) - r * sin(phi) * tan(theta), r * cos(phi) * (tan(theta) * tan(theta) + 1) + q * sin(phi) * (tan(theta) * tan(theta) + 1), 0.0, 0.0, 0.0, 0.0, (1), sin(phi) * tan(theta), cos(phi) * tan(theta),
                0.0, 0.0, 0.0, -r * cos(phi) - q * sin(phi), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, cos(phi), -sin(phi),
                0.0, 0.0, 0.0, (q * cos(phi)) / cos(theta) - (r * sin(phi)) / cos(theta), (r * cos(phi) * sin(theta)) / (cos(theta) * cos(theta)) + (q * sin(phi) * sin(theta)) / (cos(theta) * cos(theta)), 0.0, 0.0, 0.0, 0.0, 0.0, sin(phi) / cos(theta), cos(phi) / cos(theta),
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -(500 * A_x * abs(u) + 500 * A_x * u * sign(u)) / (X_u_dot + m), -(r * (Y_v_dot - m)) / (X_u_dot + m), (q * (Z_w_dot - m)) / (X_u_dot + m), 0.0, (Z_w_dot * w - m * w) / (X_u_dot + m), -(Y_v_dot * v - m * v) / (X_u_dot + m),
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, (r * (X_u_dot - m)) / (Y_v_dot + m), -(500 * A_y * abs(v) + 500 * A_y * v * sign(v)) / (Y_v_dot + m), -(p * (Z_w_dot - m)) / (Y_v_dot + m), -(Z_w_dot * w - m * w) / (Y_v_dot + m), 0.0, (X_u_dot * u - m * u) / (Y_v_dot + m),
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -(q * (X_u_dot - m)) / (Z_w_dot + m), (p * (Y_v_dot - m)) / (Z_w_dot + m), -(500 * A_z * abs(w) + 500 * A_z * w * sign(w)) / (Z_w_dot + m), (Y_v_dot * v - m * v) / (Z_w_dot + m), -(X_u_dot * u - m * u) / (Z_w_dot + m), 0.0,
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -(m * w - Z_w_dot * w + w * (Y_v_dot - m)) / (I_x + K_p_dot), (m * v - Y_v_dot * v + v * (Z_w_dot - m)) / (I_x + K_p_dot), -(K_p_dot * r + 500 * A_p * abs(p) + 500 * A_p * p * sign(p)) / (I_x + K_p_dot), (N_r_dot * r - I_z * r + r * (I_y - M_q_dot)) / (I_x + K_p_dot), -(K_p_dot * p - I_y * q + M_q_dot * q + q * (I_z - N_r_dot)) / (I_x + K_p_dot),
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, (m * w - Z_w_dot * w + w * (X_u_dot - m)) / (I_y + M_q_dot), 0.0, -(m * u - X_u_dot * u + u * (Z_w_dot - m)) / (I_y + M_q_dot), -(N_r_dot * r - I_z * r + r * (I_x - K_p_dot)) / (I_y + M_q_dot), -(500 * A_q * abs(q) + 500 * A_q * q * sign(q)) / (I_y + M_q_dot), (K_p_dot * p - I_x * p + p * (I_z - N_r_dot)) / (I_y + M_q_dot),
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -(m * v - Y_v_dot * v + v * (X_u_dot - m)) / (I_z + N_r_dot), (m * u - X_u_dot * u + u * (Y_v_dot - m)) / (I_z + N_r_dot), 0.0, (2 * K_p_dot * p - I_y * q + M_q_dot * q + q * (I_x - K_p_dot)) / (I_z + N_r_dot), -(K_p_dot * p - I_x * p + p * (I_y - M_q_dot)) / (I_z + N_r_dot), -(500 * A_r * abs(r) + 500 * A_r * r * sign(r)) / (I_z + N_r_dot);

            F = dt * F + Eigen::MatrixXd::Identity(12, 12);

            Eigen::MatrixXd D(12, 6);

            D << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                1 / (X_u_dot + m), 0.0, 0.0, 0.0, 0.0, 0.0,
                0.0, 1 / (Y_v_dot + m), 0.0, 0.0, 0.0, 0.0,
                0.0, 0.0, 1 / (Z_w_dot + m), 0.0, 0.0, 0.0,
                0.0, 0.0, 0.0, 1 / (I_x + K_p_dot), 0.0, 0.0,
                0.0, 0.0, 0.0, 0.0, 1 / (I_y + M_q_dot), 0.0,
                0.0, 0.0, 0.0, 0.0, 0.0, 1 / (I_z + N_r_dot);

            D = dt * D;

            xi_pred = F * xi_curr + D * tau;

            // wrap2Pi
            xi_pred(3) = atan2(sin(xi_pred(3)), cos(xi_pred(3)));
            xi_pred(4) = atan2(sin(xi_pred(4)), cos(xi_pred(4)));
            xi_pred(5) = atan2(sin(xi_pred(5)), cos(xi_pred(5)));

            P_pred = F * P_curr * F.transpose() + D * Q * D.transpose();

            ///////////////////////////////////////////////////////////////////////
            ///////////////////////////// PUBLISHING //////////////////////////////
            ///////////////////////////////////////////////////////////////////////

            msg.data = {xi_curr(0), xi_curr(1), xi_curr(2), xi_curr(3), xi_curr(4), xi_curr(5), xi_curr(6), xi_curr(7), xi_curr(8), xi_curr(9), xi_curr(10), xi_curr(11), mahalanobis_distance};
            est_state_pub.publish(msg);
        }

        valid_GPS = 0;
        valid_scanner = 0;
        valid_IMU = 0;
        valid_DVL = 0;
        valid_depth_sensor = 0;

        // Let ROS handle all incoming messages in a callback function

        ros::spinOnce();

        // Sleep for the remaining time to hit our 10Hz target
        loop_rate.sleep();
    }

    return 0;
}