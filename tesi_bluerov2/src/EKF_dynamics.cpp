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
#include <rosbag/bag.h>

double u1 = 0.0;
double u2 = 0.0;
double u3 = 0.0;
double u4 = 0.0;
double u5 = 0.0;
double u6 = 0.0;

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
double var_tau_motor = 0.0;
double var_tau_p = 0.0;
double var_tau_q = 0.0;

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

rosbag::Bag bag;

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

double W = 0.0;

// Distanza di Mahalanobis
double mahalanobis_distance = 0.0;

bool is_prima_volta = true;
bool first_nan_print = true;

std::string GNC_status = "NOT_READY";

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

double wrapToPi(double x)
{
    x = fmod(x * 180 / M_PI + 180, 360);
    if (x < 0)
        x += 360;
    return (x - 180) * M_PI / 180;
}

//////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////// CALLBACKS //////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////

void GNCstatusCallback(const std_msgs::String::ConstPtr &msg) // CALLBACK che riceve lo stato del GNC
{
    GNC_status = msg->data;
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
        valid_IMU = 0;
    }
    else
    {
        phi_IMU = msg->data[0];
        theta_IMU = msg->data[1];
        psi_IMU = msg->data[2];
        valid_IMU = msg->data[3];
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
        u1 = 0.0;
        u2 = 0.0;
        u3 = 0.0;
        u4 = 0.0;
        u5 = 0.0;
        u6 = 0.0;
    }
    else
    {
        u1 = msg->data[0];
        u2 = msg->data[1];
        u3 = msg->data[2];
        u4 = msg->data[3];
        u5 = msg->data[4];
        u6 = msg->data[5];
    }
}

int main(int argc, char **argv)
{
    // Initialize the ROS system and become a node.
    ros::init(argc, argv, "EKF_dynamics");

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
    n.getParam("var_tau_motor", var_tau_motor);
    n.getParam("var_tau_p", var_tau_p);
    n.getParam("var_tau_q", var_tau_q);

    Eigen::Matrix<double, 6, 6> C_rb;
    Eigen::Matrix<double, 6, 6> C_a;
    Eigen::Matrix<double, 6, 6> C;
    Eigen::Matrix<double, 6, 6> D;
    Eigen::Matrix<double, 6, 1> G;
    Eigen::Matrix<double, 6, 6> Jacobian;
    Eigen::Matrix<double, 6, 1> eta_pred;
    Eigen::Matrix<double, 6, 1> nu_pred;

    double x_r = 0.1105;
    double y_r = 0.133;
    double c_45 = cos(M_PI / 4);

    Eigen::Matrix<double, 4, 6> B;
    B << c_45, c_45, -c_45, -c_45, 0.0, 0.0,
        -c_45, c_45, c_45, -c_45, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, -1.0, -1.0,
        -(x_r + y_r) * sqrt(2) / 2, (x_r + y_r) * sqrt(2) / 2, -(x_r + y_r) * sqrt(2) / 2, (x_r + y_r) * sqrt(2) / 2, 0.0, 0.0;

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

    std::string path = ros::package::getPath("tesi_bluerov2");
    bag.open(path + "/bag/ekf_dynamics.bag", rosbag::bagmode::Write);

    bool is_init = false;

    bool is_GPS_init = false;
    bool is_scanner_init = false;
    bool is_depth_init = false;
    bool is_IMU_init = false;

    Eigen::VectorXd var_sensors(11);
    var_sensors << var_x_GPS, var_y_GPS, var_x_scanner, var_y_scanner, var_z_depth_sensor, var_phi_IMU, var_theta_IMU, var_psi_IMU, var_u_DVL, var_v_DVL, var_w_DVL;

    Eigen::MatrixXd Q(8, 8);
    Q << var_tau_motor, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, var_tau_motor, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, var_tau_motor, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, var_tau_motor, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, var_tau_motor, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, var_tau_motor, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, var_tau_p, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, var_tau_q;

    // Set the loop rate
    ros::Rate loop_rate(freq);

    // Create a publisher object
    ros::Publisher est_state_pub = n.advertise<tesi_bluerov2::Floats>("state/est_state_EKF_topic", 1000);
    ros::Publisher publisher_gnc_status = n.advertise<std_msgs::String>("manager/GNC_status_requested_topic", 10); // publisher stato richiesto al GNC

    // Create subscriber objects

    ros::Subscriber gnc_status_sub = n.subscribe("manager/GNC_status_topic", 1, GNCstatusCallback); // sottoscrizione alla topic di stato del GNC
    ros::Subscriber tau_sub = n.subscribe("tau_topic", 1000, tau_callback);
    ros::Subscriber GPS_sub = n.subscribe("sensors/GPS_topic", 1000, GPSCallback);
    ros::Subscriber scanner_sub = n.subscribe("sensors/scanner_topic", 1000, ScannerCallback);
    ros::Subscriber IMU_sub = n.subscribe("sensors/IMU_topic", 1000, IMUCallback);
    ros::Subscriber DVL_sub = n.subscribe("sensors/DVL_topic", 1000, DVLCallback);
    ros::Subscriber depth_sub = n.subscribe("sensors/depth_sensor_topic", 1000, depthSensorCallback);

    tesi_bluerov2::Floats msg;

    while (ros::ok())
    {

        //////Kalman Filter///////

        ///////////////////////////////////////////////////////////////////////
        ///////////////// INITIALIZATION OF THE FILTER ////////////////////////
        ///////////////////////////////////////////////////////////////////////
        if (GNC_status == "NOT_READY")
        {
            std::string status_req = "NAVIGATION_READY";
            std_msgs::String msg;
            msg.data = status_req;

            // Publish the message on the "topic1" topic
            publisher_gnc_status.publish(msg);
        }
        else if (!is_init && GNC_status != "NOT_READY")
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

                is_IMU_init = true;
            }

            xi_curr(6) = 0.0 + gaussianNoise(0, var_u_DVL);
            xi_curr(7) = 0.0 + gaussianNoise(0, var_v_DVL);
            xi_curr(8) = 0.0 + gaussianNoise(0, var_w_DVL);
            xi_curr(9) = 0.0 + gaussianNoise(0, var_p_IMU);
            xi_curr(10) = 0.0 + gaussianNoise(0, var_q_IMU);
            xi_curr(11) = 0.0 + gaussianNoise(0, var_r_IMU);

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
                ROS_WARN_STREAM("EKF INITIALIZATION COMPLETED\n");

                msg.data = {xi_curr(0), xi_curr(1), xi_curr(2), xi_curr(3), xi_curr(4), xi_curr(5), xi_curr(6), xi_curr(7), xi_curr(8), xi_curr(9), xi_curr(10), xi_curr(11), mahalanobis_distance};
                est_state_pub.publish(msg);

                ////////////////////////////////////////////////////////////////////
                /////////////////////////PREDICTION/////////////////////////////////
                ////////////////////////////////////////////////////////////////////

                // VETTORE DI FORZE E MOMENTI
                Eigen::VectorXd inputs(6);
                inputs << u1, u2, u3, u4, u5, u6;

                // Calcolo delle forze e momenti controllabili (u,v,w,r)
                Eigen::VectorXd tau_motor(4);
                tau_motor = B * inputs;

                // Vettore delle forze e momenti totale
                Eigen::VectorXd tau(6);
                tau << tau_motor(0), tau_motor(1), tau_motor(2), 0.0, 0.0, tau_motor(3);

                phi = xi_curr(3);
                theta = xi_curr(4);
                psi = xi_curr(5);
                u = xi_curr(6);
                v = xi_curr(7);
                w = xi_curr(8);
                p = xi_curr(9);
                q = xi_curr(10);
                r = xi_curr(11);

                W = m * 9.81;

                Eigen::MatrixXd F_k(12, 12);
                F_k << 0.0, 0.0, 0.0, v * (sin(phi) * sin(psi) + cos(phi) * cos(psi) * sin(theta)) + w * (cos(phi) * sin(psi) - cos(psi) * sin(phi) * sin(theta)), cos(psi) * (w * cos(phi) * cos(theta) - u * sin(theta) + v * cos(theta) * sin(phi)), w * (cos(psi) * sin(phi) - cos(phi) * sin(psi) * sin(theta)) - v * (cos(phi) * cos(psi) + sin(phi) * sin(psi) * sin(theta)) - u * cos(theta) * sin(psi), cos(psi) * cos(theta), cos(psi) * sin(phi) * sin(theta) - cos(phi) * sin(psi), (sin(phi) * sin(psi) + cos(phi) * cos(psi) * sin(theta)), 0.0, 0.0, 0.0,
                    0.0, 0.0, 0.0, -v * (cos(psi) * sin(phi) - cos(phi) * sin(psi) * sin(theta)) - w * (cos(phi) * cos(psi) + sin(phi) * sin(psi) * sin(theta)), sin(psi) * (w * cos(phi) * cos(theta) - u * sin(theta) + v * cos(theta) * sin(phi)), w * (sin(phi) * sin(psi) + cos(phi) * cos(psi) * sin(theta)) - v * (cos(phi) * sin(psi) - cos(psi) * sin(phi) * sin(theta)) + u * cos(psi) * cos(theta), cos(theta) * sin(psi), (cos(phi) * cos(psi) + sin(phi) * sin(psi) * sin(theta)), cos(phi) * sin(psi) * sin(theta) - cos(psi) * sin(phi), 0.0, 0.0, 0.0,
                    0.0, 0.0, 0.0, cos(theta) * (v * cos(phi) - w * sin(phi)), -u * cos(theta) - w * cos(phi) * sin(theta) - v * sin(phi) * sin(theta), 0.0, -sin(theta), cos(theta) * sin(phi), cos(phi) * cos(theta), 0.0, 0.0, 0.0,
                    0.0, 0.0, 0.0, (sin(theta) * (q * cos(phi) - r * sin(phi))) / cos(theta), (r * cos(phi) + q * sin(phi)) / (cos(theta) * cos(theta)), 0.0, 0.0, 0.0, 0.0, 1.0, sin(phi) * tan(theta), cos(phi) * tan(theta),
                    0.0, 0.0, 0.0, -r * cos(phi) - q * sin(phi), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, cos(phi), -sin(phi),
                    0.0, 0.0, 0.0, (q * cos(phi) - r * sin(phi)) / cos(theta), -(sin(theta) * (r * cos(phi) + q * sin(phi))) / (sin(theta) * sin(theta) - 1), 0.0, 0.0, 0.0, 0.0, 0.0, sin(phi) / cos(theta), cos(phi) / cos(theta),
                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -(1000 * A_x * abs(u)) / (X_u_dot + m), -(r * (Y_v_dot - m)) / (X_u_dot + m), (q * (Z_w_dot - m)) / (X_u_dot + m), 0.0, (w * (Z_w_dot - m)) / (X_u_dot + m), -(v * (Y_v_dot - m)) / (X_u_dot + m),
                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, (r * (X_u_dot - m)) / (Y_v_dot + m), -(1000 * A_y * abs(v)) / (Y_v_dot + m), -(p * (Z_w_dot - m)) / (Y_v_dot + m), -(w * (Z_w_dot - m)) / (Y_v_dot + m), 0.0, (u * (X_u_dot - m)) / (Y_v_dot + m),
                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -(q * (X_u_dot - m)) / (Z_w_dot + m), (p * (Y_v_dot - m)) / (Z_w_dot + m), -(1000 * A_z * abs(w)) / (Z_w_dot + m), (v * (Y_v_dot - m)) / (Z_w_dot + m), -(u * (X_u_dot - m)) / (Z_w_dot + m), 0.0,
                    0.0, 0.0, 0.0, (W * cos(theta) * (z_b * cos(phi) + y_b * sin(phi))) / (I_x + K_p_dot), (W * sin(theta) * (y_b * cos(phi) - z_b * sin(phi))) / (I_x + K_p_dot), 0.0, 0.0, -(w * (Y_v_dot - Z_w_dot)) / (I_x + K_p_dot), -(v * (Y_v_dot - Z_w_dot)) / (I_x + K_p_dot), -(sign(p) * (1000 * A_p * p + K_p_dot * r * sign(p))) / (I_x + K_p_dot), (r * (I_y - I_z - M_q_dot + N_r_dot)) / (I_x + K_p_dot), -(K_p_dot * p - I_y * q + M_q_dot * q + q * (I_z - N_r_dot)) / (I_x + K_p_dot),
                    0.0, 0.0, 0.0, -(W * x_b * cos(theta) * sin(phi)) / (I_y + M_q_dot), (W * (z_b * cos(theta) - x_b * cos(phi) * sin(theta))) / (I_y + M_q_dot), 0.0, (w * (X_u_dot - Z_w_dot)) / (I_y + M_q_dot), 0.0, (u * (X_u_dot - Z_w_dot)) / (I_y + M_q_dot), -(r * (I_x - I_z - K_p_dot + N_r_dot)) / (I_y + M_q_dot), -(1000 * A_q * abs(q)) / (I_y + M_q_dot), -(p * (I_x - I_z - K_p_dot + N_r_dot)) / (I_y + M_q_dot),
                    0.0, 0.0, 0.0, -(W * x_b * cos(phi) * cos(theta)) / (I_z + N_r_dot), -(W * (y_b * cos(theta) - x_b * sin(phi) * sin(theta))) / (I_z + N_r_dot), 0.0, -(v * (X_u_dot - Y_v_dot)) / (I_z + N_r_dot), -(u * (X_u_dot - Y_v_dot)) / (I_z + N_r_dot), 0.0, (2 * K_p_dot * p - I_y * q + M_q_dot * q + q * (I_x - K_p_dot)) / (I_z + N_r_dot), (p * (I_x - I_y - K_p_dot + M_q_dot)) / (I_z + N_r_dot), -(1000 * A_r * abs(r)) / (I_z + N_r_dot);

                F_k = dt * F_k + Eigen::MatrixXd::Identity(12, 12);

                Eigen::MatrixXd D_k(12, 8);

                D_k << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                    sqrt(2) / (2 * (X_u_dot + m)), sqrt(2) / (2 * (X_u_dot + m)), -sqrt(2) / (2 * (X_u_dot + m)), -sqrt(2) / (2 * (X_u_dot + m)), 0.0, 0.0, 0.0, 0.0,
                    -sqrt(2) / (2 * (Y_v_dot + m)), sqrt(2) / (2 * (Y_v_dot + m)), sqrt(2) / (2 * (Y_v_dot + m)), -sqrt(2) / (2 * (Y_v_dot + m)), 0.0, 0.0, 0.0, 0.0,
                    0.0, 0.0, 0.0, 0.0, -1 / (Z_w_dot + m), -1 / (Z_w_dot + m), 0.0, 0.0,
                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1 / (I_x + K_p_dot), 0.0,
                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1 / (I_y + M_q_dot),
                    -6203456329040103 / (36028797018963968 * (I_z + N_r_dot)), 6203456329040103 / (36028797018963968 * (I_z + N_r_dot)), -6203456329040103 / (36028797018963968 * (I_z + N_r_dot)), 6203456329040103 / (36028797018963968 * (I_z + N_r_dot)), 0.0, 0.0, 0.0, 0.0;

                D_k = dt * D_k;

                C_rb << 0.0, 0.0, 0.0, m * (y_g * q + z_g * r), -m * (x_g * q - w), -m * (x_g * r + v),
                    0.0, 0.0, 0.0, -m * (y_g * p + w), m * (z_g * r + x_g * p), -m * (y_g * r - u),
                    0.0, 0.0, 0.0, -m * (z_g * p - v), -m * (z_g * q + u), m * (x_g * p + y_g * q),
                    -m * (y_g * q + z_g * r), m * (y_g * p + w), m * (z_g * p - v), 0.0, I_z * r - I_yz * q - I_xz * p, I_xy * p + I_yz * r - I_y * q,
                    m * (x_g * q - w), -m * (z_g * r + x_g * p), m * (z_g * q + u), I_yz * q + I_xz * p - I_z * r, 0.0, -I_xz * r - I_xy * q + I_x * p,
                    m * (x_g * r + v), m * (y_g * r - u), -m * (x_g * p + y_g * q), -I_yz * r - I_xy * p + I_y * q, I_xy * q + I_xz * r - I_x * p, 0.0;

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

                C = C_rb + C_a;

                D << A_x * abs(u), 0.0, 0.0, 0.0, 0.0, 0.0,
                    0.0, A_y * abs(v), 0.0, 0.0, 0.0, 0.0,
                    0.0, 0.0, A_z * abs(w), 0.0, 0.0, 0.0,
                    0.0, 0.0, 0.0, A_p * abs(p), 0.0, 0.0,
                    0.0, 0.0, 0.0, 0.0, A_q * abs(q), 0.0,
                    0.0, 0.0, 0.0, 0.0, 0.0, A_r * abs(r);

                D = 0.5 * 1000 * D;

                // VETTORE DEI TERMINI GRAVITAZIONALI
                G << 0.0,
                    0.0,
                    0.0,
                    -(y_g - y_b) * W * cos(phi) * cos(theta) + (z_g - z_b) * W * sin(phi) * cos(theta),
                    (z_g - z_b) * W * sin(theta) + (x_g - x_b) * W * cos(theta) * cos(phi),
                    -(x_g - x_b) * W * cos(theta) * sin(phi) - (y_g - y_b) * W * sin(theta);

                if (xi_curr(2) < 0.0)
                {
                    G(2) = -W;
                }

                Jacobian << cos(psi) * cos(theta), cos(psi) * sin(phi) * sin(theta) - cos(phi) * sin(psi), sin(phi) * sin(psi) + cos(phi) * cos(psi) * sin(theta), 0.0, 0.0, 0.0,
                    cos(theta) * sin(psi), cos(phi) * cos(psi) + sin(phi) * sin(psi) * sin(theta), cos(phi) * sin(psi) * sin(theta) - cos(psi) * sin(phi), 0.0, 0.0, 0.0,
                    -sin(theta), cos(theta) * sin(phi), cos(phi) * cos(theta), 0.0, 0.0, 0.0,
                    0.0, 0.0, 0.0, 1.0, sin(phi) * tan(theta), cos(phi) * tan(theta),
                    0.0, 0.0, 0.0, 0.0, cos(phi), -sin(phi),
                    0.0, 0.0, 0.0, 0.0, sin(phi) / cos(theta), cos(phi) / cos(theta);

                eta_pred = dt * Jacobian * xi_curr.tail(6) + xi_curr.head(6);
                nu_pred = dt * M.inverse() * (tau - C * xi_curr.tail(6) - D * xi_curr.tail(6) - G) + xi_curr.tail(6);
                xi_pred << eta_pred, nu_pred;

                // wrapToPi
                xi_pred(3) = atan2(sin(xi_pred(3)), cos(xi_pred(3)));
                xi_pred(4) = atan2(sin(xi_pred(4)), cos(xi_pred(4)));
                xi_pred(5) = atan2(sin(xi_pred(5)), cos(xi_pred(5)));

                P_pred = F_k * P_curr * F_k.transpose() + D_k * Q * D_k.transpose();
            }
        }
        else
        {
            ///////////////////////////////////////////////////////////////////////
            ///////////////////////////// CORRECTION //////////////////////////////
            ///////////////////////////////////////////////////////////////////////

            // VETTORE DI MISURA
            Eigen::VectorXd z(11);
            z << x_GPS, y_GPS, x_scanner, y_scanner, z_depth_sensor, phi_IMU, theta_IMU, psi_IMU, u_DVL, v_DVL, w_DVL;

            Eigen::VectorXd valid(11);
            valid << valid_GPS, valid_GPS, valid_scanner, valid_scanner, valid_depth_sensor, valid_IMU, valid_IMU, valid_IMU, valid_DVL, valid_DVL, valid_DVL;

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

                for (int i = 3; i < 6; i++)
                {
                    xi_corr(i) = wrapToPi(xi_corr(i));
                }

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
            Eigen::VectorXd inputs(6);
            inputs << u1, u2, u3, u4, u5, u6;

            // Calcolo delle forze e momenti controllabili (u,v,w,r)
            Eigen::VectorXd tau_motor(4);
            tau_motor = B * inputs;

            // Vettore delle forze e momenti totale
            Eigen::VectorXd tau(6);
            tau << tau_motor(0), tau_motor(1), tau_motor(2), 0.0, 0.0, tau_motor(3);

            phi = xi_curr(3);
            theta = xi_curr(4);
            psi = xi_curr(5);
            u = xi_curr(6);
            v = xi_curr(7);
            w = xi_curr(8);
            p = xi_curr(9);
            q = xi_curr(10);
            r = xi_curr(11);

            W = m * 9.81;

            Eigen::MatrixXd F_k(12, 12);
            F_k << 0.0, 0.0, 0.0, v * (sin(phi) * sin(psi) + cos(phi) * cos(psi) * sin(theta)) + w * (cos(phi) * sin(psi) - cos(psi) * sin(phi) * sin(theta)), cos(psi) * (w * cos(phi) * cos(theta) - u * sin(theta) + v * cos(theta) * sin(phi)), w * (cos(psi) * sin(phi) - cos(phi) * sin(psi) * sin(theta)) - v * (cos(phi) * cos(psi) + sin(phi) * sin(psi) * sin(theta)) - u * cos(theta) * sin(psi), cos(psi) * cos(theta), cos(psi) * sin(phi) * sin(theta) - cos(phi) * sin(psi), (sin(phi) * sin(psi) + cos(phi) * cos(psi) * sin(theta)), 0.0, 0.0, 0.0,
                0.0, 0.0, 0.0, -v * (cos(psi) * sin(phi) - cos(phi) * sin(psi) * sin(theta)) - w * (cos(phi) * cos(psi) + sin(phi) * sin(psi) * sin(theta)), sin(psi) * (w * cos(phi) * cos(theta) - u * sin(theta) + v * cos(theta) * sin(phi)), w * (sin(phi) * sin(psi) + cos(phi) * cos(psi) * sin(theta)) - v * (cos(phi) * sin(psi) - cos(psi) * sin(phi) * sin(theta)) + u * cos(psi) * cos(theta), cos(theta) * sin(psi), (cos(phi) * cos(psi) + sin(phi) * sin(psi) * sin(theta)), cos(phi) * sin(psi) * sin(theta) - cos(psi) * sin(phi), 0.0, 0.0, 0.0,
                0.0, 0.0, 0.0, cos(theta) * (v * cos(phi) - w * sin(phi)), -u * cos(theta) - w * cos(phi) * sin(theta) - v * sin(phi) * sin(theta), 0.0, -sin(theta), cos(theta) * sin(phi), cos(phi) * cos(theta), 0.0, 0.0, 0.0,
                0.0, 0.0, 0.0, (sin(theta) * (q * cos(phi) - r * sin(phi))) / cos(theta), (r * cos(phi) + q * sin(phi)) / (cos(theta) * cos(theta)), 0.0, 0.0, 0.0, 0.0, 1.0, sin(phi) * tan(theta), cos(phi) * tan(theta),
                0.0, 0.0, 0.0, -r * cos(phi) - q * sin(phi), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, cos(phi), -sin(phi),
                0.0, 0.0, 0.0, (q * cos(phi) - r * sin(phi)) / cos(theta), -(sin(theta) * (r * cos(phi) + q * sin(phi))) / (sin(theta) * sin(theta) - 1), 0.0, 0.0, 0.0, 0.0, 0.0, sin(phi) / cos(theta), cos(phi) / cos(theta),
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -(1000 * A_x * abs(u)) / (X_u_dot + m), -(r * (Y_v_dot - m)) / (X_u_dot + m), (q * (Z_w_dot - m)) / (X_u_dot + m), 0.0, (w * (Z_w_dot - m)) / (X_u_dot + m), -(v * (Y_v_dot - m)) / (X_u_dot + m),
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, (r * (X_u_dot - m)) / (Y_v_dot + m), -(1000 * A_y * abs(v)) / (Y_v_dot + m), -(p * (Z_w_dot - m)) / (Y_v_dot + m), -(w * (Z_w_dot - m)) / (Y_v_dot + m), 0.0, (u * (X_u_dot - m)) / (Y_v_dot + m),
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -(q * (X_u_dot - m)) / (Z_w_dot + m), (p * (Y_v_dot - m)) / (Z_w_dot + m), -(1000 * A_z * abs(w)) / (Z_w_dot + m), (v * (Y_v_dot - m)) / (Z_w_dot + m), -(u * (X_u_dot - m)) / (Z_w_dot + m), 0.0,
                0.0, 0.0, 0.0, (W * cos(theta) * (z_b * cos(phi) + y_b * sin(phi))) / (I_x + K_p_dot), (W * sin(theta) * (y_b * cos(phi) - z_b * sin(phi))) / (I_x + K_p_dot), 0.0, 0.0, -(w * (Y_v_dot - Z_w_dot)) / (I_x + K_p_dot), -(v * (Y_v_dot - Z_w_dot)) / (I_x + K_p_dot), -(sign(p) * (1000 * A_p * p + K_p_dot * r * sign(p))) / (I_x + K_p_dot), (r * (I_y - I_z - M_q_dot + N_r_dot)) / (I_x + K_p_dot), -(K_p_dot * p - I_y * q + M_q_dot * q + q * (I_z - N_r_dot)) / (I_x + K_p_dot),
                0.0, 0.0, 0.0, -(W * x_b * cos(theta) * sin(phi)) / (I_y + M_q_dot), (W * (z_b * cos(theta) - x_b * cos(phi) * sin(theta))) / (I_y + M_q_dot), 0.0, (w * (X_u_dot - Z_w_dot)) / (I_y + M_q_dot), 0.0, (u * (X_u_dot - Z_w_dot)) / (I_y + M_q_dot), -(r * (I_x - I_z - K_p_dot + N_r_dot)) / (I_y + M_q_dot), -(1000 * A_q * abs(q)) / (I_y + M_q_dot), -(p * (I_x - I_z - K_p_dot + N_r_dot)) / (I_y + M_q_dot),
                0.0, 0.0, 0.0, -(W * x_b * cos(phi) * cos(theta)) / (I_z + N_r_dot), -(W * (y_b * cos(theta) - x_b * sin(phi) * sin(theta))) / (I_z + N_r_dot), 0.0, -(v * (X_u_dot - Y_v_dot)) / (I_z + N_r_dot), -(u * (X_u_dot - Y_v_dot)) / (I_z + N_r_dot), 0.0, (2 * K_p_dot * p - I_y * q + M_q_dot * q + q * (I_x - K_p_dot)) / (I_z + N_r_dot), (p * (I_x - I_y - K_p_dot + M_q_dot)) / (I_z + N_r_dot), -(1000 * A_r * abs(r)) / (I_z + N_r_dot);

            F_k = dt * F_k + Eigen::MatrixXd::Identity(12, 12);

            Eigen::MatrixXd D_k(12, 8);

            D_k << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                sqrt(2) / (2 * (X_u_dot + m)), sqrt(2) / (2 * (X_u_dot + m)), -sqrt(2) / (2 * (X_u_dot + m)), -sqrt(2) / (2 * (X_u_dot + m)), 0.0, 0.0, 0.0, 0.0,
                -sqrt(2) / (2 * (Y_v_dot + m)), sqrt(2) / (2 * (Y_v_dot + m)), sqrt(2) / (2 * (Y_v_dot + m)), -sqrt(2) / (2 * (Y_v_dot + m)), 0.0, 0.0, 0.0, 0.0,
                0.0, 0.0, 0.0, 0.0, -1 / (Z_w_dot + m), -1 / (Z_w_dot + m), 0.0, 0.0,
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1 / (I_x + K_p_dot), 0.0,
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1 / (I_y + M_q_dot),
                -6203456329040103 / (36028797018963968 * (I_z + N_r_dot)), 6203456329040103 / (36028797018963968 * (I_z + N_r_dot)), -6203456329040103 / (36028797018963968 * (I_z + N_r_dot)), 6203456329040103 / (36028797018963968 * (I_z + N_r_dot)), 0.0, 0.0, 0.0, 0.0;

            D_k = dt * D_k;

            C_rb << 0.0, 0.0, 0.0, m * (y_g * q + z_g * r), -m * (x_g * q - w), -m * (x_g * r + v),
                0.0, 0.0, 0.0, -m * (y_g * p + w), m * (z_g * r + x_g * p), -m * (y_g * r - u),
                0.0, 0.0, 0.0, -m * (z_g * p - v), -m * (z_g * q + u), m * (x_g * p + y_g * q),
                -m * (y_g * q + z_g * r), m * (y_g * p + w), m * (z_g * p - v), 0.0, I_z * r - I_yz * q - I_xz * p, I_xy * p + I_yz * r - I_y * q,
                m * (x_g * q - w), -m * (z_g * r + x_g * p), m * (z_g * q + u), I_yz * q + I_xz * p - I_z * r, 0.0, -I_xz * r - I_xy * q + I_x * p,
                m * (x_g * r + v), m * (y_g * r - u), -m * (x_g * p + y_g * q), -I_yz * r - I_xy * p + I_y * q, I_xy * q + I_xz * r - I_x * p, 0.0;

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

            C = C_rb + C_a;

            D << A_x * abs(u), 0.0, 0.0, 0.0, 0.0, 0.0,
                0.0, A_y * abs(v), 0.0, 0.0, 0.0, 0.0,
                0.0, 0.0, A_z * abs(w), 0.0, 0.0, 0.0,
                0.0, 0.0, 0.0, A_p * abs(p), 0.0, 0.0,
                0.0, 0.0, 0.0, 0.0, A_q * abs(q), 0.0,
                0.0, 0.0, 0.0, 0.0, 0.0, A_r * abs(r);

            D = 0.5 * 1000 * D;

            // VETTORE DEI TERMINI GRAVITAZIONALI
            G << 0.0,
                0.0,
                0.0,
                -(y_g - y_b) * W * cos(phi) * cos(theta) + (z_g - z_b) * W * sin(phi) * cos(theta),
                (z_g - z_b) * W * sin(theta) + (x_g - x_b) * W * cos(theta) * cos(phi),
                -(x_g - x_b) * W * cos(theta) * sin(phi) - (y_g - y_b) * W * sin(theta);

            if (xi_curr(2) < 0.0)
            {
                G(2) = -W;
            }

            Jacobian << cos(psi) * cos(theta), cos(psi) * sin(phi) * sin(theta) - cos(phi) * sin(psi), sin(phi) * sin(psi) + cos(phi) * cos(psi) * sin(theta), 0.0, 0.0, 0.0,
                cos(theta) * sin(psi), cos(phi) * cos(psi) + sin(phi) * sin(psi) * sin(theta), cos(phi) * sin(psi) * sin(theta) - cos(psi) * sin(phi), 0.0, 0.0, 0.0,
                -sin(theta), cos(theta) * sin(phi), cos(phi) * cos(theta), 0.0, 0.0, 0.0,
                0.0, 0.0, 0.0, 1.0, sin(phi) * tan(theta), cos(phi) * tan(theta),
                0.0, 0.0, 0.0, 0.0, cos(phi), -sin(phi),
                0.0, 0.0, 0.0, 0.0, sin(phi) / cos(theta), cos(phi) / cos(theta);

            eta_pred = dt * Jacobian * xi_curr.tail(6) + xi_curr.head(6);
            nu_pred = dt * M.inverse() * (tau - C * xi_curr.tail(6) - D * xi_curr.tail(6) - G) + xi_curr.tail(6);
            xi_pred << eta_pred, nu_pred;

            // wrapToPi
            xi_pred(3) = wrapToPi(xi_pred(3));
            xi_pred(4) = wrapToPi(xi_pred(4));
            xi_pred(5) = wrapToPi(xi_pred(5));

            P_pred = F_k * P_curr * F_k.transpose() + D_k * Q * D_k.transpose();

            if (xi_curr(0) != xi_curr(0))
            {
                if (first_nan_print)
                {
                    ROS_WARN("NaN EKF Dynamics (no imu) error");
                    first_nan_print = false;
                }
            }

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
        if (ros::Time::now().toSec() > ros::TIME_MIN.toSec())
        {
            bag.write("state/est_state_EKF_topic", ros::Time::now(), msg);
        }

        ros::spinOnce();

        // Sleep for the remaining time to hit our 10Hz target
        loop_rate.sleep();
    }
    bag.close();

    return 0;
}