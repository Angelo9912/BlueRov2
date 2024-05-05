#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include <ros/console.h>
#include <eigen3/Eigen/Dense>     // for eigen matrix
#include "tesi_bluerov2/Floats.h" // for accessing -- tesi_bluerov2 Floats()
#include <vector>
#include "yaml-cpp/yaml.h" // for yaml
#include <random>
#include <rosbag/bag.h>

double a_u = 0.0;
double a_v = 0.0;
double a_w = 0.0;

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
double p_IMU_camera = 0.0;
double q_IMU_camera = 0.0;
double r_IMU_camera = 0.0;
double z_depth_sensor = 0.0;

double valid_GPS = 0;
double valid_scanner = 0;
double valid_IMU = 0;
double valid_DVL = 0;
double valid_depth_sensor = 0;
double valid_IMU_camera = 0;

// Covariance values (process noise)
double var_acc_u = 0.0;
double var_acc_v = 0.0;
double var_acc_w = 0.0;
double var_model_p = 0.0;
double var_model_q = 0.0;
double var_model_r = 0.0;
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
double var_p_IMU_camera = 0.0;
double var_q_IMU_camera = 0.0;
double var_r_IMU_camera = 0.0;

rosbag::Bag bag;

std::string GNC_status = "";

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

double wrapToPi(double x)
{
    x = fmod(x * 180 / M_PI + 180, 360);
    if (x < 0)
        x += 360;
    return (x - 180) * M_PI / 180;
}

// Callback function for the subscriber
void GNCstatusCallback(const std_msgs::String::ConstPtr &msg) // CALLBACK che riceve lo stato del GNC
{
    GNC_status = msg->data;
}

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
void IMU_cameraCallback(const tesi_bluerov2::Floats::ConstPtr &msg)
{
    if (msg->data[0] != msg->data[0])
    {
        p_IMU_camera = 0.0;
        q_IMU_camera = 0.0;
        r_IMU_camera = 0.0;
        valid_IMU_camera = 0;
    }
    else
    {
        p_IMU_camera = msg->data[0];
        q_IMU_camera = msg->data[1];
        r_IMU_camera = msg->data[2];
        valid_IMU_camera = msg->data[3];
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

void acc_callback(const tesi_bluerov2::Floats::ConstPtr &msg)
{
    if (msg->data[0] != msg->data[0])
    {
        a_u = 0.0;
        a_v = 0.0;
        a_w = 0.0;
    }
    else
    {
        a_u = msg->data[0];
        a_v = msg->data[1];
        a_w = msg->data[2];
    }
}

int main(int argc, char **argv)
{
    // Initialize the ROS system and become a node.
    ros::init(argc, argv, "EKF");

    // Create a ROS node handle
    ros::NodeHandle n;
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
    n.getParam("var_p_IMU_camera", var_p_IMU_camera);
    n.getParam("var_q_IMU_camera", var_q_IMU_camera);
    n.getParam("var_r_IMU_camera", var_r_IMU_camera);

    n.getParam("var_acc_u", var_acc_u);
    n.getParam("var_acc_v", var_acc_v);
    n.getParam("var_acc_w", var_acc_w);
    n.getParam("var_model_p", var_model_p);
    n.getParam("var_model_q", var_model_q);
    n.getParam("var_model_r", var_model_r);
    n.getParam("var_tau_p", var_tau_p);
    n.getParam("var_tau_q", var_tau_q);
    n.getParam("var_tau_r", var_tau_r);

    Eigen::Matrix<double, 6, 6> Jacobian;
    Eigen::Matrix<double, 6, 1> eta_pred;
    Eigen::Matrix<double, 6, 1> nu_pred;

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

    // Time step
    double freq = 50;
    double dt = 1 / freq;

    std::string path = ros::package::getPath("tesi_bluerov2");
    bag.open(path + "/bag/ekf_kinematics_imu.bag", rosbag::bagmode::Write);

    bool is_init = false;

    bool is_GPS_init = false;
    bool is_scanner_init = false;
    bool is_depth_init = false;
    bool is_IMU_init = false;
    bool is_IMU_camera_init = false;

    Eigen::VectorXd var_sensors(11);
    var_sensors << var_x_GPS, var_y_GPS, var_x_scanner, var_y_scanner, var_z_depth_sensor, var_phi_IMU, var_theta_IMU, var_psi_IMU, var_u_DVL, var_v_DVL, var_w_DVL;

    Eigen::MatrixXd Q(6, 6);
    Q << var_acc_u, 0, 0, 0, 0, 0,
        0, var_acc_v, 0, 0, 0, 0,
        0, 0, var_acc_w, 0, 0, 0,
        0, 0, 0, var_p_IMU_camera, 0, 0,
        0, 0, 0, 0, var_q_IMU_camera, 0,
        0, 0, 0, 0, 0, var_r_IMU_camera;

    // Set the loop rate
    ros::Rate loop_rate(freq);

    // Create a publisher object
    ros::Publisher est_state_pub = n.advertise<tesi_bluerov2::Floats>("state/est_state_topic_no_dyn_imu", 1000);
    ros::Publisher publisher_gnc_status = n.advertise<std_msgs::String>("manager/GNC_status_requested_topic", 10); // publisher stato richiesto al GNC

    // Create subscriber objects

    ros::Subscriber gnc_status_sub = n.subscribe("manager/GNC_status_topic", 1, GNCstatusCallback); // sottoscrizione alla topic di stato del GNC
    ros::Subscriber acc_sub = n.subscribe("sensors/acc_topic", 1000, acc_callback);

    ros::Subscriber GPS_sub = n.subscribe("sensors/GPS_topic", 1000, GPSCallback);

    ros::Subscriber scanner_sub = n.subscribe("sensors/scanner_topic", 1000, ScannerCallback);

    ros::Subscriber IMU_sub = n.subscribe("sensors/IMU_topic", 1000, IMUCallback);
    ros::Subscriber IMU_camera_sub = n.subscribe("sensors/IMU_camera_topic", 1000, IMU_cameraCallback);

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

            if (valid_IMU_camera == 1.0)
            {
                xi_curr(9) = p_IMU_camera;
                xi_curr(10) = q_IMU_camera;
                xi_curr(11) = r_IMU_camera;

                is_IMU_camera_init = true;
            }

            xi_curr.setZero();

            // xi_curr(6) = 0.0 + gaussianNoise(0, var_u_DVL);
            // xi_curr(7) = 0.0 + gaussianNoise(0, var_v_DVL);
            // xi_curr(8) = 0.0 + gaussianNoise(0, var_w_DVL);
            // xi_curr(9) = 0.0 + gaussianNoise(0, var_p_IMU);
            // xi_curr(10) = 0.0 + gaussianNoise(0, var_q_IMU);
            // xi_curr(11) = 0.0 + gaussianNoise(0, var_r_IMU);

            P_curr << var_x, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                0, var_y, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                0, 0, var_z_depth_sensor, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                0, 0, 0, var_phi_IMU, 0, 0, 0, 0, 0, 0, 0, 0,
                0, 0, 0, 0, var_theta_IMU, 0, 0, 0, 0, 0, 0, 0,
                0, 0, 0, 0, 0, var_psi_IMU, 0, 0, 0, 0, 0, 0,
                0, 0, 0, 0, 0, 0, var_u_DVL, 0, 0, 0, 0, 0,
                0, 0, 0, 0, 0, 0, 0, var_v_DVL, 0, 0, 0, 0,
                0, 0, 0, 0, 0, 0, 0, 0, var_w_DVL, 0, 0, 0,
                0, 0, 0, 0, 0, 0, 0, 0, 0, var_p_IMU_camera, 0, 0,
                0, 0, 0, 0, 0, 0, 0, 0, 0, 0, var_q_IMU_camera, 0,
                0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, var_r_IMU_camera;

            /*ROS_WARN_STREAM("EKF (no dyn) nu_0: \n"
                            << xi_curr(6) << "\n"
                            << xi_curr(7) << "\n"
                            << xi_curr(8) << "\n"
                            << "\n");*/

            is_init = ((is_GPS_init || is_scanner_init) && is_depth_init && is_IMU_init && is_IMU_camera_init);

            if (is_init) // Se l'inizializzazione è completata pubblica lo stato stimato iniziale e passa alla predizione
            {
                ROS_WARN_STREAM("EKF (no dyn) INITIALIZATION COMPLETED\n");

                msg.data = {xi_curr(0), xi_curr(1), xi_curr(2), xi_curr(3), xi_curr(4), xi_curr(5), xi_curr(6), xi_curr(7), xi_curr(8), xi_curr(9), xi_curr(10), xi_curr(11), mahalanobis_distance};
                est_state_pub.publish(msg);

                ////////////////////////////////////////////////////////////////////
                /////////////////////////PREDICTION/////////////////////////////////
                ////////////////////////////////////////////////////////////////////

                // VETTORE DI FORZE E MOMENTI

                phi = xi_curr(3);
                theta = xi_curr(4);
                psi = xi_curr(5);
                u = xi_curr(6);
                v = xi_curr(7);
                w = xi_curr(8);
                p = xi_curr(9);
                q = xi_curr(10);
                r = xi_curr(11);

                Eigen::MatrixXd F_k(12, 12);
                F_k << 1.0, 0.0, 0.0, (dt * (sin(phi) * sin(psi) + cos(phi) * cos(psi) * sin(theta)) * (2 * v + a_v * dt)) / 2 + (dt * (cos(phi) * sin(psi) - cos(psi) * sin(phi) * sin(theta)) * (2 * w + a_w * dt)) / 2, (dt * cos(phi) * cos(psi) * cos(theta) * (2 * w + a_w * dt)) / 2 - (dt * cos(psi) * sin(theta) * (2 * u + a_u * dt)) / 2 + (dt * cos(psi) * cos(theta) * sin(phi) * (2 * v + a_v * dt)) / 2, (dt * (cos(psi) * sin(phi) - cos(phi) * sin(psi) * sin(theta)) * (2 * w + a_w * dt)) / 2 - (dt * (cos(phi) * cos(psi) + sin(phi) * sin(psi) * sin(theta)) * (2 * v + a_v * dt)) / 2 - (dt * cos(theta) * sin(psi) * (2 * u + a_u * dt)) / 2, dt * cos(psi) * cos(theta), dt * cos(psi) * sin(phi) * sin(theta) - dt * cos(phi) * sin(psi), dt * sin(phi) * sin(psi) + dt * cos(phi) * cos(psi) * sin(theta), 0.0, 0.0, 0.0,
                    0.0, 1.0, 0.0, -(dt * (cos(psi) * sin(phi) - cos(phi) * sin(psi) * sin(theta)) * (2 * v + a_v * dt)) / 2 - (dt * (cos(phi) * cos(psi) + sin(phi) * sin(psi) * sin(theta)) * (2 * w + a_w * dt)) / 2, (dt * cos(phi) * cos(theta) * sin(psi) * (2 * w + a_w * dt)) / 2 - (dt * sin(psi) * sin(theta) * (2 * u + a_u * dt)) / 2 + (dt * cos(theta) * sin(phi) * sin(psi) * (2 * v + a_v * dt)) / 2, (dt * (sin(phi) * sin(psi) + cos(phi) * cos(psi) * sin(theta)) * (2 * w + a_w * dt)) / 2 - (dt * (cos(phi) * sin(psi) - cos(psi) * sin(phi) * sin(theta)) * (2 * v + a_v * dt)) / 2 + (dt * cos(psi) * cos(theta) * (2 * u + a_u * dt)) / 2, dt * cos(theta) * sin(psi), dt * cos(phi) * cos(psi) + dt * sin(phi) * sin(psi) * sin(theta), dt * cos(phi) * sin(psi) * sin(theta) - dt * cos(psi) * sin(phi), 0.0, 0.0, 0.0,
                    0.0, 0.0, 1.0, (dt * cos(phi) * cos(theta) * (2 * v + a_v * dt)) / 2 - (dt * cos(theta) * sin(phi) * (2 * w + a_w * dt)) / 2, -(dt * cos(theta) * (2 * u + a_u * dt)) / 2 - (dt * cos(phi) * sin(theta) * (2 * w + a_w * dt)) / 2 - (dt * sin(phi) * sin(theta) * (2 * v + a_v * dt)) / 2, 0.0, -dt * sin(theta), dt * cos(theta) * sin(phi), dt * cos(phi) * cos(theta), 0.0, 0.0, 0.0,
                    0.0, 0.0, 0.0, dt * q * cos(phi) * tan(theta) - dt * r * sin(phi) * tan(theta) + 1, (dt * (r * cos(phi) + q * sin(phi))) / (cos(theta) * cos(theta)), 0.0, 0.0, 0.0, 0.0, dt, dt * sin(phi) * tan(theta), dt * cos(phi) * tan(theta),
                    0.0, 0.0, 0.0, -dt * (r * cos(phi) + q * sin(phi)), 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, dt * cos(phi), -dt * sin(phi),
                    0.0, 0.0, 0.0, (dt * (q * cos(phi) - r * sin(phi))) / cos(theta), (dt * sin(theta) * (r * (2 * sin(phi / 2) * sin(phi / 2) - 1) - q * sin(phi))) / (sin(theta) * sin(theta) - 1), 1.0, 0.0, 0.0, 0.0, 0.0, (dt * sin(phi)) / cos(theta), (dt * cos(phi)) / cos(theta),
                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0;

                Eigen::MatrixXd D_k(12, 6);

                D_k << (dt * dt * cos(psi) * cos(theta)) / 2, -(dt * dt * (cos(phi) * sin(psi) - cos(psi) * sin(phi) * sin(theta))) / 2, (dt * dt * (sin(phi) * sin(psi) + cos(phi) * cos(psi) * sin(theta))) / 2, 0.0, 0.0, 0.0,
                    (dt * dt * cos(theta) * sin(psi)) / 2, (dt * dt * (cos(phi) * cos(psi) + sin(phi) * sin(psi) * sin(theta))) / 2, -(dt * dt * (cos(psi) * sin(phi) - cos(phi) * sin(psi) * sin(theta))) / 2, 0.0, 0.0, 0.0,
                    -(dt * dt * sin(theta)) / 2, (dt * dt * cos(theta) * sin(phi)) / 2, (dt * dt * cos(phi) * cos(theta)) / 2, 0.0, 0.0, 0.0,
                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                    dt, 0.0, 0.0, 0.0, 0.0, 0.0,
                    0.0, dt, 0.0, 0.0, 0.0, 0.0,
                    0.0, 0.0, dt, 0.0, 0.0, 0.0,
                    0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
                    0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
                    0.0, 0.0, 0.0, 0.0, 0.0, 1.0;

                Jacobian << cos(psi) * cos(theta), cos(psi) * sin(phi) * sin(theta) - cos(phi) * sin(psi), sin(phi) * sin(psi) + cos(phi) * cos(psi) * sin(theta), 0, 0, 0,
                    cos(theta) * sin(psi), cos(phi) * cos(psi) + sin(phi) * sin(psi) * sin(theta), cos(phi) * sin(psi) * sin(theta) - cos(psi) * sin(phi), 0, 0, 0,
                    -sin(theta), cos(theta) * sin(phi), cos(phi) * cos(theta), 0, 0, 0,
                    0, 0, 0, 1, sin(phi) * tan(theta), cos(phi) * tan(theta),
                    0, 0, 0, 0, cos(phi), -sin(phi),
                    0, 0, 0, 0, sin(phi) / cos(theta), cos(phi) / cos(theta);

                Eigen::VectorXd a_m(6);
                a_m << a_u, a_v, a_w, 0, 0, 0;

                eta_pred = Jacobian * (dt * xi_curr.tail(6) + dt * dt / 2 * a_m) + xi_curr.head(6);

                double u_pred = dt * a_u + u;
                double v_pred = dt * a_v + v;
                double w_pred = dt * a_w + w;
                double p_pred = p_IMU_camera;
                double q_pred = q_IMU_camera;
                double r_pred = r_IMU_camera;

                nu_pred << u_pred, v_pred, w_pred, p_pred, q_pred, r_pred;
                xi_pred << eta_pred, nu_pred;

                // wrapToPi
                xi_pred(3) = wrapToPi(xi_pred(3));
                xi_pred(4) = wrapToPi(xi_pred(4));
                xi_pred(5) = wrapToPi(xi_pred(5));
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

            phi = xi_curr(3);
            theta = xi_curr(4);
            psi = xi_curr(5);
            u = xi_curr(6);
            v = xi_curr(7);
            w = xi_curr(8);
            p = xi_curr(9);
            q = xi_curr(10);
            r = xi_curr(11);

            Eigen::MatrixXd F_k(12, 12);

            F_k << 1.0, 0.0, 0.0, (dt * (sin(phi) * sin(psi) + cos(phi) * cos(psi) * sin(theta)) * (2 * v + a_v * dt)) / 2 + (dt * (cos(phi) * sin(psi) - cos(psi) * sin(phi) * sin(theta)) * (2 * w + a_w * dt)) / 2, (dt * cos(phi) * cos(psi) * cos(theta) * (2 * w + a_w * dt)) / 2 - (dt * cos(psi) * sin(theta) * (2 * u + a_u * dt)) / 2 + (dt * cos(psi) * cos(theta) * sin(phi) * (2 * v + a_v * dt)) / 2, (dt * (cos(psi) * sin(phi) - cos(phi) * sin(psi) * sin(theta)) * (2 * w + a_w * dt)) / 2 - (dt * (cos(phi) * cos(psi) + sin(phi) * sin(psi) * sin(theta)) * (2 * v + a_v * dt)) / 2 - (dt * cos(theta) * sin(psi) * (2 * u + a_u * dt)) / 2, dt * cos(psi) * cos(theta), dt * cos(psi) * sin(phi) * sin(theta) - dt * cos(phi) * sin(psi), dt * sin(phi) * sin(psi) + dt * cos(phi) * cos(psi) * sin(theta), 0.0, 0.0, 0.0,
                0.0, 1.0, 0.0, -(dt * (cos(psi) * sin(phi) - cos(phi) * sin(psi) * sin(theta)) * (2 * v + a_v * dt)) / 2 - (dt * (cos(phi) * cos(psi) + sin(phi) * sin(psi) * sin(theta)) * (2 * w + a_w * dt)) / 2, (dt * cos(phi) * cos(theta) * sin(psi) * (2 * w + a_w * dt)) / 2 - (dt * sin(psi) * sin(theta) * (2 * u + a_u * dt)) / 2 + (dt * cos(theta) * sin(phi) * sin(psi) * (2 * v + a_v * dt)) / 2, (dt * (sin(phi) * sin(psi) + cos(phi) * cos(psi) * sin(theta)) * (2 * w + a_w * dt)) / 2 - (dt * (cos(phi) * sin(psi) - cos(psi) * sin(phi) * sin(theta)) * (2 * v + a_v * dt)) / 2 + (dt * cos(psi) * cos(theta) * (2 * u + a_u * dt)) / 2, dt * cos(theta) * sin(psi), dt * cos(phi) * cos(psi) + dt * sin(phi) * sin(psi) * sin(theta), dt * cos(phi) * sin(psi) * sin(theta) - dt * cos(psi) * sin(phi), 0.0, 0.0, 0.0,
                0.0, 0.0, 1.0, (dt * cos(phi) * cos(theta) * (2 * v + a_v * dt)) / 2 - (dt * cos(theta) * sin(phi) * (2 * w + a_w * dt)) / 2, -(dt * cos(theta) * (2 * u + a_u * dt)) / 2 - (dt * cos(phi) * sin(theta) * (2 * w + a_w * dt)) / 2 - (dt * sin(phi) * sin(theta) * (2 * v + a_v * dt)) / 2, 0.0, -dt * sin(theta), dt * cos(theta) * sin(phi), dt * cos(phi) * cos(theta), 0.0, 0.0, 0.0,
                0.0, 0.0, 0.0, dt * q * cos(phi) * tan(theta) - dt * r * sin(phi) * tan(theta) + 1, (dt * (r * cos(phi) + q * sin(phi))) / (cos(theta) * cos(theta)), 0.0, 0.0, 0.0, 0.0, dt, dt * sin(phi) * tan(theta), dt * cos(phi) * tan(theta),
                0.0, 0.0, 0.0, -dt * (r * cos(phi) + q * sin(phi)), 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, dt * cos(phi), -dt * sin(phi),
                0.0, 0.0, 0.0, (dt * (q * cos(phi) - r * sin(phi))) / cos(theta), (dt * sin(theta) * (r * (2 * sin(phi / 2) * sin(phi / 2) - 1) - q * sin(phi))) / (sin(theta) * sin(theta) - 1), 1.0, 0.0, 0.0, 0.0, 0.0, (dt * sin(phi)) / cos(theta), (dt * cos(phi)) / cos(theta),
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0;

            Eigen::MatrixXd D_k(12, 6);

            D_k << (dt * dt * cos(psi) * cos(theta)) / 2, -(dt * dt * (cos(phi) * sin(psi) - cos(psi) * sin(phi) * sin(theta))) / 2, (dt * dt * (sin(phi) * sin(psi) + cos(phi) * cos(psi) * sin(theta))) / 2, 0.0, 0.0, 0.0,
                (dt * dt * cos(theta) * sin(psi)) / 2, (dt * dt * (cos(phi) * cos(psi) + sin(phi) * sin(psi) * sin(theta))) / 2, -(dt * dt * (cos(psi) * sin(phi) - cos(phi) * sin(psi) * sin(theta))) / 2, 0.0, 0.0, 0.0,
                -(dt * dt * sin(theta)) / 2, (dt * dt * cos(theta) * sin(phi)) / 2, (dt * dt * cos(phi) * cos(theta)) / 2, 0.0, 0.0, 0.0,
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                dt, 0.0, 0.0, 0.0, 0.0, 0.0,
                0.0, dt, 0.0, 0.0, 0.0, 0.0,
                0.0, 0.0, dt, 0.0, 0.0, 0.0,
                0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
                0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
                0.0, 0.0, 0.0, 0.0, 0.0, 1.0;

            Jacobian << cos(psi) * cos(theta), cos(psi) * sin(phi) * sin(theta) - cos(phi) * sin(psi), sin(phi) * sin(psi) + cos(phi) * cos(psi) * sin(theta), 0, 0, 0,
                cos(theta) * sin(psi), cos(phi) * cos(psi) + sin(phi) * sin(psi) * sin(theta), cos(phi) * sin(psi) * sin(theta) - cos(psi) * sin(phi), 0, 0, 0,
                -sin(theta), cos(theta) * sin(phi), cos(phi) * cos(theta), 0, 0, 0,
                0, 0, 0, 1, sin(phi) * tan(theta), cos(phi) * tan(theta),
                0, 0, 0, 0, cos(phi), -sin(phi),
                0, 0, 0, 0, sin(phi) / cos(theta), cos(phi) / cos(theta);

            Eigen::VectorXd a_m(6);
            a_m << a_u, a_v, a_w, 0, 0, 0;

            eta_pred = Jacobian * (dt * xi_curr.tail(6) + dt * dt / 2 * a_m) + xi_curr.head(6);

            double u_pred = dt * a_u + u;
            double v_pred = dt * a_v + v;
            double w_pred = dt * a_w + w;
            double p_pred = p_IMU_camera;
            double q_pred = q_IMU_camera;
            double r_pred = r_IMU_camera;

            nu_pred << u_pred, v_pred, w_pred, p_pred, q_pred, r_pred;
            xi_pred << eta_pred, nu_pred;

            // wrapToPi
            xi_pred(3) = wrapToPi(xi_pred(3));
            xi_pred(4) = wrapToPi(xi_pred(4));
            xi_pred(5) = wrapToPi(xi_pred(5));

            P_pred = F_k * P_curr * F_k.transpose() + D_k * Q * D_k.transpose();

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
        valid_IMU_camera = 0;

        // Let ROS handle all incoming messages in a callback function
        if (ros::Time::now().toSec() > ros::TIME_MIN.toSec())
        {
            bag.write("state/est_state_topic_no_dyn_imu", ros::Time::now(), msg);
        }

        ros::spinOnce();

        // Sleep for the remaining time to hit our 10Hz target
        loop_rate.sleep();
    }
    bag.close();
    return 0;
}