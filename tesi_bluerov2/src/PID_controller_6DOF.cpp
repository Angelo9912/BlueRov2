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
#include <random>

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
double wait_for_controller = 0.0;
std::string GNC_status = "NOT_READY";

double gaussianNoise(double mean, double var)
{
    double stddev = sqrt(var);
    std::random_device rd;
    std::mt19937 gen(rd());
    std::normal_distribution<> d(mean, stddev);
    return d(gen);
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
    ros::init(argc, argv, "PID_controller_6DOF");
    ros::NodeHandle n;
    rosbag::Bag tau_bag;
    std::string path = ros::package::getPath("tesi_bluerov2");
    tau_bag.open(path + "/bag/tau.bag", rosbag::bagmode::Write);

    ros::Publisher chatter_pub = n.advertise<tesi_bluerov2::Floats>("tau_topic", 1);
    ros::Publisher publisher_gnc_status = n.advertise<std_msgs::String>("manager/GNC_status_requested_topic", 10); // publisher stato richiesto al GNC

    ros::Subscriber sub_gnc_status = n.subscribe("manager/GNC_status_topic", 1, GNCstatusCallback); // sottoscrizione alla topic di stato del GNC
    ros::Subscriber sub_des_state = n.subscribe("state/desired_state_topic", 1, desStateCallback);
    ros::Subscriber sub_est_state = n.subscribe("state/est_state_EKF_no_dyn_imu_topic", 1, estStateCallback);
    //ros::Subscriber sub_est_state = n.subscribe("state/state_topic", 1, estStateCallback);

    double freq = 200;
    double dt = 1 / freq;
    ros::Rate loop_rate(freq);

    n.getParam("wait_for_controller", wait_for_controller);

    bool is_init = true;
    double init_time;

    // Define the desired state and the estimated state vectors
    Eigen::Matrix<double, 6, 1> des_pose;
    Eigen::Matrix<double, 6, 1> des_pos_dot;

    Eigen::Matrix<double, 6, 1> est_pose;

    // Define error vector
    Eigen::Matrix<double, 3, 1> error_lin;

    Eigen::Matrix<double, 3, 1> error_lin_int;
    error_lin_int.setZero();

    Eigen::Matrix<double, 3, 1> error_lin_dot;

    double error_psi;
    double error_psi_int = 0.0;
    double error_psi_dot;

    Eigen::Matrix<double, 6, 1> nu;

    Eigen::Matrix<double, 3, 3> J1;

    Eigen::Matrix<double, 3, 1> est_pose_lin_dot;
    Eigen::Matrix<double, 3, 1> est_pose_lin;

    // Define the gains
    Eigen::Matrix<double, 3, 3> K_p_lin;
   //K_p_lin = (400.0) * Eigen::Matrix<double, 3, 3>::Identity();

    K_p_lin = 30*Eigen::Matrix<double, 3, 3>::Identity();

    Eigen::Matrix<double, 3, 3> K_d_lin;
    //K_d_lin = (200.0) * Eigen::Matrix<double, 3, 3>::Identity();
    K_d_lin = 10*Eigen::Matrix<double, 3, 3>::Identity();

    Eigen::Matrix<double, 3, 3> K_i_lin;
   // K_i_lin = (0.1) * Eigen::Matrix<double, 3, 3>::Identity();
    K_i_lin = 0.8*Eigen::Matrix<double, 3, 3>::Identity();


    double K_p_psi;
   // K_p_psi = 50.0;

    K_p_psi = 30.0;

    double K_d_psi;
    K_d_psi = 0.0;


    double K_i_psi;
    K_i_psi = 5.0;

    double x_r = 0.1105;
    double y_r = 0.133;
    double c_45 = cos(M_PI / 4);

    Eigen::Matrix<double, 4, 6> B;
    B << c_45, c_45, -c_45, -c_45, 0.0, 0.0,
        -c_45, c_45, c_45, -c_45, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, -1.0, -1.0,
        -(x_r + y_r) * sqrt(2) / 2, (x_r + y_r) * sqrt(2) / 2, -(x_r + y_r) * sqrt(2) / 2, (x_r + y_r) * sqrt(2) / 2, 0.0, 0.0;

    Eigen::Matrix<double, 6, 4> B_cross;
    B_cross = B.transpose() * (B * B.transpose()).inverse();

    Eigen::Vector3d tau_lin;
    double tau_psi;

    Eigen::Vector3d anti_windup_lin;
    anti_windup_lin.setZero();
    double anti_windup_psi = 0.0;

    Eigen::Matrix<double, 6, 1> inputs;

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

            // Define Jacobian Matrix
            J1 << cos(psi_hat) * cos(theta_hat), cos(psi_hat) * sin(phi_hat) * sin(theta_hat) - cos(phi_hat) * sin(psi_hat), sin(phi_hat) * sin(psi_hat) + cos(phi_hat) * cos(psi_hat) * sin(theta_hat),
                cos(theta_hat) * sin(psi_hat), cos(phi_hat) * cos(psi_hat) + sin(phi_hat) * sin(psi_hat) * sin(theta_hat), cos(phi_hat) * sin(psi_hat) * sin(theta_hat) - cos(psi_hat) * sin(phi_hat),
                -sin(theta_hat), cos(theta_hat) * sin(phi_hat), cos(phi_hat) * cos(theta_hat);

            nu << u_hat, v_hat, w_hat, p_hat, q_hat, r_hat;

            // Define the error vector
            error_lin = J1.transpose() * (des_pose.head(3) - est_pose.head(3));

            error_lin_dot = J1.transpose() * (des_pos_dot.head(3)) - nu.head(3);

            error_lin_int += error_lin * dt;

            error_psi = angleDifference(des_pose(5) - est_pose(5));
            error_psi_dot = des_pos_dot(5) - nu(5);
            error_psi_int += error_psi * dt;

            tau_lin = K_p_lin * error_lin + K_d_lin * error_lin_dot + K_i_lin * (error_lin_int + anti_windup_lin);
            tau_psi = K_p_psi * error_psi + K_d_psi * error_psi_dot + K_i_psi * (error_psi_int + anti_windup_psi);

            // Define the torques vector
            Eigen::Matrix<double, 4, 1> torques_vec;

            torques_vec << tau_lin(0), tau_lin(1), tau_lin(2), tau_psi;

            // Post saturation torques
            double sat_tau_u = torques_vec(0);
            double sat_tau_v = torques_vec(1);
            double sat_tau_w = torques_vec(2);
            double sat_tau_r = tau_psi;

            std::vector<double> torques = {torques_vec(0), torques_vec(1), torques_vec(2), 0.0, 0.0, torques_vec(3)};

            if (torques_vec(3) > 37.471)
            {
                sat_tau_r = 37.471;
                torques[5] = sat_tau_r;
            }
            else if (torques_vec(3) < -37.471)
            {
                sat_tau_r = -37.471;
                torques[5] = sat_tau_r;
            }

            if (torques_vec(0) > 141.42)
            {
                sat_tau_u = 141.42;
                torques[0] = sat_tau_u;
            }
            else if (torques_vec(0) < -141.42)
            {
                sat_tau_u = -141.42;
                torques[0] = sat_tau_u;
            }

            if (torques_vec(1) > 141.42)
            {
                sat_tau_v = 141.42;
                torques[1] = sat_tau_v;
            }
            else if (torques_vec(1) < -141.42)
            {
                sat_tau_v = -141.42;
                torques[1] = sat_tau_v;
            }

            if (torques_vec(2) > 70.71)
            {
                sat_tau_w = 70.71;
                torques[2] = sat_tau_w;
            }
            else if (torques_vec(2) < -70.71)
            {
                sat_tau_w = -70.71;
                torques[2] = sat_tau_w;
            }

            Eigen::Vector4d tau_out_sat;
            
            
            tau_out_sat << sat_tau_u, sat_tau_v, sat_tau_w, sat_tau_r;


            anti_windup_lin = tau_out_sat.head(3) - tau_lin;
            anti_windup_psi = tau_out_sat(3) - tau_psi;

            inputs = B_cross * torques_vec;

            std::vector<double> inputs_vec = {inputs(0), inputs(1), inputs(2), inputs(3), inputs(4), inputs(5)};

            // Publishing the torques
            tesi_bluerov2::Floats torques_msg;
            torques_msg.data = inputs_vec;

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