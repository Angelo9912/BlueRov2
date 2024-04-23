#include <ros/ros.h>
#include <std_msgs/String.h>
#include <rosbag/bag.h>
#include <eigen3/Eigen/Dense>
#include "tesi_bluerov2/Floats.h" // for accessing -- tesi_bluerov2 Floats()
#include "tesi_bluerov2/waypoints.h"
#include <random>

rosbag::Bag bag;
rosbag::Bag waypoints_bag;

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

double x_hat_EKF = 0.0;
double y_hat_EKF = 0.0;
double z_hat_EKF = 0.0;
double phi_hat_EKF = 0.0;
double theta_hat_EKF = 0.0;
double psi_hat_EKF = 0.0;
double u_hat_EKF = 0.0;
double v_hat_EKF = 0.0;
double w_hat_EKF = 0.0;
double p_hat_EKF = 0.0;
double q_hat_EKF = 0.0;
double r_hat_EKF = 0.0;

double x_hat_UKF = 0.0;
double y_hat_UKF = 0.0;
double z_hat_UKF = 0.0;
double phi_hat_UKF = 0.0;
double theta_hat_UKF = 0.0;
double psi_hat_UKF = 0.0;
double u_hat_UKF = 0.0;
double v_hat_UKF = 0.0;
double w_hat_UKF = 0.0;
double p_hat_UKF = 0.0;
double q_hat_UKF = 0.0;
double r_hat_UKF = 0.0;

double mahalanobis_EKF = 0.0;
double mahalanobis_UKF = 0.0;

double DELTA = 20;
double w_d = -0.1;

bool flag = true;

// Function to generate Gaussian random number
double gaussianNoise(double mean, double var)
{
    double stddev = sqrt(var);
    std::random_device rd;
    std::mt19937 gen(rd());
    std::normal_distribution<> d(mean, stddev);
    return d(gen);
}

void state_callback(const tesi_bluerov2::Floats::ConstPtr &msg)
{
    if (msg->data[0] != msg->data[0])
    {
        x = 0.0;
        y = 0.0;
        z = 0.0;
        phi = 0.0;
        theta = 0.0;
        psi = 0.0;
        u = 0.0;
        v = 0.0;
        w = 0.0;
        p = 0.0;
        q = 0.0;
        r = 0.0;
    }
    else
    {
        x = msg->data[0];
        y = msg->data[1];
        z = msg->data[2];
        phi = msg->data[3];
        theta = msg->data[4];
        psi = msg->data[5];
        u = msg->data[6];
        v = msg->data[7];
        w = msg->data[8];
        p = msg->data[9];
        q = msg->data[10];
        r = msg->data[11];
    }
}

void est_state_callback(const tesi_bluerov2::Floats::ConstPtr &msg)
{
    if (msg->data[0] != msg->data[0])
    {
        x_hat_EKF = 0.0;
        y_hat_EKF = 0.0;
        z_hat_EKF = 0.0;
        phi_hat_EKF = 0.0;
        theta_hat_EKF = 0.0;
        psi_hat_EKF = 0.0;
        u_hat_EKF = 0.0;
        v_hat_EKF = 0.0;
        w_hat_EKF = 0.0;
        p_hat_EKF = 0.0;
        q_hat_EKF = 0.0;
        r_hat_EKF = 0.0;
        ROS_WARN("NaN KF error");
    }
    else
    {
        x_hat_EKF = msg->data[0];
        y_hat_EKF = msg->data[1];
        z_hat_EKF = msg->data[2];
        phi_hat_EKF = msg->data[3];
        theta_hat_EKF = msg->data[4];
        psi_hat_EKF = msg->data[5];
        u_hat_EKF = msg->data[6];
        v_hat_EKF = msg->data[7];
        w_hat_EKF = msg->data[8];
        p_hat_EKF = msg->data[9];
        q_hat_EKF = msg->data[10];
        r_hat_EKF = msg->data[11];
        mahalanobis_EKF = msg->data[12];
    }
}

void est_state_UKF_callback(const tesi_bluerov2::Floats::ConstPtr &msg)
{
    if (msg->data[0] != msg->data[0])
    {
        x_hat_UKF = 0.0;
        y_hat_UKF = 0.0;
        z_hat_UKF = 0.0;
        phi_hat_UKF = 0.0;
        theta_hat_UKF = 0.0;
        psi_hat_UKF = 0.0;
        u_hat_UKF = 0.0;
        v_hat_UKF = 0.0;
        w_hat_UKF = 0.0;
        p_hat_UKF = 0.0;
        q_hat_UKF = 0.0;
        r_hat_UKF = 0.0;

        ROS_WARN("NaN UKF error");
    }
    else
    {
        x_hat_UKF = msg->data[0];
        y_hat_UKF = msg->data[1];
        z_hat_UKF = msg->data[2];
        phi_hat_UKF = msg->data[3];
        theta_hat_UKF = msg->data[4];
        psi_hat_UKF = msg->data[5];
        u_hat_UKF = msg->data[6];
        v_hat_UKF = msg->data[7];
        w_hat_UKF = msg->data[8];
        p_hat_UKF = msg->data[9];
        q_hat_UKF = msg->data[10];
        r_hat_UKF = msg->data[11];
        mahalanobis_UKF = msg->data[12];
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

int main(int argc, char **argv)
{
    ros::init(argc, argv, "err_vis");

    ros::NodeHandle n;

    ros::Subscriber sub1 = n.subscribe("state/state_topic", 1000, state_callback);
    ros::Subscriber sub2 = n.subscribe("state/est_state_topic", 1000, est_state_callback);
    ros::Subscriber sub3 = n.subscribe("state/est_state_UKF_topic", 1000, est_state_UKF_callback);
    ros::Publisher pub = n.advertise<tesi_bluerov2::Floats>("error_topic", 1000);
    // ros::Publisher pub2 = n.advertise<tesi_bluerov2::Floats>("tau_topic", 1000);
    // ros::Publisher pub3 = n.advertise<tesi_bluerov2::Floats>("desired_state_topic", 1000);
    ros::Rate loop_rate(100);

    bool is_first_loop = true;
    double t1;

    bool f1 = false;
    bool f2 = false;
    bool f3 = false;
    bool f4 = false;

    Eigen::Matrix<double, 6, 6> J;

    std::string path = ros::package::getPath("tesi_bluerov2");
    bag.open(path + "/bag/test.bag", rosbag::bagmode::Write);
    waypoints_bag.open(path + "/bag/waypoints.bag", rosbag::bagmode::Write);

    double x_d = 0;
    double y_d = 0;
    double z_d = 0;
    double psi_d = 0;

    ros::Duration(20).sleep();
    ros::spinOnce();
    int i = 0;
    double dt = 0.01;
    std::vector<double> way = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    while (ros::ok())
    {
        if (is_first_loop)
        {
            is_first_loop = false;
            f1 = true;
            t1 = ros::Time::now().toSec();
        }

        double err_x_EKF = x - x_hat_EKF;
        double err_y_EKF = y - y_hat_EKF;
        double err_z_EKF = z - z_hat_EKF;
        double err_phi_EKF = phi - phi_hat_EKF;
        double err_theta_EKF = theta - theta_hat_EKF;
        double err_psi_EKF = psi - psi_hat_EKF;
        double err_u_EKF = u - u_hat_EKF;
        double err_v_EKF = v - v_hat_EKF;
        double err_w_EKF = w - w_hat_EKF;
        double err_p_EKF = p - p_hat_EKF;
        double err_q_EKF = q - q_hat_EKF;
        double err_r_EKF = r - r_hat_EKF;

        double err_x_UKF = x - x_hat_UKF;
        double err_y_UKF = y - y_hat_UKF;
        double err_z_UKF = z - z_hat_UKF;
        double err_phi_UKF = phi - phi_hat_UKF;
        double err_theta_UKF = theta - theta_hat_UKF;
        double err_psi_UKF = psi - psi_hat_UKF;
        double err_u_UKF = u - u_hat_UKF;
        double err_v_UKF = v - v_hat_UKF;
        double err_w_UKF = w - w_hat_UKF;
        double err_p_UKF = p - p_hat_UKF;
        double err_q_UKF = q - q_hat_UKF;
        double err_r_UKF = r - r_hat_UKF;

        std::vector<double> error = {err_x_EKF, err_y_EKF, err_z_EKF, err_phi_EKF, err_theta_EKF, err_psi_EKF, err_u_EKF, err_v_EKF, err_w_EKF, err_p_EKF, err_q_EKF, err_r_EKF, mahalanobis_EKF, err_x_UKF, err_y_UKF, err_z_UKF, err_phi_UKF, err_theta_UKF, err_psi_UKF, err_u_UKF, err_v_UKF, err_w_UKF, err_p_UKF, err_q_UKF, err_r_UKF, mahalanobis_UKF};

        for (int i = 3; i < 6; i++)
        {
            error[i] = angleDifference(error[i]);
        }

        for (int i = 16; i < 19; i++)
        {
            error[i] = angleDifference(error[i]);
        }

        error[3] = error[3] * 180 / M_PI;
        error[4] = error[4] * 180 / M_PI;
        error[5] = error[5] * 180 / M_PI;

        error[9] = error[9] * 180 / M_PI;
        error[10] = error[10] * 180 / M_PI;
        error[11] = error[11] * 180 / M_PI;

        error[16] = error[16] * 180 / M_PI;
        error[17] = error[17] * 180 / M_PI;
        error[18] = error[18] * 180 / M_PI;

        error[22] = error[22] * 180 / M_PI;
        error[23] = error[23] * 180 / M_PI;
        error[24] = error[24] * 180 / M_PI;

        tesi_bluerov2::Floats error_msg;
        error_msg.data = error;
        pub.publish(error_msg);
        std::vector<double> des;

        // if (f1)
        // {
        //     ROS_WARN("f1");
        //     tau = {30.0, 0.0, 0.0, 0.0, 0.0, 0.0};

        //     if (ros::Time::now().toSec() - t1 > 10)
        //     {
        //         f1 = false;
        //         f2 = true;
        //         t1 = ros::Time::now().toSec();
        //     }
        // }
        // else if (f2)
        // {
        //     ROS_WARN("f2");
        //     tau = {0.0, 0.0, 2.0, 0.0, 0.0, 0.0};

        //     if (ros::Time::now().toSec() - t1 > 10)
        //     {
        //         f2 = false;
        //         f3 = true;
        //         t1 = ros::Time::now().toSec();
        //     }
        // }
        // else if (f3)
        // {
        //     ROS_WARN("f3");
        //     tau = {0.0, 0.0, 0.0, 0.0, 0.0, 5};

        //     if (ros::Time::now().toSec() - t1 > 10)
        //     {
        //         f3 = false;
        //         f4 = true;
        //         t1 = ros::Time::now().toSec();
        //     }
        // }
        // else if (f4)
        // {
        //     ROS_WARN("f4");
        //     tau = {0.0, 0.0, -2.0, 0.0, 0.0, 0.0};

        //     if (ros::Time::now().toSec() - t1 > 10)
        //     {
        //         f4 = false;
        //         f1 = true;
        //         t1 = ros::Time::now().toSec();
        //     }
        // }
        double u_d = 0.5;
        double r_d = 10 * M_PI / 180;

        if (i / 100 >= DELTA)
        {
            w_d = -w_d;
            DELTA = DELTA + 20;
        }

        i++;
        psi_d = psi_d + r_d * dt;

        psi_d = atan2(sin(psi_d), cos(psi_d));

        x_d = x_d + dt * u_d * cos(psi_d);
        y_d = y_d + dt * u_d * sin(psi_d);
        z_d = z_d + dt * w_d;

        // Define Jacobian Matrix
        J << cos(psi_d) * cos(0.0), cos(psi_d) * sin(0.0) * sin(0.0) - cos(0.0) * sin(psi_d), sin(0.0) * sin(psi_d) + cos(0.0) * cos(psi_d) * sin(theta_hat_UKF), 0, 0, 0,
            cos(0.0) * sin(psi_d), cos(0.0) * cos(psi_d) + sin(0.0) * sin(psi_d) * sin(0.0), cos(0.0) * sin(psi_d) * sin(0.0) - cos(psi_d) * sin(0.0), 0, 0, 0,
            -sin(0.0), cos(0.0) * sin(0.0), cos(0.0) * cos(0.0), 0, 0, 0,
            0, 0, 0, 1, sin(0.0) * tan(0.0), cos(0.0) * tan(0.0),
            0, 0, 0, 0, cos(0.0), -sin(0.0),
            0, 0, 0, 0, sin(0.0) / cos(0.0), cos(0.0) / cos(0.0);

        Eigen::VectorXd nu_d(6);
        nu_d << u_d, 0, w_d, 0, 0, r_d;

        Eigen::VectorXd eta_dot_d(6);
        eta_dot_d << J * nu_d;

        // des = {x_d, y_d, z_d, 0.0, 0.0, psi_d, eta_dot_d(0), eta_dot_d(1), eta_dot_d(2), eta_dot_d(3), eta_dot_d(4), eta_dot_d(5)};

        // tesi_bluerov2::Floats des_msg;
        // des_msg.data = des;
        // pub3.publish(des_msg);

        if (ros::Time::now().toSec() > ros::TIME_MIN.toSec())
        {
            bag.write("error_topic", ros::Time::now(), error_msg);
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    bag.close();

    return 0;
}