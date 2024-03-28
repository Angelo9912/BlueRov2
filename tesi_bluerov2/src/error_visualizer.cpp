#include <ros/ros.h>
#include <std_msgs/String.h>
#include <rosbag/bag.h>
#include "tesi_bluerov2/Floats.h" // for accessing -- tesi_bluerov2 Floats()
#include <random>

rosbag::Bag bag;

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

int main(int argc, char **argv)
{
    ros::init(argc, argv, "err_vis");

    ros::NodeHandle n;

    ros::Subscriber sub1 = n.subscribe("state_topic", 1000, state_callback);
    ros::Subscriber sub2 = n.subscribe("est_state_topic", 1000, est_state_callback);
    ros::Subscriber sub3 = n.subscribe("est_state_UKF_topic", 1000, est_state_UKF_callback);
    ros::Publisher pub = n.advertise<tesi_bluerov2::Floats>("error_topic", 1000);
    ros::Publisher pub2 = n.advertise<tesi_bluerov2::Floats>("tau_topic", 1000);
    ros::Rate loop_rate(100);

    bool is_first_loop = true;
    double start_time;

    std::string path = ros::package::getPath("tesi_bluerov2");
    bag.open(path + "/bag/test.bag", rosbag::bagmode::Write);

    while (ros::ok())
    {
        if (is_first_loop)
        {
            is_first_loop = false;
            start_time = ros::Time::now().toSec();
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

        std::vector<double> error = {err_x_EKF, err_y_EKF, err_z_EKF, err_phi_EKF, err_theta_EKF, err_psi_EKF, err_u_EKF, err_v_EKF, err_w_EKF, err_p_EKF, err_q_EKF, err_r_EKF,mahalanobis_EKF, err_x_UKF, err_y_UKF, err_z_UKF, err_phi_UKF, err_theta_UKF, err_psi_UKF, err_u_UKF, err_v_UKF, err_w_UKF, err_p_UKF, err_q_UKF, err_r_UKF, mahalanobis_UKF};

        for (int i = 3; i < 6; i++)
        {
            if (error[i] > 0)
            {
                if (error[i] > 2 * M_PI - error[i])
                {
                    error[i] = -(2 * M_PI - error[i]);
                }
                else
                {
                    error[i] = error[i];
                }
            }
            else
            {
                error[i] = -error[i];
                if (error[i] > 2 * M_PI - error[i])
                {
                    error[i] = 2 * M_PI - error[i];
                }
                else
                {
                    error[i] = -error[i];
                }
            }
        }

        for (int i = 15; i < 18; i++)
        {
            if (error[i] > 0)
            {
                if (error[i] > 2 * M_PI - error[i])
                {
                    error[i] = -(2 * M_PI - error[i]);
                }
                else
                {
                    error[i] = error[i];
                }
            }
            else
            {
                error[i] = -error[i];
                if (error[i] > 2 * M_PI - error[i])
                {
                    error[i] = 2 * M_PI - error[i];
                }
                else
                {
                    error[i] = -error[i];
                }
            }
        }

        error[3] = error[3] * 180 / M_PI;
        error[4] = error[4] * 180 / M_PI;
        error[5] = error[5] * 180 / M_PI;

        error[9] = error[9] * 180 / M_PI;
        error[10] = error[10] * 180 / M_PI;
        error[11] = error[11] * 180 / M_PI;

        error[15] = error[15] * 180 / M_PI;
        error[16] = error[16] * 180 / M_PI;
        error[17] = error[17] * 180 / M_PI;

        error[21] = error[21] * 180 / M_PI;
        error[22] = error[22] * 180 / M_PI;
        error[23] = error[23] * 180 / M_PI;

        tesi_bluerov2::Floats error_msg;
        error_msg.data = error;
        pub.publish(error_msg);
        std::vector<double> tau;
        if (ros::Time::now().toSec() - start_time < 0.1)
        {
            tau = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        }
        else
        {
            tau = {1.0, 0.0, 0.0, M_PI / 180 * 0.0, M_PI / 180 * 0.0, M_PI / 180 * 0.0};
        }

        tesi_bluerov2::Floats tau_msg;
        tau_msg.data = tau;
        pub2.publish(tau_msg);

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