#include "ros/ros.h"
#include "tesi_bluerov2/Floats.h" // for accessing -- tesi_bluerov2 Floats()
#include <random>

double phi_hat = 0.0;
double theta_hat = 0.0;
double psi_hat = 0.0;
double p_hat = 0.0;
double q_hat = 0.0;
double r_hat = 0.0;

double var_phi = 0.01;
double var_theta = 0.01;
double var_psi = 0.01;
double var_p = 0.0007;
double var_q = 0.0007;
double var_r = 0.0007;

// Function to generate Gaussian random number
double gaussianNoise(double mean, double var)
{
    double stddev = sqrt(var);
    std::random_device rd;
    std::mt19937 gen(rd());
    std::normal_distribution<> d(mean, stddev);
    return d(gen);
}

// Callback function for the state_topic subscriber
void stateCallback(const tesi_bluerov2::Floats::ConstPtr &msg)
{
    if (msg->data[0] != msg->data[0])
    {
        phi_hat = 0.0;
        theta_hat = 0.0;
        psi_hat = 0.0;
        p_hat = 0.0;
        q_hat = 0.0;
        r_hat = 0.0;
    }
    else
    {
        phi_hat = msg->data[3];
        theta_hat = msg->data[4];
        psi_hat = msg->data[5];
        p_hat = msg->data[9];
        q_hat = msg->data[10];
        r_hat = msg->data[11];
    }
}

int main(int argc, char **argv)
{
    // Initialize the ROS system and become a node.
    ros::init(argc, argv, "imu_publisher");

    // Create a handle to this process' node.
    ros::NodeHandle nh;

    // Create a publisher object.
    ros::Publisher pub = nh.advertise<tesi_bluerov2::Floats>("IMU_topic", 1000);

    // Create a subscriber object.
    ros::Subscriber sub = nh.subscribe("state_topic", 1000, stateCallback);

    // Seed the random number generator.
    srand(time(0));

    bool is_first_loop = true;
    double start_time;

    // Loop at 10Hz, publishing messages until this node is shut down.
    ros::Rate rate(300);
    while (ros::ok())
    {
        if (is_first_loop)
        {
            is_first_loop = false;
            start_time = ros::Time::now().toSec();
        }
        tesi_bluerov2::Floats msg;

        double valid = 1.0;

        // Add noise to the IMU data and fill the message
        std::vector<double> imu_data = {phi_hat + gaussianNoise(0, var_phi), theta_hat + gaussianNoise(0, var_theta), psi_hat + gaussianNoise(0, var_psi), p_hat + gaussianNoise(0, var_p), q_hat + gaussianNoise(0, var_q), r_hat + gaussianNoise(0, var_r),valid};
        msg.data = imu_data;

        if(ros::Time::now().toSec() - start_time > 5)
        {
            // Publish the message.
            pub.publish(msg);
        }
        // Send any pending callbacks.
        ros::spinOnce();

        // Wait until it's time for another iteration.
        rate.sleep();
    }

    return 0;
}