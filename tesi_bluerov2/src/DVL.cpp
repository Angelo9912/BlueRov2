#include "ros/ros.h"
#include "tesi_bluerov2/Floats.h" // for accessing -- tesi_bluerov2 Floats()
#include <random>

double u_hat = 0.0;
double v_hat = 0.0;
double w_hat = 0.0;

double var_u = 0.0189375;
double var_v = 0.0189375;
double var_w = 0.0189375;

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
        u_hat = 0.0;
        v_hat = 0.0;
        w_hat = 0.0;
    }
    else
    {
        u_hat = msg->data[6];
        v_hat = msg->data[7];
        w_hat = msg->data[8];
    }
}

int main(int argc, char **argv)
{
    // Initialize the ROS system and become a node.
    ros::init(argc, argv, "DVL_publisher");

    // Create a handle to this process' node.
    ros::NodeHandle nh;

    // Create a publisher object.
    ros::Publisher pub = nh.advertise<tesi_bluerov2::Floats>("sensors/DVL_topic", 1000);

    // Create a subscriber object.
    ros::Subscriber sub = nh.subscribe("state/state_topic", 1000, stateCallback);

    // Seed the random number generator.
    srand(time(0));

    bool is_first_loop = true;
    double start_time;

    // Loop at 10Hz, publishing messages until this node is shut down.
    ros::Rate rate(5);
    while (ros::ok())
    {
        if (is_first_loop)
        {
            start_time = ros::Time::now().toSec();
            is_first_loop = false;
        }
        tesi_bluerov2::Floats msg;
        double valid = 0.0;

        // Add noise to the DVL data and fill the message
        std::vector<double> DVL_data = {u_hat + gaussianNoise(0, var_u), v_hat + gaussianNoise(0, var_v), w_hat + gaussianNoise(0, var_w), valid};
        msg.data = DVL_data;

        if (ros::Time::now().toSec() - start_time > 5)
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