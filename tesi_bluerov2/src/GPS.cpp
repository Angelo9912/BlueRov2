#include "ros/ros.h"
#include "tesi_bluerov2/Floats.h" // for accessing -- tesi_bluerov2 Floats()
#include <random>

double x_hat = 0.0;
double y_hat = 0.0;
double z_hat = 0.0;

double var_x = 1;
double var_y = 1;

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
        x_hat = 0.0;
        y_hat = 0.0;
        z_hat = 0.0;
    }
    else
    {
        x_hat = msg->data[0];
        y_hat = msg->data[1];
        z_hat = msg->data[2];
    }
}

int main(int argc, char **argv)
{
    // Initialize the ROS system and become a node.
    ros::init(argc, argv, "GPS_publisher");

    // Create a handle to this process' node.
    ros::NodeHandle nh;

    // Create a publisher object.
    ros::Publisher pub = nh.advertise<tesi_bluerov2::Floats>("sensors/GPS_topic", 1000);

    // Create a subscriber object.
    ros::Subscriber sub = nh.subscribe("state/state_topic", 1000, stateCallback);

    // Seed the random number generator.
    srand(time(0));

    bool is_first_loop = true;
    double start_time;

    // Loop at 10Hz, publishing messages until this node is shut down.
    ros::Rate rate(1);
    while (ros::ok())
    {
        if (is_first_loop)
        {
            is_first_loop = false;
            start_time = ros::Time::now().toSec();
        }

        tesi_bluerov2::Floats msg;
        double valid;

        if (z_hat > 0.5)
            valid = 0.0;
        else
            valid = 1.0;

        // Add noise to the GPS data and fill the message
        std::vector<double> GPS_data = {x_hat + gaussianNoise(0, var_x), y_hat + gaussianNoise(0, var_y), valid};
        msg.data = GPS_data;
        // ROS_WARN("GPS data: x: %f, y: %f", GPS_data[0], GPS_data[1]);

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