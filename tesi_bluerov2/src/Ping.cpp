#include "ros/ros.h"
#include "tesi_bluerov2/Floats.h" // for accessing -- tesi_bluerov2 Floats()
#include <random>

double x_hat = 0.0;
double y_hat = 0.0;

double var_x = 1;
double var_y = 1;

// Function to generate Gaussian random number
double gaussianNoise(double mean, double stddev)
{
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
    }
    else
    {
        x_hat = msg->data[0];
        y_hat = msg->data[1];
    }
}

int main(int argc, char **argv)
{
    // Initialize the ROS system and become a node.
    ros::init(argc, argv, "ping_publisher");

    // Create a handle to this process' node.
    ros::NodeHandle nh;

    // Create a publisher object.
    ros::Publisher pub = nh.advertise<tesi_bluerov2::Floats>("scanner_topic", 1000);

    // Create a subscriber object.
    ros::Subscriber sub = nh.subscribe("state_topic", 1000, stateCallback);

    // Seed the random number generator.
    srand(time(0));

    // Loop at 10Hz, publishing messages until this node is shut down.
    ros::Rate rate(5);
    while (ros::ok())
    {
        tesi_bluerov2::Floats msg;
        double valid = 1.0;

        // Add noise to the scanner data and fill the message
        std::vector<double> scanner_data = {x_hat + gaussianNoise(0, var_x), y_hat + gaussianNoise(0, var_y), valid};
        msg.data = scanner_data;

        // Publish the message.
        pub.publish(msg);

        // Send any pending callbacks.
        ros::spinOnce();

        // Wait until it's time for another iteration.
        rate.sleep();
    }

    return 0;
}