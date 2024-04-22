#include "ros/ros.h"
#include "tesi_bluerov2/Floats.h" // for accessing -- tesi_bluerov2 Floats()
#include <random>

double u_dot = 0.0;
double v_dot = 0.0;
double w_dot = 0.0;

double var_u_dot = 0.000096236;
double var_v_dot = 0.000096236;
double var_w_dot = 0.000096236;


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
void rawAccCallback(const tesi_bluerov2::Floats::ConstPtr &msg)
{
    if (msg->data[0] != msg->data[0])
    {
        u_dot = 0.0;
        v_dot = 0.0;
        w_dot = 0.0;
    }
    else
    {
        u_dot = msg->data[0];
        v_dot = msg->data[1];
        w_dot = msg->data[2];
    }
}

int main(int argc, char **argv)
{
    // Initialize the ROS system and become a node.
    ros::init(argc, argv, "accelerometer_publisher");

    // Create a handle to this process' node.
    ros::NodeHandle nh;

    // Create a publisher object.
    ros::Publisher pub = nh.advertise<tesi_bluerov2::Floats>("sensors/acc_topic", 1000);

    // Create a subscriber object.
    ros::Subscriber sub = nh.subscribe("state/raw_acc_topic", 1000, rawAccCallback);

    // Seed the random number generator.
    srand(time(0));

    bool is_first_loop = true;
    double start_time;

    // Loop at 50Hz, publishing messages until this node is shut down.
    ros::Rate rate(300);
    while (ros::ok())
    {
        if (is_first_loop)
        {
            is_first_loop = false;
            start_time = ros::Time::now().toSec();
        }

        tesi_bluerov2::Floats msg;

        // Add noise to the accelerometer data and fill the message
        std::vector<double> acc_data = {u_dot + gaussianNoise(0, var_u_dot), v_dot + gaussianNoise(0, var_v_dot), w_dot + gaussianNoise(0, var_w_dot)};
        msg.data = acc_data;

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