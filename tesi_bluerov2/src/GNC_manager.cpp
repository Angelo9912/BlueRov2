#include <ros/ros.h>
#include <std_msgs/String.h>

std::string status_req = "";

std::string status_true = "NOT_READY";

bool first_print = true;

// Callback function for the subscriber
void status_reqCallback(const std_msgs::String::ConstPtr &msg)
{
    status_req = msg->data.c_str();
}

int main(int argc, char **argv)
{
    // Initialize the ROS node
    ros::init(argc, argv, "GNC_manager_node");
    ros::NodeHandle nh;

    // Create a publisher that publishes messages of type std_msgs::String on the "topic1" topic
    ros::Publisher publisher = nh.advertise<std_msgs::String>("manager/GNC_status_topic", 10);

    // Create a subscriber that subscribes to messages of type std_msgs::String on the "topic2" topic
    ros::Subscriber subscriber = nh.subscribe("manager/GNC_status_requested_topic", 10, status_reqCallback);

    // Set the loop rate (in Hz)
    ros::Rate loop_rate(10);

    int count = 0;
    while (ros::ok())
    {
        // Check if the status is "NOT_READY"
        if (status_true == "NOT_READY")
        {
            if (first_print)
            {
                ROS_WARN("GNC IS NOT READY");
                first_print = false;
            }
            // Check if the status requested is "READY"
            if (status_req == "NAVIGATION_READY")
            {
                // Change the status to "READY"
                status_true = "NAVIGATION_READY";
                ROS_WARN("NAVIGATION READY");
            }
        }
        else
        {
            if (status_req == "CONTROLLER_READY" && status_true == "NAVIGATION_READY")
            {
                status_true = "CONTROLLER_READY";
                ROS_WARN("CONTROLLER READY");
            }
            else if (status_req == "GUIDANCE_READY")
            {
                status_true = "GNC_READY";
                if (!first_print)
                {
                    first_print = true;
                    ROS_WARN("GNC READY");
                }
            }
        }
        // Create a message object of type std_msgs::String
        std_msgs::String msg;
        msg.data = status_true;

        // Publish the message on the "topic1" topic
        publisher.publish(msg);

        // Process any incoming messages
        ros::spinOnce();

        // Sleep for the remaining time to achieve the desired loop rate
        loop_rate.sleep();
    }

    return 0;
}