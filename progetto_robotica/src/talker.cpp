#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include <geometry_msgs/Pose.h>            // for accessing -- geometry_msgs Pose()
#include <geometry_msgs/Point.h>           // for accessing -- geometry_msgs Point()
#include <geometry_msgs/Quaternion.h>      // for accessing -- geometry_msgs Quaternion()
#include <tf2/LinearMath/Quaternion.h>     // for tf2 quaternion

int main(int argc, char **argv)
{
    ros::init(argc, argv, "talker");

    ros::NodeHandle n;

    ros::Publisher chatter_pub = n.advertise<std_msgs::String>("pose_topic", 1000);

    ros::Rate loop_rate(10);
    double count = 0;
    tf2::Quaternion q;
    while (ros::ok())
    {
        q.setRPY(0, 0, 0);
        geometry_msgs::Point point;
        point.x = count;
        point.y = 0;
        point.z = 0;
        geometry_msgs::Quaternion quaternion;
        quaternion.x = q.x();
        quaternion.y = q.y();
        quaternion.z = q.z();
        quaternion.w = q.w();
        

        
        
        geometry_msgs::Pose pose;
        pose.position = point;
        pose.orientation = quaternion;
        
        chatter_pub.publish(pose);

        ros::spinOnce();

        loop_rate.sleep();
        count += 0.1;
    }

    return 0;
}