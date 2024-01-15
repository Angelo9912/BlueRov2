#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include <geometry_msgs/Pose.h>            // for accessing -- geometry_msgs Pose()
#include <geometry_msgs/Point.h>           // for accessing -- geometry_msgs Point()
#include <geometry_msgs/Quaternion.h>      // for accessing -- geometry_msgs Quaternion()
#include <tf2/LinearMath/Quaternion.h>     // for tf2 quaternion

int main(int argc, char **argv)
{
    ros::init(argc, argv, "dynamics");

    ros::NodeHandle n;

    ros::Publisher chatter_pub = n.advertise<geometry_msgs::Pose>("pose_topic", 1);
    double freq = 100;
    ros::Rate loop_rate(freq);
    double count = 0;
    tf2::Quaternion q;
    double vel = 1;

    double x = 0, y = 0;

    while (ros::ok())
    {
        q.setRPY(0, 0, count);
        geometry_msgs::Point point;
        point.x = 1/freq * vel*cos(count) + x;
        point.y = 1/freq * vel*sin(count) + y;
        point.z = 0;

        geometry_msgs::Quaternion quaternion;
        quaternion.x = q.x();
        quaternion.y = q.y();
        quaternion.z = q.z();
        quaternion.w = q.w();
        
        x = point.x;
        y = point.y;
        
        geometry_msgs::Pose pose;
        pose.position = point;
        pose.orientation = quaternion;
        
        chatter_pub.publish(pose);

        ROS_INFO("%f", point.x);


        ros::spinOnce();

        count += 1/freq*1;
        loop_rate.sleep();
        
    }

    return 0;
}