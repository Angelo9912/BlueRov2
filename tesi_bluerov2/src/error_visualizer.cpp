#include <ros/ros.h>
#include <std_msgs/String.h>
#include <rosbag/bag.h>
#include "tesi_bluerov2/Floats.h" // for accessing -- tesi_bluerov2 Floats()

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

double x_hat = 0.0;
double y_hat = 0.0;
double z_hat = 0.0;
double phi_hat = 0.0;
double theta_hat = 0.0;
double psi_hat = 0.0;
double u_hat = 0.0;
double v_hat = 0.0;
double w_hat = 0.0;
double p_hat = 0.0;
double q_hat = 0.0;
double r_hat = 0.0;

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
        x_hat = 0.0;
        y_hat = 0.0;
        z_hat = 0.0;
        phi_hat = 0.0;
        theta_hat = 0.0;
        psi_hat = 0.0;
        u_hat = 0.0;
        v_hat = 0.0;
        w_hat = 0.0;
        p_hat = 0.0;
        q_hat = 0.0;
        r_hat = 0.0;
        ROS_WARN("SONO NAN PORCO CAZZO!");
    }
    else
    {
        x_hat = msg->data[0];
        y_hat = msg->data[1];
        z_hat = msg->data[2];
        phi_hat = msg->data[3];
        theta_hat = msg->data[4];
        psi_hat = msg->data[5];
        u_hat = msg->data[6];
        v_hat = msg->data[7];
        w_hat = msg->data[8];
        p_hat = msg->data[9];
        q_hat = msg->data[10];
        r_hat = msg->data[11];
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "err_vis");

    ros::NodeHandle n;

    ros::Subscriber sub1 = n.subscribe("state_topic", 1000, state_callback);
    ros::Subscriber sub2 = n.subscribe("est_state_topic", 1000, est_state_callback);
    ros::Publisher pub = n.advertise<tesi_bluerov2::Floats>("error_topic", 1000);
    ros::Publisher pub2 = n.advertise<tesi_bluerov2::Floats>("tau_topic", 1000);
    ros::Rate loop_rate(100);

    std::string path = ros::package::getPath("tesi_bluerov2");
    bag.open(path + "/bag/test.bag", rosbag::bagmode::Write);

    while (ros::ok())
    {
        double err_x = x - x_hat;
        double err_y = y - y_hat;
        double err_z = z - z_hat;
        double err_phi = phi - phi_hat;
        double err_theta = theta - theta_hat;
        double err_psi = psi - psi_hat;
        double err_u = u - u_hat;
        double err_v = v - v_hat;
        double err_w = w - w_hat;
        double err_p = p - p_hat;
        double err_q = q - q_hat;
        double err_r = r - r_hat;

        // ROS_WARN("X VERA: %f, X STIMATA: %f", x, x_hat);

        std::vector<double> error = {err_x, err_y, err_z, err_phi, err_theta, err_psi, err_u, err_v, err_w, err_p, err_q, err_r};

        tesi_bluerov2::Floats error_msg;
        error_msg.data = error;
        pub.publish(error_msg);

        std::vector<double> tau = {0.2, 0.0, 0.0, 0.0, 0.0, 0.0};
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