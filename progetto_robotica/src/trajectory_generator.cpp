#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include <ros/console.h>
#include <eigen3/Eigen/Dense>                // for eigen matrix
#include "progetto_robotica/Floats.h"        // for accessing -- progetto_robotica Floats()
#include "progetto_robotica/Floats_String.h" // for accessing -- progetto_robotica Floats_String()
#include <vector>
#include "yaml-cpp/yaml.h" // for yaml

// Declare callback variables
double x_hat = 0.0;
double y_hat = 0.0;
double z_hat = 0.0;
double psi_hat = 0.0;
double u_hat = 0.0;
double v_hat = 0.0;
double w_hat = 0.0;
double r_hat = 0.0;

double x_1 = 0.0;
double y_1 = 0.0;
double z_1 = 0.0;

double x_2 = 0.0;
double y_2 = 0.0;
double z_2 = 0.0;

double x_3 = 0.0;
double y_3 = 0.0;
double z_3 = 0.0;

double x_b = 0.0;
double y_b = 0.0;
double z_b = 0.0;
double x_p = 0.0;
double y_p = 0.0;
double z_p = 0.0;

std::string strategy = "";
std::string mission_status = "";

void statusCallback(const std_msgs::String::ConstPtr &msg)
{
    mission_status = msg->data.c_str();
}

void waypointCallback(const progetto_robotica::Floats_String::ConstPtr &msg)
{
    if (msg->data[0] != msg->data[0])
    {
        x_b = 0.0;
        y_b = 0.0;
        z_b = 0.0;
        strategy = "";
    }
    else
    {
        if (msg->strategy == "Circumference")
        {
            x_b = msg->data[0];
            y_b = msg->data[1];
            z_b = msg->data[2];
            x_p = msg->data[3];
            y_p = msg->data[4];
            z_p = msg->data[5];
            strategy = msg->strategy;
            x_1 = x_hat;
            y_1 = y_hat;
            z_1 = z_hat;
        }
        else if (msg->strategy == "UP_DOWN")
        {
            x_b = msg->data[0];
            y_b = msg->data[1];
            z_b = msg->data[2];
            strategy = msg->strategy;
            x_1 = x_hat;
            y_1 = y_hat;
            z_1 = z_hat;
        }
        else if (msg->strategy == "Spline")
        {
            x_1 = x_hat;
            y_1 = y_hat;
            z_1 = z_hat;
            x_2 = msg->data[0];
            y_2 = msg->data[1];
            z_2 = msg->data[2];
            x_3 = msg->data[3];
            y_3 = msg->data[4];
            z_3 = msg->data[5];
            strategy = msg->strategy;
        }
    }
}

void estStateCallback(const progetto_robotica::Floats::ConstPtr &msg)
{
    if (msg->data[0] != msg->data[0])
    {
        x_hat = 0.0;
        y_hat = 0.0;
        z_hat = 0.0;
        psi_hat = 0.0;
        u_hat = 0.0;
        v_hat = 0.0;
        w_hat = 0.0;
        r_hat = 0.0;
    }
    else
    {
        x_hat = msg->data[0];
        y_hat = msg->data[1];
        z_hat = msg->data[2];
        psi_hat = msg->data[3];
        u_hat = msg->data[4];
        v_hat = msg->data[5];
        w_hat = msg->data[6];
        r_hat = msg->data[7];
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "trajectory_generator");
    ros::NodeHandle n;
    ros::Publisher chatter_pub = n.advertise<progetto_robotica::Floats>("desired_state_topic", 1);
    ros::Publisher publisher_status = n.advertise<std_msgs::String>("manager/status_requested_topic", 10);

    ros::Subscriber sub_est_state = n.subscribe("state_topic", 1, estStateCallback);
    ros::Subscriber sub_waypoint = n.subscribe("waypoints_topic", 1, waypointCallback);
    ros::Subscriber sub_status = n.subscribe("manager/status_topic", 1, statusCallback);
    double freq = 10.0;
    double dt = 1 / freq;
    ros::Rate loop_rate(freq);

    double x_d = 0.0;
    double y_d = 0.0;
    double z_d = 0.0;
    double psi_d = 0.0;
    double x_dot_d = 0.0;
    double y_dot_d = 0.0;
    double z_dot_d = 0.0;
    double psi_dot_d = 0.0;

    double dist_min;

    // int i_min = 0;
    while (ros::ok())
    {
        if (mission_status == "PAUSED")
        {
            x_d = x_hat;
            y_d = y_hat;
            z_d = z_hat;
            psi_d = psi_hat;
            x_dot_d = 0.0;
            y_dot_d = 0.0;
            z_dot_d = 0.0;
            psi_dot_d = 0.0;
        }
        if (mission_status == "RUNNING")
        {
            if (strategy == "Spline")
            {
                double u_d = 0.5;
                double v_d = 0.0;
                double w_d = 0.0;
                double r_d = 0.0;
                double vel = sqrt(u_d * u_d + v_d * v_d + w_d * w_d);
                double waypoint_distance = (sqrt(pow(x_2 - x_1, 2) + pow(y_2 - y_1, 2) + pow(z_2 - z_1, 2)) + sqrt(pow(x_3 - x_2, 2) + pow(y_3 - y_2, 2) + pow(z_3 - z_2, 2))) / 2;

                double h_ott = waypoint_distance / vel;

                // spline sulle x

                double Mx = 3 / (2 * h_ott * h_ott) * (x_1 - 2 * x_2 + x_3);
                double bx1 = 0;
                double bx2 = Mx / 2;
                double dx1 = x_1;
                double dx2 = x_2;
                double ax1 = Mx / (6 * h_ott);
                double ax2 = -Mx / (6 * h_ott);
                double cx1 = (x_2 - x_1) / h_ott - Mx * h_ott / 6;
                double cx2 = (x_3 - x_2) / h_ott - Mx * h_ott / 3;

                // spline sulle y

                double My = 3 / (2 * h_ott * h_ott) * (y_1 - 2 * y_2 + y_3);
                double by1 = 0;
                double by2 = My / 2;
                double dy1 = y_1;
                double dy2 = y_2;
                double ay1 = My / (6 * h_ott);
                double ay2 = -My / (6 * h_ott);
                double cy1 = (y_2 - y_1) / h_ott - My * h_ott / 6;
                double cy2 = (y_3 - y_2) / h_ott - My * h_ott / 3;

                // spline sulle z

                double Mz = 3 / (2 * h_ott * h_ott) * (z_1 - 2 * z_2 + z_3);
                double bz1 = 0;
                double bz2 = Mz / 2;
                double dz1 = z_1;
                double dz2 = z_2;
                double az1 = Mz / (6 * h_ott);
                double az2 = -Mz / (6 * h_ott);
                double cz1 = (z_2 - z_1) / h_ott - Mz * h_ott / 6;
                double cz2 = (z_3 - z_2) / h_ott - Mz * h_ott / 3;
                int t0 = 0;

                double x[298];
                double y[298];
                double z[298];
                double psi[298];
                Eigen::VectorXd dist(298);

                for (int i = 0; i < 149; i++)
                {

                    // Troviamo il tempo corrispondente all'indice i
                    double t = (i + 1) * h_ott / 149;

                    x[i] = ax1 * pow(t - t0, 3.0) + bx1 * pow(t - t0, 2.0) + cx1 * (t - t0) + dx1;
                    y[i] = ay1 * pow(t - t0, 3.0) + by1 * pow(t - t0, 2.0) + cy1 * (t - t0) + dy1;
                    z[i] = az1 * pow(t - t0, 3.0) + bz1 * pow(t - t0, 2.0) + cz1 * (t - t0) + dz1;
                    psi[i] = atan2(3 * ay1 * pow(t - t0, 2.0) + 2 * by1 * (t - t0) + cy1, 3 * ax1 * pow(t - t0, 2.0) + 2 * bx1 * (t - t0) + cx1);
                }

                for (int i = 149; i < 298; i++)
                {

                    // Troviamo il tempo corrispondente all'indice i
                    double t = (i + 1) * h_ott / 149;

                    x[i] = ax2 * pow(t - t0 - h_ott, 3.0) + bx2 * pow(t - t0 - h_ott, 2.0) + cx2 * (t - t0 - h_ott) + dx2;
                    y[i] = ay2 * pow(t - t0 - h_ott, 3.0) + by2 * pow(t - t0 - h_ott, 2.0) + cy2 * (t - t0 - h_ott) + dy2;
                    z[i] = az2 * pow(t - t0 - h_ott, 3.0) + bz2 * pow(t - t0 - h_ott, 2.0) + cz2 * (t - t0 - h_ott) + dz2;
                    psi[i] = atan2(3 * ay2 * pow(t - t0 - h_ott, 2.0) + 2 * by2 * (t - t0 - h_ott) + cy2, 3 * ax2 * pow(t - t0 - h_ott, 2.0) + 2 * bx2 * (t - t0 - h_ott) + cx2);
                }

                int i_dist_min = 0;
                for (int i = 0; i < 298; i++)
                {
                    dist(i) = sqrt(pow(x_hat - x[i], 2.0) + pow(y_hat - y[i], 2.0) + pow(z_hat - z[i], 2.0));
                    if (i == 0)
                    {
                        dist_min = dist(i);
                    }
                    else
                    {
                        if (dist(i) < dist_min)
                        {
                            dist_min = dist(i);
                            i_dist_min = i;
                        }
                    }
                }

                x_d = x[i_dist_min];
                y_d = y[i_dist_min];
                z_d = z[i_dist_min];
                psi_d = psi[i_dist_min];

                if (i_dist_min > 296)
                {
                    u_d = 0.0;
                    v_d = 0.0;
                    w_d = 0.0;
                    r_d = 0.0;
                    std::string status_req = "PAUSED";
                    std_msgs::String msg;
                    msg.data = status_req;
                    publisher_status.publish(msg);
                }
                x_dot_d = u_d * cos(psi_d) - v_d * sin(psi_d);
                y_dot_d = u_d * sin(psi_d) + v_d * cos(psi_d);
                z_dot_d = w_d;
                psi_dot_d = r_d;
            }
            else if (strategy == "Circumference")
            {
                double x[199];
                double y[199];
                double z[199];
                double psi[199];

                double u_d = 0.5;
                double v_d = 0.0;
                double w_d = 0.0;
                double r_d = 0.0;

                Eigen::VectorXd dist(199);
                double dx = (x_p - x_1) / 100;
                double dy = (y_p - y_1) / 100;
                double dz = (z_p - z_1) / 100;

                for (int i = 0; i < 99; i++)
                {
                    x[i] = x_1 + (i + 1) * dx;
                    y[i] = y_1 + (i + 1) * dy;
                    z[i] = z_1 + (i + 1) * dz;
                    psi[i] = atan2(dy, dx);
                }

                double k = 2 * M_PI / 100;
                double beta = atan2(y_p - y_b, x_p - x_b);

                for (int i = 99; i < 199; i++)
                {
                    double t = (i - 99) * k;

                    x[i] = x_b + cos(beta + t);
                    y[i] = y_b + sin(beta + t);
                    z[i] = z_b;
                    psi[i] = atan2(cos(beta + t), -sin(beta + t));
                }

                int i_dist_min = 0;
                for (int i = 0; i < 199; i++)
                {
                    dist(i) = sqrt(pow(x_hat - x[i], 2.0) + pow(y_hat - y[i], 2.0) + pow(z_hat - z[i], 2.0));
                    if (i == 0)
                    {
                        dist_min = dist(i);
                    }
                    else
                    {
                        if (dist(i) < dist_min)
                        {
                            dist_min = dist(i);
                            i_dist_min = i;
                        }
                    }
                }

                x_d = x[i_dist_min];
                y_d = y[i_dist_min];
                z_d = z[i_dist_min];
                psi_d = psi[i_dist_min];

                if (i_dist_min > 50 && i_dist_min <= 196)
                {
                    u_d = 0.1;
                    v_d = 0.0;
                    w_d = 0.0;
                    r_d = 0.0;
                }
                else if (i_dist_min > 196)
                {
                    u_d = 0.0;
                    v_d = 0.0;
                    w_d = 0.0;
                    r_d = 0.0;
                    std::string status_req = "PAUSED";
                    std_msgs::String msg;
                    msg.data = status_req;
                    publisher_status.publish(msg);
                }
                x_dot_d = u_d * cos(psi_d) - v_d * sin(psi_d);
                y_dot_d = u_d * sin(psi_d) + v_d * cos(psi_d);
                z_dot_d = w_d;
                psi_dot_d = r_d;
            }
            else if (strategy == "UP_DOWN")
            {
                double x[119];
                double y[119];
                double z[119];
                double psi[119];

                double u_d = 0.5;
                double v_d = 0.0;
                double w_d = 0.0;
                double r_d = 0.0;

                Eigen::VectorXd dist(119);
                double d = sqrt(pow(x_b - x_1, 2) + pow(y_b - y_1, 2)) - 1.0;
                double alpha = atan2(y_b - y_1, x_b - x_1); // angolo tra la boa e la posizione del robot
                double x_p = x_1 + d * cos(alpha);
                double y_p = y_1 + d * sin(alpha);
                double z_p = z_b;
                double dx = (x_p - x_1) / 100;
                double dy = (y_p - y_1) / 100;
                double dz = (z_p - z_1) / 100;

                for (int i = 0; i < 99; i++)
                {
                    x[i] = x_1 + (i + 1) * dx;
                    y[i] = y_1 + (i + 1) * dy;
                    z[i] = z_1 + (i + 1) * dz;
                    psi[i] = atan2(dy, dx);
                }

                double dz2 = 0.5 / 20; // passo di risalita su asse z per ripianificazione locale

                for (int i = 99; i < 119; i++)
                {
                    x[i] = x_p;
                    y[i] = y_p;
                    z[i] = z[i - 1] + dz2;
                    psi[i] = psi[i - 1];
                }

                int i_dist_min = 0;
                for (int i = 0; i < 119; i++)
                {
                    dist(i) = sqrt(pow(x_hat - x[i], 2.0) + pow(y_hat - y[i], 2.0) + pow(z_hat - z[i], 2.0));
                    if (i == 0)
                    {
                        dist_min = dist(i);
                    }
                    else
                    {
                        if (dist(i) < dist_min)
                        {
                            dist_min = dist(i);
                            i_dist_min = i;
                        }
                    }
                }

                x_d = x[i_dist_min];
                y_d = y[i_dist_min];
                z_d = z[i_dist_min];
                psi_d = psi[i_dist_min];

                if (i_dist_min > 50 && i_dist_min < 99)
                {
                    u_d = 0.1;
                    v_d = 0.0;
                    w_d = 0.0;
                    r_d = 0.0;
                }
                else if (i_dist_min >= 99 && i_dist_min < 116)
                {
                    u_d = 0.0;
                    v_d = 0.0;
                    w_d = 0.1;
                    r_d = 0.0;
                }
                else if (i_dist_min >= 116)
                {
                    u_d = 0.0;
                    v_d = 0.0;
                    w_d = 0.0;
                    r_d = 0.0;
                    std::string status_req = "PAUSED";
                    std_msgs::String msg;
                    msg.data = status_req;
                    publisher_status.publish(msg);
                }
                x_dot_d = u_d * cos(psi_d) - v_d * sin(psi_d);
                y_dot_d = u_d * sin(psi_d) + v_d * cos(psi_d);
                z_dot_d = w_d;
                psi_dot_d = r_d;
            }
        }

        // Publishing the state
        std::vector<double> des_state = {x_d, y_d, z_d, psi_d, x_dot_d, y_dot_d, z_dot_d, psi_dot_d};
        progetto_robotica::Floats des_state_msg;
        des_state_msg.data = des_state;

        chatter_pub.publish(des_state_msg);
        ros::spinOnce();

        loop_rate.sleep();
    }

    return 0;
}