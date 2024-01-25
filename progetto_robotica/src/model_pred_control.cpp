#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include <ros/console.h>
#include <eigen3/Eigen/Dense>         // for eigen matrix
#include "progetto_robotica/Floats.h" // for accessing -- progetto_robotica Floats()
#include <vector>
#include "yaml-cpp/yaml.h" // for yaml

/// Declare callback variables
double x_d = 0.0;
double y_d = 0.0;
double z_d = 0.0;
double psi_d = 0.0;
double x_dot_d = 0.0;
double y_dot_d = 0.0;
double z_dot_d = 0.0;
double psi_dot_d = 0.0;
double x_hat = 0.0;
double y_hat = 0.0;
double z_hat = 0.0;
double psi_hat = 0.0;
double u_hat = 0.0;
double v_hat = 0.0;
double w_hat = 0.0;
double r_hat = 0.0;

// Define callback functions
void desStateCallback(const progetto_robotica::Floats::ConstPtr &msg)
{
    if (msg->data[0] != msg->data[0])
    {
        x_d = 0.0;
        y_d = 0.0;
        z_d = 0.0;
        psi_d = 0.0;
        x_dot_d = 0.0;
        y_dot_d = 0.0;
        z_dot_d = 0.0;
        psi_dot_d = 0.0;
    }
    else
    {
        x_d = msg->data[0];
        y_d = msg->data[1];
        z_d = msg->data[2];
        psi_d = msg->data[3];
        x_dot_d = msg->data[4];
        y_dot_d = msg->data[5];
        z_dot_d = msg->data[6];
        psi_dot_d = msg->data[7];
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
    ros::init(argc, argv, "model_pred_control");
    ros::NodeHandle n;
    ros::Publisher chatter_pub = n.advertise<progetto_robotica::Floats>("tau_topic", 1);
    ros::Subscriber sub_des_state = n.subscribe("desired_state_topic", 1, desStateCallback);
    ros::Subscriber sub_est_state = n.subscribe("state_topic", 1, estStateCallback);
    double freq = 10;
    double dt = 1 / freq;
    ros::Rate loop_rate(freq);
    // Import parameters from YAML file
    double m = 0.0;
    double d = 0.0;
    double l = 0.0;
    double I = 0.0;
    double X_u_dot = 0.0;
    double Y_v_dot = 0.0;
    double X_u = 0.0;
    double Y_r = 0.0;
    double Y_v = 0.0;
    double N_r = 0.0;
    double N_r_r = 0.0;
    double Z_w = 0.0;
    double N_v = 0.0;

    n.getParam("m", m);
    n.getParam("d", d);
    n.getParam("l", l);
    I = 1.0 / 12.0 * m * (pow(d, 2) + pow(l, 2));
    n.getParam("X_u_dot", X_u_dot);
    n.getParam("Y_v_dot", Y_v_dot);
    n.getParam("X_u", X_u);
    n.getParam("Y_r", Y_r);
    n.getParam("Y_v", Y_v);
    n.getParam("N_r", N_r);
    n.getParam("N_r_r", N_r_r);
    n.getParam("Z_w", Z_w);
    n.getParam("N_v", N_v);

    while (ros::ok())
    {
        // Computing the control law
        Eigen::Matrix<double, 8, 8> A_cont;
        A_cont << 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
        Eigen::Matrix<double, 8, 8> A = dt * A_cont + Eigen::Matrix<double, 8, 8>::Identity();

        Eigen::Matrix<double, 8, 4> B_cont;
        B_cont << 0.0, 0.0, 0.0, 0.0,
            1.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0,
            0.0, 1.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 1.0, 0.0,
            0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 1.0;

        Eigen::Matrix<double, 8, 4> B = dt * B_cont;

        Eigen::Matrix<double, 8, 8> C = Eigen::Matrix<double, 8, 8>::Identity();

        Eigen::Matrix<double, 8, 4> Zero;
        Zero.setZero();
        Eigen::Matrix<double, 32, 16> PHI;
        PHI << C * B, Zero, Zero, Zero,
            C * A * B, C * B, Zero, Zero,
            C * A * A * B, C * A * B, C * B, Zero,
            C * A * A * A * B, C * A * A * B, C * A * B, C * B;

        Eigen::Matrix<double, 32, 8> F;
        F << C * A,
            C * A * A,
            C * A * A * A,
            C * A * A * A * A;

        Eigen::Matrix<double, 16, 16> Rc = 0.01 * Eigen::Matrix<double, 16, 16>::Identity();

        // Estrazione valori desired state

        Eigen::Matrix<double, 8, 1> des_state;
        des_state << x_d, x_dot_d, y_d, y_dot_d, z_d, z_dot_d, psi_d, psi_dot_d;

        Eigen::Matrix<double, 32, 1> r;
        r << des_state, des_state, des_state, des_state;

        // Trasformazione da body a world frame
        Eigen::Matrix<double, 4, 4> J;
        J << cos(psi_hat), -sin(psi_hat), 0.0, 0.0,
            sin(psi_hat), cos(psi_hat), 0.0, 0.0,
            0.0, 0.0, 1.0, 0.0,
            0.0, 0.0, 0.0, 1.0;

        Eigen::Matrix<double, 4, 1> est_nu;
        est_nu << u_hat, v_hat, w_hat, r_hat;

        Eigen::Matrix<double, 4, 1> est_vel;
        est_vel = J * est_nu;

        Eigen::Matrix<double, 8, 1> est_state;
        est_state << x_hat, est_vel(0), y_hat, est_vel(1), z_hat, est_vel(2), psi_hat, est_vel(3);

        Eigen::Matrix<double, 32, 1> prediction;
        prediction = F * est_state;
        for (int i = 6; i < 32; i += 8)
        {
            double rem = std::fmod(prediction(i) + M_PI, 2 * M_PI);
            if (rem < 0)
            {
                rem += 2 * M_PI;
            }
            prediction(i) = rem - M_PI;
        }

        Eigen::Matrix<double, 32, 1> error;
        error = r - prediction;

        for (int i = 6; i < 32; i += 8)
        {
            double angle_tmp = error(i);
            if (angle_tmp > 0)
            {
                if (angle_tmp > 2 * M_PI - angle_tmp)
                {
                    error(i) = angle_tmp - 2 * M_PI;
                }
                else
                {
                    error(i) = angle_tmp;
                }
            }
            else
            {
                angle_tmp = -angle_tmp;
                if (angle_tmp > 2 * M_PI - angle_tmp)
                {
                    error(i) = 2 * M_PI - angle_tmp;
                }
                else
                {
                    error(i) = -angle_tmp;
                }
            }
        }

        Eigen::Matrix<double, 16, 1> DeltaU;
        DeltaU = (PHI.transpose() * PHI + Rc).inverse() * PHI.transpose() * (error);
        Eigen::Matrix<double, 4, 1> u;
        u << DeltaU(0), DeltaU(1), DeltaU(2), DeltaU(3);

        // Computing the torques
        // Actually, the torques are computed transforming the MPC control law in feedback linearization input
        Eigen::Matrix<double, 4, 1> torques_vec;
        torques_vec(0) = m * u(0) * cos(psi_hat) - X_u * u_hat + m * u(1) * sin(psi_hat);
        torques_vec(1) = m * u(1) * cos(psi_hat) - Y_v * v_hat - Y_r * r_hat - m * u(0) * sin(psi_hat);
        torques_vec(2) = m * u(2) - Z_w * w_hat;
        torques_vec(3) = I * u(3) - N_v * v_hat - N_r * r_hat;

        std::vector<double> torques = {torques_vec(0), torques_vec(1), torques_vec(2), torques_vec(3)};

        // Publishing the torques
        progetto_robotica::Floats torques_msg;
        torques_msg.data = torques;

        chatter_pub.publish(torques_msg);

        ros::spinOnce();

        loop_rate.sleep();
    }

    return 0;
}
