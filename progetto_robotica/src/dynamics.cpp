#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include <geometry_msgs/Pose.h>            // for accessing -- geometry_msgs Pose()
#include <geometry_msgs/Point.h>           // for accessing -- geometry_msgs Point()
#include <geometry_msgs/Quaternion.h>      // for accessing -- geometry_msgs Quaternion()
#include <tf2/LinearMath/Quaternion.h>     // for tf2 quaternion
#include <eigen3/Eigen/Dense>              // for eigen matrix

double tau_u = 0;
double tau_v = 0;
double tau_w = 0;
double tau_r = 0;

double x = 0;
double y = 0;
double z = 0;
double u = 0;
double v = 0;
double w = 0;
double phi = 0;
double theta = 0;
double psi = 0;
double r = 0;

Eigen::Matrix<double,4,1> eta;
eta << x, y, z, psi;

Eigen::Matrix<double,4,1> nu;
nu << u, v, w, r;


void tauCallback(const geometry_msgs::Point::ConstPtr& msg)
{
    tau_u = msg->x;
    tau_v = msg->y;
    tau_w = msg->z;
    tau_r = msg->r;
}




int main(int argc, char **argv)
{

    ros::init(argc, argv, "dynamics");

    // DEFINIAMO LE COSTANTI DEL SISTEMA

    double const m = 12;
    double const d = 0.338;
    double const l = 0.457;

    double I = 1/12*m*(pow(d,2)+pow(l,2));

    double const X_u_dot = 0;
    double const Y_v_dot = 0;
    double const X_u = 0;
    double const Y_r = 0;
    double const Y_v = 0;
    double const N_r = 0;
    double const N_r_r = 0;
    double const Z_w = 0;

    Eigen::Matrix<double,4,4> M;
    M << m, 0, 0, 0,
         0, m, 0, 0,
         0, 0, m, 0,
         0, 0, 0, I;

    ros::NodeHandle n;

    ros::Publisher chatter_pub = n.advertise<geometry_msgs::Pose>("pose_topic", 1);
    ros::Subscriber sub = n.subscribe("tau_topic", 1000, tauCallback);
    double freq = 100;
    double dt = 1/freq;
    ros::Rate loop_rate(freq);
    tf2::Quaternion q;


    while (ros::ok())
    {

        // DEFINIAMO LA DINAMICA DEL SISTEMA
        
        // MATRICE DI CORIOLIS
        Eigen::Matrix<double,4,4> C;
        C << 0, 0, 0, Y_v_dot*v, + Y_r*r,
             0, 0, 0, -X_u_dot*u,
             0, 0, 0, 0,
            -Y_v_dot*v - Y_r*r, X_u_dot*u, 0, 0;
        

        // MATRICE DI AMMORTIZZAMENTO
        Eigen::Matrix<double,4,4> D;
        C.diagonal() << -X_u, -Y_v, -Z_w, -N_r-N_r_r*r;
        
        // VETTORE DI FORZE E MOMENTI
        Eigen::Matrix<double,3,1> tau;
        tau << tau_u, tau_v, tau_w, tau_r;

        // VETTORE DELLE VELOCITA'
        Eigen::Matrix<double,4,1> nu_k1;
        nu_k1 = dt*M.inverse()*(tau - C*nu - D*nu) + nu;
        nu = nu_k1;

        // VETTORE DELLE POSIZIONI
        Eigen::Matrix<double,4,4> Jacobian;
        Jacobian << cos(psi), -sin(psi), 0, 0,
                    sin(psi), cos(psi), 0, 0,
                    0, 0, 1, 0,
                    0, 0, 0, 1;

        Eigen::Matrix<double,4,1> eta_dot;
        eta_dot = Jacobian*nu;
        Eigen::Matrix<double,4,1> eta_k1;
        eta_k1 = dt*eta_dot + eta;
        eta = eta_k1;


        // SETTARE LA POSIZIONE DEL MODELLO
        q.setRPY(0, 0, psi);
        geometry_msgs::Point point;
        point.x = 0;
        point.y = 0;
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

        loop_rate.sleep();
        
    }

    return 0;
}