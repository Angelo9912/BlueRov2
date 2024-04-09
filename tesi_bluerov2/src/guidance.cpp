#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include <ros/console.h>
#include <eigen3/Eigen/Dense>        // for eigen matrix
#include "tesi_bluerov2/Floats.h"    // for accessing -- tesi_bluerov2 Floats()
#include "tesi_bluerov2/waypoints.h" // for accessing -- tesi_bluerov2 Floats_String()
#include <vector>
#include "yaml-cpp/yaml.h" // for yaml

// Declare callback variables
double x_hat = 0.0;     // x stimata
double y_hat = 0.0;     // y stimata
double z_hat = 0.0;     // z stimata
double phi_hat = 0.0;   // phi stimata
double theta_hat = 0.0; // theta stimata
double psi_hat = 0.0;   // psi stimata
double u_hat = 0.0;     // velocità di surge stimata
double v_hat = 0.0;     // velocità di sway stimata
double w_hat = 0.0;     // velocità di heave stimata
double p_hat = 0.0;     // velocità angolare di roll stimata
double q_hat = 0.0;     // velocità angolare di pitch stimata
double r_hat = 0.0;     // velocità angolare di yaw stimata
double speed = 0.0;     // velocità di crociera

// coordiate spline a 3 punti

double x_1 = 0.0; // coordiata x del primo waypoint
double y_1 = 0.0; // coordiata y del primo waypoint
double z_1 = 0.0; // coordiata z del primo waypoint

double x_t = 0.0; // coordiata x del target da raggiungere a fine missione
double y_t = 0.0; // coordiata y del target da raggiungere a fine missione
double z_t = 0.0; // coordiata z del target da raggiungere a fine missione

std::string strategy = "";       // strategia di controllo
std::string mission_status = ""; // stato di missione
std::vector<double> way_spline;  // vettore che contiene i waypoint

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////// DEFINIZIONE CALLBACK /////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void statusCallback(const std_msgs::String::ConstPtr &msg) // CALLBACK macchina a stati
{
    mission_status = msg->data.c_str();
}

void waypointCallback(const tesi_bluerov2::waypoints::ConstPtr &msg) // CALLBACK che riceve i waypoint
{
    if (msg->strategy == "Spline")
    {
        x_1 = x_hat;
        y_1 = y_hat;
        z_1 = z_hat;
        way_spline = msg->waypoints;
        speed = msg->speed;
        strategy = msg->strategy;
    }
    else if (msg->strategy == "Target") // quando interrompiamo l'esplorazione e ci dirigiamo al target
    {
        x_1 = x_hat;
        y_1 = y_hat;
        z_1 = z_hat;
        x_t = msg->waypoints[0];
        y_t = msg->waypoints[1];
        z_t = msg->waypoints[2];
        speed = msg->speed;
        strategy = msg->strategy;
    }
}

void estStateCallback(const tesi_bluerov2::Floats::ConstPtr &msg) // CALLBACK in cui prendiamo gli stati "stimati"
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
        r_hat = 0.0;
    }
    else
    {
        x_hat = msg->data[0];
        y_hat = msg->data[1];
        z_hat = msg->data[2];
        psi_hat = msg->data[3];
        phi_hat = msg->data[4];
        theta_hat = msg->data[5];
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
    ros::init(argc, argv, "guidance");
    ros::NodeHandle n;
    // Definisco publisher e subscriber
    ros::Publisher guidance_pub = n.advertise<tesi_bluerov2::Floats>("desired_state_topic", 1);            // publisher stato desiderato
    ros::Publisher publisher_status = n.advertise<std_msgs::String>("manager/status_requested_topic", 10); // publisher stato richiesto al manager di missione

    ros::Subscriber sub_est_state = n.subscribe("est_state_UKF_topic", 1, estStateCallback); // sottoscrizione alla topic di stato stimato
    ros::Subscriber sub_waypoint = n.subscribe("waypoints_topic", 1, waypointCallback);      // sottoscrizione alla topic di waypoint
    ros::Subscriber sub_status = n.subscribe("manager/status_topic", 1, statusCallback);     // sottoscrizione alla topic di mission status
    double freq = 10.0;                                                                      // frequenza di lavoro
    double dt = 1 / freq;                                                                    // tempo di campionamento
    ros::Rate loop_rate(freq);

    // Inizializzo i valori di stato desiderato
    double x_d = 0.0;         // x desiderata
    double y_d = 0.0;         // y desiderata
    double z_d = 0.0;         // z desiderata
    double phi_d = 0.0;       // phi desiderata
    double theta_d = 0.0;     // theta desiderata
    double psi_d = 0.0;       // psi desiderata
    double x_dot_d = 0.0;     // x derivata desiderata
    double y_dot_d = 0.0;     // y derivata desiderata
    double z_dot_d = 0.0;     // z derivata desiderata
    double phi_dot_d = 0.0;   // phi derivata desiderata
    double theta_dot_d = 0.0; // theta derivata desiderata
    double psi_dot_d = 0.0;   // psi derivata desiderata
    double u_d = 0.0;
    double v_d = 0.0;
    double w_d = 0.0;
    double p_d = 0.0;
    double q_d = 0.0;
    double r_d = 0.0;

    double dist_min;
    double waypoint_distance; // distanza tra i waypoint
    double h;                 // intervallo di tempo tra due waypoint
    double t0;                // tempo iniziale
    double delta_t;           // intervallo di tempo tra due punti
    int dim;                  // dimensione del vettore

    // Definisco le matrici e i vettori necessari
    int n_waypoints;
    Eigen::MatrixXd J(6, 6); // matrice jacobiana
    Eigen::VectorXd nu_d(6); // vettore delle velocità
    Eigen::VectorXd pose_d_dot(6);
    Eigen::VectorXd Mx(1);
    Eigen::VectorXd My(1);
    Eigen::VectorXd Mz(1);
    Eigen::VectorXd Mx_tmp(1);
    Eigen::VectorXd My_tmp(1);
    Eigen::VectorXd Mz_tmp(1);
    Eigen::VectorXd Sx(1);
    Eigen::VectorXd Sy(1);
    Eigen::VectorXd Sz(1);
    Eigen::VectorXd ax(1);
    Eigen::VectorXd ay(1);
    Eigen::VectorXd az(1);
    Eigen::VectorXd bx(1);
    Eigen::VectorXd by(1);
    Eigen::VectorXd bz(1);
    Eigen::VectorXd cx(1);
    Eigen::VectorXd cy(1);
    Eigen::VectorXd cz(1);
    Eigen::VectorXd dx(1);
    Eigen::VectorXd dy(1);
    Eigen::VectorXd dz(1);
    Eigen::VectorXd way_spline_x(1); // vettore che contiene le x delle spline
    Eigen::VectorXd way_spline_y(1); // vettore che contiene le y delle spline
    Eigen::VectorXd way_spline_z(1); // vettore che contiene le z delle splineu

    // int i_min = 0;
    while (ros::ok())
    {

        if (mission_status == "COMPLETED")
        {
            // Vogliamo stare fermi nella posizione in cui ci troviamo
            x_d = x_hat;
            y_d = y_hat;
            z_d = z_hat;
            phi_d = phi_hat;
            theta_d = theta_hat;
            psi_d = psi_hat;
            x_dot_d = 0.0;
            y_dot_d = 0.0;
            z_dot_d = 0.0;
            phi_dot_d = 0.0;
            theta_dot_d = 0.0;
            psi_dot_d = 0.0;
        }
        else if (mission_status == "RUNNING")
        {
            // Calcolo la distanza minima tra il ROV e i waypoint
            if (strategy == "Spline")
            {
                n_waypoints = way_spline.size() / 3 + 1;
                way_spline_x.resize(n_waypoints);
                way_spline_y.resize(n_waypoints);
                way_spline_z.resize(n_waypoints);
                way_spline_x(0) = x_1;
                way_spline_y(0) = y_1;
                way_spline_z(0) = z_1;
                for (int i = 0; i < n_waypoints - 1; i++)
                {
                    way_spline_x(i + 1) = way_spline[i * 3];
                    way_spline_y(i + 1) = way_spline[i * 3 + 1];
                    way_spline_z(i + 1) = way_spline[i * 3 + 2];
                }
                ROS_WARN_STREAM("WAYPOINT X: " << way_spline_x);
                ROS_WARN_STREAM("WAYPOINT Y: " << way_spline_y);
                ROS_WARN_STREAM("WAYPOINT Z: " << way_spline_z);

                u_d = 0.5;
                v_d = 0.0;
                w_d = 0.0;
                p_d = 0.0;
                q_d = 0.0;
                r_d = 0.0;
                // double vel = sqrt(u_d * u_d + v_d * v_d + w_d * w_d);
                waypoint_distance;
                for (int i = 0; i < n_waypoints - 1; i++)
                {
                    waypoint_distance += sqrt(pow(way_spline_x(i + 1) - way_spline_x(i), 2) + pow(way_spline_y(i + 1) - way_spline_y(i), 2) + pow(way_spline_z(i + 1) - way_spline_z(i), 2)) / n_waypoints;
                }
                h = waypoint_distance / speed;
                ROS_WARN("H: %f", h);
                // interpolazione mediante natural cubic spline
                Eigen::MatrixXd A(n_waypoints - 2, n_waypoints - 2); // matrice moltiplicata al vettore M
                A << 4 * Eigen::MatrixXd::Identity(n_waypoints - 2, n_waypoints - 2);
                if (A.rows() > 1 && A.cols() > 1)
                {
                    for (int i = 0; i < n_waypoints - 2; i++)
                    {
                        A(i, i + 1) = 1;
                        A(i + 1, i) = 1;
                    }
                }
                ROS_WARN_STREAM("A: " << A);

                ROS_WARN("SONO PRIMA DEI RESIZE");
                Mx.resize(n_waypoints);
                My.resize(n_waypoints);
                Mz.resize(n_waypoints);
                Mx_tmp.resize(n_waypoints - 2);
                My_tmp.resize(n_waypoints - 2);
                Mz_tmp.resize(n_waypoints - 2);
                ROS_WARN("M DEFINITE");
                Sx.resize(n_waypoints - 2);
                Sy.resize(n_waypoints - 2);
                Sz.resize(n_waypoints - 2);
                ROS_WARN("S DEFINITE");
                ax.resize(n_waypoints - 1);
                ay.resize(n_waypoints - 1);
                az.resize(n_waypoints - 1);
                ROS_WARN("AX DEFINITE");
                bx.resize(n_waypoints - 1);
                by.resize(n_waypoints - 1);
                bz.resize(n_waypoints - 1);
                ROS_WARN("BX DEFINITE");
                cx.resize(n_waypoints - 1);
                cy.resize(n_waypoints - 1);
                cz.resize(n_waypoints - 1);
                ROS_WARN("CX DEFINITE");
                dx.resize(n_waypoints - 1);
                dy.resize(n_waypoints - 1);
                dz.resize(n_waypoints - 1);
                ROS_WARN("SONO DOPO I RESIZE");

                for (int i = 0; i < n_waypoints - 2; i++)
                {
                    Sx(i) = 6 / (h * h) * (way_spline_x(i) - 2 * way_spline_x(i + 1) + way_spline_x(i + 2));
                    Sy(i) = 6 / (h * h) * (way_spline_y(i) - 2 * way_spline_y(i + 1) + way_spline_y(i + 2));
                    Sz(i) = 6 / (h * h) * (way_spline_z(i) - 2 * way_spline_z(i + 1) + way_spline_z(i + 2));
                }
                Mx_tmp = A.inverse() * Sx;
                Mx << 0, Mx_tmp, 0;
                My_tmp = A.inverse() * Sy;
                My << 0, My_tmp, 0;
                Mz_tmp = A.inverse() * Sz;
                Mz << 0, Mz_tmp, 0;
                ROS_WARN("MATRICI DEFINITE");
                ROS_WARN_STREAM("Mx: " << Mx);
                ROS_WARN_STREAM("My: " << My);
                ROS_WARN_STREAM("Mz: " << Mz);

                for (int i = 0; i < n_waypoints - 1; i++)
                {
                    ax(i) = (Mx(i + 1) - Mx(i)) / (6 * h);
                    ay(i) = (My(i + 1) - My(i)) / (6 * h);
                    az(i) = (Mz(i + 1) - Mz(i)) / (6 * h);
                    bx(i) = Mx(i) / 2;
                    by(i) = My(i) / 2;
                    bz(i) = Mz(i) / 2;
                    cx(i) = (way_spline_x(i + 1) - way_spline_x(i)) / h - h / 6 * (2 * Mx(i) + Mx(i + 1));
                    cy(i) = (way_spline_y(i + 1) - way_spline_y(i)) / h - h / 6 * (2 * My(i) + My(i + 1));
                    cz(i) = (way_spline_z(i + 1) - way_spline_z(i)) / h - h / 6 * (2 * Mz(i) + Mz(i + 1));
                    dx(i) = way_spline_x(i);
                    dy(i) = way_spline_y(i);
                    dz(i) = way_spline_z(i);
                }

                t0 = 0;
                delta_t = 0.1;
                dim = (int)(n_waypoints * h / delta_t);
                ROS_WARN("DIM: %d", dim);
                Eigen::VectorXd x(dim);
                Eigen::VectorXd y(dim);
                Eigen::VectorXd z(dim);
                ROS_WARN("XYZ DEFINITI");
                Eigen::VectorXd phi(dim);
                Eigen::VectorXd theta(dim);
                Eigen::VectorXd psi(dim);
                ROS_WARN("PHI THETA PSI DEFINITI");
                Eigen::VectorXd dist(dim);
                int j_prec = 0;
                for (int i = 0; i < n_waypoints; i++)
                {
                    ROS_WARN("SONO NEL PRIMO FOR");
                    for (int j = j_prec; j < j_prec + h / delta_t; j++)
                    {
                        ROS_WARN("SONO NEL SECONDO FOR");
                        x(j) = ax(i) * pow(t0 + j * delta_t, 3) + bx(i) * pow(t0 + j * delta_t, 2) + cx(i) * (t0 + j * delta_t) + dx(i);
                        y(j) = ay(i) * pow(t0 + j * delta_t, 3) + by(i) * pow(t0 + j * delta_t, 2) + cy(i) * (t0 + j * delta_t) + dy(i);
                        z(j) = az(i) * pow(t0 + j * delta_t, 3) + bz(i) * pow(t0 + j * delta_t, 2) + cz(i) * (t0 + j * delta_t) + dz(i);
                        phi(j) = 0.0;
                        theta(j) = 0.0;
                        psi(j) = atan2(3 * ay(i) * pow(t0 + j * delta_t, 2) + 2 * by(i) * (t0 + j * delta_t) + cy(i), 3 * ax(i) * pow(t0 + j * delta_t, 2) + 2 * bx(i) * (t0 + j * delta_t) + cx(i));
                    }
                    j_prec = j_prec + (int)(h / delta_t);
                    t0 = (i + 1) * h;
                }

                int i_dist_min = 0;
                for (int i = 0; i < dim - 1; i++)
                {
                    dist(i) = sqrt(pow(x(i + 1) - x(i), 2.0) + pow(y(i + 1) - y(i), 2.0) + pow(z(i + 1) - z(i), 2.0));
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

                x_d = x(i_dist_min);
                y_d = y(i_dist_min);
                z_d = z(i_dist_min);
                phi_d = phi(i_dist_min);
                theta_d = theta(i_dist_min);
                psi_d = psi(i_dist_min);
                u_d = 0.5;
                v_d = 0.0;
                if (z_hat < z_d)
                {
                    w_d = 0.1;
                }
                else if (z_hat == z_d)
                {
                    w_d = 0.0;
                }
                else
                {
                    w_d = -0.1;
                }
                p_d = 0.0;
                q_d = 0.0;
                r_d = 0.0;
                if (i_dist_min > dim - 10)
                {
                    // u_d = 0.0;
                    // v_d = 0.0;
                    // w_d = 0.0;
                    // r_d = 0.0;
                    std::string status_req = "PAUSED";
                    std_msgs::String msg;
                    msg.data = status_req;
                    publisher_status.publish(msg);
                }
                // Define Jacobian Matrix
                J << cos(psi_d) * cos(theta_d), cos(psi_d) * sin(phi_d) * sin(theta_d) - cos(phi_d) * sin(psi_d), sin(phi_d) * sin(psi_d) + cos(phi_d) * cos(psi_d) * sin(theta_d), 0, 0, 0,
                    cos(theta_d) * sin(psi_d), cos(phi_d) * cos(psi_d) + sin(phi_d) * sin(psi_d) * sin(theta_d), cos(phi_d) * sin(psi_d) * sin(theta_d) - cos(psi_d) * sin(phi_d), 0, 0, 0,
                    -sin(theta_d), cos(theta_d) * sin(phi_d), cos(phi_d) * cos(theta_d), 0, 0, 0,
                    0, 0, 0, 1, sin(phi_d) * tan(theta_d), cos(phi_d) * tan(theta_d),
                    0, 0, 0, 0, cos(phi_d), -sin(phi_d),
                    0, 0, 0, 0, sin(phi_d) / cos(theta_d), cos(phi_d) / cos(theta_d);

                nu_d << u_d, v_d, w_d, p_d, q_d, r_d;

                pose_d_dot = J * nu_d;
                x_dot_d = pose_d_dot(0);
                y_dot_d = pose_d_dot(1);
                z_dot_d = pose_d_dot(2);
                phi_dot_d = pose_d_dot(3);
                theta_dot_d = pose_d_dot(4);
                psi_dot_d = pose_d_dot(5);

                // else if (strategy == "Target")
                // {
                // }

                std::vector<double> des_state = {x_d, y_d, z_d, phi_d, theta_d, psi_d, x_dot_d, y_dot_d, z_dot_d, phi_dot_d, theta_dot_d, psi_dot_d};
                tesi_bluerov2::Floats des_state_msg;
                des_state_msg.data = des_state;
                // Publishing the state

                guidance_pub.publish(des_state_msg);
                ROS_WARN("HO PUBBLICATO");
            }
        }
        else if (mission_status == "PAUSED")
        {
            std::string status_req = "RUNNING";
            std_msgs::String msg;
            msg.data = status_req;
            publisher_status.publish(msg);
        }

        ros::spinOnce();

        loop_rate.sleep();
    }
    return 0;
}