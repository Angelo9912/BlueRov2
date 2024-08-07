#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include <ros/console.h>
#include <rosbag/bag.h>
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
int clockwise = 0;      // flag per indicare se il giro è in senso orario
int up = 0;             // flag per indicare se la traiettoria è in salita
int CIRCUMFERENCE_phase = 1; // fase della circonferenza

double x_1 = 0.0;   // coordiata x del primo waypoint
double y_1 = 0.0;   // coordiata y del primo waypoint
double z_1 = 0.0;   // coordiata z del primo waypoint
double psi_1 = 0.0; // orientamento del primo waypoint

double x_b = 0.0; // coordiata x del centro della circonferenza
double y_b = 0.0; // coordiata y del centro della circonferenza
double z_b = 0.0; // coordiata z del centro della circonferenza

double x_p = 0.0; // coordiata x del punto della circonferenza
double y_p = 0.0; // coordiata y del punto della circonferenza
double z_p = 0.0; // coordiata z del punto della circonferenza

double x_t = 0.0;     // coordiata x del target da raggiungere a fine missione
double y_t = 0.0;     // coordiata y del target da raggiungere a fine missione
double z_t = 0.0;     // coordiata z del target da raggiungere a fine missione
bool is_psi_adjusted; // flag per controllare se psi è stato aggiustato
bool is_z_adjusted;   // flag per regolare la z e portarci sul piano xy in cui fare la guida
int way_counter = 0;  // contatore per waypoint
bool UP_DOWN_first_phase = true;
std::string strategy = "";       // strategia di controllo
std::string mission_status = ""; // stato di missione
std::vector<double> way_spline;  // vettore che contiene i waypoint
std::string GNC_status = "NOT_READY";

// velocità costanti da comandare al robot
double const surge_speed = 0.5;            // velocità da seguire normalmente
double const surge_speed_replanning = 0.2; // velocità da seguire durante ripianificazioni

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////// DEFINIZIONE CALLBACK /////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void statusCallback(const std_msgs::String::ConstPtr &msg) // CALLBACK macchina a stati
{
    mission_status = msg->data.c_str();
}

void GNCstatusCallback(const std_msgs::String::ConstPtr &msg) // CALLBACK che riceve lo stato del GNC
{
    GNC_status = msg->data;
}

void waypointCallback(const tesi_bluerov2::waypoints::ConstPtr &msg) // CALLBACK che riceve i waypoint
{
    if (msg->strategy == "Spline")
    {
        x_1 = x_hat;
        y_1 = y_hat;
        z_1 = z_hat;
        psi_1 = psi_hat;
        way_counter = 1;
        is_psi_adjusted = false;
        way_spline = msg->waypoints;
        speed = msg->speed;
        strategy = msg->strategy;
    }
    else if (msg->strategy == "Rect")
    {
        x_1 = x_hat;
        y_1 = y_hat;
        z_1 = z_hat;
        psi_1 = psi_hat;
        way_counter = 1;
        is_psi_adjusted = false;
        is_z_adjusted = false;
        way_spline = msg->waypoints;
        speed = msg->speed;
        strategy = msg->strategy;
    }
    else if (msg->strategy == "Circumference")
    {
        x_1 = x_hat;
        y_1 = y_hat;
        z_1 = z_hat;
        psi_1 = psi_hat;
        is_psi_adjusted = false;
        way_counter = 1;
        is_psi_adjusted = false;
        CIRCUMFERENCE_phase = 1;
        x_b = msg->waypoints[0];
        y_b = msg->waypoints[1];
        z_b = msg->waypoints[2];
        speed = msg->speed;
        strategy = msg->strategy;
        clockwise = msg->approach;
    }
    else if (msg->strategy == "UP_DOWN")
    {
        x_1 = x_hat;
        y_1 = y_hat;
        z_1 = z_hat;
        psi_1 = psi_hat;
        way_counter = 1;
        is_psi_adjusted = false;
        x_p = msg->waypoints[0];
        y_p = msg->waypoints[1];
        z_p = msg->waypoints[2];
        speed = msg->speed;
        strategy = msg->strategy;
        up = msg->approach;
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

double angleDifference(double e)
{
    if (e > 0)
    {
        if (e > 2 * M_PI - e)
        {
            e = -(2 * M_PI - e);
        }
        else
        {
            e = e;
        }
    }
    else
    {
        e = -e;
        if (e > 2 * M_PI - e)
        {
            e = 2 * M_PI - e;
        }
        else
        {
            e = -e;
        }
    }
    return e;
}

double wrapToPi(double x)
{
    x = fmod(x * 180 / M_PI + 180, 360);
    if (x < 0)
        x += 360;
    return (x - 180) * M_PI / 180;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "guidance");
    ros::NodeHandle n;
    // Definisco publisher e subscriber
    ros::Publisher guidance_pub = n.advertise<tesi_bluerov2::Floats>("state/desired_state_topic", 1);              // publisher stato desiderato
    ros::Publisher publisher_status = n.advertise<std_msgs::String>("manager/mission_status_requested_topic", 10); // publisher stato richiesto al manager di missione
    ros::Publisher publisher_gnc_status = n.advertise<std_msgs::String>("manager/GNC_status_requested_topic", 10); // publisher stato richiesto al GNC

    ros::Subscriber sub_gnc_status = n.subscribe("manager/GNC_status_topic", 1, GNCstatusCallback); // sottoscrizione alla topic di stato del GNC
    // ros::Subscriber sub_est_state = n.subscribe("state/est_state_topic_no_dyn_imu", 1, estStateCallback);          // sottoscrizione alla topic di stato stimato
    ros::Subscriber sub_est_state = n.subscribe("state/est_state_EKF_no_dyn_imu_topic", 1, estStateCallback); // sottoscrizione alla topic di stato stimato
    ros::Subscriber sub_waypoint = n.subscribe("waypoints_topic", 1, waypointCallback);                       // sottoscrizione alla topic di waypoint
    ros::Subscriber sub_status = n.subscribe("manager/mission_status_topic", 1, statusCallback);              // sottoscrizione alla topic di mission status

    double freq = 50.0;   // frequenza di lavoro
    double dt = 1 / freq; // tempo di campionamento
    ros::Rate loop_rate(freq);
    double i_psi = 0.0;
    int post_up_down = 0;        // intero che mi dice se la guida rect deve ripartire a seguito di un up_down
    double post_up_down_x = 0.0; // x del punto di ripartenza
    double post_up_down_y = 0.0; // y del punto di ripartenza
    double post_up_down_z = 0.0; // z del punto di ripartenza
    rosbag::Bag des_state_bag;
    std::string path = ros::package::getPath("tesi_bluerov2");
    des_state_bag.open(path + "/bag/des_state.bag", rosbag::bagmode::Write);

    tesi_bluerov2::Floats des_state_msg;

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

    double u_d_tmp; // velocità di surge per tenere conto della velocità a cui siamo arrivati
    double w_d_tmp; // velocità di heave per tenere conto della velocità a cui siamo arrivati
    double max_w_d = 0.3;
    double delta_w = 0.03;
    double delta_u = 0.05;
    double step = 1.0 / 50.0; // lunghezza segmentini che costituiscono il segmento che congiunge due waypoint successivi
    bool is_u_d_tmp_initialized = false;
    bool is_w_d_tmp_initialized = false;

    double dist_min;
    double dist_min_psi;

    double waypoint_distance; // distanza tra i waypoint
    double h;                 // intervallo di tempo tra due waypoint
    double delta_t = 0.1;     // intervallo di tempo tra due punti
    int dim;                  // dimensione del vettore

    // Definisco le matrici e i vettori necessari
    int n_waypoints;
    Eigen::MatrixXd J(6, 6); // matrice jacobiana
    Eigen::VectorXd nu_d(6); // vettore delle velocità
    Eigen::VectorXd pose_d_dot(6);
    Eigen::VectorXd Mx(1);
    Eigen::VectorXd My(1);
    Eigen::VectorXd Mx_tmp(1);
    Eigen::VectorXd My_tmp(1);
    Eigen::VectorXd Sx(1);
    Eigen::VectorXd Sy(1);
    Eigen::VectorXd ax(1);
    Eigen::VectorXd ay(1);
    Eigen::VectorXd bx(1);
    Eigen::VectorXd by(1);
    Eigen::VectorXd cx(1);
    Eigen::VectorXd cy(1);
    Eigen::VectorXd dx(1);
    Eigen::VectorXd dy(1);
    Eigen::VectorXd way_spline_x(1); // vettore che contiene le x delle spline
    Eigen::VectorXd way_spline_y(1); // vettore che contiene le y delle spline
    Eigen::VectorXd way_spline_z(1); // vettore che contiene le z delle spline
    Eigen::VectorXd dist_psi(1);     // vettore che contiene le distanze tra psi desiderato e psi stimato
    // int i_min = 0;
    bool is_init = true;
    double init_time;
    while (ros::ok())
    {
        if (GNC_status == "CONTROLLER_READY")
        {
            std::string status_req = "GUIDANCE_READY";
            std_msgs::String msg;
            msg.data = status_req;
            if (is_init)
            {
                init_time = ros::Time::now().toSec();
                is_init = false;
            }
            if (ros::Time::now().toSec() - init_time >= 15.0)
            {
                publisher_gnc_status.publish(msg);
            }
        }
        else if (GNC_status == "GNC_READY")
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

                    waypoint_distance = 0;
                    Eigen::VectorXd waypoint_distance_vector(n_waypoints - 1);
                    Eigen::VectorXd delta_t_vector(n_waypoints - 1);

                    for (int i = 0; i < n_waypoints - 1; i++)
                    {
                        waypoint_distance_vector(i) = sqrt(pow(way_spline_x(i + 1) - way_spline_x(i), 2) + pow(way_spline_y(i + 1) - way_spline_y(i), 2));
                    }

                    waypoint_distance = waypoint_distance_vector.sum();
                    h = waypoint_distance / speed;
                    dim = 0;
                    for (int i = 0; i < n_waypoints - 1; i++)
                    {
                        delta_t_vector(i) = h * 0.05 / waypoint_distance_vector(i); // Un punto ogni 5cm
                        dim += (int)((n_waypoints - 1) * h / delta_t_vector(i));
                    }
                    dim = dim++;

                    // interpolazione mediante natural cubic spline
                    Eigen::MatrixXd A(n_waypoints - 2, n_waypoints - 2); // matrice moltiplicata al vettore M
                    A << 4 * Eigen::MatrixXd::Identity(n_waypoints - 2, n_waypoints - 2);

                    if (A.rows() > 1 && A.cols() > 1)
                    {
                        for (int i = 1; i < n_waypoints - 2; i++)
                        {
                            A(i - 1, i) = 1;
                            A(i, i - 1) = 1;
                        }
                    }

                    Mx.resize(n_waypoints);
                    My.resize(n_waypoints);
                    Mx_tmp.resize(n_waypoints - 2);
                    My_tmp.resize(n_waypoints - 2);
                    Sx.resize(n_waypoints - 2);
                    Sy.resize(n_waypoints - 2);
                    ax.resize(n_waypoints - 1);
                    ay.resize(n_waypoints - 1);
                    bx.resize(n_waypoints - 1);
                    by.resize(n_waypoints - 1);
                    cx.resize(n_waypoints - 1);
                    cy.resize(n_waypoints - 1);
                    dx.resize(n_waypoints - 1);
                    dy.resize(n_waypoints - 1);
                    for (int i = 0; i < n_waypoints - 2; i++)
                    {
                        Sx(i) = 6 / (h * h) * (way_spline_x(i) - 2 * way_spline_x(i + 1) + way_spline_x(i + 2));
                        Sy(i) = 6 / (h * h) * (way_spline_y(i) - 2 * way_spline_y(i + 1) + way_spline_y(i + 2));
                    }
                    Mx_tmp = A.inverse() * Sx;
                    Mx << 0, Mx_tmp, 0;
                    My_tmp = A.inverse() * Sy;
                    My << 0, My_tmp, 0;

                    for (int i = 0; i < n_waypoints - 1; i++)
                    {
                        ax(i) = (Mx(i + 1) - Mx(i)) / (6 * h);
                        ay(i) = (My(i + 1) - My(i)) / (6 * h);
                        bx(i) = Mx(i) / 2;
                        by(i) = My(i) / 2;
                        cx(i) = (way_spline_x(i + 1) - way_spline_x(i)) / h - h / 6 * (2 * Mx(i) + Mx(i + 1));
                        cy(i) = (way_spline_y(i + 1) - way_spline_y(i)) / h - h / 6 * (2 * My(i) + My(i + 1));
                        dx(i) = way_spline_x(i);
                        dy(i) = way_spline_y(i);
                    }

                    Eigen::VectorXd x2(dim);
                    Eigen::VectorXd y2(dim);
                    Eigen::VectorXd z2(dim);
                    Eigen::VectorXd phi2(dim);
                    Eigen::VectorXd theta2(dim);
                    Eigen::VectorXd psi2(dim);
                    double dist;
                    Eigen::VectorXi j_waypoint(n_waypoints); // Vettore che contiene gli indici relativi ai waypoint
                    j_waypoint(0) = 0;                       // all'interno della struttura dati della spline
                    int j_prec = 0;
                    double h_i = h;
                    for (int i = 0; i < n_waypoints - 1; i++)
                    {
                        delta_t = delta_t_vector(i);
                        h_i = floor(h / delta_t) * delta_t;
                        for (int j = 0; j < h_i / delta_t; j++)
                        {
                            x2(j + j_prec) = ax(i) * pow(j * delta_t, 3) + bx(i) * pow(j * delta_t, 2) + cx(i) * (j * delta_t) + dx(i);
                            y2(j + j_prec) = ay(i) * pow(j * delta_t, 3) + by(i) * pow(j * delta_t, 2) + cy(i) * (j * delta_t) + dy(i);
                            z2(j + j_prec) = way_spline_z(i);
                            phi2(j + j_prec) = 0.0;
                            theta2(j + j_prec) = 0.0;
                            psi2(j + j_prec) = atan2(3 * ay(i) * pow(j * delta_t, 2) + 2 * by(i) * (j * delta_t) + cy(i), 3 * ax(i) * pow(j * delta_t, 2) + 2 * bx(i) * (j * delta_t) + cx(i));
                        }
                        j_prec = j_prec + (int)(h_i / delta_t);
                        j_waypoint(i + 1) = j_prec;
                    }

                    x2(dim - 1) = way_spline_x(n_waypoints - 1);
                    y2(dim - 1) = way_spline_y(n_waypoints - 1);
                    z2(dim - 1) = way_spline_z(n_waypoints - 1);
                    phi2(dim - 1) = 0.0;
                    theta2(dim - 1) = 0.0;
                    psi2(dim - 1) = psi2(dim - 2);

                    int dim1 = 0;
                    if (angleDifference(psi2(0) - psi_1) > 10 * (M_PI / 180))
                    {
                        dim1 = (int)(angleDifference(psi2(0) - psi_1) * freq / (5 * (M_PI / 180)));
                    }
                    else if (angleDifference(psi2(0) - psi_1) < -10 * (M_PI / 180))
                    {
                        dim1 = (int)(angleDifference(psi_1 - psi2(0)) * freq / (-5 * (M_PI / 180)));
                    }
                    else
                    {
                        is_psi_adjusted = true;
                    }

                    Eigen::VectorXi middle_waypoints(n_waypoints);
                    Eigen::VectorXd pos_rel_x(n_waypoints);
                    Eigen::VectorXd pos_rel_y(n_waypoints);
                    Eigen::VectorXd pos_rel_z(n_waypoints);
                    Eigen::VectorXd dz(n_waypoints);
                    Eigen::VectorXd dist_to_targ(n_waypoints);
                    pos_rel_x(way_counter - 1) = way_spline_x(way_counter) - way_spline_x(way_counter - 1);
                    pos_rel_y(way_counter - 1) = way_spline_y(way_counter) - way_spline_y(way_counter - 1);
                    pos_rel_z(way_counter - 1) = way_spline_z(way_counter) - way_spline_z(way_counter - 1);

                    dist_to_targ(way_counter - 1) = sqrt(pow(pos_rel_x(way_counter - 1), 2) + pow(pos_rel_y(way_counter - 1), 2) + pow(pos_rel_z(way_counter - 1), 2));
                    middle_waypoints(way_counter - 1) = (int)(floor(pos_rel_z(way_counter - 1) / step)); // numero di waypoint intermedi
                    Eigen::VectorXd z(middle_waypoints(way_counter - 1));
                    Eigen::VectorXd distz(middle_waypoints(way_counter - 1));
                    dz(way_counter - 1) = pos_rel_z(way_counter - 1) / middle_waypoints(way_counter - 1);
                    if (abs(pos_rel_z(way_counter - 1)) <= 0.5)
                    {
                        is_z_adjusted = true;
                    }
                    double error_z_to_waypoint = way_spline_z(way_counter) - z_hat;

                    if (!is_psi_adjusted)
                    {
                        // FASE DI REGOLAZIONE DI PSI

                        double delta_psi = angleDifference(psi2(0) - psi_1) / dim1;

                        x_d = x_1;
                        y_d = y_1;
                        z_d = z_1;
                        if (z_d <= 0.0)
                        {
                            z_d = 0.15;
                        }
                        phi_d = 0.0;
                        theta_d = 0.0;
                        psi_d = psi_1 + i_psi * delta_psi;
                        psi_d = wrapToPi(psi_d);
                        i_psi++;
                        if (i_psi >= dim1 - 1)
                        {
                            is_psi_adjusted = true;
                        }

                        u_d = 0.0;
                        v_d = 0.0;
                        w_d = 0.0;
                        p_d = 0.0;
                        q_d = 0.0;
                        r_d = 0.0;
                        if (angleDifference(psi2(0) - psi_1) > 10 * (M_PI / 180))
                        {
                            r_d = 5.0 * M_PI / 180;
                        }
                        else if ((angleDifference(psi2(0)) - psi_1) < -10 * (M_PI / 180))
                        {
                            r_d = -5.0 * M_PI / 180;
                        }
                        else
                        {
                            r_d = 0.0;
                        }
                    }
                    else if (!is_z_adjusted && is_psi_adjusted)
                    {
                        x_d = way_spline_x(way_counter - 1);
                        y_d = way_spline_y(way_counter - 1);
                        phi_d = 0.0;
                        theta_d = 0.0;
                        psi_d = psi2(0);
                        u_d = 0.0;
                        v_d = 0.0;
                        p_d = 0.0;
                        q_d = 0.0;
                        r_d = 0.0;

                        //////////////////////////////////////////////////
                        /////////////////Regolazione di z/////////////////
                        //////////////////////////////////////////////////
                        int i_dist_min = 0;
                        for (int j = 1; j < middle_waypoints(way_counter - 1); j++)
                        {
                            ROS_WARN("Sono nel for con j: %d", j);
                            ROS_WARN("I middle waypoints sono: %d", middle_waypoints(way_counter - 1));
                            ROS_WARN_STREAM("la dimensione di z e' : " << z.size());
                            z(j - 1) = way_spline_z(way_counter - 1) + j * step;
                            distz(j - 1) = abs(z(j - 1) - z_hat);
                            if (j == 1)
                            {
                                dist_min = distz(j - 1);
                            }
                            else
                            {
                                if (distz(j - 1) < dist_min)
                                {
                                    dist_min = distz(j - 1);
                                    i_dist_min = j - 1;
                                }
                            }
                        }
                        ROS_WARN("i_dist_min: %d", i_dist_min);
                        z_d = z(i_dist_min);
                        ROS_WARN("z_d: %f", z_d);

                        //////////////////////////////////////////////////
                        /////////////////Regolazione di w/////////////////
                        //////////////////////////////////////////////////

                        if (pos_rel_z(way_counter - 1) > 0.5)
                        {
                            w_d = w_d + delta_w * dt;
                            if (w_d >= max_w_d)
                            {
                                w_d = max_w_d;
                            }

                            if (abs(error_z_to_waypoint) <= 1.0 && abs(error_z_to_waypoint) > 0.5)
                            {
                                if (!is_w_d_tmp_initialized)
                                {
                                    w_d_tmp = w_d;
                                    is_w_d_tmp_initialized = true;
                                }
                                w_d = w_d_tmp / 2;
                            }
                            else if (abs(error_z_to_waypoint) <= 0.5 && abs(error_z_to_waypoint) > 0.1 && is_w_d_tmp_initialized)
                            {
                                w_d = w_d_tmp / 3;
                            }
                            if (abs(error_z_to_waypoint) <= 0.1 && is_w_d_tmp_initialized)
                            {
                                w_d = 0.0;
                                is_z_adjusted = true;
                            }

                            // Se, in fase di decelerazione (is_w_d_tmp_initialized = true), la velocità di discesa calcolata
                            // risulta troppo bassa, la si imposta a 0.05 m/s
                            if (abs(w_d) < 0.05 && (w_d != 0.0) && is_w_d_tmp_initialized)
                            {
                                w_d = 0.05; // Velocità minima di salita
                            }
                        }
                        else if (pos_rel_z(way_counter - 1) < -0.5)
                        {
                            w_d = w_d - delta_w * dt;
                            if (w_d <= -max_w_d)
                            {
                                w_d = -max_w_d;
                            }
                            if (abs(error_z_to_waypoint) <= 1.0 && abs(error_z_to_waypoint) > 0.5)
                            {
                                if (!is_w_d_tmp_initialized)
                                {
                                    w_d_tmp = w_d;
                                    is_w_d_tmp_initialized = true;
                                }
                                w_d = w_d_tmp / 2;
                            }
                            else if (abs(error_z_to_waypoint) <= 0.5 && abs(error_z_to_waypoint) > 0.1 && is_w_d_tmp_initialized)
                            {
                                w_d = w_d_tmp / 3;
                            }
                            else if (abs(error_z_to_waypoint) <= 0.1 && is_w_d_tmp_initialized)
                            {
                                w_d = 0.0;
                                is_z_adjusted = true;
                            }

                            // Se, in fase di decelerazione (is_w_d_tmp_initialized = true), la velocità di risalita calcolata
                            // risulta troppo bassa, la si imposta a -0.05 m/s
                            if (abs(w_d) < 0.05 && (w_d != 0.0) && is_w_d_tmp_initialized)
                            {
                                w_d = -0.05; // Velocità minima di discesa
                            }
                        }
                    }
                    else
                    {
                        // FASE DI SPLINE
                        // Calcolo la distanza minima tra il ROV e i waypoint
                        i_psi = 0;
                        int i_dist_min = j_waypoint(way_counter - 1);

                        for (int i = j_waypoint(way_counter - 1); i < j_waypoint(way_counter); i++)
                        {
                            dist = sqrt(pow(x2(i) - x_hat, 2.0) + pow(y2(i) - y_hat, 2.0) + pow(z2(i) - z_hat, 2.0));
                            if (i == j_waypoint(way_counter - 1))
                            {
                                dist_min = dist;
                            }
                            else
                            {
                                if (dist < dist_min)
                                {
                                    dist_min = dist;
                                    i_dist_min = i;
                                }
                            }
                        }

                        x_d = x2(i_dist_min);
                        y_d = y2(i_dist_min);
                        z_d = way_spline_z(way_counter);
                        if (z_d <= 0.0)
                        {
                            z_d = 0.15;
                        }

                        if (z_d <= 0.0)
                        {
                            z_d = 0.2;
                        }

                        phi_d = phi2(i_dist_min);
                        theta_d = theta2(i_dist_min);
                        psi_d = psi2(i_dist_min);
                        double delta_u = 0.05;
                        u_d = u_d + delta_u * dt;
                        if (u_d >= speed)
                        {
                            u_d = speed;
                        }
                        p_d = 0.0;
                        q_d = 0.0;
                        r_d = 0.0;

                        double error_x_to_waypoint = way_spline_x(way_counter) - x_hat;
                        double error_y_to_waypoint = way_spline_y(way_counter) - y_hat;
                        double error_z_to_waypoint = way_spline_z(way_counter) - z_hat;

                        double dist_to_waypoint = sqrt(pow(error_x_to_waypoint, 2) + pow(error_y_to_waypoint, 2) + pow(error_z_to_waypoint, 2));
                        double dist_to_waypoint_xy = sqrt(pow(error_x_to_waypoint, 2) + pow(error_y_to_waypoint, 2));

                        Eigen::VectorXd pos_rel_z(way_counter);
                        pos_rel_z(way_counter - 1) = way_spline_z(way_counter) - way_spline_z(way_counter - 1);

                        //////////////////////////////////////////////////
                        /////////////////Regolazione di u/////////////////
                        //////////////////////////////////////////////////

                        if (dist_to_waypoint_xy <= 1.25 && dist_to_waypoint_xy > 0.75)
                        {
                            if (!is_u_d_tmp_initialized)
                            {
                                u_d_tmp = u_d;
                                is_u_d_tmp_initialized = true;
                            }

                            u_d = u_d_tmp / 1.5;
                        }
                        else if (dist_to_waypoint_xy <= 0.75 && dist_to_waypoint_xy > 0.5 && is_u_d_tmp_initialized)
                        {
                            u_d = u_d_tmp / 2;
                        }
                        else if (dist_to_waypoint_xy <= 0.5 && dist_to_waypoint_xy > 0.25 && is_u_d_tmp_initialized)
                        {
                            u_d = u_d_tmp / 3;
                        }
                        else if (dist_to_waypoint_xy <= 0.25 && dist_to_waypoint_xy > 0.15 && is_u_d_tmp_initialized)
                        {
                            u_d = u_d_tmp / 4;
                        }

                        if (u_d < 0.05)
                        {
                            u_d = 0.05; // Velocità minima di avanzamento
                        }

                        //////////////////////////////////////////////////
                        /////////////////Regolazione di v/////////////////
                        //////////////////////////////////////////////////

                        Eigen::MatrixXd Body2NED(3, 3);
                        Body2NED << cos(psi_hat) * cos(theta_hat), cos(psi_hat) * sin(phi_hat) * sin(theta_hat) - cos(phi_hat) * sin(psi_hat), sin(phi_hat) * sin(psi_hat) + cos(phi_hat) * cos(psi_hat) * sin(theta_hat),
                            cos(theta_hat) * sin(psi_hat), cos(phi_hat) * cos(psi_hat) + sin(phi_hat) * sin(psi_hat) * sin(theta_hat), cos(phi_hat) * sin(psi_hat) * sin(theta_hat) - cos(psi_hat) * sin(phi_hat),
                            -sin(theta_hat), cos(theta_hat) * sin(phi_hat), cos(phi_hat) * cos(theta_hat);

                        Eigen::VectorXd pose_d(3);
                        pose_d << x_d, y_d, z_d;
                        Eigen::VectorXd pose_hat(3);
                        pose_hat << x_hat, y_hat, z_hat;

                        Eigen::VectorXd pose_error_body = Body2NED.transpose() * (pose_d - pose_hat);

                        if (pose_error_body(1) > 0.5)
                        {
                            v_d = 0.05;
                        }
                        else if (pose_error_body(1) < -0.5)
                        {
                            v_d = -0.05;
                        }
                        else
                        {
                            v_d = 0.0;
                        }

                        //////////////////////////////////////////////////
                        /////////////////Regolazione di w/////////////////
                        //////////////////////////////////////////////////

                        double max_w_d = abs(pos_rel_z(way_counter - 1)) * speed / dist_to_waypoint_xy;
                        double delta_w = abs(pos_rel_z(way_counter - 1)) * delta_u / dist_to_waypoint_xy;

                        if (pos_rel_z(way_counter - 1) > 0.5)
                        {
                            w_d = w_d + delta_w * dt;
                            if (w_d >= max_w_d)
                            {
                                w_d = max_w_d;
                            }

                            if (abs(error_z_to_waypoint) <= 1.0 && abs(error_z_to_waypoint) > 0.5)
                            {
                                if (!is_w_d_tmp_initialized)
                                {
                                    w_d_tmp = w_d;
                                    is_w_d_tmp_initialized = true;
                                }
                                w_d = w_d_tmp / 2;
                            }
                            else if (abs(error_z_to_waypoint) <= 0.5 && abs(error_z_to_waypoint) > 0.2 && is_w_d_tmp_initialized)
                            {
                                w_d = w_d_tmp / 3;
                            }

                            if (abs(w_d) < 0.05)
                            {
                                w_d = 0.05; // Velocità minima di salita
                            }

                            if (abs(error_z_to_waypoint) <= 0.1 && is_w_d_tmp_initialized)
                            {
                                w_d = 0.0;
                            }
                        }
                        else if (pos_rel_z(way_counter - 1) < -0.5)
                        {
                            w_d = w_d - delta_w * dt;
                            if (w_d <= -max_w_d)
                            {
                                w_d = -max_w_d;
                            }
                            if (abs(error_z_to_waypoint) <= 1.0 && abs(error_z_to_waypoint) > 0.5)
                            {
                                if (!is_w_d_tmp_initialized)
                                {
                                    w_d_tmp = w_d;
                                    is_w_d_tmp_initialized = true;
                                }
                                w_d = w_d_tmp / 2;
                            }
                            else if (abs(error_z_to_waypoint) <= 0.5 && abs(error_z_to_waypoint) > 0.2 && is_w_d_tmp_initialized)
                            {
                                w_d = w_d_tmp / 3;
                            }

                            if (abs(w_d) < 0.05)
                            {
                                w_d = -0.05; // Velocità minima di discesa
                            }

                            if (abs(error_z_to_waypoint) <= 0.1 && is_w_d_tmp_initialized)
                            {
                                w_d = 0.0;
                            }
                        }

                        if (dist_to_waypoint < 0.2)
                        {
                            is_u_d_tmp_initialized = false;
                            is_w_d_tmp_initialized = false;
                            is_z_adjusted = false;
                            z_1 = z_d;
                            if (way_counter < n_waypoints - 1)
                            {
                                way_counter++;
                            }
                            else
                            {
                                u_d = 0.0;
                            }
                        }
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

                    std::vector<double> des_state = {x_d, y_d, z_d, phi_d, theta_d, psi_d, x_dot_d, y_dot_d, z_dot_d, phi_dot_d, theta_dot_d, psi_dot_d};
                    des_state_msg.data = des_state;
                    // Publishing the state
                    guidance_pub.publish(des_state_msg);
                }
                else if (strategy == "Rect")
                {
                    n_waypoints = way_spline.size() / 3 + 1;
                    way_spline_x.resize(n_waypoints);
                    way_spline_y.resize(n_waypoints);
                    way_spline_z.resize(n_waypoints);
                    if (post_up_down == 0)
                    {
                        way_spline_x(0) = x_1;
                        way_spline_y(0) = y_1;
                        way_spline_z(0) = z_1;
                    }
                    else
                    {
                        post_up_down = 0;
                        way_spline_x(0) = post_up_down_x;
                        way_spline_y(0) = post_up_down_y;
                        way_spline_z(0) = post_up_down_z;
                    }

                    for (int i = 0; i < n_waypoints - 1; i++)
                    {
                        way_spline_x(i + 1) = way_spline[i * 3];
                        way_spline_y(i + 1) = way_spline[i * 3 + 1];
                        way_spline_z(i + 1) = way_spline[i * 3 + 2];
                    }

                    Eigen::VectorXd x_to_go(n_waypoints - 1);
                    Eigen::VectorXd y_to_go(n_waypoints - 1);
                    Eigen::VectorXd z_to_go(n_waypoints - 1);
                    Eigen::VectorXd psi_to_go(n_waypoints - 1);
                    Eigen::VectorXi dim1(n_waypoints - 1);
                    Eigen::VectorXd delta_psi(n_waypoints - 1);
                    Eigen::VectorXd pos_rel_x(n_waypoints);
                    Eigen::VectorXd pos_rel_y(n_waypoints);
                    Eigen::VectorXd pos_rel_z(n_waypoints);
                    Eigen::VectorXd dist_to_targ(n_waypoints);
                    Eigen::VectorXi middle_waypoints(n_waypoints);
                    Eigen::VectorXd dx(n_waypoints);
                    Eigen::VectorXd dy(n_waypoints);
                    Eigen::VectorXd dz(n_waypoints);
                    pos_rel_x(way_counter - 1) = way_spline_x(way_counter) - way_spline_x(way_counter - 1);
                    pos_rel_y(way_counter - 1) = way_spline_y(way_counter) - way_spline_y(way_counter - 1);
                    pos_rel_z(way_counter - 1) = way_spline_z(way_counter) - way_spline_z(way_counter - 1);

                    dist_to_targ(way_counter - 1) = sqrt(pow(pos_rel_x(way_counter - 1), 2) + pow(pos_rel_y(way_counter - 1), 2) + pow(pos_rel_z(way_counter - 1), 2));
                    middle_waypoints(way_counter - 1) = (int)(floor(dist_to_targ(way_counter - 1) / step)); // numero di waypoint intermedi

                    dx(way_counter - 1) = pos_rel_x(way_counter - 1) / middle_waypoints(way_counter - 1);
                    dy(way_counter - 1) = pos_rel_y(way_counter - 1) / middle_waypoints(way_counter - 1);
                    dz(way_counter - 1) = pos_rel_z(way_counter - 1) / middle_waypoints(way_counter - 1);

                    Eigen::VectorXd x(middle_waypoints(way_counter - 1));
                    Eigen::VectorXd y(middle_waypoints(way_counter - 1));
                    Eigen::VectorXd z(middle_waypoints(way_counter - 1));
                    Eigen::VectorXd psi(middle_waypoints(way_counter - 1));
                    Eigen::VectorXd dist(middle_waypoints(way_counter - 1));

                    if (way_counter < n_waypoints)
                    {

                        psi_to_go(way_counter - 1) = atan2(pos_rel_y(way_counter - 1), pos_rel_x(way_counter - 1)); // angolo da percorrere

                        if (angleDifference(psi_to_go(way_counter - 1) - psi_1) > 10 * (M_PI / 180))
                        {
                            dim1(way_counter - 1) = (int)(angleDifference(psi_to_go(way_counter - 1) - psi_1) * freq / (5 * (M_PI / 180)));
                        }
                        else if (angleDifference(psi_to_go(way_counter - 1) - psi_1) < -10 * (M_PI / 180))
                        {
                            dim1(way_counter - 1) = (int)(angleDifference(psi_1 - psi_to_go(way_counter - 1)) * freq / (5 * (M_PI / 180)));
                        }
                        else
                        {
                            is_psi_adjusted = true;
                        }

                        if (abs(pos_rel_z(way_counter - 1)) <= 0.5)
                        {
                            is_z_adjusted = true;
                        }

                        double error_x_to_waypoint = way_spline_x(way_counter) - x_hat;
                        double error_y_to_waypoint = way_spline_y(way_counter) - y_hat;
                        double error_z_to_waypoint = way_spline_z(way_counter) - z_hat;

                        double dist_to_waypoint = sqrt(pow(error_x_to_waypoint, 2) + pow(error_y_to_waypoint, 2) + pow(error_z_to_waypoint, 2));
                        double dist_to_waypoint_xy = sqrt(pow(error_x_to_waypoint, 2) + pow(error_y_to_waypoint, 2));

                        if (!is_psi_adjusted)
                        {

                            ///////////////////////////////////////////////////
                            ///////////////REGOLAZIONE DI PSI//////////////////
                            ///////////////////////////////////////////////////

                            delta_psi(way_counter - 1) = angleDifference(psi_to_go(way_counter - 1) - psi_1) / dim1(way_counter - 1);
                            x_d = way_spline_x(way_counter - 1);
                            y_d = way_spline_y(way_counter - 1);
                            z_d = way_spline_z(way_counter - 1);
                            phi_d = 0.0;
                            theta_d = 0.0;
                            psi_d = psi_1 + i_psi * delta_psi(way_counter - 1);
                            psi_d = wrapToPi(psi_d);
                            i_psi++;

                            if (i_psi >= dim1(way_counter - 1) - 1)
                            {
                                is_psi_adjusted = true;
                            }

                            u_d = 0.0;
                            v_d = 0.0;
                            w_d = 0.0;
                            p_d = 0.0;
                            q_d = 0.0;

                            if (angleDifference(psi_to_go(way_counter - 1) - psi_1) > 10 * (M_PI / 180))
                            {
                                r_d = 5.0 * M_PI / 180;
                            }
                            else if (angleDifference(psi_to_go(way_counter - 1) - psi_1) < -10 * (M_PI / 180))
                            {
                                r_d = -5.0 * M_PI / 180;
                            }
                            else
                            {
                                r_d = 0.0;
                            }
                        }
                        else if (!is_z_adjusted && is_psi_adjusted)
                        {
                            x_d = way_spline_x(way_counter - 1);
                            y_d = way_spline_y(way_counter - 1);
                            phi_d = 0.0;
                            theta_d = 0.0;
                            psi_d = psi_to_go(way_counter - 1);
                            u_d = 0.0;
                            v_d = 0.0;
                            p_d = 0.0;
                            q_d = 0.0;
                            r_d = 0.0;

                            //////////////////////////////////////////////////
                            /////////////////Regolazione di z/////////////////
                            //////////////////////////////////////////////////
                            int i_dist_min = 0;
                            for (int j = 1; j < middle_waypoints(way_counter - 1); j++)
                            {
                                z(j - 1) = way_spline_z(way_counter - 1) + j * dz(way_counter - 1);
                                dist(j - 1) = abs(z(j - 1) - z_hat);
                                if (j == 1)
                                {
                                    dist_min = dist(j - 1);
                                }
                                else
                                {
                                    if (dist(j - 1) < dist_min)
                                    {
                                        dist_min = dist(j - 1);
                                        i_dist_min = j - 1;
                                    }
                                }
                            }
                            z_d = z(i_dist_min);

                            //////////////////////////////////////////////////
                            /////////////////Regolazione di w/////////////////
                            //////////////////////////////////////////////////

                            if (pos_rel_z(way_counter - 1) > 0.5)
                            {
                                w_d = w_d + delta_w * dt;
                                if (w_d >= max_w_d)
                                {
                                    w_d = max_w_d;
                                }

                                if (abs(error_z_to_waypoint) <= 1.0 && abs(error_z_to_waypoint) > 0.5)
                                {
                                    if (!is_w_d_tmp_initialized)
                                    {
                                        w_d_tmp = w_d;
                                        is_w_d_tmp_initialized = true;
                                    }
                                    w_d = w_d_tmp / 2;
                                }
                                else if (abs(error_z_to_waypoint) <= 0.5 && abs(error_z_to_waypoint) > 0.1 && is_w_d_tmp_initialized)
                                {
                                    w_d = w_d_tmp / 3;
                                }
                                if (abs(error_z_to_waypoint) <= 0.1 && is_w_d_tmp_initialized)
                                {
                                    w_d = 0.0;
                                    is_z_adjusted = true;
                                }

                                // Se, in fase di decelerazione (is_w_d_tmp_initialized = true), la velocità di discesa calcolata
                                // risulta troppo bassa, la si imposta a 0.05 m/s
                                if (abs(w_d) < 0.05 && (w_d != 0.0) && is_w_d_tmp_initialized)
                                {
                                    w_d = 0.05; // Velocità minima di salita
                                }
                            }
                            else if (pos_rel_z(way_counter - 1) < -0.5)
                            {
                                w_d = w_d - delta_w * dt;
                                if (w_d <= -max_w_d)
                                {
                                    w_d = -max_w_d;
                                }
                                if (abs(error_z_to_waypoint) <= 1.0 && abs(error_z_to_waypoint) > 0.5)
                                {
                                    if (!is_w_d_tmp_initialized)
                                    {
                                        w_d_tmp = w_d;
                                        is_w_d_tmp_initialized = true;
                                    }
                                    w_d = w_d_tmp / 2;
                                }
                                else if (abs(error_z_to_waypoint) <= 0.5 && abs(error_z_to_waypoint) > 0.1 && is_w_d_tmp_initialized)
                                {
                                    w_d = w_d_tmp / 3;
                                }
                                else if (abs(error_z_to_waypoint) <= 0.1 && is_w_d_tmp_initialized)
                                {
                                    w_d = 0.0;
                                    is_z_adjusted = true;
                                }

                                // Se, in fase di decelerazione (is_w_d_tmp_initialized = true), la velocità di risalita calcolata
                                // risulta troppo bassa, la si imposta a -0.05 m/s
                                if (abs(w_d) < 0.05 && (w_d != 0.0) && is_w_d_tmp_initialized)
                                {
                                    w_d = -0.05; // Velocità minima di discesa
                                }
                            }
                        }
                        else
                        {

                            /////////////////////////////////////////////////////////////
                            ///////////////////FASE DI AVANZAMENTO///////////////////////
                            /////////////////////////////////////////////////////////////

                            i_psi = 0;
                            u_d = u_d + delta_u * dt;
                            if (u_d >= speed)
                            {
                                u_d = speed;
                            }

                            v_d = 0.0;
                            w_d = 0.0;
                            r_d = 0.0;

                            int i_dist_min = 0;

                            for (int j = 1; j < middle_waypoints(way_counter - 1); j++)
                            {
                                x(j - 1) = way_spline_x(way_counter - 1) + j * dx(way_counter - 1);
                                y(j - 1) = way_spline_y(way_counter - 1) + j * dy(way_counter - 1);
                                dist(j - 1) = sqrt(pow(x_hat - x(j - 1), 2.0) + pow(y_hat - y(j - 1), 2.0));
                                if (j == 1)
                                {
                                    dist_min = dist(j - 1);
                                }
                                else
                                {
                                    if (dist(j - 1) < dist_min)
                                    {
                                        dist_min = dist(j - 1);
                                        i_dist_min = j - 1;
                                    }
                                }
                            }

                            x_d = x(i_dist_min);
                            y_d = y(i_dist_min);
                            z_d = way_spline_z(way_counter);
                            if (z_d == 0.0)
                            {
                                z_d = 0.2;
                            }

                            phi_d = 0.0;
                            theta_d = 0.0;
                            psi_d = psi_to_go(way_counter - 1);
                            // ROS_WARN("i_dist_min: %d", i_dist_min);

                            //////////////////////////////////////////////////
                            /////////////////Regolazione di u/////////////////
                            //////////////////////////////////////////////////

                            if (dist_to_waypoint_xy <= 1.25 && dist_to_waypoint_xy > 0.75)
                            {
                                if (!is_u_d_tmp_initialized)
                                {
                                    u_d_tmp = u_d;
                                    is_u_d_tmp_initialized = true;
                                }

                                u_d = u_d_tmp / 1.5;
                            }
                            else if (dist_to_waypoint_xy <= 0.75 && dist_to_waypoint_xy > 0.5 && is_u_d_tmp_initialized)
                            {
                                u_d = u_d_tmp / 2;
                            }
                            else if (dist_to_waypoint_xy <= 0.5 && dist_to_waypoint_xy > 0.25 && is_u_d_tmp_initialized)
                            {
                                u_d = u_d_tmp / 3;
                            }
                            else if (dist_to_waypoint_xy <= 0.25 && dist_to_waypoint_xy > 0.15 && is_u_d_tmp_initialized)
                            {
                                u_d = u_d_tmp / 4;
                            }

                            if (u_d < 0.1)
                            {
                                u_d = 0.1; // Velocità minima di avanzamento
                            }

                            if (dist_to_waypoint_xy <= 0.15 && is_u_d_tmp_initialized)
                            {
                                u_d = 0.0;
                            }

                            //////////////////////////////////////////////////
                            /////////////////Regolazione di v/////////////////
                            //////////////////////////////////////////////////

                            // Eigen::MatrixXd Body2NED(3, 3);
                            // Body2NED << cos(psi_hat) * cos(theta_hat), cos(psi_hat) * sin(phi_hat) * sin(theta_hat) - cos(phi_hat) * sin(psi_hat), sin(phi_hat) * sin(psi_hat) + cos(phi_hat) * cos(psi_hat) * sin(theta_hat),
                            //     cos(theta_hat) * sin(psi_hat), cos(phi_hat) * cos(psi_hat) + sin(phi_hat) * sin(psi_hat) * sin(theta_hat), cos(phi_hat) * sin(psi_hat) * sin(theta_hat) - cos(psi_hat) * sin(phi_hat),
                            //     -sin(theta_hat), cos(theta_hat) * sin(phi_hat), cos(phi_hat) * cos(theta_hat);

                            // Eigen::VectorXd pose_d(3);
                            // pose_d << x_d, y_d, z_d;
                            // Eigen::VectorXd pose_hat(3);
                            // pose_hat << x_hat, y_hat, z_hat;

                            // Eigen::VectorXd pose_error_body = Body2NED.transpose() * (pose_d - pose_hat);

                            // if (pose_error_body(1) > 0.5)
                            // {
                            //     v_d = 0.05;
                            // }
                            // else if (pose_error_body(1) < -0.5)
                            // {
                            //     v_d = -0.05;
                            // }
                            // else
                            // {
                            //     v_d = 0.0;
                            // }

                            v_d = 0.0;

                            if (dist_to_waypoint < 0.2)
                            {
                                is_psi_adjusted = false;
                                is_z_adjusted = false;
                                is_u_d_tmp_initialized = false;
                                is_w_d_tmp_initialized = false;
                                way_counter++;
                                psi_1 = psi_d;
                                z_1 = z_d;
                            }
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

                        std::vector<double> des_state = {x_d, y_d, z_d, phi_d, theta_d, psi_d, x_dot_d, y_dot_d, z_dot_d, phi_dot_d, theta_dot_d, psi_dot_d};
                        des_state_msg.data = des_state;
                        // Publishing the state
                        guidance_pub.publish(des_state_msg);
                    }
                }
                else if (strategy == "Circumference")
                {
                    double dist_to_buoy = sqrt(pow(x_1 - x_b, 2.0) + pow(y_1 - y_b, 2.0));
                    int n_elements_circ = (int)(dist_to_buoy * 100);        // divido la circonferenza in 100 elementi
                    int n_elements_semi = (int)(n_elements_circ / 2);       // divido la circonferenza in 2 semicirconferenze
                    int n_elements_tot = n_elements_circ + n_elements_semi; // percorriamo una circonferenza e mezzo
                    double x[n_elements_tot];
                    double y[n_elements_tot];
                    double z[n_elements_tot];
                    double x_dot;
                    double y_dot;
                    double phi;
                    double theta;
                    double psi[n_elements_tot];
                    double beta = atan2(y_1 - y_b, x_1 - x_b);
                    int n_elements_psi;
                    Eigen::VectorXd dist(n_elements_tot);
                    double delta_psi_circ;
                    for (int i = 0; i < n_elements_tot; i++)
                    {
                        if (clockwise == 1)
                        {
                            x[i] = x_b + dist_to_buoy * cos(2 * M_PI * i / n_elements_circ + beta);
                            y[i] = y_b + dist_to_buoy * sin(2 * M_PI * i / n_elements_circ + beta);
                        }
                        else
                        {
                            x[i] = x_b + dist_to_buoy * cos(-2 * M_PI * i / n_elements_circ + beta);
                            y[i] = y_b + dist_to_buoy * sin(-2 * M_PI * i / n_elements_circ + beta);
                        }
                        z[i] = z_b;
                        if (i < n_elements_tot - 1)
                        {
                            if (clockwise == 1)
                            {
                                x_dot = -dist_to_buoy * sin(2 * M_PI * i / n_elements_circ + beta) * 2 * M_PI / n_elements_circ;
                                y_dot = dist_to_buoy * cos(2 * M_PI * i / n_elements_circ + beta) * 2 * M_PI / n_elements_circ;
                            }
                            else
                            {
                                x_dot = dist_to_buoy * sin(-2 * M_PI * i / n_elements_circ + beta) * 2 * M_PI / n_elements_circ;
                                y_dot = -dist_to_buoy * cos(-2 * M_PI * i / n_elements_circ + beta) * 2 * M_PI / n_elements_circ;
                            }
                            psi[i] = atan2(y_dot, x_dot);
                        }
                        else
                        {
                            psi[i] = psi[0];
                        }
                    }

                    if (angleDifference(psi[0] - psi_1) > 10 * (M_PI / 180))
                    {
                        n_elements_psi = (int)(angleDifference(psi[0] - psi_1) * freq / (5 * (M_PI / 180)));
                    }
                    else if (angleDifference(psi[0] - psi_1) < -10 * (M_PI / 180))
                    {
                        n_elements_psi = (int)(angleDifference(psi_1 - psi[0]) * freq / (5 * (M_PI / 180)));
                    }
                    else
                    {
                        is_psi_adjusted = true;
                    }
                    if (!is_psi_adjusted)
                    {
                        ///////////////////////////////////////////////////
                        ///////////////REGOLAZIONE DI PSI//////////////////
                        ///////////////////////////////////////////////////
                        ROS_WARN("Regolazione di psi");
                        delta_psi_circ = angleDifference(psi[0] - psi_1) / n_elements_psi;
                        x_d = x_1;
                        y_d = y_1;
                        z_d = z_1;
                        phi_d = 0.0;
                        theta_d = 0.0;
                        psi_d = psi_1 + i_psi * delta_psi_circ;
                        psi_d = wrapToPi(psi_d);
                        i_psi++;
                        u_d = 0.0;
                        v_d = 0.0;
                        w_d = 0.0;
                        p_d = 0.0;
                        q_d = 0.0;
                        if (angleDifference(psi[0] - psi_1) > 10 * (M_PI / 180))
                        {
                            r_d = 5.0 * M_PI / 180;
                        }
                        else if (angleDifference(psi[0] - psi_1) < -10 * (M_PI / 180))
                        {
                            r_d = -5.0 * M_PI / 180;
                        }
                        else
                        {
                            r_d = 0.0;
                        }

                        if (i_psi >= n_elements_psi - 1)
                        {
                            is_psi_adjusted = true;
                        }
                    }
                    else
                    {
                        int i_dist_min = 0;
                        if (CIRCUMFERENCE_phase == 1)
                        {
                            for (int i = 0; i < n_elements_semi; i++)
                            {
                                dist(i) = sqrt(pow(x_hat - x[i], 2.0) + pow(y_hat - y[i], 2.0));
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
                            ROS_WARN("i_dist_min: %d di %d", i_dist_min, n_elements_semi);

                            if (i_dist_min >= n_elements_semi - 3)
                            {
                                CIRCUMFERENCE_phase = 2;
                            }
                        }
                        else if (CIRCUMFERENCE_phase == 2)
                        {
                            i_dist_min = n_elements_semi;
                            for (int i = n_elements_semi; i < n_elements_circ; i++)
                            {
                                dist(i) = sqrt(pow(x_hat - x[i], 2.0) + pow(y_hat - y[i], 2.0));
                                if (i == n_elements_semi)
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
                            ROS_WARN("i_dist_min: %d di %d", i_dist_min, n_elements_circ);

                            if (i_dist_min >= n_elements_circ - 3)
                            {
                                CIRCUMFERENCE_phase = 3;
                            }
                        }
                        else if (CIRCUMFERENCE_phase == 3)
                        {
                            i_dist_min = n_elements_circ;
                            for (int i = n_elements_circ; i < n_elements_tot; i++)
                            {
                                dist(i) = sqrt(pow(x_hat - x[i], 2.0) + pow(y_hat - y[i], 2.0));
                                if (i == n_elements_circ)
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
                                ROS_WARN("i_dist_min: %d di %d", i_dist_min, n_elements_tot);
                            }
                        }

                        x_d = x[i_dist_min];
                        y_d = y[i_dist_min];
                        z_d = z[i_dist_min];
                        phi_d = 0.0;
                        theta_d = 0.0;
                        psi_d = psi[i_dist_min];
                        u_d = speed;
                        v_d = 0.0;
                        p_d = 0.0;
                        q_d = 0.0;
                        r_d = 0.0;
                        if (i_dist_min >= n_elements_tot - 3)
                        {
                            x_d = x[n_elements_tot - 3];
                            y_d = y[n_elements_tot - 3];
                            z_d = z[n_elements_tot - 3];
                            phi_d = 0.0;
                            theta_d = 0.0;
                            psi_d = psi[n_elements_tot - 3];
                            u_d = 0.0;
                            v_d = 0.0;
                            w_d = 0.0;
                            p_d = 0.0;
                            q_d = 0.0;
                            r_d = 0.0;
                            std::string status_req = "PAUSED";
                            std_msgs::String msg;
                            msg.data = status_req;
                            publisher_status.publish(msg);
                            i_psi = 0;
                        }
                    }

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

                    std::vector<double> des_state = {x_d, y_d, z_d, phi_d, theta_d, psi_d, x_dot_d, y_dot_d, z_dot_d, phi_dot_d, theta_dot_d, psi_dot_d};
                    des_state_msg.data = des_state;
                    // Publishing the state
                    guidance_pub.publish(des_state_msg);
                }
                else if (strategy == "UP_DOWN")
                {
                    double x[119];
                    double y[119];
                    double z[119];
                    double phi[119];
                    double theta[119];
                    double psi[119];

                    double u_d = surge_speed;
                    double v_d = 0.0;
                    double w_d = 0.0;
                    double p_d = 0.0;
                    double q_d = 0.0;
                    double r_d = 0.0;

                    Eigen::VectorXd dist(119);

                    double dx = (x_p - x_1) / 100;
                    double dy = (y_p - y_1) / 100;
                    double dz = (z_p - z_1) / 100;

                    w_d = dz * 10;

                    // FASE DI AVVICINAMENTO
                    for (int i = 0; i < 99; i++)
                    {
                        x[i] = x_1 + (i + 1) * dx;
                        y[i] = y_1 + (i + 1) * dy;
                        z[i] = z_1 + (i + 1) * dz;
                        phi[i] = 0.0;
                        theta[i] = 0.0;
                        psi[i] = atan2(dy, dx);
                    }

                    // FASE DI RISALITA
                    double dz2;
                    if (up == 0)
                        dz2 = 0.5 / 20; // passo di risalita su asse z per ripianificazione locale
                    else if (up == 1)
                        dz2 = -0.5 / 20; // passo di discesa su asse z per ripianificazione locale
                    for (int i = 99; i < 119; i++)
                    {
                        x[i] = x_p;
                        y[i] = y_p;
                        z[i] = z[i - 1] + dz2;
                        phi[i] = 0.0;
                        theta[i] = 0.0;
                        psi[i] = psi[i - 1];
                    }
                    int i_dist_min;
                    if (UP_DOWN_first_phase)
                    {
                        i_dist_min = 0;

                        // Scelta del waypoint più vicino

                        for (int i = 0; i < 99; i++)
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
                        double dist_to_way = sqrt(pow(x_p - x_hat, 2.0) + pow(y_p - y_hat, 2.0) + pow(z_p - z_hat, 2.0));
                        if (dist_to_way < 0.1)
                        {
                            UP_DOWN_first_phase = false;
                        }
                    }
                    else
                    {
                        // ROS_WARN("z: %f", z_hat);
                        i_dist_min = 99;

                        // Scelta del waypoint più vicino

                        for (int i = 99; i < 119; i++)
                        {
                            dist(i) = sqrt(pow(x_hat - x[i], 2.0) + pow(y_hat - y[i], 2.0) + pow(z_hat - z[i], 2.0));
                            if (i == 99)
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
                    }

                    x_d = x[i_dist_min];
                    y_d = y[i_dist_min];
                    z_d = z[i_dist_min];
                    phi_d = 0.0;
                    theta_d = 0.0;
                    psi_d = psi[i_dist_min];

                    if (i_dist_min > 50 && i_dist_min < 99)
                    {
                        u_d = surge_speed_replanning;
                        v_d = 0.0;
                        r_d = 0.0;
                        if (i_dist_min > 70)
                        {
                            w_d = (z_p - z_hat) / abs(z_hat - z_p) * 0.1;
                        }
                    }
                    else if (i_dist_min >= 99)
                    {
                        u_d = 0.0;
                        v_d = 0.0;
                        if (up == 1)
                        {
                            ROS_WARN_STREAM("RISALITA");
                            w_d = -0.1;
                        }
                        else
                        {
                            ROS_WARN_STREAM("DISCESA");
                            w_d = 0.1;
                        }
                        r_d = 0.0;
                    }
                    post_up_down = 1;
                    if (i_dist_min >= 115)
                    {
                        std::string status_req = "PAUSED";

                        post_up_down_x = x[i_dist_min];
                        post_up_down_y = y[i_dist_min];
                        post_up_down_z = z[i_dist_min];
                        std_msgs::String msg;
                        msg.data = status_req;
                        publisher_status.publish(msg);
                        UP_DOWN_first_phase = true;
                        ros::Duration(1).sleep();
                    }
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

                    std::vector<double> des_state = {x_d, y_d, z_d, phi_d, theta_d, psi_d, x_dot_d, y_dot_d, z_dot_d, phi_dot_d, theta_dot_d, psi_dot_d};
                    des_state_msg.data = des_state;
                    // Publishing the state
                    guidance_pub.publish(des_state_msg);
                }
            }
        }

        if (ros::Time::now().toSec() > ros::TIME_MIN.toSec())
        {
            des_state_bag.write("state/desired_state_topic", ros::Time::now(), des_state_msg);
        }

        ros::spinOnce();

        loop_rate.sleep();
    }
    des_state_bag.close();
    return 0;
}
