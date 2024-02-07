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
double x_hat = 0.0;   // x stimata
double y_hat = 0.0;   // y stimata
double z_hat = 0.0;   // z stimata
double psi_hat = 0.0; // psi stimata
double u_hat = 0.0;   // velocità di surge stimata
double v_hat = 0.0;   // velocità di sway stimata
double w_hat = 0.0;   // velocità di heave stimata
double r_hat = 0.0;   // velocità angolare di yaw stimata

// coordiate spline a 3 punti

double x_1 = 0.0; // coordiata x del primo waypoint
double y_1 = 0.0; // coordiata y del primo waypoint
double z_1 = 0.0; // coordiata z del primo waypoint

double x_2 = 0.0; // coordiata x del secondo waypoint
double y_2 = 0.0; // coordiata y del secondo waypoint
double z_2 = 0.0; // coordiata z del secondo waypoint

double x_3 = 0.0; // coordiata x del terzo waypoint
double y_3 = 0.0; // coordiata y del terzo waypoint
double z_3 = 0.0; // coordiata z del terzo waypoint

double x_b = 0.0; // coordiata x della boa vista
double y_b = 0.0; // coordiata y della boa vista
double z_b = 0.0; // coordiata z della boa vista
double x_p = 0.0; // coordiata x del punto da raggiungere per ripianificazione locale
double y_p = 0.0; // coordiata y del punto da raggiungere per ripianificazione locale
double z_p = 0.0; // coordiata z del punto da raggiungere per ripianificazione locale

double x_t = 0.0; // coordiata x del target da raggiungere a fine missione
double y_t = 0.0; // coordiata y del target da raggiungere a fine missione
double z_t = 0.0; // coordiata z del target da raggiungere a fine missione

double clockwise; // Verso di rotazione della circonferenza

// spezziamo la traiettoria di circonferenza e di UP_DOWN in più fasi, queste variabili servono per gestire le varie fasi
bool UP_DOWN_first_phase = true; // booleano per gesire le fasi di up_down(2 fasi)
int CIRCUMFERENCE_phase = 1;     // intero per gestire le fasi di circumference(3 fasi)

double w = 0.0; // Comando di velocità di heave (variabile nelle Splines)

std::string strategy = "";       // strategia di controllo
std::string mission_status = ""; // stato di missione

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////// DEFINIZIONE CALLBACK /////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void statusCallback(const std_msgs::String::ConstPtr &msg) // CALLBACK macchina a stati
{
    mission_status = msg->data.c_str();
}

void waypointCallback(const progetto_robotica::Floats_String::ConstPtr &msg) // CALLBACK che riceve i waypoint
{
    if (msg->data[0] != msg->data[0])
    {
        x_b = 0.0;
        y_b = 0.0;
        z_b = 0.0;
        strategy = "";
    }
    else // in base alla strategia, cambia il numero di informazioni che prendiamo dal messaggio
    {
        if (msg->strategy == "Circumference")
        {
            x_b = msg->data[0];
            y_b = msg->data[1];
            z_b = msg->data[2];
            x_p = msg->data[3];
            y_p = msg->data[4];
            z_p = msg->data[5];
            clockwise = msg->data[6];
            strategy = msg->strategy;
            x_1 = x_hat;
            y_1 = y_hat;
            z_1 = z_hat;
        }
        else if (msg->strategy == "UP_DOWN")
        {
            x_p = msg->data[0];
            y_p = msg->data[1];
            z_p = msg->data[2];
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
            w = msg->data[6];
            strategy = msg->strategy;
        }
        else if (msg->strategy == "Target") // quando interrompiamo l'esplorazione e ci dirigiamo al target
        {
            x_1 = x_hat;
            y_1 = y_hat;
            z_1 = z_hat;
            x_t = msg->data[0];
            y_t = msg->data[1];
            z_t = msg->data[2];
            strategy = msg->strategy;
        }
        else if (msg->strategy == "Initial") // prima fase, in cui ci allontaniamo dal muro
        {
            x_1 = x_hat;
            y_1 = y_hat;
            z_1 = z_hat;
            x_2 = msg->data[0];
            y_2 = msg->data[1];
            z_2 = msg->data[2];
            strategy = msg->strategy;
        }
    }
}

void estStateCallback(const progetto_robotica::Floats::ConstPtr &msg) // CALLBACK in cui prendiamo gli stati "stimati"
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
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////// DEFINIZIONE NODO ROS ///////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    ros::init(argc, argv, "trajectory_generator");
    ros::NodeHandle n;
    // Definisco publisher e subscriber
    ros::Publisher chatter_pub = n.advertise<progetto_robotica::Floats>("desired_state_topic", 1);         // publisher stato desiderato
    ros::Publisher publisher_status = n.advertise<std_msgs::String>("manager/status_requested_topic", 10); // publisher stato richiesto al manager di missione

    ros::Subscriber sub_est_state = n.subscribe("state_topic", 1, estStateCallback);     // sottoscrizione alla topic di stato stimato
    ros::Subscriber sub_waypoint = n.subscribe("waypoints_topic", 1, waypointCallback);  // sottoscrizione alla topic di waypoint
    ros::Subscriber sub_status = n.subscribe("manager/status_topic", 1, statusCallback); // sottoscrizione alla topic di mission status
    double freq = 10.0;                                                                  // frequenza di lavoro
    double dt = 1 / freq;                                                                // tempo di campionamento
    ros::Rate loop_rate(freq);

    // Inizializzo i valori di stato desiderato
    double x_d = 0.0;       // x desiderata
    double y_d = 0.0;       // y desiderata
    double z_d = 0.0;       // z desiderata
    double psi_d = 0.0;     // psi desiderata
    double x_dot_d = 0.0;   // x derivata desiderata
    double y_dot_d = 0.0;   // y derivata desiderata
    double z_dot_d = 0.0;   // z derivata desiderata
    double psi_dot_d = 0.0; // psi derivata desiderata

    double dist_min;

    // int i_min = 0;
    while (ros::ok())
    {
        if (mission_status == "PAUSED" || mission_status == "COMPLETED")
        {
            // Vogliamo stare fermi nella posizione in cui ci troviamo
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
            if (strategy == "Initial")
            {
                // inizio missione
                double u_d = 0.5; // velocità di surge
                double v_d = 0.0; // velocità di sway
                double w_d = 0.0; // velocità di heave
                double r_d = 0.0; // velocità angolare di yaw

                // definisco la posizione relativa tra i primi due waypoint
                double pos_rel_x = x_2 - x_1; // posizione relativa nelle x
                double pos_rel_y = y_2 - y_1; // posizione relativa nelle y
                double pos_rel_z = z_2 - z_1; // posizione relativa nelle z

                // il secondo waypoint è il nostro target, definiamo la distanza da esso e vogliamo spezzare questo segmento in
                // tanti segmentini, con un passo di 10 cm
                double dist_to_targ = sqrt(pow(pos_rel_x, 2) + pow(pos_rel_y, 2) + pow(pos_rel_z, 2)); // distanza dal target
                double step = 0.1;                                                                     // passo dei segmentini

                int n_waypoints = (int)(dist_to_targ / step); // il numero di waypoint intermedi dipende dalla distanza e dallo step scelto

                double dx = pos_rel_x / n_waypoints; // spostamento nelle x
                double dy = pos_rel_y / n_waypoints; // spostamento nelle y
                double dz = pos_rel_z / n_waypoints; // spostamento nelle z

                w_d = 0.3 * dz / abs(dz); // VELOCITA' DI HEAVE COSTANTE CON SEGNO DIPENDENTE DAL SEGNO DI dz

                // definiamo dei vettori di x,y,z di dimensione pari al numero di waypoint
                double x[n_waypoints];
                double y[n_waypoints];
                double z[n_waypoints];
                double psi[n_waypoints];

                Eigen::VectorXd dist(n_waypoints); // definiamo un vettore di distanze di dimensione pari al numero di waypoint

                for (int i = 1; i <= n_waypoints; i++)
                { // riempiamo i vettori
                    x[i - 1] = x_1 + i * dx;
                    y[i - 1] = y_1 + i * dy;
                    z[i - 1] = z_1 + i * dz;
                    psi[i - 1] = atan2(dy, dx);
                }
                int i_dist_min = 0; // indice che mi dice qual è il waypoint intermedio a distanza minima
                // la strategia consiste nel dividere la retta in tanti segmentini delimitati dai waypoint e inseguire sempre
                // quello a distanza minore
                for (int i = 1; i <= n_waypoints; i++)
                {
                    dist(i - 1) = sqrt(pow(x_hat - x[i - 1], 2.0) + pow(y_hat - y[i - 1], 2.0) + pow(z_hat - z[i - 1], 2.0));
                    if (i - 1 == 0)
                    {
                        dist_min = dist(i - 1);
                    }
                    else
                    {
                        if (dist(i - 1) < dist_min)
                        {
                            dist_min = dist(i - 1);
                            i_dist_min = i - 1;
                        }
                    }
                }

                // a ogni iterazione, x,y,z desiderati saranno quelli a distanza minima (psi di conseguenza)
                x_d = x[i_dist_min];
                y_d = y[i_dist_min];
                z_d = z[i_dist_min];
                psi_d = psi[i_dist_min];

                if (i_dist_min > n_waypoints - 3) // poco prima della fine della traiettoria annulliamo le velocità per dare il tempo
                                                  // al robot di fermarsi
                {
                    u_d = 0.0;
                    v_d = 0.0;
                    w_d = 0.0;
                    r_d = 0.0;

                    // quando arriviamo nell'intorno del punto di arrivo desiderato ci fermiamo e mandiamo in PAUSED
                    if (x_hat > x_d - 0.2 && x_hat < x_d + 0.2 && y_hat > y_d - 0.2 && y_hat < y_d + 0.2 && z_hat > z_d - 0.2 && z_hat < z_d + 0.2)
                    {
                        std::string status_req = "PAUSED";
                        std_msgs::String msg;
                        msg.data = status_req;
                        publisher_status.publish(msg);
                    }
                }

                // derivate di x,y,z,psi
                x_dot_d = u_d * cos(psi_d) - v_d * sin(psi_d);
                y_dot_d = u_d * sin(psi_d) + v_d * cos(psi_d);
                z_dot_d = w_d;
                psi_dot_d = r_d;
            }
            else if (strategy == "Target") // è passato troppo tempo quindi ci disinteressiamo dell'esplorazione e andiamo a destinazione
            {
                double u_d = 0.5;
                double v_d = 0.0;
                double w_d = 0.0;
                double r_d = 0.0;

                // ragionamento analogo a quello visto per la strategia initial
                double pos_rel_x = x_t - x_1;
                double pos_rel_y = y_t - y_1;
                double pos_rel_z = z_t - z_1;

                double dist_to_targ = sqrt(pow(pos_rel_x, 2) + pow(pos_rel_y, 2) + pow(pos_rel_z, 2));
                double step = 0.1;

                int n_waypoints = (int)(dist_to_targ / step);
                ROS_WARN("n_waypoints : %d", n_waypoints);
                double dx = pos_rel_x / n_waypoints;
                double dy = pos_rel_y / n_waypoints;
                double dz = pos_rel_z / n_waypoints;
                ROS_WARN("[dx dy dz] : [%f %f %f]", dx, dy, dz);

                double x[n_waypoints];
                double y[n_waypoints];
                double z[n_waypoints];
                double psi[n_waypoints];
                Eigen::VectorXd dist(n_waypoints);

                for (int i = 1; i <= n_waypoints; i++)
                {
                    x[i - 1] = x_1 + i * dx;
                    y[i - 1] = y_1 + i * dy;
                    z[i - 1] = z_1 + i * dz;
                    psi[i - 1] = atan2(dy, dx);
                }
                int i_dist_min = 0;
                for (int i = 1; i <= n_waypoints; i++)
                {
                    dist(i - 1) = sqrt(pow(x_hat - x[i - 1], 2.0) + pow(y_hat - y[i - 1], 2.0) + pow(z_hat - z[i - 1], 2.0));
                    if (i - 1 == 0)
                    {
                        dist_min = dist(i - 1);
                    }
                    else
                    {
                        if (dist(i - 1) < dist_min)
                        {
                            dist_min = dist(i - 1);
                            i_dist_min = i - 1;
                        }
                    }
                }

                x_d = x[i_dist_min];
                y_d = y[i_dist_min];
                z_d = z[i_dist_min];
                psi_d = psi[i_dist_min];

                if (i_dist_min > n_waypoints - 3)
                {
                    u_d = 0.0;
                    v_d = 0.0;
                    w_d = 0.0;
                    r_d = 0.0;
                    if (x_hat > x_d - 0.2 && x_hat < x_d + 0.2 && y_hat > y_d - 0.2 && y_hat < y_d + 0.2 && z_hat > z_d - 0.2 && z_hat < z_d + 0.2)
                    {
                        std::string status_req = "COMPLETED";
                        std_msgs::String msg;
                        msg.data = status_req;
                        publisher_status.publish(msg);
                    }
                }

                x_dot_d = u_d * cos(psi_d) - v_d * sin(psi_d);
                y_dot_d = u_d * sin(psi_d) + v_d * cos(psi_d);
                z_dot_d = w_d;
                psi_dot_d = r_d;
            }

            else if (strategy == "Spline")
            {
                double u_d = 0.5;
                double v_d = 0.0;
                double w_d = w;
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

                if (i_dist_min > 150 && i_dist_min < 296)
                {
                    w_d = -w;
                }
                else if (i_dist_min > 296)
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

                double u_d = 0.3;
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
                    double t;
                    if (clockwise == 1.0)
                    {
                        t = -(i - 99) * k;
                        psi[i] = atan2(-cos(beta + t), sin(beta + t));
                    }
                    else
                    {
                        t = (i - 99) * k;
                        psi[i] = atan2(cos(beta + t), -sin(beta + t));
                    }
                    x[i] = x_b + cos(beta + t);
                    y[i] = y_b + sin(beta + t);
                    z[i] = z_b;
                }

                int i_dist_min;

                if (CIRCUMFERENCE_phase == 1)
                {
                    ROS_WARN("RETTA");
                    i_dist_min = 0;
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
                    if (i_dist_min >= 97)
                    {
                        CIRCUMFERENCE_phase = 2;
                    }
                }
                else if (CIRCUMFERENCE_phase == 2)
                {
                    ROS_WARN("CIRCONFERENZA 1");
                    i_dist_min = 99;
                    for (int i = 99; i < 149; i++)
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
                    if (i_dist_min >= 147)
                    {
                        CIRCUMFERENCE_phase = 3;
                    }
                }
                else
                {
                    ROS_WARN("CIRCONFERENZA 2");

                    i_dist_min = 149;
                    for (int i = 149; i < 199; i++)
                    {
                        dist(i) = sqrt(pow(x_hat - x[i], 2.0) + pow(y_hat - y[i], 2.0) + pow(z_hat - z[i], 2.0));
                        if (i == 149)
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
                psi_d = psi[i_dist_min];
                ROS_WARN("i_dist_min: %d", i_dist_min);

                if (i_dist_min > 99 && i_dist_min <= 196)
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
                    CIRCUMFERENCE_phase = 1;
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

                double u_d = 0.3;
                double v_d = 0.0;
                double w_d = 0.0;
                double r_d = 0.0;

                Eigen::VectorXd dist(119);

                double dx = (x_p - x_1) / 100;
                double dy = (y_p - y_1) / 100;
                double dz = (z_p - z_1) / 100;

                if (dz > 0)
                    w_d = 0.1;
                else
                    w_d = -0.1;

                // FASE DI AVVICINAMENTO
                for (int i = 0; i < 99; i++)
                {
                    x[i] = x_1 + (i + 1) * dx;
                    y[i] = y_1 + (i + 1) * dy;
                    z[i] = z_1 + (i + 1) * dz;
                    psi[i] = atan2(dy, dx);
                }

                // FASE DI RISALITA
                double dz2 = 0.5 / 20; // passo di risalita su asse z per ripianificazione locale

                for (int i = 99; i < 119; i++)
                {
                    x[i] = x_p;
                    y[i] = y_p;
                    z[i] = z[i - 1] + dz2;
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
                    if (i_dist_min >= 97)
                    {
                        UP_DOWN_first_phase = false;
                    }
                }
                else
                {
                    ROS_WARN("z: %f", z_hat);
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
                psi_d = psi[i_dist_min];

                if (i_dist_min > 50 && i_dist_min < 99)
                {
                    u_d = 0.1;
                    v_d = 0.0;
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
                    UP_DOWN_first_phase = true;
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
