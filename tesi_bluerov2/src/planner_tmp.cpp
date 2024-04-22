#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include <ros/console.h>
#include <eigen3/Eigen/Dense>            // for eigen matrix
#include "tesi_bluerov2/Floats.h"        // for accessing --  Floats()
#include "tesi_bluerov2/Floats_String.h" // for accessing --  buoy()
#include "tesi_bluerov2/waypoints.h"     // for accessing --  waypoints()
#include "tesi_bluerov2/buoy_msg.h"      // for accessing --  waypoints()
#include <vector>
#include "yaml-cpp/yaml.h" // for yaml
#include <ctime>
#include "geodetic_functions.h"

// Definizioni variabili da richiamare nelle callback
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

double x_1 = 0.0;
double y_1 = 0.0;
double z_1 = 0.0;
double x_b = 0.0;
double y_b = 0.0;
double z_b = 0.0;
std::string strategy = "";
int approach = 0;
std::string mission_status = "";
std::string status_req = "";
bool buoy_seen = false;

double target_x = 0.0;
double target_y = 0.0;
double target_z = 0.0;
double coord_1 = 0.0;
double coord_2 = 0.0;
double coord_3 = 0.0;
double coord_4 = 0.0;
double MAP_NE_x = 0.0; // coord1toned
double MAP_NE_y = 0.0; // coord1toned
double MAP_SE_x = 0.0; // coord2toned
double MAP_SE_y = 0.0; // coord2toned
double MAP_SW_x = 0.0; // coord3toned
double MAP_SW_y = 0.0; // coord3toned
double MAP_NW_x = 0.0; // coord4toned
double MAP_NW_y = 0.0; // coord4toned

double const camera_range = 1.5;
// Angoli di visione della camera
double const camera_angle_vertical = 70 * M_PI / 180;    // Angolo verticale della camera
double const camera_angle_horizontal = 120 * M_PI / 180; // Angolo orizzontale della camera

// Range di visione della camera

double const camera_range_lateral = sqrt((camera_range / cos(camera_angle_horizontal / 2)) * (camera_range / cos(camera_angle_horizontal / 2)) - (1.5 * 1.5));
double const camera_range_vertical = sqrt((camera_range / cos(camera_angle_vertical / 2)) * (camera_range / cos(camera_angle_vertical / 2)) - (1.5 * 1.5));

double const total_range = sqrt(camera_range * camera_range + camera_range_lateral * camera_range_lateral + camera_range_vertical * camera_range_vertical);

Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> buoys_pos(1, 3);
Eigen::Matrix<bool, Eigen::Dynamic, Eigen::Dynamic> is_used(1, 1);
int n_buoys = 0;              // numero boe indiviudate, verrà utilizzato per scartarae le boe già visitate
double const SAFE_DIST = 2.6; // distanza di sicurezza che deve essere mantenuta dalle boe = Range di visione della camera
bool buoy_seen_prec = false;  // variabile di stato per capire se la camera ha visto una boa nella scorsa iterazione

std::string exploration_strategy = "Rect";

////////////////////////////////////// Subscriber callback function //////////////////////////////////////

// variabile di stato missione, PAUSED/RUNNING/COMPLETED
void statusCallback(const std_msgs::String::ConstPtr &msg)
{
    mission_status = msg->data.c_str();
}

// Sottoscrizione alla topic degli stati stimati (allo stato attuale si assume la presenza del filtro di kalman, mentre sulla
// topic al momento pubblichiamo gli stati della dinamica

void estStateCallback(const ::Floats::ConstPtr &msg)
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

// callback dei dati ricevuti dalla camera
void buoyCallback(const tesi_bluerov2::buoy_msg::ConstPtr &msg)
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
        x_b = msg->buoy_pos[0];
        y_b = msg->buoy_pos[1];
        z_b = msg->buoy_pos[2];
        strategy = msg->strategy;
        if (msg->clockwise = "")
            approach = msg->up;
        else
            approach = msg->clockwise;
        bool flag_new_buoy = true;

        // Calcolo delle coordinate della boa in terna body
        double pos_rel_x = cos(psi_hat) * (x_b - x_hat) + sin(psi_hat) * (y_b - y_hat);
        double pos_rel_y = -sin(psi_hat) * (x_b - x_hat) + cos(psi_hat) * (y_b - y_hat);
        double pos_rel_z = z_b - z_hat;

        // Condizione che deve essere rispettata per considerare la boa all'interno del range di visione della camera
        bool is_in_range = (pos_rel_x <= camera_range) && (pos_rel_y <= tan(camera_angle_horizontal / 2) * pos_rel_x) && (pos_rel_y >= -tan(camera_angle_horizontal / 2) * pos_rel_x) && (pos_rel_z <= tan(camera_angle_vertical / 2) * pos_rel_x) && (pos_rel_z >= -tan(camera_angle_vertical / 2));

        // valutiamo la prima boa avvistata
        if (n_buoys == 0 && is_in_range)
        {
            buoys_pos << x_b, y_b, z_b; // se ne aggiungono le coordinate nella matrice buoys_pos (sarà una matrice con n righe
                                        // e 3 colonne
            is_used << false;           // è un vettore colonna con n righe, il primo elemento ora si setta a false in modo da
                                        // non ripescare questa boa
            n_buoys++;
        }
        else if (n_buoys > 0)
        {
            // scorriamo il vettore delle boe, se la boa individuata dalla camera è già stata visitata, non la consideriamo, altrimenti
            // la aggiungiamo alla matrice
            for (int i = 0; i < n_buoys; i++)
            {
                if (buoys_pos(i, 0) >= x_b - 1.0 && buoys_pos(i, 0) <= x_b + 1.0 && buoys_pos(i, 1) >= y_b - 1.0 && buoys_pos(i, 1) <= y_b + 1.0 && buoys_pos(i, 2) >= z_b - 1.0 && buoys_pos(i, 2) <= z_b + 1.0)
                {
                    flag_new_buoy = false;
                }
            }
            if (flag_new_buoy && is_in_range) // la boa vista è "nuova"
            {
                // incremento delle dimensioni delle matrici dinamiche con le coordinate della nuova boa
                buoys_pos.conservativeResize(n_buoys + 1, 3);
                buoys_pos(n_buoys, 0) = x_b;
                buoys_pos(n_buoys, 1) = y_b;
                buoys_pos(n_buoys, 2) = z_b;
                is_used.conservativeResize(n_buoys + 1, 1);
                is_used(n_buoys, 0) = false;
                n_buoys++;
            }
        }
    }
}

int main(int argc, char **argv)
{
    // Initialize the ROS node
    ros::init(argc, argv, "planner_node");

    // Create a ROS node handle
    ros::NodeHandle nh;

    // Create a publisher object
    ros::Publisher publisher = nh.advertise<tesi_bluerov2::waypoints>("waypoints_topic", 10);
    ros::Publisher publisher_status = nh.advertise<std_msgs::String>("manager/status_requested_topic", 10);

    // Create a subscriber object
    ros::Subscriber subscriber = nh.subscribe("buoy_topic", 10, buoyCallback);
    ros::Subscriber subscriber_state = nh.subscribe("est_state_UKF_topic", 10, estStateCallback);
    ros::Subscriber subscriber_status = nh.subscribe("manager/status_topic", 10, statusCallback);

    // Get parameters from YAML file
    nh.getParam("target_x", target_x);
    nh.getParam("target_y", target_y);
    nh.getParam("target_z", target_z);

    double target[3] = {target_x, target_y, target_z}; // coordinate del target da raggiungere a fine missione
    bool condition_to_run;                             // condizione che deve essere rispettata per ripianificazione locale
    bool flag_start_mission = true;                    // flag che serve per avviare la missione
    bool inversion = false;                            // flag per fare l'inversione evitando collisioni con i muri
    bool timer_init = true;                            // flag per avviare il timer
    bool to_target = false;                            // flag che ci dice quando puntare al target di fine missione
    bool is_second_expl = false;
    bool delete_mission = false; // booleano che mi dice se quando vado in paused devo cancellare la missione e crearne una nuova

    bool is_first_spline = true; // la spline è divisa in due "sotto-spline", con questo flag passiamo da una all'altra

    double dist_wall_up;    // distanza dal muro in alto
    double dist_wall_down;  // distanza dal muro in basso
    double dist_wall_left;  // distanza dal muro a sinistra
    double dist_wall_right; // distanza dal muro a destra
    double dist_wall[4];    // metto le distanze in un vettore di quattro elementi

    std::string exploring_direction = "";  // "up", "down", "left", "right" -> direzione verso cui faccio l'inversion
    std::string direction = "";            // "up", "down", "left", "right" -> direzione di avanzamento tenendo il muro sulla destra
    int i_wall_direction;                  // indice associato al muro che ho davanti mentre avanzo
    int i_wall_direc_expl;                 // indice associato al muro che ho davanti nella direzione di esplorazione
    tesi_bluerov2::waypoints waypoint_msg; // messaggio da pubblicare sulla topic dei waypoint

    // coordinate dei waypoint per la spline

    std::vector waypoints = {0.0, 2.0, 0.0, 2.0, 2.0, 0.0, 2.0, -2.0, 0.0, 0.0, -2.0, 0.0, 0.0, 0.0, 4.0, 0.0, 2.0, 4.0, 2.0, 2.0, 4.0, 2.0, -2.0, 4.0, 0.0, -2.0, 4.0, 0.0, 0.0, 4.0};

    Eigen::Vector<bool, Eigen::Dynamic> waypoint_passed(size(waypoints) / 3);
    for (int i = 0; i < waypoint_passed; i++)
    {
        waypoint_passed(i) << false;
    }
    // Comando di velocità di heave
    double w;

    // Tempi di start & end
    double start_time;
    double end_time;

    // Angoli di visione della camera
    double camera_angle_vertical = 70 * M_PI / 180;    // Angolo verticale della camera
    double camera_angle_horizontal = 120 * M_PI / 180; // Angolo orizzontale della camera

    // Range di visione della camera

    double const camera_range_lateral = sqrt((camera_range / cos(camera_angle_horizontal / 2)) * (camera_range / cos(camera_angle_horizontal / 2)) - (1.5 * 1.5));
    double const camera_range_vertical = sqrt((camera_range / cos(camera_angle_vertical / 2)) * (camera_range / cos(camera_angle_vertical / 2)) - (1.5 * 1.5));

    // Set the loop rate (in Hz)
    ros::Rate loop_rate(5);
    // Main loop
    while (ros::ok())
    {
        // il primo waypoint è la posizione in cui mi trovo, che prendo dal "filtro di kalman"
        x_1 = x_hat;
        y_1 = y_hat;
        z_1 = z_hat;

        // Calcolo delle coordinate della boa in terna body
        double pos_rel_x = cos(psi_hat) * (x_b - x_hat) + sin(psi_hat) * (y_b - y_hat);
        double pos_rel_y = -sin(psi_hat) * (x_b - x_hat) + cos(psi_hat) * (y_b - y_hat);
        double pos_rel_z = z_b - z_hat;

        // Condizione che deve essere rispettata per considerare la boa all'interno del range di visione della camera
        bool buoy_seen = (pos_rel_x <= camera_range) && (pos_rel_y <= tan(camera_angle_horizontal / 2) * pos_rel_x) && (pos_rel_y >= -tan(camera_angle_horizontal / 2) * pos_rel_x) && (pos_rel_z <= tan(camera_angle_vertical / 2) * pos_rel_x) && (pos_rel_z >= -tan(camera_angle_vertical / 2));

        // Compute distance from walls
        dist_wall_up = abs(MAP_NE_y - y_hat);
        dist_wall_down = abs(MAP_SW_y - y_hat);
        dist_wall_left = abs(MAP_SW_x - x_hat);
        dist_wall_right = abs(MAP_NE_x - x_hat);
        dist_wall[0] = dist_wall_up;
        dist_wall[1] = dist_wall_right;
        dist_wall[2] = dist_wall_down;
        dist_wall[3] = dist_wall_left;

        // Calcolo del muro a distanza minima
        int i_wall_min = 0;
        for (int i = 0; i < 4; i++)
        {
            if (dist_wall[i] < dist_wall[i_wall_min])
            {
                i_wall_min = i;
            }
        }

        if (n_buoys == 0)
        {
            condition_to_run = false;
        }
        else
        {
            // definisco il valore booleano della condizione per ripianificazione
            condition_to_run = buoys_pos(n_buoys - 1, 0) == x_b && buoys_pos(n_buoys - 1, 1) == y_b && buoys_pos(n_buoys - 1, 2) == z_b && !is_used(n_buoys - 1, 0);
        }

        // per puntare al target, associamo un timeout di missione: dopo un certo tempo (scelto considerando l'autonomia del robot)
        // il robot interrompe la sua esplorazione
        if (ros::Time::now().toSec() > end_time && !timer_init)
        {
            if (mission_status == "PAUSED")
            {

                ROS_WARN("TIMEOUT DA PAUSED");
                status_req = "READY";
                std_msgs::String status_req_msg;
                status_req_msg.data = status_req;
                publisher_status.publish(status_req_msg);

                ros::Duration(0.5).sleep(); // sleep
            }
            else if (mission_status == "READY")
            {
                if (!to_target)
                {
                    waypoint_msg.strategy = "Target";
                    std::vector<double> waypoint_pos = {target_x, target_y, target_z};
                    waypoint_msg.waypoints = waypoint_pos;
                    waypoint_msg.speed = 0.5;
                    publisher.publish(waypoint_msg);

                    ros::Duration(0.5).sleep(); // sleep
                    to_target = true;
                }
            }
            else if (mission_status == "RUNNING" && !to_target)
            {
                ROS_WARN("TIMEOUT DA RUNNING");

                status_req = "PAUSED";
                std_msgs::String status_req_msg;
                status_req_msg.data = status_req;
                publisher_status.publish(status_req_msg);

                ros::Duration(0.5).sleep(); // sleep
            }
        }

        else
        {
            if (mission_status == "IDLE")
            {
                status_req = "READY";

                // Publish the message
                std_msgs::String status_req_msg;
                status_req_msg.data = status_req;

                // Publish the message
                publisher_status.publish(status_req_msg);
            }

            else if (mission_status == "PAUSED")
            {
                // se la posizione del robot è nell'intorno dell'ultimo waypoint, significa che questa
                // parte di missione è stata completata e posso caricare nuovi waypoint, quindi vado a ready
                double dist_to_last_waypoint = sqrt((x_hat - waypoints_x.tail(1)) ^ 2 + (y_hat - waypoints_y.tail(1)) ^ 2 + (z_hat - waypoints_z.tail(1)) ^ 2);
                if (dist_to_last_waypoint < 0.5)
                    delete_mission = true;

                if (delete_mission)
                {
                    status_req = "READY";

                    // Publish the message
                    std_msgs::String status_req_msg;
                    status_req_msg.data = status_req;

                    // Publish the message
                    publisher_status.publish(status_req_msg);
                    delete_mission = false;
                }
                else
                {
                    status_req = "RUNNINNG";

                    // Publish the message
                    std_msgs::String status_req_msg;
                    status_req_msg.data = status_req;

                    // Publish the message
                    publisher_status.publish(status_req_msg);
                }
            }

            else if (mission_status == "READY")
            {
                if (to_target)
                {
                    ROS_WARN("MAPPA ESPLORATA, POSSO ANDARE AL TARGET FINALE");
                    status_req = "RUNNING";

                    // Publish the message
                    std_msgs::String status_req_msg;
                    status_req_msg.data = status_req;

                    // Publish the message
                    publisher_status.publish(status_req_msg);
                    waypoint_msg.strategy = "Target";
                    std::vector<double> waypoint_pos = {target_x, target_y, target_z};
                    waypoint_msg.waypoints = waypoint_pos;
                    waypoint_msg.speed = 0.5;
                    publisher.publish(waypoint_msg);

                    ros::Duration(0.5).sleep(); // sleep
                }
                else if (strategy == "Circumference" && buoy_seen_prec && condition_to_run)
                {
                    buoy_seen_prec = false;
                    ROS_WARN("CIRCUMFERENCE");
                    is_used(n_buoys - 1, 0) = true;
                    waypoint_msg.strategy = "Circumference";

                    // Calcolo dei punti in terna body

                    double x_boa_body = cos(psi_hat) * (x_b - x_hat) + sin(psi_hat) * (y_b - y_hat);
                    double y_boa_body = -sin(psi_hat) * (x_b - x_hat) + cos(psi_hat) * (y_b - y_hat);
                    double z_boa_body = z_b - z_hat;

                    double alpha = atan2(y_boa_body, x_boa_body);

                    // Calcolo del punto di tangenza alla circonferenza in terna body
                    double x_p_body = x_boa_body + cos(alpha);
                    double y_p_body = y_boa_body + sin(alpha);
                    double z_p_body = z_boa_body;

                    // Trasformazione in terna mondo

                    double x_p = cos(psi_hat) * x_p_body - sin(psi_hat) * y_p_body + x_hat;
                    double y_p = sin(psi_hat) * x_p_body + cos(psi_hat) * y_p_body + y_hat;
                    double z_p = z_p_body + z_hat;

                    std::vector<double> waypoint_pos = {x_b, y_b, z_b, x_p, y_p, z_p};
                    waypoint_msg.waypoints = waypoint_pos;
                    waypoint_msg.speed = 0.2;
                    waypoint_msg.approach = clockwise;
                    status_req = "RUNNING";
                    // Publish the message
                    std_msgs::String status_req_msg;
                    status_req_msg.data = status_req;
                    // Publish the message
                    publisher_status.publish(status_req_msg);
                    publisher.publish(waypoint_msg);
                    ros::Duration(0.5).sleep(); // sleep
                }
                else if (strategy == "UP_DOWN" && buoy_seen_prec && condition_to_run)
                {
                    buoy_seen_prec = false;
                    ROS_WARN("UP_DOWN");
                    is_used(n_buoys - 1, 0) = true;
                    waypoint_msg.strategy = "UP_DOWN";

                    // Calcolo dei punti in terna body

                    double x_boa_body = cos(psi_hat) * (x_b - x_hat) + sin(psi_hat) * (y_b - y_hat);
                    double y_boa_body = -sin(psi_hat) * (x_b - x_hat) + cos(psi_hat) * (y_b - y_hat);
                    double z_boa_body = z_b - z_hat;

                    double alpha = atan2(y_boa_body, x_boa_body);

                    if (y_boa_body > 0)
                    {
                        alpha = alpha - M_PI / 2;
                    }
                    else
                    {
                        alpha = alpha + M_PI / 2;
                    }

                    // Calcolo del punto di tangenza alla circonferenza in terna body
                    double x_p_body = x_boa_body + cos(alpha);
                    double y_p_body = y_boa_body + sin(alpha);
                    double z_p_body = z_boa_body;

                    // Trasformazione in terna mondo

                    double x_p = cos(psi_hat) * x_p_body - sin(psi_hat) * y_p_body + x_hat;
                    double y_p = sin(psi_hat) * x_p_body + cos(psi_hat) * y_p_body + y_hat;
                    double z_p = z_p_body + z_hat;

                    std::vector<double> waypoint_pos = {x_p, y_p, z_p};
                    waypoint_msg.waypoints = waypoint_pos;
                    waypoint_msg.speed = 0.5;

                    status_req = "RUNNING";

                    // Publish the message
                    std_msgs::String status_req_msg;
                    status_req_msg.data = status_req;

                    // Publish the message
                    publisher_status.publish(status_req_msg);
                    publisher.publish(waypoint_msg);
                    ros::Duration(0.5).sleep(); // sleep for half a second
                }
                /*else if (inversion)
                {
                    ROS_WARN("INVERSION");
                    waypoint_msg.strategy = exploration_strategy;

                    if (direction == "up")
                    {

                        direction = "down";
                        i_wall_direction = 2;
                        if (exploring_direction == "left")
                        {
                            x_2 = x_1 - 2 * camera_range_lateral;
                            y_2 = y_1;
                            x_3 = x_2;
                            y_3 = y_2 - 0.5;
                        }
                        else if (exploring_direction == "right")
                        {
                            x_2 = x_1 + 2 * camera_range_lateral;
                            y_2 = y_1;
                            x_3 = x_2;
                            y_3 = y_2 - 0.5;
                        }
                    }
                    else if (direction == "down")
                    {
                        direction = "up";
                        i_wall_direction = 0;
                        if (exploring_direction == "left")
                        {
                            x_2 = x_1 - 2 * camera_range_lateral;
                            y_2 = y_1;
                            x_3 = x_2;
                            y_3 = y_2 + 0.5;
                        }
                        else if (exploring_direction == "right")
                        {
                            x_2 = x_1 + 2 * camera_range_lateral;
                            y_2 = y_1;
                            x_3 = x_2;
                            y_3 = y_2 + 0.5;
                        }
                    }
                    else if (direction == "left")
                    {
                        direction = "right";
                        i_wall_direction = 1;
                        if (exploring_direction == "up")
                        {
                            x_2 = x_1;
                            y_2 = y_1 + 2 * camera_angle_horizontal;
                            x_3 = x_2 + 0.5;
                            y_3 = y_2;
                        }
                        else if (exploring_direction == "down")
                        {
                            x_2 = x_1;
                            y_2 = y_1 - 2 * camera_angle_horizontal;
                            x_3 = x_2 + 0.5;
                            y_3 = y_2;
                        }
                    }
                    else if (direction == "right")
                    {
                        direction = "left";
                        i_wall_direction = 3;
                        if (exploring_direction == "up")
                        {
                            x_2 = x_1;
                            y_2 = y_1 + 2 * camera_angle_horizontal;
                            x_3 = x_2 - 0.5;
                            y_3 = y_2;
                        }
                        else if (exploring_direction == "down")
                        {
                            x_2 = x_1;
                            y_2 = y_1 - 2 * camera_angle_horizontal;
                            x_3 = x_2 - 0.5;
                            y_3 = y_2;
                        }
                    }

                    if (!is_second_expl)
                        z_2 = 1.5;
                    else if (is_second_expl)
                        z_2 = 3.5;
                    else
                        z_2 = 2.5;

                    z_3 = z_2;

                    std::vector<double> waypoint_pos = {x_2, y_2, z_2, x_3, y_3, z_3};
                    waypoint_msg.waypoints = waypoint_pos;
                    status_req = "RUNNING";
                    // Publish the message
                    std_msgs::String status_req_msg;
                    status_req_msg.data = status_req;
                    // Publish the message
                    publisher_status.publish(status_req_msg);
                    inversion = false;
                    publisher.publish(waypoint_msg);
                    ros::Duration(0.5).sleep(); // sleep for half a second
                }*/
                else
                {
                    //////////////////////////////////////////////////////////////////////////////////////////////////////
                    /////////////////////////////////// INIZIALIZZAZIONE DELLA MISSIONE //////////////////////////////////
                    //////////////////////////////////////////////////////////////////////////////////////////////////////
                    ////////// Si decide la direzione iniziale di esplorazione in base alla posizione del robot //////////
                    //////////////////////////////////////////////////////////////////////////////////////////////////////

                    if (flag_start_mission)
                    {
                        ros::Duration(2.0).sleep(); // sleep for half a second
                        if (timer_init)
                        {
                            start_time = ros::Time::now().toSec();
                            end_time = start_time + 60.0 * 40;
                            timer_init = false;
                        }

                        flag_start_mission = false;

                        if (i_wall_min == 0)
                        {
                            if (x_hat >= 0)
                            {
                                direction = "left";
                                i_wall_direction = 3;
                            }
                            else
                            {
                                direction = "right";
                                i_wall_direction = 1;
                            }

                            exploring_direction = "down";
                            i_wall_direc_expl = 2;

                            x_2 = x_1;
                        }
                        else if (i_wall_min == 1)
                        {
                            if (y_hat >= 0)
                            {
                                direction = "down";
                                i_wall_direction = 2;
                            }
                            else
                            {
                                direction = "up";
                                i_wall_direction = 0;
                            }

                            exploring_direction = "left";
                            i_wall_direc_expl = 3;

                            y_2 = y_1;
                        }
                        else if (i_wall_min == 2)
                        {
                            if (x_hat >= 0)
                            {
                                direction = "left";
                                i_wall_direction = 3;
                            }
                            else
                            {
                                direction = "right";
                                i_wall_direction = 1;
                            }

                            exploring_direction = "up";
                            i_wall_direc_expl = 0;

                            x_2 = x_1;
                        }
                        else if (i_wall_min == 3)
                        {
                            if (y_hat >= 0)
                            {
                                direction = "down";
                                i_wall_direction = 2;
                            }
                            else
                            {
                                direction = "up";
                                i_wall_direction = 0;
                            }

                            exploring_direction = "right";
                            i_wall_direc_expl = 1;

                            y_2 = y_1;
                        }

                        if (dist_wall[0] <= SAFE_DIST && dist_wall[1] <= SAFE_DIST)
                        {
                            direction = "left";
                            exploring_direction = "down";

                            // Muro a sinistra
                            i_wall_direction = 3;
                            i_wall_direc_expl = 2;

                            x_2 = 20 - SAFE_DIST;
                            y_2 = 20 - SAFE_DIST;
                        }
                        else if (dist_wall[1] <= SAFE_DIST && dist_wall[2] <= SAFE_DIST)
                        {
                            direction = "up";
                            exploring_direction = "left";

                            // Muro sopra
                            i_wall_direction = 0;
                            i_wall_direc_expl = 3;

                            x_2 = 20 - SAFE_DIST;
                            y_2 = -20 + SAFE_DIST;
                        }
                        else if (dist_wall[2] <= SAFE_DIST && dist_wall[3] <= SAFE_DIST)
                        {
                            direction = "right";
                            exploring_direction = "up";

                            // Muro a destra
                            i_wall_direction = 1;
                            i_wall_direc_expl = 0;

                            x_2 = -20 + SAFE_DIST;
                            y_2 = -20 + SAFE_DIST;
                        }
                        else if (dist_wall[3] <= SAFE_DIST && dist_wall[0] <= SAFE_DIST)
                        {
                            direction = "down";
                            exploring_direction = "right";

                            // Muro sotto
                            i_wall_direction = 2;
                            i_wall_direc_expl = 1;

                            x_2 = -20 + SAFE_DIST;
                            y_2 = 20 - SAFE_DIST;
                        }
                        if (!is_second_expl)
                            z_2 = 1.5;
                        else
                            z_2 = 3.5;

                        ROS_WARN("AWAY FROM THE WALL");
                        waypoint_msg.strategy = "Initial";
                        std::vector<double> waypoint_pos = {x_2, y_2, z_2};
                        waypoint_msg.waypoints = waypoint_pos;
                        waypoint_msg.speed = 0.1;
                        status_req = "RUNNING";
                        // Publish the message
                        std_msgs::String status_req_msg;
                        status_req_msg.data = status_req;
                        // Publish the message
                        publisher_status.publish(status_req_msg);
                        publisher.publish(waypoint_msg);
                        ros::Duration(0.5).sleep(); // sleep for half a second
                    }
                    else
                    {

                        if (direction == "up")
                        {
                        }
                        if (direction == "down")
                        {
                        }
                        if (direction == "left")
                        {
                        }
                        if (direction == "right")
                        {
                        }
                        waypoint_msg.strategy = exploration_strategy;
                        std::vector<double> waypoint_pos = {x_2, y_2, z_2, x_3, y_3, z_3, w};
                        waypoint_msg.waypoints = waypoint_pos;
                        waypoint_msg.speed = 0.5;
                        status_req = "RUNNING";
                        // Publish the message
                        std_msgs::String status_req_msg;
                        status_req_msg.data = status_req;
                        // Publish the message
                        publisher_status.publish(status_req_msg);
                        publisher.publish(waypoint_msg);
                        ros::Duration(0.5).sleep(); // sleep for half a second
                    }
                }
            }

            // CASO DI EMERGENZA (se il robot si trova troppo vicino al muro)
            else if (mission_status == "RUNNING")
            {
                if (dist_wall[i_wall_direction] <= SAFE_DIST && !inversion && !to_target)
                {
                    if (dist_wall[i_wall_direc_expl] <= 2 * SAFE_DIST)
                    {
                        status_req = "PAUSED";
                        if (!is_second_expl)
                        {
                            is_second_expl = !is_second_expl;
                            flag_start_mission = true;
                        }
                        else
                        {
                            to_target = true;
                            delete_mission = true;
                        }

                        // Publish the message
                        std_msgs::String status_req_msg;
                        status_req_msg.data = status_req;
                        publisher_status.publish(status_req_msg);
                        ros::Duration(0.5).sleep(); // sleep for half a second
                    }
                    else
                    {
                        inversion = true;
                        status_req = "PAUSED";

                        // Publish the message
                        std_msgs::String status_req_msg;
                        status_req_msg.data = status_req;

                        publisher_status.publish(status_req_msg);
                        ros::Duration(0.5).sleep(); // sleep for 1.5 seconds
                    }
                }
                // PASSAGGIO DA SPLINE A BOA
                else if (buoy_seen && waypoint_msg.strategy == exploration_strategy && condition_to_run)
                {
                    status_req = "PAUSED";
                    delete_mission = true;
                    ROS_WARN("BUOY SEEN");
                    buoy_seen_prec = true;
                    // Publish the message
                    std_msgs::String status_req_msg;
                    status_req_msg.data = status_req;
                    // Publish the message
                    publisher_status.publish(status_req_msg);
                    ros::Duration(0.5).sleep(); // sleep for half a second
                }
            }
        }
        // Process any incoming messages
        ros::spinOnce();

        // Sleep to maintain the loop rate
        loop_rate.sleep();
    }
    return 0;
}