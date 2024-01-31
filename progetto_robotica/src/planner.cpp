#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include <ros/console.h>
#include <eigen3/Eigen/Dense>                // for eigen matrix
#include "progetto_robotica/Floats.h"        // for accessing -- progetto_robotica Floats()
#include "progetto_robotica/Floats_String.h" // for accessing -- progetto_robotica buoy()
#include <vector>
#include "yaml-cpp/yaml.h" // for yaml
#include <ctime>

// Definizioni variabili da richiamare nelle callback
double x_hat = 0.0;
double y_hat = 0.0;
double z_hat = 0.0;
double psi_hat = 0.0;
double u_hat = 0.0;
double v_hat = 0.0;
double w_hat = 0.0;
double r_hat = 0.0;

double x_b = 0.0;
double y_b = 0.0;
double z_b = 0.0;
std::string strategy = "";
std::string mission_status = "";
std::string status_req = "";
bool buoy_seen = false;

double target_x = 0.0;
double target_y = 0.0;
double target_z = 0.0;
double MAP_NE_x = 0.0;
double MAP_NE_y = 0.0;
double MAP_SW_x = 0.0;
double MAP_SW_y = 0.0;
double MAP_NW_x = 0.0;
double MAP_NW_y = 0.0;
double MAP_SE_x = 0.0;
double MAP_SE_y = 0.0;

Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> buoys_pos(1, 3);
Eigen::Matrix<bool, Eigen::Dynamic, Eigen::Dynamic> is_used(1, 1);
int n_buoys = 0;              // numero boe indiviudate, verrà utilizzato per scartarae le boe già visitate
double const SAFE_DIST = 3.0; // distanza di sicurezza che deve essere mantenuta dalle boe

////////////////////////////////////// Subscriber callback function //////////////////////////////////////

// variabile di stato missione, PAUSED/RUNNING/COMPLETED
void statusCallback(const std_msgs::String::ConstPtr &msg)
{
    mission_status = msg->data.c_str();
}

// Sottoscrizione alla topic degli stati stimati (allo stato attuale si assume la presenza del filtro di kalman, mentre sulla
// topic al momento pubblichiamo gli stati della dinamica

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

// callback dei dati ricevuti dalla camera
void buoyCallback(const progetto_robotica::Floats_String::ConstPtr &msg)
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
        x_b = msg->data[0];
        y_b = msg->data[1];
        z_b = msg->data[2];
        strategy = msg->strategy;
        bool flag_new_buoy = true;
        bool is_in_range = sqrt(pow(x_b - x_hat, 2) + pow(y_b - y_hat, 2) + pow(z_b - z_hat, 2)) < 3.0; // se la boa è a meno di 3 metri
                                                                                                        // dal robot, la consideriamo
                                                                                                        // valida (altrimenti potrebbe
                                                                                                        // essere un falso positivo)
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
    ros::Publisher publisher = nh.advertise<progetto_robotica::Floats_String>("waypoints_topic", 10);
    ros::Publisher publisher_status = nh.advertise<std_msgs::String>("manager/status_requested_topic", 10);

    // Create a subscriber object
    ros::Subscriber subscriber = nh.subscribe("buoy_topic", 10, buoyCallback);
    ros::Subscriber subscriber_state = nh.subscribe("state_topic", 10, estStateCallback);
    ros::Subscriber subscriber_status = nh.subscribe("manager/status_topic", 10, statusCallback);

    // Get parameters from YAML file
    nh.getParam("target_x", target_x);
    nh.getParam("target_y", target_y);
    nh.getParam("target_z", target_z);
    nh.getParam("MAP_NE_x", MAP_NE_x);
    nh.getParam("MAP_NE_y", MAP_NE_y);
    nh.getParam("MAP_SW_x", MAP_SW_x);
    nh.getParam("MAP_SW_y", MAP_SW_y);
    nh.getParam("MAP_NW_x", MAP_NW_x);
    nh.getParam("MAP_NW_y", MAP_NW_y);
    nh.getParam("MAP_SE_x", MAP_SE_x);
    nh.getParam("MAP_SE_y", MAP_SE_y);

    double target[3] = {target_x, target_y, target_z}; // coordinate del target da raggiungere a fine missione
    bool condition_to_run;                             // condizione che deve essere rispettata per ripianificazione locale
    bool flag_start_mission = true;                    // flag che serve per avviare la missione
    bool inversion = false;                            // flag per fare l'inversione evitando collisioni con i muri
    bool timer_init = true;                            // flag per avviare il timer
    bool to_target = false;                            // flag che ci dice quando puntare al target di fine missione

    bool is_first_spline = true; // la spline è divisa in due "sotto-spline", con questo flag passiamo da una all'altra

    double dist_wall_up;    // distanza dal muro in alto
    double dist_wall_down;  // distanza dal muro in basso
    double dist_wall_left;  // distanza dal muro a sinistra
    double dist_wall_right; // distanza dal muro a destra
    double dist_wall[4];    // metto le distanze in un vettore di quattro elementi

    // definisco gli step delle spline nei 3 passi: ogni spline deve spostarsi di 3 metri nelle x, 3 metri nelle y e 3.5 metri nelle z
    double const SPLINE_STEP_X = 3.0;
    double const SPLINE_STEP_Y = 3.0;
    double const SPLINE_STEP_Z = 3.5;

    std::string exploring_direction = ""; // "up", "down", "left", "right" -> direzione verso cui faccio l'inversion
    std::string direction = "";           // "up", "down", "left", "right" -> direzione di avanzamento tenendo il muro sulla destra
    int i_wall_direction;                 // indice associato al muro che ho davanti mentre avanzo
    int i_wall_direc_expl;                // indice associato al muro che ho davanti nella direzione di esplorazione
    progetto_robotica::Floats_String waypoint_msg; // messaggio da pubblicare sulla topic dei waypoint

    // coordinate dei waypoint per la spline
    double x_1;
    double y_1;
    double z_1;
    double x_2;
    double y_2;
    double z_2;
    double x_3;
    double y_3;
    double z_3;

    // Comando di velocità di heave
    double w;

    // Tempi di start & end
    double start_time;
    double end_time;

    // Set the loop rate (in Hz)
    ros::Rate loop_rate(5);
    // Main loop
    while (ros::ok())
    {
        // il primo waypoint è la posizione in cui mi trovo, che prendo dal "filtro di kalman"
        x_1 = x_hat;
        y_1 = y_hat;
        z_1 = z_hat;

        // Compute distance from nearest buoy
        double dist2buoy = sqrt(pow(x_b - x_hat, 2) + pow(y_b - y_hat, 2) + pow(z_b - z_hat, 2));
        buoy_seen = dist2buoy < 3.0;
        ROS_WARN("Boa vista (main)? %d", buoy_seen);

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
            ROS_WARN("Ultima boa vista [%f,%f,%f]", buoys_pos(n_buoys - 1, 0), buoys_pos(n_buoys - 1, 1), buoys_pos(n_buoys - 1, 2));
            ROS_WARN("Boa utilizzata? %d", is_used(n_buoys - 1, 0));
            ROS_WARN("Condizione per ripianificazione: %d", condition_to_run);
        }

        // per puntare al target, associamo un timeout di missione: dopo un certo tempo (scelto considerando l'autonomia del robot)
        // il robot interrompe la sua esplo
        if (ros::Time::now().toSec() > end_time && !timer_init && mission_status == "PAUSED")
        {

            ROS_WARN("TIMEOUT DA PAUSED");
            status_req = "RUNNING";
            std_msgs::String status_req_msg;
            status_req_msg.data = status_req;
            publisher_status.publish(status_req_msg);

            ros::Duration(0.5).sleep(); // sleep

            if (!to_target)
            {
                waypoint_msg.strategy = "Target";
                std::vector<double> waypoint_pos = {target_x, target_y, target_z};
                waypoint_msg.data = waypoint_pos;
                publisher.publish(waypoint_msg);

                ros::Duration(0.5).sleep(); // sleep
                to_target = true;
            }
        }
        else if (ros::Time::now().toSec() > end_time && !timer_init && mission_status == "RUNNING" && !to_target)
        {
            ROS_WARN("TIMEOUT DA RUNNING");

            status_req = "PAUSED";
            std_msgs::String status_req_msg;
            status_req_msg.data = status_req;
            publisher_status.publish(status_req_msg);

            ros::Duration(0.5).sleep(); // sleep
        }

        else if (mission_status == "PAUSED")
        {
            if (strategy == "Circumference" && buoy_seen && condition_to_run)
            {
                ROS_WARN("CIRCUMFERENCE");
                is_used(n_buoys - 1, 0) = true;
                waypoint_msg.strategy = "Circumference";

                // Calcolo dei punti in terna body

                double x_boa_body = cos(psi_hat) * (x_b - x_hat) + sin(psi_hat) * (y_b - y_hat);
                double y_boa_body = -sin(psi_hat) * (x_b - x_hat) + cos(psi_hat) * (y_b - y_hat);
                double z_boa_body = z_b - z_hat;

                double alpha = atan2(y_boa_body, x_boa_body);
                double clockwise;
                if (y_boa_body > 0)
                {
                    alpha = alpha - M_PI / 2;
                    clockwise = 0.0;
                }
                else
                {
                    alpha = alpha + M_PI / 2;
                    clockwise = 1.0;
                }

                // Calcolo del punto di tangenza alla circonferenza in terna body
                double x_p_body = x_boa_body + cos(alpha);
                double y_p_body = y_boa_body + sin(alpha);
                double z_p_body = z_boa_body;

                // Trasformazione in terna mondo

                double x_p = cos(psi_hat) * x_p_body - sin(psi_hat) * y_p_body + x_hat;
                double y_p = sin(psi_hat) * x_p_body + cos(psi_hat) * y_p_body + y_hat;
                double z_p = z_p_body + z_hat;

                std::vector<double> waypoint_pos = {x_b, y_b, z_b, x_p, y_p, z_p, clockwise};
                waypoint_msg.data = waypoint_pos;
                status_req = "RUNNING";
                // Publish the message
                std_msgs::String status_req_msg;
                status_req_msg.data = status_req;
                // Publish the message
                publisher_status.publish(status_req_msg);
                publisher.publish(waypoint_msg);
                ros::Duration(0.5).sleep(); // sleep
            }
            else if (strategy == "UP_DOWN" && buoy_seen && condition_to_run)
            {
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
                waypoint_msg.data = waypoint_pos;
                status_req = "RUNNING";

                // Publish the message
                std_msgs::String status_req_msg;
                status_req_msg.data = status_req;

                // Publish the message
                publisher_status.publish(status_req_msg);
                publisher.publish(waypoint_msg);
                ros::Duration(0.5).sleep(); // sleep for half a second
            }
            else if (inversion)
            {
                ROS_WARN("INVERSION");
                waypoint_msg.strategy = "Spline";

                if (direction == "up")
                {

                    direction = "down";
                    i_wall_direction = 2;
                    if (exploring_direction == "left")
                    {
                        x_2 = x_1 - SPLINE_STEP_X / 2;
                        y_2 = y_1 - SPLINE_STEP_Y / 2;
                        z_2 = z_1;
                        x_3 = x_2 - SPLINE_STEP_X / 2;
                        y_3 = y_2 - SPLINE_STEP_Y / 2;
                        z_3 = z_1;
                    }
                    else if (exploring_direction == "right")
                    {
                        x_2 = x_1 + SPLINE_STEP_X / 2;
                        y_2 = y_1 - SPLINE_STEP_Y / 2;
                        z_2 = z_1;
                        x_3 = x_2 + SPLINE_STEP_X / 2;
                        y_3 = y_2 - SPLINE_STEP_Y / 2;
                        z_3 = z_1;
                    }
                }
                else if (direction == "down")
                {
                    direction = "up";
                    i_wall_direction = 0;
                    if (exploring_direction == "left")
                    {
                        x_2 = x_1 - SPLINE_STEP_X / 2;
                        y_2 = y_1 + SPLINE_STEP_Y / 2;
                        z_2 = z_1;
                        x_3 = x_2 - SPLINE_STEP_X / 2;
                        y_3 = y_2 + SPLINE_STEP_Y / 2;
                        z_3 = z_1;
                    }
                    else if (exploring_direction == "right")
                    {
                        x_2 = x_1 + SPLINE_STEP_X / 2;
                        y_2 = y_1 + SPLINE_STEP_Y / 2;
                        z_2 = z_1;
                        x_3 = x_2 + SPLINE_STEP_X / 2;
                        y_3 = y_2 + SPLINE_STEP_Y / 2;
                        z_3 = z_1;
                    }
                }
                else if (direction == "left")
                {
                    direction = "right";
                    i_wall_direction = 1;
                    if (exploring_direction == "up")
                    {
                        x_2 = x_1 + SPLINE_STEP_X / 2;
                        y_2 = y_1 + SPLINE_STEP_Y / 2;
                        z_2 = z_1;
                        x_3 = x_2 + SPLINE_STEP_X / 2;
                        y_3 = y_2 + SPLINE_STEP_Y / 2;
                        z_3 = z_1;
                    }
                    else if (exploring_direction == "down")
                    {
                        x_2 = x_1 + SPLINE_STEP_X / 2;
                        y_2 = y_1 - SPLINE_STEP_Y / 2;
                        z_2 = z_1;
                        x_3 = x_2 + SPLINE_STEP_X / 2;
                        y_3 = y_2 - SPLINE_STEP_Y / 2;
                        z_3 = z_1;
                    }
                }
                else if (direction == "right")
                {
                    direction = "left";
                    i_wall_direction = 3;
                    if (exploring_direction == "up")
                    {
                        x_2 = x_1 - SPLINE_STEP_X / 2;
                        y_2 = y_1 + SPLINE_STEP_Y / 2;
                        z_2 = z_1;
                        x_3 = x_2 - SPLINE_STEP_X / 2;
                        y_3 = y_2 + SPLINE_STEP_Y / 2;
                        z_3 = z_1;
                    }
                    else if (exploring_direction == "down")
                    {
                        x_2 = x_1 - SPLINE_STEP_X / 2;
                        y_2 = y_1 - SPLINE_STEP_Y / 2;
                        z_2 = z_1;
                        x_3 = x_2 - SPLINE_STEP_X / 2;
                        y_3 = y_2 - SPLINE_STEP_Y / 2;
                        z_3 = z_1;
                    }
                }

                std::vector<double> waypoint_pos = {x_2, y_2, z_2, x_3, y_3, z_3};
                waypoint_msg.data = waypoint_pos;
                status_req = "RUNNING";
                // Publish the message
                std_msgs::String status_req_msg;
                status_req_msg.data = status_req;
                // Publish the message
                publisher_status.publish(status_req_msg);
                inversion = false;
                publisher.publish(waypoint_msg);
                ros::Duration(1.5).sleep(); // sleep for half a second
            }
            else
            {
                //////////////////////////////////////////////////////////////////////////////////////////////////////
                /////////////////////////////////// INIZIALIZZAZIONE DELLA MISSIONE///////////////////////////////////
                //////////////////////////////////////////////////////////////////////////////////////////////////////
                ////////// Si decide la direzione iniziale di esplorazione in base alla posizione del robot //////////
                //////////////////////////////////////////////////////////////////////////////////////////////////////

                if (flag_start_mission)
                {
                    if (timer_init)
                    {
                        start_time = ros::Time::now().toSec();
                        end_time = start_time + 60.0 * 40;
                        timer_init = false;
                        ROS_WARN("START: %f", start_time);
                        ROS_WARN("END: %f", end_time);
                    }

                    flag_start_mission = false;

                    if (i_wall_min == 0)
                    {
                        direction = "left";
                        exploring_direction = "down";

                        // Muro a sinistra
                        i_wall_direction = 3;
                        i_wall_direc_expl = 2;

                        x_2 = x_1;
                        y_2 = y_1 - SPLINE_STEP_Y / 2 - SAFE_DIST / 2;
                        z_2 = z_1;
                        x_3 = x_1;
                        y_3 = y_2 - SPLINE_STEP_Y / 2 - SAFE_DIST / 2;
                        z_3 = z_1;
                    }
                    else if (i_wall_min == 1)
                    {
                        direction = "up";
                        exploring_direction = "left";

                        // Muro sopra
                        i_wall_direction = 0;
                        i_wall_direc_expl = 3;

                        x_2 = x_1 - SPLINE_STEP_X / 2 - SAFE_DIST / 2;
                        y_2 = y_1;
                        z_2 = z_1;
                        x_3 = x_2 - SPLINE_STEP_X / 2 - SAFE_DIST / 2;
                        y_3 = y_1;
                        z_3 = z_1;
                    }
                    else if (i_wall_min == 2)
                    {
                        direction = "right";
                        exploring_direction = "up";

                        // Muro a destra
                        i_wall_direction = 1;
                        i_wall_direc_expl = 0;

                        x_2 = x_1;
                        y_2 = y_1 + SPLINE_STEP_Y / 2 + SAFE_DIST / 2;
                        z_2 = z_1;
                        x_3 = x_1;
                        y_3 = y_2 + SPLINE_STEP_Y / 2 + SAFE_DIST / 2;
                        z_3 = z_1;
                    }
                    else if (i_wall_min == 3)
                    {
                        direction = "down";
                        exploring_direction = "right";

                        // Muro sotto
                        i_wall_direction = 2;
                        i_wall_direc_expl = 1;

                        x_2 = x_1 + SPLINE_STEP_X / 2 + SAFE_DIST / 2;
                        y_2 = y_1;
                        z_2 = z_1;
                        x_3 = x_2 + SPLINE_STEP_X / 2 + SAFE_DIST / 2;
                        y_3 = y_1;
                        z_3 = z_1;
                    }

                    if (dist_wall[0] <= SAFE_DIST && dist_wall[1] <= SAFE_DIST)
                    {
                        direction = "left";
                        exploring_direction = "down";

                        // Muro a sinistra
                        i_wall_direction = 3;
                        i_wall_direc_expl = 2;

                        x_2 = x_1 - SPLINE_STEP_X - SAFE_DIST;
                        y_2 = y_1 - SPLINE_STEP_Y - SAFE_DIST;
                    }
                    else if (dist_wall[1] <= SAFE_DIST && dist_wall[2] <= SAFE_DIST)
                    {
                        direction = "up";
                        exploring_direction = "left";

                        // Muro sopra
                        i_wall_direction = 0;
                        i_wall_direc_expl = 3;

                        x_2 = x_1 - SPLINE_STEP_X - SAFE_DIST;
                        y_2 = y_1 + SPLINE_STEP_Y + SAFE_DIST;
                    }
                    else if (dist_wall[2] <= SAFE_DIST && dist_wall[3] <= SAFE_DIST)
                    {
                        direction = "right";
                        exploring_direction = "up";

                        // Muro a destra
                        i_wall_direction = 1;
                        i_wall_direc_expl = 0;

                        x_2 = x_1 + SPLINE_STEP_X + SAFE_DIST;
                        y_2 = y_1 + SPLINE_STEP_Y + SAFE_DIST;
                    }
                    else if (dist_wall[3] <= SAFE_DIST && dist_wall[0] <= SAFE_DIST)
                    {
                        direction = "down";
                        exploring_direction = "right";

                        // Muro sotto
                        i_wall_direction = 2;
                        i_wall_direc_expl = 1;

                        x_2 = x_1 + SPLINE_STEP_X + SAFE_DIST;
                        y_2 = y_1 - SPLINE_STEP_Y - SAFE_DIST;
                    }
                    z_2 = 2.5;

                    ROS_WARN("ALLONTANAMENTO DAL MURO");
                    waypoint_msg.strategy = "Initial";
                    std::vector<double> waypoint_pos = {x_2, y_2, z_2};
                    waypoint_msg.data = waypoint_pos;
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
                        if (is_first_spline)
                        {
                            is_first_spline = !is_first_spline;
                            x_2 = x_1 + SPLINE_STEP_X / 2;
                            y_2 = y_1 + SPLINE_STEP_Y / 2;
                            z_2 = z_1 + SPLINE_STEP_Z / 2;
                            x_3 = x_1;
                            y_3 = y_2 + SPLINE_STEP_Y / 2;
                            z_3 = z_1;
                            w = 0.4;
                            if (z_2 > 4.25)
                            {
                                z_2 = 4.25;
                                z_3 = 2.5;
                            }
                        }
                        else
                        {
                            is_first_spline = !is_first_spline;
                            x_2 = x_1 - SPLINE_STEP_X / 2;
                            y_2 = y_1 + SPLINE_STEP_Y / 2;
                            z_2 = z_1 - SPLINE_STEP_Z / 2;
                            x_3 = x_1;
                            y_3 = y_2 + SPLINE_STEP_Y / 2;
                            z_3 = z_1;
                            w = -0.4;
                            if (z_2 < 0.75)
                            {
                                z_2 = 0.75;
                                z_3 = 2.5;
                            }
                        }
                    }
                    if (direction == "down")
                    {
                        if (is_first_spline)
                        {
                            is_first_spline = !is_first_spline;
                            x_2 = x_1 - SPLINE_STEP_X / 2;
                            y_2 = y_1 - SPLINE_STEP_Y / 2;
                            z_2 = z_1 + SPLINE_STEP_Z / 2;
                            x_3 = x_1;
                            y_3 = y_2 - SPLINE_STEP_Y / 2;
                            z_3 = z_1;
                            w = 0.4;
                            if (z_2 > 4.25)
                            {
                                z_2 = 4.25;
                                z_3 = 2.5;
                            }
                        }
                        else
                        {
                            is_first_spline = !is_first_spline;
                            x_2 = x_1 + SPLINE_STEP_X / 2;
                            y_2 = y_1 - SPLINE_STEP_Y / 2;
                            z_2 = z_1 - SPLINE_STEP_Z / 2;
                            x_3 = x_1;
                            y_3 = y_2 - SPLINE_STEP_Y / 2;
                            z_3 = z_1;
                            w = -0.4;
                            if (z_2 < 0.75)
                            {
                                z_2 = 0.75;
                                z_3 = 2.5;
                            }
                        }
                    }
                    if (direction == "left")
                    {
                        if (is_first_spline)
                        {
                            is_first_spline = !is_first_spline;
                            x_2 = x_1 - SPLINE_STEP_X / 2;
                            y_2 = y_1 + SPLINE_STEP_Y / 2;
                            z_2 = z_1 + SPLINE_STEP_Z / 2;
                            x_3 = x_2 - SPLINE_STEP_X / 2;
                            y_3 = y_1;
                            z_3 = z_1;
                            w = 0.4;
                            if (z_2 > 4.25)
                            {
                                z_2 = 4.25;
                                z_3 = 2.5;
                            }
                        }
                        else
                        {
                            is_first_spline = !is_first_spline;
                            x_2 = x_1 - SPLINE_STEP_X / 2;
                            y_2 = y_1 - SPLINE_STEP_Y / 2;
                            z_2 = z_1 - SPLINE_STEP_Z / 2;
                            x_3 = x_2 - SPLINE_STEP_X / 2;
                            y_3 = y_1;
                            z_3 = z_1;
                            w = -0.4;
                            if (z_2 < 0.75)
                            {
                                z_2 = 0.75;
                                z_3 = 2.5;
                            }
                        }
                    }
                    if (direction == "right")
                    {
                        if (is_first_spline)
                        {
                            is_first_spline = !is_first_spline;
                            x_2 = x_1 + SPLINE_STEP_X / 2;
                            y_2 = y_1 - SPLINE_STEP_Y / 2;
                            z_2 = z_1 + SPLINE_STEP_Z / 2;
                            x_3 = x_2 + SPLINE_STEP_X / 2;
                            y_3 = y_1;
                            z_3 = z_1;
                            w = 0.4;
                            if (z_2 > 4.25)
                            {
                                z_2 = 4.25;
                                z_3 = 2.5;
                            }
                        }
                        else
                        {
                            is_first_spline = !is_first_spline;
                            x_2 = x_1 + SPLINE_STEP_X / 2;
                            y_2 = y_1 + SPLINE_STEP_Y / 2;
                            z_2 = z_1 - SPLINE_STEP_Z / 2;
                            x_3 = x_2 + SPLINE_STEP_X / 2;
                            y_3 = y_1;
                            z_3 = z_1;
                            w = -0.4;
                            if (z_2 < 0.75)
                            {
                                z_2 = 0.75;
                                z_3 = 2.5;
                            }
                        }
                    }
                    ROS_WARN("SPLINE");
                    waypoint_msg.strategy = "Spline";
                    std::vector<double> waypoint_pos = {x_2, y_2, z_2, x_3, y_3, z_3, w};
                    waypoint_msg.data = waypoint_pos;
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
        else if (mission_status == "RUNNING" && dist_wall[i_wall_direction] <= SAFE_DIST && !inversion)
        {
            if (dist_wall[i_wall_direc_expl] <= 2 * SPLINE_STEP_X)
            {
                ROS_WARN("I_WALL_DIRECTION: %d I_WALL_DIRECTION_EXP %d", i_wall_direction, i_wall_direc_expl);
                status_req = "PAUSED";
                flag_start_mission = true;
                // Publish the message
                std_msgs::String status_req_msg;
                status_req_msg.data = status_req;
                publisher_status.publish(status_req_msg);
                ros::Duration(0.5).sleep(); // sleep for half a second
            }
            else
            {
                ROS_WARN("SONO NELL'ELSE DI INVERSION");
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
        else if (mission_status == "RUNNING" && buoy_seen && waypoint_msg.strategy == "Spline" && condition_to_run)
        {
            status_req = "PAUSED";
            ROS_WARN("BUOY SEEN");
            // Publish the message
            std_msgs::String status_req_msg;
            status_req_msg.data = status_req;
            // Publish the message
            publisher_status.publish(status_req_msg);
            ros::Duration(0.5).sleep(); // sleep for half a second
        }
        // Process any incoming messages
        bool tmp = mission_status == "RUNNING" && buoy_seen && waypoint_msg.strategy == "Spline" && condition_to_run;
        ROS_WARN("POSSO ANDARE A FARE UP_DOWN: %d", tmp);
        ros::spinOnce();

        // Sleep to maintain the loop rate
        loop_rate.sleep();
    }
    return 0;
}
