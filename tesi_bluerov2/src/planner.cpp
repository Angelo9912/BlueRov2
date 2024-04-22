#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include <ros/console.h>
#include <eigen3/Eigen/Dense>            // for eigen matrix
#include "tesi_bluerov2/Floats.h"        // for accessing --  Floats()
#include "tesi_bluerov2/Floats_String.h" // for accessing --  buoy()
#include "tesi_bluerov2/waypoints.h"     // for accessing --  waypoints()
#include "tesi_bluerov2/buoy.h"          // for accessing --  buoy_msg()
#include <vector>
#include "yaml-cpp/yaml.h" // for yaml
#include <ctime>
#include "geodetic_functions.h"

/////////////// Definizioni variabili da richiamare nelle callback ///////////////

// Variabili di stato stimate
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

// coordinate del robot nel momento in cui arriva la stima del KF
double x_1 = 0.0;
double y_1 = 0.0;
double z_1 = 0.0;

// variabili per il messaggio della camera
double x_b = 0.0;
double y_b = 0.0;
double z_b = 0.0;
int clockwise = 0;
int up = 0;
// variabili per il waypoint msg
std::vector<double> waypoints_to_go;
std::string strategy = "";
int approach = 0;

// variabili per la gestione della macchina a stati
std::string mission_status = "";
std::string status_req = "";

bool buoy_seen = false; // booleano che ci dice se è stata avvistata una boa

/////////////// Definizioni variabili dal file yaml ///////////////

// coordinate del target
double target_x = 0.0;
double target_y = 0.0;
double target_z = 0.0;

// coordinate del punto di partenza in geodetico e in ned
double start_lat = 0.0;
double start_long = 0.0;
double start_x = 0.0;
double start_y = 0.0;

// coordinate dei 4 vertici della vasca in coordinate geodetiche
double coord_1_lat = 0.0;
double coord_1_long = 0.0;
double coord_2_lat = 0.0;
double coord_2_long = 0.0;
double coord_3_lat = 0.0;
double coord_3_long = 0.0;
double coord_4_lat = 0.0;
double coord_4_long = 0.0;

Eigen::Vector2d coord_1;
Eigen::Vector2d coord_2;
Eigen::Vector2d coord_3;
Eigen::Vector2d coord_4;

// coordinate dei vertici della vasca in NED

Eigen::Vector2d MAP_NE;
Eigen::Vector2d MAP_SE;
Eigen::Vector2d MAP_SW;
Eigen::Vector2d MAP_NW;

// velocità costanti da comandare al robot
double const surge_speed = 0.5;            // velocità da seguire normalmente
double const surge_speed_replanning = 0.2; // velocità da seguire durante ripianificazioni

// dati della camera
double const camera_range = 1.5;

// Angoli di visione della camera
double const camera_angle_vertical = 70 * M_PI / 180;    // Angolo verticale della camera
double const camera_angle_horizontal = 120 * M_PI / 180; // Angolo orizzontale della camera

// Range di visione della camera
double const camera_range_lateral = sqrt((camera_range / cos(camera_angle_horizontal / 2)) * (camera_range / cos(camera_angle_horizontal / 2)) - (1.5 * 1.5));
double const camera_range_vertical = sqrt((camera_range / cos(camera_angle_vertical / 2)) * (camera_range / cos(camera_angle_vertical / 2)) - (1.5 * 1.5));
double const total_range = sqrt(camera_range * camera_range + camera_range_lateral * camera_range_lateral + camera_range_vertical * camera_range_vertical);

// Matrici per salvare posizioni delle boe e vedere se sono state già viste o meno
Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> buoys_pos(1, 3);
Eigen::Matrix<bool, Eigen::Dynamic, Eigen::Dynamic> is_used(1, 1);
int n_buoys = 0;              // numero boe indiviudate, verrà utilizzato per scartarae le boe già visitate
double const SAFE_DIST = 2.6; // distanza di sicurezza che deve essere mantenuta dalle boe = Range di visione della camera
bool buoy_seen_prec = false;  // variabile di stato per capire se la camera ha visto una boa nella scorsa iterazione

// strategia di guida con cui raggiungere i waypoint (Rect o Spline)
std::string exploration_strategy = "Rect";

//////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////// Subscriber callback function //////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////

// variabile di stato missione, IDLE/READY/PAUSED/RUNNING/COMPLETED

void statusCallback(const std_msgs::String::ConstPtr &msg)
{
    mission_status = msg->data.c_str();
}

// Sottoscrizione alla topic degli stati stimati

void estStateCallback(const tesi_bluerov2::Floats::ConstPtr &msg)
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
void buoyCallback(const tesi_bluerov2::buoy::ConstPtr &msg)
{
    if (msg->buoy_pos[0] != msg->buoy_pos[0])
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
        if (strategy == "UP_DOWN")
            approach = msg->up;
        else if (strategy == "Circumference")
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
//////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////// Subscriber callback function //////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////

int main(int argc, char **argv)
{
    // Initialize the ROS node
    ros::init(argc, argv, "planner_node");

    // Create a ROS node handle
    ros::NodeHandle nh;

    ////////////////////////// Create a publisher object //////////////////////////
    ros::Publisher publisher = nh.advertise<tesi_bluerov2::waypoints>("waypoints_topic", 10);
    ros::Publisher publisher_status = nh.advertise<std_msgs::String>("manager/status_requested_topic", 10);

    ////////////////////////// Create a subscriber object //////////////////////////
    ros::Subscriber subscriber = nh.subscribe("sensors/buoy_topic", 10, buoyCallback);
    ros::Subscriber subscriber_state = nh.subscribe("state/est_state_UKF_topic", 10, estStateCallback);
    ros::Subscriber subscriber_status = nh.subscribe("manager/status_topic", 10, statusCallback);

    ////////////////////////// Get parameters from YAML file //////////////////////////

    nh.getParam("target_x", target_x);
    nh.getParam("target_y", target_y);
    nh.getParam("target_z", target_z);
    nh.getParam("start_lat", start_lat);
    nh.getParam("start_long", start_long);
    nh.getParam("coord_1_lat", coord_1_lat);
    nh.getParam("coord_1_long", coord_1_long);
    nh.getParam("coord_2_lat", coord_2_lat);
    nh.getParam("coord_2_long", coord_2_long);
    nh.getParam("coord_3_lat", coord_3_lat);
    nh.getParam("coord_3_long", coord_3_long);
    nh.getParam("coord_4_lat", coord_4_lat);
    nh.getParam("coord_4_long", coord_4_long);

    coord_1 << coord_1_lat, coord_1_long;
    coord_2 << coord_2_lat, coord_2_long;
    coord_3 << coord_3_lat, coord_3_long;
    coord_4 << coord_4_lat, coord_4_long;

    // posizione dei vertici della vasca in NED

    double MAP_NE_x = 0.0; // coord1toned
    double MAP_NE_y = 0.0; // coord1toned
    double MAP_SE_x = 0.0; // coord2toned
    double MAP_SE_y = 0.0; // coord2toned
    double MAP_SW_x = 0.0; // coord3toned
    double MAP_SW_y = 0.0; // coord3toned
    double MAP_NW_x = 0.0; // coord4toned
    double MAP_NW_y = 0.0; // coord4toned

    MAP_NE = ll2ne(coord_1, coord_1);
    MAP_SE = ll2ne(coord_1, coord_2);
    MAP_SW = ll2ne(coord_1, coord_3);
    MAP_NW = ll2ne(coord_1, coord_4);
    MAP_NE_x = MAP_NE(0);
    MAP_NE_y = MAP_NE(1);
    MAP_SE_x = MAP_SE(0);
    MAP_SE_y = MAP_SE(1);
    MAP_SW_x = MAP_SW(0);
    MAP_SW_y = MAP_SW(1);
    MAP_NW_x = MAP_NW(0);
    MAP_NW_y = MAP_NW(1);

    // coordinate di partenza in NED
    Eigen::Vector2d start = ll2ne(coord_1, {start_lat, start_long});
    start_x = start(0);
    start_y = start(1);

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

    // waypoint da seguire per completare la missione
    int k = 0;
    waypoints_to_go = {0.0, 2.0, 0.0, 4.0, 2.0, 0.0, 2.0, -2.0, 0.0, 0.0, -2.0, 0.0, 0.0, 0.0, 4.0, 0.0, 2.0, 4.0, 4.0, 2.0, 4.0, 2.0, -2.0, 4.0, 0.0, -2.0, 4.0, 0.0, 0.0, 4.0};
    for (int i = 0; i < waypoints_to_go.size(); i++)
    {
        waypoints_to_go[i] = waypoints_to_go[i] * 3.0;

    }

    waypoints_to_go[14] = 4.0;
    waypoints_to_go[17] = 4.0;
    waypoints_to_go[20] = 4.0;
    waypoints_to_go[23] = 4.0;
    waypoints_to_go[26] = 4.0;
    waypoints_to_go[29] = 4.0;
    

    int n_waypoints = waypoints_to_go.size() / 3;
    Eigen::VectorXd waypoints_x(n_waypoints);
    Eigen::VectorXd waypoints_y(n_waypoints);
    Eigen::VectorXd waypoints_z(n_waypoints);

    for (int i = 0; i < n_waypoints; i++)
    {
        waypoints_x(i) = waypoints_to_go[3 * i];
        waypoints_y(i) = waypoints_to_go[3 * i + 1];
        waypoints_z(i) = waypoints_to_go[3 * i + 2];
    }
    Eigen::VectorXi waypoint_passed(n_waypoints);
    waypoint_passed.setZero();

    double start_time;
    double end_time;

    ros::Rate loop_rate(20); // 20 Hz
    while (ros::ok())
    {
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
        ///////////////////////////////////////////////////////////////////////////////////////////
        //////////////////////////////// GESTIONE MACCHINA A STATI ////////////////////////////////
        ///////////////////////////////////////////////////////////////////////////////////////////

        if (mission_status == "IDLE")
        {
            /* Condizione idle solo a inizio missione, appena parte il pianificatore ci deve portare
            in stato ready per poter caricare i waypoint */
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
            Eigen::Vector3d last_waypoint;
            last_waypoint << waypoints_x.tail(1), waypoints_y.tail(1), waypoints_z.tail(1);
            double dist_to_last_waypoint = sqrt(pow((x_hat - last_waypoint(0)), 2) + pow((y_hat - last_waypoint(0)), 2) + pow((z_hat - last_waypoint(0)), 2));
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
                status_req = "RUNNING";

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
                waypoint_msg.strategy = "Target";
                std::vector<double> waypoint_pos = {target_x, target_y, target_z};
                waypoint_msg.waypoints = waypoint_pos;
                waypoint_msg.speed = surge_speed;
                publisher.publish(waypoint_msg);
                ROS_WARN("MAPPA ESPLORATA, POSSO ANDARE AL TARGET FINALE");
                status_req = "RUNNING";

                // Publish the message
                std_msgs::String status_req_msg;
                status_req_msg.data = status_req;

                // Publish the message
                publisher_status.publish(status_req_msg);

                ros::Duration(0.5).sleep(); // sleep
            }
            else if (strategy == "Circumference" && buoy_seen_prec && condition_to_run)
            {
                delete_mission = true;
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

                // Trasformazione in terna NED

                double x_p = cos(psi_hat) * x_p_body - sin(psi_hat) * y_p_body + x_hat;
                double y_p = sin(psi_hat) * x_p_body + cos(psi_hat) * y_p_body + y_hat;
                double z_p = z_p_body + z_hat;

                std::vector<double> waypoint_pos = {x_b, y_b, z_b, x_p, y_p, z_p};
                waypoint_msg.waypoints = waypoint_pos;
                waypoint_msg.speed = surge_speed_replanning;
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
                delete_mission = true;
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
                waypoint_msg.speed = surge_speed;
                waypoint_msg.approach = up;

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
                std::vector<double> waypoint_pos = {};
                for (int i = 0; i < n_waypoints; i++)
                {
                    if (waypoint_passed(i) == 0)
                    {
                        ROS_WARN("WAYPOINT %d", i);
                        waypoint_msg.strategy = exploration_strategy;
                        std::vector<double> waypoint_tmp = {waypoints_x(i), waypoints_y(i), waypoints_z(i)};
                        waypoint_pos.insert(waypoint_pos.end(), waypoint_tmp.begin(), waypoint_tmp.end());
                    }
                }
                waypoint_msg.waypoints = waypoint_pos;
                waypoint_msg.speed = surge_speed;

                publisher.publish(waypoint_msg);
                status_req = "RUNNING";

                // Publish the message
                std_msgs::String status_req_msg;
                status_req_msg.data = status_req;
                // Publish the message
                publisher_status.publish(status_req_msg);
            }
        }
        // CASO DI EMERGENZA (se il robot si trova troppo vicino al muro)
        else if (mission_status == "RUNNING")
        {
            double dist_to_next_waypoint;
            for (int i = 0; i < n_waypoints; i++)
            {
                if (waypoint_passed(i) == 0)
                {
                    dist_to_next_waypoint = sqrt(pow((x_hat - waypoints_x(i)), 2) + pow((y_hat - waypoints_y(i)), 2) + pow((z_hat - waypoints_z(i)), 2));
                    if (dist_to_next_waypoint < 0.1)
                    {
                        if (i == 0)
                        {
                            ROS_WARN("SONO PASSATO DAL WAYPOINT %d", i);
                            waypoint_passed(i) = 1;
                        }
                        else
                        {
                            if (waypoint_passed(i - 1) == 1)
                            {
                                ROS_WARN("SONO PASSATO DAL WAYPOINT %d", i);
                                waypoint_passed(i) = 1;
                            }
                        }
                    }
                }
            }
            /*if (dist_wall[i_wall_direction] <= SAFE_DIST && !to_target)
            {
                status_req = "PAUSED";

                // Publish the message
                std_msgs::String status_req_msg;
                status_req_msg.data = status_req;
                delete_mission = true;
                publisher_status.publish(status_req_msg);
                ros::Duration(0.5).sleep(); // sleep for 0.5 seconds
                // if (dist_wall[i_wall_direc_expl] <= 2 * SAFE_DIST)
                // {
                //     status_req = "PAUSED";
                //     // if (!is_second_expl)
                //     // {
                //     //     is_second_expl = !is_second_expl;
                //     //     flag_start_mission = true;
                //     // }
                //     // else
                //     // {
                //     //     to_target = true;
                //     //     delete_mission = true;
                //     // }
                //     // Publish the message

                //     std_msgs::String status_req_msg;
                //     status_req_msg.data = status_req;
                //     publisher_status.publish(status_req_msg);
                //     ros::Duration(0.5).sleep(); // sleep for half a second
                // }
                // else
                // {
                //     status_req = "PAUSED";

                //     // Publish the message
                //     std_msgs::String status_req_msg;
                //     status_req_msg.data = status_req;

                //     publisher_status.publish(status_req_msg);
                //     ros::Duration(0.5).sleep(); // sleep for .5 seconds
                // }
            }
            // PASSAGGIO DA SPLINE A BOA
            else */
            if (buoy_seen && waypoint_msg.strategy == exploration_strategy && condition_to_run)
            {
                delete_mission = true;
                ROS_WARN("BUOY SEEN");
                buoy_seen_prec = true;

                std_msgs::String status_req_msg;
                status_req = "PAUSED";

                status_req_msg.data = status_req;
                // Publish the message
                publisher_status.publish(status_req_msg);
                ros::Duration(0.5).sleep(); // sleep for half a second
            }
        }
        // Process any incoming messages
        ros::spinOnce();

        // Sleep to maintain the loop rate
        loop_rate.sleep();
    }

    return 0;
}
