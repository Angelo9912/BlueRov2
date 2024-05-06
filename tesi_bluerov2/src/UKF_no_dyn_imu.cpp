#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include <ros/console.h>
#include <eigen3/Eigen/Dense>     // for eigen matrix
#include "tesi_bluerov2/Floats.h" // for accessing -- tesi_bluerov2 Floats()
#include <vector>
#include "yaml-cpp/yaml.h" // for yaml
#include "UnscentedOutput.h"
#include <random>
#include <rosbag/bag.h>

rosbag::Bag bag;

double a_u = 0.0;
double a_v = 0.0;
double a_w = 0.0;

double x_GPS = 0.0;
double y_GPS = 0.0;
double x_scanner = 0.0;
double y_scanner = 0.0;
double phi_IMU = 0.0;
double theta_IMU = 0.0;
double psi_IMU = 0.0;
double u_DVL = 0.0;
double v_DVL = 0.0;
double w_DVL = 0.0;
double p_IMU_camera = 0.0;
double q_IMU_camera = 0.0;
double r_IMU_camera = 0.0;
double z_depth_sensor = 0.0;

double valid_GPS = 0;
double valid_scanner = 0;
double valid_IMU = 0;
double valid_DVL = 0;
double valid_depth_sensor = 0;
double valid_IMU_camera = 0;

// Covariance values (process noise)
double var_acc_u = 0.0;
double var_acc_v = 0.0;
double var_acc_w = 0.0;
double var_tau_p = 0.0;
double var_tau_q = 0.0;
double var_tau_r = 0.0;
// Covariance values (sensor noise)

double var_x_GPS = 0.0;
double var_y_GPS = 0.0;
double var_x_scanner = 0.0;
double var_y_scanner = 0.0;
double var_z_depth_sensor = 0.0;
double var_phi_IMU = 0.0;
double var_theta_IMU = 0.0;
double var_psi_IMU = 0.0;
double var_u_DVL = 0.0;
double var_v_DVL = 0.0;
double var_w_DVL = 0.0;
double var_p_IMU_camera = 0.0;
double var_q_IMU_camera = 0.0;
double var_r_IMU_camera = 0.0;

// Parameters for the Unscented Kalman Filter

// Import parameters from YAML file
double m = 0.0;
double x_g = 0.0;
double y_g = 0.0;
double z_g = 0.0;
double x_b = 0.0;
double y_b = 0.0;
double z_b = 0.0;
double I_x = 0.0;
double I_y = 0.0;
double I_z = 0.0;
double I_xy = 0.0;
double I_xz = 0.0;
double I_yz = 0.0;
double I_yx = 0.0;
double I_zx = 0.0;
double I_zy = 0.0;
double A_x = 0.0;
double A_y = 0.0;
double A_z = 0.0;
double A_p = 0.0;
double A_q = 0.0;
double A_r = 0.0;
double X_u_dot = 0.0;
double X_v_dot = 0.0;
double X_w_dot = 0.0;
double X_p_dot = 0.0;
double X_q_dot = 0.0;
double X_r_dot = 0.0;
double Y_u_dot = 0.0;
double Y_v_dot = 0.0;
double Y_w_dot = 0.0;
double Y_p_dot = 0.0;
double Y_q_dot = 0.0;
double Y_r_dot = 0.0;
double Z_u_dot = 0.0;
double Z_v_dot = 0.0;
double Z_w_dot = 0.0;
double Z_p_dot = 0.0;
double Z_q_dot = 0.0;
double Z_r_dot = 0.0;
double K_u_dot = 0.0;
double K_v_dot = 0.0;
double K_w_dot = 0.0;
double K_p_dot = 0.0;
double K_q_dot = 0.0;
double K_r_dot = 0.0;
double M_u_dot = 0.0;
double M_v_dot = 0.0;
double M_w_dot = 0.0;
double M_p_dot = 0.0;
double M_q_dot = 0.0;
double M_r_dot = 0.0;
double N_u_dot = 0.0;
double N_v_dot = 0.0;
double N_w_dot = 0.0;
double N_p_dot = 0.0;
double N_q_dot = 0.0;
double N_r_dot = 0.0;

std::string GNC_status = "NOT_READY";

// Distanza di Mahalanobis
double mahalanobis_distance = 0.0;

// Function to generate Gaussian random number
double gaussianNoise(double mean, double var)
{
    double stddev = sqrt(var);
    std::random_device rd;
    std::mt19937 gen(rd());
    std::normal_distribution<> d(mean, stddev);
    return d(gen);
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

// Trasformata unscented (predizione)
UnscentedOutput UnscentedTransform_Prediction(Eigen::VectorXd xi_k, Eigen::VectorXd acc_k, Eigen::VectorXd gyro_k, Eigen::MatrixXd P_kk, Eigen::MatrixXd Q, double dt)
{

    // ROS_WARN_STREAM("acc_k " << acc_k);

    // Il vettore utilizzato effettivamente nella trasformata
    Eigen::VectorXd xi_prediction(xi_k.size() + Q.rows());
    Eigen::VectorXd zeros(Q.rows());
    zeros.setZero();
    xi_prediction << xi_k, zeros;

    int n = xi_prediction.size();
    int n_sigma = 2 * n + 1;
    int alpha = 1.0;
    double kappa = 0.0;
    double beta = 2.0;
    double lambda = alpha * alpha * (n + kappa) - n;

    Eigen::MatrixXd Zeros(xi_k.size(), Q.rows());
    Zeros.setZero();

    Eigen::MatrixXd ZerosT(Q.rows(), xi_k.size());
    ZerosT.setZero();

    Eigen::MatrixXd Sigma(n, n);
    Sigma << P_kk, Zeros,
        ZerosT, Q;

    // SVD decomposition
    Eigen::MatrixXd U(n, n); // compute the SVD decomposition of Sigma
    Eigen::VectorXd Singular_values(n);
    Singular_values = Sigma.bdcSvd(Eigen::ComputeFullU).singularValues();
    U = Sigma.bdcSvd(Eigen::ComputeFullU).matrixU();

    // Create sigma points
    Eigen::MatrixXd sigma_points(n, n_sigma);

    double weights[n_sigma];

    for (int i = 0; i < n; i++)
    {
        sigma_points.col(i) = xi_prediction + sqrt(n + lambda) * U.col(i) * sqrt(Singular_values(i));
        sigma_points.col(i)(3) = wrapToPi(sigma_points.col(i)(3));
        sigma_points.col(i)(4) = wrapToPi(sigma_points.col(i)(4));
        sigma_points.col(i)(5) = wrapToPi(sigma_points.col(i)(5));

        sigma_points.col(i + n) = xi_prediction - sqrt(n + lambda) * U.col(i) * sqrt(Singular_values(i));
        sigma_points.col(i + n)(3) = wrapToPi(sigma_points.col(i + n)(3));
        sigma_points.col(i + n)(4) = wrapToPi(sigma_points.col(i + n)(4));
        sigma_points.col(i + n)(5) = wrapToPi(sigma_points.col(i + n)(5));

        weights[i] = 1 / (2 * (n + lambda));
        weights[i + n] = 1 / (2 * (n + lambda));
    }

    sigma_points.col(n_sigma - 1) = xi_prediction;

    weights[n_sigma - 1] = lambda / (n + lambda);

    Eigen::MatrixXd sigma_points_out(12, n_sigma);

    // Compute the transformed sigma points

    for (int i = 0; i < n_sigma; i++)
    {
        Eigen::VectorXd xi_aug(n);
        xi_aug = sigma_points.col(i);

        Eigen::VectorXd eta(6);
        eta = xi_aug.head(6);

        Eigen::VectorXd nu(6);
        nu = xi_aug(Eigen::seq(6, 11));

        Eigen::VectorXd acc_noise(6);
        acc_noise = xi_aug.tail(6);

        double phi = eta(3);
        double theta = eta(4);
        double psi = eta(5);

        double u = nu(0);
        double v = nu(1);
        double w = nu(2);
        
        nu(3) = gyro_k(0);
        nu(4) = gyro_k(1);
        nu(5) = gyro_k(2);

        double p = nu(3);
        double q = nu(4);
        double r = nu(5);
        // Dinamica del sistema

        double u_pred = dt * (acc_k(0) + acc_noise(0)) + u;
        double v_pred = dt * (acc_k(1) + acc_noise(1)) + v;
        double w_pred = dt * (acc_k(2) + acc_noise(2)) + w;
        double p_pred = p + acc_noise(3);
        double q_pred = q + acc_noise(4);
        double r_pred = r + acc_noise(5);

    

        Eigen::Matrix<double, 6, 1> nu_k1;

        nu_k1 << u_pred, v_pred, w_pred, p_pred, q_pred, r_pred;

        // VETTORE DELLE POSIZIONI
        Eigen::Matrix<double, 6, 6> Jacobian;
        Jacobian << cos(psi) * cos(theta), cos(psi) * sin(phi) * sin(theta) - cos(phi) * sin(psi), sin(phi) * sin(psi) + cos(phi) * cos(psi) * sin(theta), 0, 0, 0,
            cos(theta) * sin(psi), cos(phi) * cos(psi) + sin(phi) * sin(psi) * sin(theta), cos(phi) * sin(psi) * sin(theta) - cos(psi) * sin(phi), 0, 0, 0,
            -sin(theta), cos(theta) * sin(phi), cos(phi) * cos(theta), 0, 0, 0,
            0, 0, 0, 1, sin(phi) * tan(theta), cos(phi) * tan(theta),
            0, 0, 0, 0, cos(phi), -sin(phi),
            0, 0, 0, 0, sin(phi) / cos(theta), cos(phi) / cos(theta);

        Eigen::VectorXd eta_k1(6);
        Eigen::VectorXd a_m(6);
        a_m << acc_k(0) + acc_noise(0), acc_k(1) + acc_noise(1), acc_k(2) + acc_noise(2), 0, 0, 0;

        eta_k1 = Jacobian * (dt * nu + dt * dt / 2 * a_m) + eta;

        // wrapToPi(eta_k1(phi));
        eta_k1(3) = wrapToPi(eta_k1(3));
        // wrapToPi(eta_k1(theta));
        eta_k1(4) = wrapToPi(eta_k1(4));
        // wrapToPi(eta_k1(psi));
        eta_k1(5) = wrapToPi(eta_k1(5));
        sigma_points_out.col(i) << eta_k1, nu_k1;
    }

    // Compute the transformed mean

    Eigen::VectorXd xi_out(xi_k.size());
    xi_out.setZero();

    for (int i = 0; i < n_sigma; i++)
    {
        xi_out += sigma_points_out.col(i) * weights[i];
    }

    // Transformed mean for angular variables

    double x_phi = 0.0;
    double x_theta = 0.0;
    double x_psi = 0.0;
    double y_phi = 0.0;
    double y_theta = 0.0;
    double y_psi = 0.0;

    for (int i = 0; i < n_sigma; i++)
    {
        x_phi += cos(sigma_points_out(3, i)) * weights[i];
        x_theta += cos(sigma_points_out(4, i)) * weights[i];
        x_psi += cos(sigma_points_out(5, i)) * weights[i];
        y_phi += sin(sigma_points_out(3, i)) * weights[i];
        y_theta += sin(sigma_points_out(4, i)) * weights[i];
        y_psi += sin(sigma_points_out(5, i)) * weights[i];
    }

    xi_out(3) = atan2(y_phi, x_phi);
    xi_out(4) = atan2(y_theta, x_theta);
    xi_out(5) = atan2(y_psi, x_psi);

    // Per la costruzione dei momenti di ordine secondo si usa un peso diverso solo per l'elemento w0
    weights[n_sigma - 1] = weights[n_sigma - 1] + (1 - alpha * alpha + beta);

    // Compute the transformed covariance
    Eigen::MatrixXd SigmaX_out(xi_k.size(), xi_k.size());
    SigmaX_out.setZero();

    Eigen::VectorXd Deviation(xi_k.size());

    for (int i = 0; i < n_sigma; i++)
    {
        Deviation = sigma_points_out.col(i) - xi_out;
        Deviation(3) = angleDifference(Deviation(3));
        Deviation(4) = angleDifference(Deviation(4));
        Deviation(5) = angleDifference(Deviation(5));

        SigmaX_out += weights[i] * Deviation * Deviation.transpose();
    }

    // Initialize the cross-covariance matrix
    Eigen::MatrixXd SigmaXY_out(xi_k.size(), xi_k.size());
    SigmaXY_out.setZero();

    return UnscentedOutput(xi_out, SigmaX_out, SigmaXY_out);
}

UnscentedOutput UnscentedTransform_Correction(Eigen::VectorXd xi_k, Eigen::VectorXd z_k, Eigen::MatrixXd P_kk, Eigen::VectorXd valid, Eigen::VectorXd var_used)
{

    // Definiamo i dati secondo le misure che abbiamo ricevuto

    Eigen::VectorXd xi_correction(xi_k.size());
    Eigen::MatrixXd R(var_used.size(), var_used.size());
    R = var_used.asDiagonal();

    xi_correction << xi_k;

    int n = xi_correction.size();

    Eigen::MatrixXd Sigma(n, n);
    Sigma << P_kk;

    int n_sigma = 2 * n + 1;
    int alpha = 1.0;
    double kappa = 0.0;
    double beta = 2.0;
    double lambda = alpha * alpha * (n + kappa) - n;

    // SVD decomposition
    Eigen::MatrixXd U(n, n); // compute the SVD decomposition of Sigma
    Eigen::VectorXd Singular_values(n);

    Singular_values = Sigma.bdcSvd(Eigen::ComputeFullU).singularValues();
    U = Sigma.bdcSvd(Eigen::ComputeFullU).matrixU();

    // Create sigma points
    Eigen::MatrixXd sigma_points(n, n_sigma);

    double weights[n_sigma];

    for (int i = 0; i < n; i++)
    {
        sigma_points.col(i) = xi_correction + sqrt(n + lambda) * U.col(i) * sqrt(Singular_values(i));
        sigma_points.col(i)(3) = wrapToPi(sigma_points.col(i)(3));
        sigma_points.col(i)(4) = wrapToPi(sigma_points.col(i)(4));
        sigma_points.col(i)(5) = wrapToPi(sigma_points.col(i)(5));

        sigma_points.col(i + n) = xi_correction - sqrt(n + lambda) * U.col(i) * sqrt(Singular_values(i));
        sigma_points.col(i + n)(3) = wrapToPi(sigma_points.col(i + n)(3));
        sigma_points.col(i + n)(4) = wrapToPi(sigma_points.col(i + n)(4));
        sigma_points.col(i + n)(5) = wrapToPi(sigma_points.col(i + n)(5));

        weights[i] = 1 / (2 * (n + lambda));
        weights[i + n] = 1 / (2 * (n + lambda));
    }

    sigma_points.col(n_sigma - 1) = xi_correction;
    weights[n_sigma - 1] = lambda / (n + lambda);

    // Pesco solo i valori dello stato predetto associati a misure valide
    // in modo tale da poter calcolare l'innovazione in maniera corretta
    // Eventualmente salvo la posizione della variabile angolare phi se è stata misurata
    int id_phi = -1;

    Eigen::MatrixXd sigma_points_out(var_used.size(), n_sigma);

    for (int i = 0; i < n_sigma; i++)
    {
        int j = 0;

        for (int k = 0; k < valid.size(); k++)
        {
            if (valid(k) == 1.0) // Controllo la validità della misura ricevuta
            {
                if (k < 2) // Se è una misura di posizione del GPS k=j
                {
                    sigma_points_out(j, i) = sigma_points(k, i);
                }
                else // Altrimenti k = j + 2
                {
                    sigma_points_out(j, i) = sigma_points(k - 2, i);

                    if (k == 5) // Salvo la posizione della variabile angolare phi
                    {
                        id_phi = j;
                    }
                }
                j++;
            }
        }
    }

    // Calcoliamo la misura data dai valori predetti come media pesata dei sigma points

    Eigen::VectorXd z_out(var_used.size());

    z_out.setZero();

    for (int i = 0; i < n_sigma; i++)
    {
        z_out += sigma_points_out.col(i) * weights[i];
    }

    // Calcolo la media pesata dei sigma points per le variabili angolari

    if (valid(5) == 1 && id_phi >= 0) // dati dell'IMU validi
    {

        double x_phi = 0.0;
        double x_theta = 0.0;
        double x_psi = 0.0;
        double y_phi = 0.0;
        double y_theta = 0.0;
        double y_psi = 0.0;

        for (int i = 0; i < n_sigma; i++)
        {
            x_phi += cos(sigma_points(3, i)) * weights[i];
            x_theta += cos(sigma_points(4, i)) * weights[i];
            x_psi += cos(sigma_points(5, i)) * weights[i];
            y_phi += sin(sigma_points(3, i)) * weights[i];
            y_theta += sin(sigma_points(4, i)) * weights[i];
            y_psi += sin(sigma_points(5, i)) * weights[i];
        }

        z_out(id_phi) = atan2(y_phi, x_phi);
        z_out(id_phi + 1) = atan2(y_theta, x_theta);
        z_out(id_phi + 2) = atan2(y_psi, x_psi);
    }

    // Calcolo l'innovazione
    Eigen::VectorXd e_k(var_used.size());
    e_k = z_k - z_out;

    if (id_phi >= 0)
    {
        for (int i = id_phi; i < id_phi + 3; i++)
        {
            e_k(i) = angleDifference(e_k(i));
        }
    }

    // Calcolo la matrice di covarianza pesata dei sigma points

    Eigen::MatrixXd Pzz_out(var_used.size(), var_used.size());
    Pzz_out.setZero();

    weights[n_sigma - 1] = weights[n_sigma - 1] + (1 - alpha * alpha + beta);

    // Calcolo lo scarto dei sigma_points_out
    Eigen::MatrixXd sigma_points_out_diff(var_used.size(), n_sigma);

    // Calcolo lo scarto dei sigma_points
    Eigen::VectorXd DeviationZ(n);

    for (int i = 0; i < n_sigma; i++)
    {
        DeviationZ = sigma_points_out.col(i) - z_out;
        if (id_phi >= 0)
        {
            for (int j = id_phi; j < id_phi + 3; j++)
            {
                DeviationZ(j) = angleDifference(DeviationZ(j));
            }
        }

        Pzz_out += weights[i] * DeviationZ * DeviationZ.transpose();
    }

    // Calcolo la matrice di covarianza incrociata

    Eigen::MatrixXd Pxz_out(n, var_used.size());
    Pxz_out.setZero();

    Eigen::VectorXd DeviationX(n);

    for (int i = 0; i < n_sigma; i++)
    {
        DeviationX = sigma_points.col(i) - xi_correction;
        DeviationX(3) = angleDifference(DeviationX(3));
        DeviationX(4) = angleDifference(DeviationX(4));
        DeviationX(5) = angleDifference(DeviationX(5));

        DeviationZ = sigma_points_out.col(i) - z_out;
        if (id_phi >= 0)
        {
            for (int j = id_phi; j < id_phi + 3; j++)
            {
                DeviationZ(j) = angleDifference(DeviationZ(j));
            }
        }

        Pxz_out += weights[i] * DeviationX * DeviationZ.transpose();
    }

    return UnscentedOutput(e_k, Pzz_out + R, Pxz_out);
}

void GNCstatusCallback(const std_msgs::String::ConstPtr &msg) // CALLBACK che riceve lo stato del GNC
{
    GNC_status = msg->data;
}

// Callback function for the subscriber
void GPSCallback(const tesi_bluerov2::Floats::ConstPtr &msg)
{
    if (msg->data[0] != msg->data[0])
    {
        x_GPS = 0.0;
        y_GPS = 0.0;
        valid_GPS = 0;
    }
    else
    {
        x_GPS = msg->data[0];
        y_GPS = msg->data[1];
        valid_GPS = msg->data[2];
    }
}

void ScannerCallback(const tesi_bluerov2::Floats::ConstPtr &msg)
{
    if (msg->data[0] != msg->data[0])
    {
        x_scanner = 0.0;
        y_scanner = 0.0;
        valid_scanner = 0;
    }
    else
    {
        x_scanner = msg->data[0];
        y_scanner = msg->data[1];
        valid_scanner = msg->data[2];
    }
}

void IMUCallback(const tesi_bluerov2::Floats::ConstPtr &msg)
{
    if (msg->data[0] != msg->data[0])
    {
        phi_IMU = 0.0;
        theta_IMU = 0.0;
        psi_IMU = 0.0;
        valid_IMU = 0;
    }
    else
    {
        phi_IMU = msg->data[0];
        theta_IMU = msg->data[1];
        psi_IMU = msg->data[2];
        valid_IMU = msg->data[3];
    }
}
void IMU_cameraCallback(const tesi_bluerov2::Floats::ConstPtr &msg)
{
    if (msg->data[0] != msg->data[0])
    {
        p_IMU_camera = 0.0;
        q_IMU_camera = 0.0;
        r_IMU_camera = 0.0;
        valid_IMU_camera = 0;
    }
    else
    {
        p_IMU_camera = msg->data[0];
        q_IMU_camera = msg->data[1];
        r_IMU_camera = msg->data[2];
        valid_IMU_camera = msg->data[3];
    }
}

void DVLCallback(const tesi_bluerov2::Floats::ConstPtr &msg)
{
    if (msg->data[0] != msg->data[0])
    {
        u_DVL = 0.0;
        v_DVL = 0.0;
        w_DVL = 0.0;
        valid_DVL = 0;
    }
    else
    {
        u_DVL = msg->data[0];
        v_DVL = msg->data[1];
        w_DVL = msg->data[2];
        valid_DVL = msg->data[3];
    }
}

void depthSensorCallback(const tesi_bluerov2::Floats::ConstPtr &msg)
{
    if (msg->data[0] != msg->data[0])
    {
        z_depth_sensor = 0.0;
        valid_depth_sensor = 0;
    }
    else
    {
        z_depth_sensor = msg->data[0];
        valid_depth_sensor = msg->data[1];
    }
}

void acc_callback(const tesi_bluerov2::Floats::ConstPtr &msg)
{
    if (msg->data[0] != msg->data[0])
    {
        a_u = 0.0;
        a_v = 0.0;
        a_w = 0.0;
    }
    else
    {
        a_u = msg->data[0];
        a_v = msg->data[1];
        a_w = msg->data[2];
    }
}

int main(int argc, char **argv)
{
    // Initialize the ROS system and become a node.
    ros::init(argc, argv, "UKF_no_dyn_imu_node");
    // Create a ROS node handle
    ros::NodeHandle n;
    n.getParam("var_x_GPS", var_x_GPS);
    n.getParam("var_y_GPS", var_y_GPS);
    n.getParam("var_x_scanner", var_x_scanner);
    n.getParam("var_y_scanner", var_y_scanner);
    n.getParam("var_z_depth_sensor", var_z_depth_sensor);
    n.getParam("var_phi_IMU", var_phi_IMU);
    n.getParam("var_theta_IMU", var_theta_IMU);
    n.getParam("var_psi_IMU", var_psi_IMU);
    n.getParam("var_u_DVL", var_u_DVL);
    n.getParam("var_v_DVL", var_v_DVL);
    n.getParam("var_w_DVL", var_w_DVL);
    n.getParam("var_p_IMU_camera", var_p_IMU_camera);
    n.getParam("var_q_IMU_camera", var_q_IMU_camera);
    n.getParam("var_r_IMU_camera", var_r_IMU_camera);

    n.getParam("var_acc_u", var_acc_u);
    n.getParam("var_acc_v", var_acc_v);
    n.getParam("var_acc_w", var_acc_w);
    n.getParam("var_tau_p", var_tau_p);
    n.getParam("var_tau_q", var_tau_q);
    n.getParam("var_tau_r", var_tau_r);

    // Current corrected state vector (12x1)
    Eigen::VectorXd xi_curr(12);
    xi_curr.setZero();

    // Matrice MMSE
    Eigen::Matrix<double, 12, 12> P_curr;
    P_curr.setZero();

    Eigen::VectorXd xi_pred(12);
    xi_pred.setZero();

    Eigen::Matrix<double, 12, 12> P_pred;
    P_pred.setZero();

    // Time step
    double freq = 50;
    double dt = 1 / freq;
    std::string path = ros::package::getPath("tesi_bluerov2");
    bag.open(path + "/bag/ukf_kinematics_imu.bag", rosbag::bagmode::Write);

    bool is_init = false;

    bool is_GPS_init = false;
    bool is_scanner_init = false;
    bool is_depth_init = false;
    bool is_IMU_init = false;
    bool is_IMU_camera_init = false;

    Eigen::VectorXd var_sensors(11);
    var_sensors << var_x_GPS, var_y_GPS, var_x_scanner, var_y_scanner, var_z_depth_sensor, var_phi_IMU, var_theta_IMU, var_psi_IMU, var_u_DVL, var_v_DVL, var_w_DVL;

    Eigen::MatrixXd Q(6, 6);
    Q << var_acc_u, 0, 0, 0, 0, 0,
        0, var_acc_v, 0, 0, 0, 0,
        0, 0, var_acc_w, 0, 0, 0,
        0, 0, 0, var_p_IMU_camera, 0, 0,
        0, 0, 0, 0, var_q_IMU_camera, 0,
        0, 0, 0, 0, 0, var_r_IMU_camera;

    // Set the loop rate
    ros::Rate loop_rate(freq);

    // Create a publisher object
    ros::Publisher est_state_pub = n.advertise<tesi_bluerov2::Floats>("state/est_state_UKF_no_dyn_imu_topic", 1000);

    ros::Publisher publisher_gnc_status = n.advertise<std_msgs::String>("manager/GNC_status_requested_topic", 10); // publisher stato richiesto al GNC

    // Create subscriber objects

    ros::Subscriber gnc_status_sub = n.subscribe("manager/GNC_status_topic", 1, GNCstatusCallback); // sottoscrizione alla topic di stato del GNC
    ros::Subscriber acc_sub = n.subscribe("sensors/acc_topic", 1000, acc_callback);
    ros::Subscriber GPS_sub = n.subscribe("sensors/GPS_topic", 1000, GPSCallback);
    ros::Subscriber scanner_sub = n.subscribe("sensors/scanner_topic", 1000, ScannerCallback);
    ros::Subscriber IMU_sub = n.subscribe("sensors/IMU_topic", 1000, IMUCallback);
    ros::Subscriber IMU_camera_sub = n.subscribe("sensors/IMU_camera_topic", 1000, IMU_cameraCallback);
    ros::Subscriber DVL_sub = n.subscribe("sensors/DVL_topic", 1000, DVLCallback);
    ros::Subscriber depth_sub = n.subscribe("sensors/depth_sensor_topic", 1000, depthSensorCallback);

    // Create a message object
    tesi_bluerov2::Floats msg;

    while (ros::ok())
    {

        //////Kalman Filter///////

        ///////////////////////////////////////////////////////////////////////
        ///////////////// INITIALIZATION OF THE FILTER ////////////////////////
        ///////////////////////////////////////////////////////////////////////
        if (GNC_status == "NOT_READY")
        {
            std::string status_req = "NAVIGATION_READY";
            std_msgs::String msg;
            msg.data = status_req;

            // Publish the message on the "topic1" topic
            publisher_gnc_status.publish(msg);
        }
        else if (!is_init && GNC_status != "NOT_READY")
        {
            double var_x;
            double var_y;

            if (valid_scanner == 1.0)
            {
                xi_curr(0) = x_scanner;
                xi_curr(1) = y_scanner;
                var_x = var_x_scanner;
                var_y = var_y_scanner;
                is_scanner_init = true;
            }
            else if (valid_GPS == 1.0)
            {
                xi_curr(0) = x_GPS;
                xi_curr(1) = y_GPS;
                var_x = var_x_GPS;
                var_y = var_y_GPS;
                is_GPS_init = true;
            }

            if (valid_depth_sensor == 1.0)
            {
                xi_curr(2) = z_depth_sensor;
                is_depth_init = true;
            }

            if (valid_IMU == 1.0)
            {
                xi_curr(3) = phi_IMU;
                xi_curr(4) = theta_IMU;
                xi_curr(5) = psi_IMU;

                is_IMU_init = true;
            }
            if (valid_IMU_camera == 1.0)
            {
                xi_curr(9) = p_IMU_camera;
                xi_curr(10) = q_IMU_camera;
                xi_curr(11) = r_IMU_camera;

                is_IMU_camera_init = true;
            }
            xi_curr.setZero();
            // xi_curr(6) = 0.0 + gaussianNoise(0, var_u_DVL);
            // xi_curr(7) = 0.0 + gaussianNoise(0, var_v_DVL);
            // xi_curr(8) = 0.0 + gaussianNoise(0, var_w_DVL);
            // xi_curr(9) = 0.0 + gaussianNoise(0, var_p_IMU);
            // xi_curr(10) = 0.0 + gaussianNoise(0, var_q_IMU);
            // xi_curr(11) = 0.0 + gaussianNoise(0, var_r_IMU);

            P_curr << var_x, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                0, var_y, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                0, 0, var_z_depth_sensor, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                0, 0, 0, var_phi_IMU, 0, 0, 0, 0, 0, 0, 0, 0,
                0, 0, 0, 0, var_theta_IMU, 0, 0, 0, 0, 0, 0, 0,
                0, 0, 0, 0, 0, var_psi_IMU, 0, 0, 0, 0, 0, 0,
                0, 0, 0, 0, 0, 0, var_u_DVL, 0, 0, 0, 0, 0,
                0, 0, 0, 0, 0, 0, 0, var_v_DVL, 0, 0, 0, 0,
                0, 0, 0, 0, 0, 0, 0, 0, var_w_DVL, 0, 0, 0,
                0, 0, 0, 0, 0, 0, 0, 0, 0, var_p_IMU_camera, 0, 0,
                0, 0, 0, 0, 0, 0, 0, 0, 0, 0, var_q_IMU_camera, 0,
                0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, var_r_IMU_camera;

            is_init = ((is_GPS_init || is_scanner_init) && is_depth_init && is_IMU_init && is_IMU_camera_init);
            if (is_init) // Se l'inizializzazione è completata pubblica lo stato stimato iniziale e passa alla predizione
            {
                ROS_WARN_STREAM("UKF INITIALIZATION COMPLETED \n");

                msg.data = {xi_curr(0), xi_curr(1), xi_curr(2), xi_curr(3), xi_curr(4), xi_curr(5), xi_curr(6), xi_curr(7), xi_curr(8), xi_curr(9), xi_curr(10), xi_curr(11), mahalanobis_distance};
                est_state_pub.publish(msg);

                ////////////////////////////////////////////////////////////////////
                /////////////////////////PREDICTION/////////////////////////////////
                ////////////////////////////////////////////////////////////////////

                // VETTORE DI FORZE E MOMENTI
                Eigen::VectorXd acc(3);
                acc << a_u, a_v, a_w;
                Eigen::VectorXd gyro(3);
                gyro << p_IMU_camera, q_IMU_camera, r_IMU_camera;

                // Predizionegyro
                UnscentedOutput Prediction_out = UnscentedTransform_Prediction(xi_curr, acc, gyro, P_curr, Q, dt);

                xi_pred = Prediction_out.getX();

                P_pred = Prediction_out.getSigmaX();
            }
        }
        else
        {
            ///////////////////////////////////////////////////////////////////////
            ///////////////////////////// CORRECTION //////////////////////////////
            ///////////////////////////////////////////////////////////////////////

            // VETTORE DI MISURA
            Eigen::VectorXd z(11);
            z << x_GPS, y_GPS, x_scanner, y_scanner, z_depth_sensor, phi_IMU, theta_IMU, psi_IMU, u_DVL, v_DVL, w_DVL;

            Eigen::VectorXd valid(11);
            valid << valid_GPS, valid_GPS, valid_scanner, valid_scanner, valid_depth_sensor, valid_IMU, valid_IMU, valid_IMU, valid_DVL, valid_DVL, valid_DVL;

            // Pesco le misure valide
            Eigen::VectorXd z_valid(1);
            z_valid.setZero();
            bool is_first_element = true;
            int n_valid = 0;

            for (int i = 0; i < valid.size(); i++)
            {
                if (valid(i) == 1.0)
                {
                    if (z_valid.size() == 1 && is_first_element)
                    {
                        z_valid(0) = z(i);
                        is_first_element = false;
                    }
                    else
                    {
                        n_valid = z_valid.size();
                        z_valid.conservativeResize(n_valid + 1);
                        n_valid++;
                        z_valid(n_valid - 1) = z(i);
                    }
                }
            }

            Eigen::VectorXd var_used(z_valid.size());
            var_used.setZero();
            int j = 0;
            for (int i = 0; i < valid.size(); i++)
            {
                if (valid(i) == 1.0)
                {
                    var_used(j) = var_sensors(i);
                    j++;
                }
            }

            if (z_valid.size() == 1 && z_valid(0) == 0.0) // Nessuna misura valida
            {
                ROS_WARN("NO VALID MEASURES");
                xi_curr = xi_pred;
                P_curr = P_pred;
                mahalanobis_distance = 0.0;
            }
            else
            {
                // Correzione
                UnscentedOutput Correction_out = UnscentedTransform_Correction(xi_pred, z_valid, P_pred, valid, var_used);
                int n_z = Correction_out.getX().size();
                Eigen::VectorXd e_k(n_z);
                e_k = Correction_out.getX();

                int row_s = Correction_out.getSigmaX().rows();
                int col_s = Correction_out.getSigmaX().cols();
                Eigen::MatrixXd S_k(row_s, col_s);
                S_k = Correction_out.getSigmaX();

                int row_pxz = Correction_out.getSigmaXY().rows();
                int col_pxz = Correction_out.getSigmaXY().cols();
                Eigen::MatrixXd P_xz(row_pxz, col_pxz);
                P_xz = Correction_out.getSigmaXY();

                // Calcolo il guadagno di Kalman
                Eigen::MatrixXd K(row_pxz, col_s);
                K = P_xz * S_k.inverse();

                // Calcolo la stima corretta

                Eigen::VectorXd xi_corr(xi_pred.size());

                xi_corr = xi_pred + K * e_k;

                xi_corr(3) = wrapToPi(xi_corr(3));
                xi_corr(4) = wrapToPi(xi_corr(4));
                xi_corr(5) = wrapToPi(xi_corr(5));

                // Calcolo la matrice di covarianza corretta
                Eigen::MatrixXd P_corr(row_pxz, row_pxz);
                P_corr = P_pred - K * S_k * K.transpose();

                xi_curr = xi_corr;
                P_curr = P_corr;
                mahalanobis_distance = sqrt(e_k.transpose() * S_k.inverse() * e_k);
            }

            ////////////////////////////////////////////////////////////////////
            /////////////////////////PREDICTION/////////////////////////////////
            ////////////////////////////////////////////////////////////////////

            // VETTORE DI FORZE E MOMENTI
            Eigen::VectorXd acc(3);
            acc << a_u, a_v, a_w;
            Eigen::VectorXd gyro(3);
            gyro << p_IMU_camera, q_IMU_camera, r_IMU_camera;

            // Predizione
            UnscentedOutput Prediction_out = UnscentedTransform_Prediction(xi_curr, acc, gyro, P_curr, Q, dt);

            xi_pred = Prediction_out.getX();

            P_pred = Prediction_out.getSigmaX();

            ///////////////////////////////////////////////////////////////////////
            ///////////////////////////// PUBLISHING //////////////////////////////
            ///////////////////////////////////////////////////////////////////////

            // Create a message object
            tesi_bluerov2::Floats msg;

            msg.data = {xi_curr(0), xi_curr(1), xi_curr(2), xi_curr(3), xi_curr(4), xi_curr(5), xi_curr(6), xi_curr(7), xi_curr(8), xi_curr(9), xi_curr(10), xi_curr(11), mahalanobis_distance};

            est_state_pub.publish(msg);
            if (ros::Time::now().toSec() > ros::TIME_MIN.toSec())
            {
                bag.write("state/est_state_UKF_no_dyn_imu_topic", ros::Time::now(), msg);
            }
        }

        valid_GPS = 0;
        valid_scanner = 0;
        valid_IMU = 0;
        valid_DVL = 0;
        valid_depth_sensor = 0;
        valid_IMU_camera = 0;
        // Let ROS handle all incoming messages in a callback function
        ros::spinOnce();

        // Sleep for the remaining time to hit our 10Hz target
        loop_rate.sleep();
    }
    bag.close();
    return 0;
}