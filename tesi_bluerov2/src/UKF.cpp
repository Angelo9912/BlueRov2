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

double tau_u = 0.0;
double tau_v = 0.0;
double tau_w = 0.0;
double tau_p = 0.0;
double tau_q = 0.0;
double tau_r = 0.0;

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
double p_IMU = 0.0;
double q_IMU = 0.0;
double r_IMU = 0.0;
double z_depth_sensor = 0.0;

double valid_GPS = 0;
double valid_scanner = 0;
double valid_IMU = 0;
double valid_DVL = 0;
double valid_depth_sensor = 0;

// Covariance values (process noise)
double var_tau_u = 0.0;
double var_tau_v = 0.0;
double var_tau_w = 0.0;
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
double var_p_IMU = 0.0;
double var_q_IMU = 0.0;
double var_r_IMU = 0.0;

// Parameters for the Unscented Kalman Filter

// Import parameters from YAML file
double m = 0.0;
double x_g = 0.0;
double y_g = 0.0;
double z_g = 0.0;
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

// Trasformata unscented (predizione)
UnscentedOutput UnscentedTransform_Prediction(Eigen::VectorXd xi_k, Eigen::VectorXd tau_k, Eigen::MatrixXd P_kk, Eigen::MatrixXd Q, double dt, Eigen::MatrixXd M)
{

    // Il vettore utilizzato effettivamente nella trasformata
    Eigen::VectorXd xi_prediction(xi_k.size() + tau_k.size());
    Eigen::VectorXd zeros(tau_k.size());
    zeros.setZero();
    xi_prediction << xi_k, zeros; // 6 = dim(Q) zeri

    int n = xi_prediction.size();
    int n_sigma = 2 * n + 1;
    int alpha = 1.0;
    double kappa = 0.0;
    double beta = 2.0;
    double lambda = alpha * alpha * (n + kappa) - n;

    Eigen::MatrixXd Zeros(xi_k.size(), tau_k.size());
    Zeros.setZero();

    Eigen::MatrixXd Sigma(n, n);
    Sigma << P_kk, Zeros,
        Zeros.transpose(), Q;

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
        sigma_points.col(i + n) = xi_prediction - sqrt(n + lambda) * U.col(i) * sqrt(Singular_values(i));
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

        Eigen::VectorXd tau_noise(6);
        tau_noise = xi_aug.tail(6);

        double phi = eta(3);
        double theta = eta(4);
        double psi = eta(5);

        double u = nu(0);
        double v = nu(1);
        double w = nu(2);
        double p = nu(3);
        double q = nu(4);
        double r = nu(5);

        // MATRICE DI CORIOLIS
        Eigen::Matrix<double, 6, 6> C_rb;

        C_rb << 0.0, 0.0, 0.0, m * (y_g * q + z_g * r), -m * (x_g * q - w), -m * (x_g * r + v),
            0.0, 0.0, 0.0, -m * (y_g * p + w), m * (z_g * r + x_g * p), -m * (y_g * r - u),
            0.0, 0.0, 0.0, -m * (z_g * p - v), -m * (z_g * q + u), m * (x_g * p + y_g * q),
            -m * (y_g * q + z_g * r), m * (y_g * p + w), m * (z_g * p - v), 0.0, I_z * r - I_yz * q - I_xz * p, I_xy * p + I_yz * r - I_y * q,
            m * (x_g * q - w), -m * (z_g * r + x_g * p), m * (z_g * q + u), I_yz * q + I_xz * p - I_z*r, 0.0, -I_xz * r - I_xy * q + I_x * p,
            m * (x_g * r + v), m * (y_g * r - u), -m * (x_g * p + y_g * q), -I_yz * r - I_xy * p + I_y * q, I_xy * q + I_xz * r - I_x * p, 0.0;

        Eigen::Matrix<double, 6, 6> C_a;

        double a1 = X_u_dot * u + X_v_dot * v + X_w_dot * w + X_p_dot * p + X_q_dot * q + X_r_dot * r;
        double a2 = X_v_dot * u + Y_v_dot * v + Y_w_dot * w + Y_p_dot * p + Y_q_dot * q + Y_r_dot * r;
        double a3 = X_w_dot * u + Y_w_dot * v + Z_w_dot * w + Z_p_dot * p + Z_q_dot * q + Z_r_dot * r;
        double b1 = X_p_dot * u + Y_p_dot * v + Z_p_dot * w + K_p_dot * p + K_q_dot * q + K_r_dot * r;
        double b2 = X_q_dot * u + Y_q_dot * v + Z_q_dot * w + K_p_dot * p + M_q_dot * q + M_r_dot * r;
        double b3 = X_r_dot * u + Y_r_dot * v + Z_r_dot * w + K_r_dot * p + M_r_dot * q + N_r_dot * r;

        C_a << 0.0, 0.0, 0.0, 0.0, -a3, a2,
            0.0, 0.0, 0.0, a3, 0.0, -a1,
            0.0, 0.0, 0.0, -a2, a1, 0.0,
            0.0, -a3, a2, 0.0, -b3, b2,
            a3, 0.0, -a1, b3, 0.0, -b1,
            -a2, a1, 0.0, -b2, b1, 0.0;

        Eigen::Matrix<double, 6, 6> C;
        C = C_rb + C_a;

        // MATRICE DI DAMPING
        Eigen::Matrix<double, 6, 6> D;

        D << A_x * abs(u), 0, 0, 0, 0, 0,
            0, A_y * abs(v), 0, 0, 0, 0,
            0, 0, A_z * abs(w), 0, 0, 0,
            0, 0, 0, A_p * abs(p), 0, 0,
            0, 0, 0, 0, A_q * abs(q), 0,
            0, 0, 0, 0, 0, A_r * abs(r);

        D = 0.5 * 1000 * D;

        // Dinamica del sistema

        Eigen::Matrix<double, 6, 1> nu_k1;
        nu_k1 = (dt * M.inverse() * (tau_k - C * nu - D * nu)) + nu;

        // VETTORE DELLE POSIZIONI
        Eigen::Matrix<double, 6, 6> Jacobian;
        Jacobian << cos(psi) * cos(theta), cos(psi) * sin(phi) * sin(theta) - cos(phi) * sin(psi), sin(phi) * sin(psi) + cos(phi) * cos(psi) * sin(theta), 0, 0, 0,
            cos(theta) * sin(psi), cos(phi) * cos(psi) + sin(phi) * sin(psi) * sin(theta), cos(phi) * sin(psi) * sin(theta) - cos(psi) * sin(phi), 0, 0, 0,
            -sin(theta), cos(theta) * sin(phi), cos(phi) * cos(theta), 0, 0, 0,
            0, 0, 0, 1, sin(phi) * tan(theta), cos(phi) * tan(theta),
            0, 0, 0, 0, cos(phi), -sin(phi),
            0, 0, 0, 0, sin(phi) / cos(theta), cos(phi) / cos(theta);

        Eigen::VectorXd eta_k1(6);
        eta_k1 = dt * (Jacobian * nu) + eta;

        // wrapToPi(eta_k1(phi));
        eta_k1(3) = atan2(sin(eta_k1(3)), cos(eta_k1(3)));

        // wrapToPi(eta_k1(theta));
        eta_k1(4) = atan2(sin(eta_k1(4)), cos(eta_k1(4)));

        // wrapToPi(eta_k1(psi));
        eta_k1(5) = atan2(sin(eta_k1(5)), cos(eta_k1(5)));

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

    for (int i = 0; i < n_sigma; i++)
    {
        SigmaX_out += weights[i] * (sigma_points_out.col(i) - xi_out) * (sigma_points_out.col(i) - xi_out).transpose();
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
        sigma_points.col(i + n) = xi_correction - sqrt(n + lambda) * U.col(i) * sqrt(Singular_values(i));
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
            if (e_k(i) > 0)
            {
                if (e_k(i) > 2 * M_PI - e_k(i))
                {
                    e_k(i) = -(2 * M_PI - e_k(i));
                }
                else
                {
                    e_k(i) = e_k(i);
                }
            }
            else
            {
                e_k(i) = -e_k(i);
                if (e_k(i) > 2 * M_PI - e_k(i))
                {
                    e_k(i) = 2 * M_PI - e_k(i);
                }
                else
                {
                    e_k(i) = -e_k(i);
                }
            }
        }
    }

    // Calcolo la matrice di covarianza pesata dei sigma points

    Eigen::MatrixXd Pzz_out(var_used.size(), var_used.size());
    Pzz_out.setZero();

    weights[n_sigma - 1] = weights[n_sigma - 1] + (1 - alpha * alpha + beta);

    for (int i = 0; i < n_sigma; i++)
    {
        Pzz_out += weights[i] * (sigma_points_out.col(i) - z_out) * (sigma_points_out.col(i) - z_out).transpose();
    }

    // Calcolo la matrice di covarianza incrociata

    Eigen::MatrixXd Pxz_out(n, var_used.size());
    Pxz_out.setZero();

    for (int i = 0; i < n_sigma; i++)
    {
        Pxz_out += weights[i] * (sigma_points.col(i) - xi_correction) * (sigma_points_out.col(i) - z_out).transpose();
    }

    return UnscentedOutput(e_k, Pzz_out + R, Pxz_out);
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
        p_IMU = 0.0;
        q_IMU = 0.0;
        r_IMU = 0.0;
        valid_IMU = 0;
    }
    else
    {
        phi_IMU = msg->data[0];
        theta_IMU = msg->data[1];
        psi_IMU = msg->data[2];
        p_IMU = msg->data[3];
        q_IMU = msg->data[4];
        r_IMU = msg->data[5];
        valid_IMU = msg->data[6];
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

void tau_callback(const tesi_bluerov2::Floats::ConstPtr &msg)
{
    if (msg->data[0] != msg->data[0])
    {
        tau_u = 0.0;
        tau_v = 0.0;
        tau_w = 0.0;
        tau_p = 0.0;
        tau_q = 0.0;
        tau_r = 0.0;
    }
    else
    {
        tau_u = msg->data[0];
        tau_v = msg->data[1];
        tau_w = msg->data[2];
        tau_p = msg->data[3];
        tau_q = msg->data[4];
        tau_r = msg->data[5];
    }
}

int main(int argc, char **argv)
{
    // Initialize the ROS system and become a node.
    ros::init(argc, argv, "UKF");

    // Create a ROS node handle
    ros::NodeHandle n;
    n.getParam("m", m);
    n.getParam("x_g", x_g);
    n.getParam("y_g", y_g);
    n.getParam("z_g", z_g);
    n.getParam("I_x", I_x);
    n.getParam("I_y", I_y);
    n.getParam("I_z", I_z);
    n.getParam("I_xy", I_xy);
    n.getParam("I_xz", I_xz);
    n.getParam("I_yz", I_yz);
    n.getParam("I_yx", I_yx);
    n.getParam("I_zx", I_zx);
    n.getParam("I_zy", I_zy);
    n.getParam("A_x", A_x);
    n.getParam("A_y", A_y);
    n.getParam("A_z", A_z);
    n.getParam("A_p", A_p);
    n.getParam("A_q", A_q);
    n.getParam("A_r", A_r);
    n.getParam("X_u_dot", X_u_dot);
    n.getParam("X_v_dot", X_v_dot);
    n.getParam("X_w_dot", X_w_dot);
    n.getParam("X_p_dot", X_p_dot);
    n.getParam("X_q_dot", X_q_dot);
    n.getParam("X_r_dot", X_r_dot);
    n.getParam("Y_u_dot", Y_u_dot);
    n.getParam("Y_v_dot", Y_v_dot);
    n.getParam("Y_w_dot", Y_w_dot);
    n.getParam("Y_p_dot", Y_p_dot);
    n.getParam("Y_q_dot", Y_q_dot);
    n.getParam("Y_r_dot", Y_r_dot);
    n.getParam("Z_u_dot", Z_u_dot);
    n.getParam("Z_v_dot", Z_v_dot);
    n.getParam("Z_w_dot", Z_w_dot);
    n.getParam("Z_p_dot", Z_p_dot);
    n.getParam("Z_q_dot", Z_q_dot);
    n.getParam("Z_r_dot", Z_r_dot);
    n.getParam("K_u_dot", K_u_dot);
    n.getParam("K_v_dot", K_v_dot);
    n.getParam("K_w_dot", K_w_dot);
    n.getParam("K_p_dot", K_p_dot);
    n.getParam("K_q_dot", K_q_dot);
    n.getParam("K_r_dot", K_r_dot);
    n.getParam("M_u_dot", M_u_dot);
    n.getParam("M_v_dot", M_v_dot);
    n.getParam("M_w_dot", M_w_dot);
    n.getParam("M_p_dot", M_p_dot);
    n.getParam("M_q_dot", M_q_dot);
    n.getParam("M_r_dot", M_r_dot);
    n.getParam("N_u_dot", N_u_dot);
    n.getParam("N_v_dot", N_v_dot);
    n.getParam("N_w_dot", N_w_dot);
    n.getParam("N_p_dot", N_p_dot);
    n.getParam("N_q_dot", N_q_dot);
    n.getParam("N_r_dot", N_r_dot);
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
    n.getParam("var_p_IMU", var_p_IMU);
    n.getParam("var_q_IMU", var_q_IMU);
    n.getParam("var_r_IMU", var_r_IMU);

    n.getParam("var_tau_u", var_tau_u);
    n.getParam("var_tau_v", var_tau_v);
    n.getParam("var_tau_w", var_tau_w);
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

    // MATRICE DI MASSA

    Eigen::Matrix<double, 6, 6> M_rb;
    M_rb << m, 0.0, 0.0, 0.0, m * z_g, -m * y_g,
        0.0, m, 0.0, -m * z_g, 0.0, m * x_g,
        0.0, 0.0, m, m * y_g, -m * x_g, 0.0,
        0.0, -m * z_g, m * y_g, I_x, -I_xy, -I_xz,
        m * z_g, 0.0, -m * x_g, -I_xy, I_y, -I_yz,
        -m * y_g, m * x_g, 0.0, -I_xz, -I_yz, I_z;

    Eigen::Matrix<double, 6, 6> M_a;
    M_a << X_u_dot, X_v_dot, X_w_dot, X_p_dot, X_q_dot, X_r_dot,
        Y_u_dot, Y_v_dot, Y_w_dot, Y_p_dot, Y_q_dot, Y_r_dot,
        Z_u_dot, Z_v_dot, Z_w_dot, Z_p_dot, Z_q_dot, Z_r_dot,
        K_u_dot, K_v_dot, K_w_dot, K_p_dot, K_q_dot, K_r_dot,
        M_u_dot, M_v_dot, M_w_dot, M_p_dot, M_q_dot, M_r_dot,
        N_u_dot, N_v_dot, N_w_dot, N_p_dot, N_q_dot, N_r_dot;

    Eigen::Matrix<double, 6, 6> M;
    M = M_rb + M_a;

    // Time step
    double freq = 50;
    double dt = 1 / freq;

    bool is_init = false;

    bool is_GPS_init = false;
    bool is_scanner_init = false;
    bool is_depth_init = false;
    bool is_IMU_init = false;

    Eigen::VectorXd var_sensors(14);
    var_sensors << var_x_GPS, var_y_GPS, var_x_scanner, var_y_scanner, var_z_depth_sensor, var_phi_IMU, var_theta_IMU, var_psi_IMU, var_u_DVL, var_v_DVL, var_w_DVL, var_p_IMU, var_q_IMU, var_r_IMU;

    Eigen::MatrixXd Q(6, 6);
    Q << var_tau_u, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, var_tau_v, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, var_tau_w, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, var_tau_p, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, var_tau_q, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, var_tau_r;

    // Set the loop rate
    ros::Rate loop_rate(freq);

    // Create a publisher object
    ros::Publisher est_state_pub = n.advertise<tesi_bluerov2::Floats>("est_state_UKF_topic", 1000);

    // Create subscriber objects

    ros::Subscriber tau_sub = n.subscribe("tau_topic", 1000, tau_callback);

    ros::Subscriber GPS_sub = n.subscribe("GPS_topic", 1000, GPSCallback);

    ros::Subscriber scanner_sub = n.subscribe("scanner_topic", 1000, ScannerCallback);

    ros::Subscriber IMU_sub = n.subscribe("IMU_topic", 1000, IMUCallback);

    ros::Subscriber DVL_sub = n.subscribe("DVL_topic", 1000, DVLCallback);

    ros::Subscriber depth_sub = n.subscribe("depth_sensor_topic", 1000, depthSensorCallback);

    // Create a message object
    tesi_bluerov2::Floats msg;

    while (ros::ok())
    {

        //////Kalman Filter///////

        ///////////////////////////////////////////////////////////////////////
        ///////////////// INITIALIZATION OF THE FILTER ////////////////////////
        ///////////////////////////////////////////////////////////////////////

        if (!is_init)
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

            xi_curr(6) = 0.0 + gaussianNoise(0, var_u_DVL);
            xi_curr(7) = 0.0 + gaussianNoise(0, var_v_DVL);
            xi_curr(8) = 0.0 + gaussianNoise(0, var_w_DVL);
            xi_curr(9) = 0.0 + gaussianNoise(0, var_p_IMU);
            xi_curr(10) = 0.0 + gaussianNoise(0, var_q_IMU);
            xi_curr(11) = 0.0 + gaussianNoise(0, var_r_IMU);

            P_curr << var_x, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                0, var_y, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                0, 0, var_z_depth_sensor, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                0, 0, 0, var_phi_IMU, 0, 0, 0, 0, 0, 0, 0, 0,
                0, 0, 0, 0, var_theta_IMU, 0, 0, 0, 0, 0, 0, 0,
                0, 0, 0, 0, 0, var_psi_IMU, 0, 0, 0, 0, 0, 0,
                0, 0, 0, 0, 0, 0, var_u_DVL, 0, 0, 0, 0, 0,
                0, 0, 0, 0, 0, 0, 0, var_v_DVL, 0, 0, 0, 0,
                0, 0, 0, 0, 0, 0, 0, 0, var_w_DVL, 0, 0, 0,
                0, 0, 0, 0, 0, 0, 0, 0, 0, var_p_IMU, 0, 0,
                0, 0, 0, 0, 0, 0, 0, 0, 0, 0, var_q_IMU, 0,
                0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, var_r_IMU;

            is_init = ((is_GPS_init || is_scanner_init) && is_depth_init && is_IMU_init);

            if (is_init) // Se l'inizializzazione è completata pubblica lo stato stimato iniziale e passa alla predizione
            {
                ROS_WARN_STREAM("INITIALIZATION COMPLETED\n");

                msg.data = {xi_curr(0), xi_curr(1), xi_curr(2), xi_curr(3), xi_curr(4), xi_curr(5), xi_curr(6), xi_curr(7), xi_curr(8), xi_curr(9), xi_curr(10), xi_curr(11),mahalanobis_distance};
                est_state_pub.publish(msg);

                ////////////////////////////////////////////////////////////////////
                /////////////////////////PREDICTION/////////////////////////////////
                ////////////////////////////////////////////////////////////////////

                // VETTORE DI FORZE E MOMENTI
                Eigen::VectorXd tau(6);
                tau << tau_u, tau_v, tau_w, tau_p, tau_q, tau_r;
                // Predizione
                UnscentedOutput Prediction_out = UnscentedTransform_Prediction(xi_curr, tau, P_curr, Q, dt, M);

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
            Eigen::VectorXd z(14);
            z << x_GPS, y_GPS, x_scanner, y_scanner, z_depth_sensor, phi_IMU, theta_IMU, psi_IMU, u_DVL, v_DVL, w_DVL, p_IMU, q_IMU, r_IMU;

            Eigen::VectorXd valid(14);
            valid << valid_GPS, valid_GPS, valid_scanner, valid_scanner, valid_depth_sensor, valid_IMU, valid_IMU, valid_IMU, valid_DVL, valid_DVL, valid_DVL, valid_IMU, valid_IMU, valid_IMU;

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

            if(j == z_valid.size())
            {
                ROS_WARN_STREAM("GIUSTO");
            }
            else
            {
                ROS_WARN_STREAM("SBAGLIATO");
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

                // ROS_WARN_STREAM("e_k UKF: \n"
                //                 << e_k.transpose());

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
                // K = P_xz * S_k.completeOrthogonalDecomposition().pseudoInverse();
                K = P_xz * S_k.inverse();

                // Calcolo la stima corretta

                Eigen::VectorXd xi_corr(xi_pred.size());

                xi_corr = xi_pred + K * e_k;

                // Calcolo la matrice di covarianza corretta
                Eigen::MatrixXd P_corr(row_pxz, row_pxz);
                P_corr = P_pred - K * S_k * K.transpose();

                // ROS_WARN_STREAM("eigs: \n"
                //                 << P_corr.eigenvalues().real().minCoeff() << " , " << P_corr.eigenvalues().real().maxCoeff());

                xi_curr = xi_corr;
                P_curr = P_corr;
                mahalanobis_distance = sqrt(e_k.transpose() * S_k.inverse() * e_k);
            }

            ////////////////////////////////////////////////////////////////////
            /////////////////////////PREDICTION/////////////////////////////////
            ////////////////////////////////////////////////////////////////////

            // VETTORE DI FORZE E MOMENTI
            Eigen::VectorXd tau(6);
            tau << tau_u, tau_v, tau_w, tau_p, tau_q, tau_r;

            // Predizione
            UnscentedOutput Prediction_out = UnscentedTransform_Prediction(xi_curr, tau, P_curr, Q, dt, M);

            xi_pred = Prediction_out.getX();

            P_pred = Prediction_out.getSigmaX();

            ///////////////////////////////////////////////////////////////////////
            ///////////////////////////// PUBLISHING //////////////////////////////
            ///////////////////////////////////////////////////////////////////////

            // Create a message object
            tesi_bluerov2::Floats msg;

            msg.data = {xi_curr(0), xi_curr(1), xi_curr(2), xi_curr(3), xi_curr(4), xi_curr(5), xi_curr(6), xi_curr(7), xi_curr(8), xi_curr(9), xi_curr(10), xi_curr(11), mahalanobis_distance};

            est_state_pub.publish(msg);
        }

        valid_GPS = 0;
        valid_scanner = 0;
        valid_IMU = 0;
        valid_DVL = 0;
        valid_depth_sensor = 0;

        // Let ROS handle all incoming messages in a callback function
        ros::spinOnce();

        // Sleep for the remaining time to hit our 10Hz target
        loop_rate.sleep();
    }

    return 0;
}