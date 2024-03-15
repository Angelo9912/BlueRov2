#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include <ros/console.h>
#include <eigen3/Eigen/Dense>         // for eigen matrix
#include "progetto_robotica/Floats.h" // for accessing -- progetto_robotica Floats()
#include <vector>
#include "yaml-cpp/yaml.h" // for yaml

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

int valid_GPS = 0;
int valid_scanner = 0;
int valid_IMU = 0;
int valid_DVL = 0;
int valid_depth_sensor = 0;

// Current corrected state vector (12x1)
Eigen::VectorXd xi_curr(12);

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

Eigen::VectorXd var_sensors(14);
var_sensors << var_x_GPS, var_y_GPS, var_x_scanner, var_y_scanner, var_z_depth_sensor, var_phi_IMU, var_theta_IMU, var_psi_IMU, var_u_DVL, var_v_DVL, var_w_DVL, var_p_IMU, var_q_IMU, var_r_IMU;

// Covariance values (process noise)
double var_tau_u = 0.0;
double var_tau_v = 0.0;
double var_tau_w = 0.0;
double var_tau_p = 0.0;
double var_tau_q = 0.0;
double var_tau_r = 0.0;

Eigen::MatrixXd Q(6, 6);
Q << var_tau_u, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, var_tau_v, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, var_tau_w, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, var_tau_p, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, var_tau_q, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, var_tau_r;

// Parameters for the Unscented Kalman Filter

// Import parameters from YAML file
double m = 0.0;
double d = 0.0;
double l = 0.0;
double X_u_dot = 0.0;
double Y_v_dot = 0.0;
double X_u = 0.0;
double Y_r = 0.0;
double Y_v = 0.0;
double N_r = 0.0;
double N_r_r = 0.0;
double Z_w = 0.0;
double N_v = 0.0;
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

n.getParam("m", m);
n.getParam("d", d);
n.getParam("l", l);
// I = 1.0 / 12.0 * m * (pow(d, 2) + pow(l, 2));
n.getParam("X_u_dot", X_u_dot);
n.getParam("Y_v_dot", Y_v_dot);
n.getParam("X_u", X_u);
n.getParam("Y_r", Y_r);
n.getParam("Y_v", Y_v);
n.getParam("N_r", N_r);
n.getParam("N_r_r", N_r_r);
n.getParam("Z_w", Z_w);
n.getParam("N_v", N_v);
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

// MATRICE DI MASSA

Eigen::Matrix<double, 6, 6> M_rb;
M_rb << m, 0.0, 0.0, 0.0, m *z_g, -m *y_g,
    0.0, m, 0.0, -m *z_g, 0.0, m *x_g,
    0.0, 0.0, m, m *y_g, -m *x_g, 0.0,
    0.0, -m *z_g, m *y_g, I_x, -I_xy, -I_xz,
    m *z_g, 0.0, -m *x_g, -I_xy, I_y, -I_yz,
    -m *y_g, m *x_g, 0.0, -I_xz, -I_yz, I_z;

Eigen::Matrix<double, 6, 6> M_a;
M_a << X_u_dot, X_v_dot, X_w_dot, X_p_dot, X_q_dot, X_r_dot,
    Y_u_dot, Y_v_dot, Y_w_dot, Y_p_dot, Y_q_dot, Y_r_dot,
    Z_u_dot, Z_v_dot, Z_w_dot, Z_p_dot, Z_q_dot, Z_r_dot,
    K_u_dot, K_v_dot, K_w_dot, K_p_dot, K_q_dot, K_r_dot,
    M_u_dot, M_v_dot, M_w_dot, M_p_dot, M_q_dot, M_r_dot,
    N_u_dot, N_v_dot, N_w_dot, N_p_dot, N_q_dot, N_r_dot;

Eigen::Matrix<double, 6, 6> M;
M = M_rb + M_a;

double dt = 0.01;
double freq = 100;

// MATRICE DI MASSA

Eigen::Matrix<double, 6, 6> M_rb;
M_rb << m, 0.0, 0.0, 0.0, m *z_g, -m *y_g,
    0.0, m, 0.0, -m *z_g, 0.0, m *x_g,
    0.0, 0.0, m, m *y_g, -m *x_g, 0.0,
    0.0, -m *z_g, m *y_g, I_x, -I_xy, -I_xz,
    m *z_g, 0.0, -m *x_g, -I_xy, I_y, -I_yz,
    -m *y_g, m *x_g, 0.0, -I_xz, -I_yz, I_z;

Eigen::Matrix<double, 6, 6> M_a;
M_a << X_u_dot, X_v_dot, X_w_dot, X_p_dot, X_q_dot, X_r_dot,
    Y_u_dot, Y_v_dot, Y_w_dot, Y_p_dot, Y_q_dot, Y_r_dot,
    Z_u_dot, Z_v_dot, Z_w_dot, Z_p_dot, Z_q_dot, Z_r_dot,
    K_u_dot, K_v_dot, K_w_dot, K_p_dot, K_q_dot, K_r_dot,
    M_u_dot, M_v_dot, M_w_dot, M_p_dot, M_q_dot, M_r_dot,
    N_u_dot, N_v_dot, N_w_dot, N_p_dot, N_q_dot, N_r_dot;

Eigen::Matrix<double, 6, 6> M;
M = M_rb + M_a;


// Matrice MMSE
Eigen::MatrixXd P_curr(12, 12);

P_curr << 




// Trasformata unscented (predizione)
UnscentedOutput UnscentedTransform_Prediction(Eigen::VectorXd xi_k, Eigen::VectorXd tau_k, Eigen::MatrixXd P_kk)
{

    // Il vettore utilizzato effettivamente nella trasformata
    Eigen::VectorXd xi_prediction;
    Eigen::VectorXd zeros(tau_k.size()).setZero();
    xi_prediction << xi_k, zeros; // 6 = dim(Q) zeri

    int n = xi_prediction.size();
    int n_sigma = 2 * n + 1;
    int alpha = 1;
    double kappa = 0.0;
    double beta = 2.0;
    double lambda = alpha * alpha * (n + kappa) - n;

    // Cholesky decomposition
    Eigen::MatrixXd GAMMA(n, n); // compute the Cholesky decomposition of Sigma

    Eigen::MatrixXd Zeros(xi_prediction.size(), tau_k.size()).setZero();

    Eigen::MatrixXd Sigma(n, n);
    Sigma << P_kk, Zeros,
        Zeros, Q;

    GAMMA = Sigma.llt().matrixL();

    // Create sigma points
    Eigen::MatrixXd sigma_points(n, n_sigma);

    double weights[n_sigma];

    for (int i = 0; i < n; i++)
    {
        sigma_points.col(i) = xi_prediction + sqrt(n + lambda) * GAMMA.col(i);
        sigma_points.col(i + n) = xi_prediction - sqrt(n + lambda) * GAMMA.col(i);
        weights[i] = 1 / (2 * (n + lambda));
    }

    sigma_points.col(n_sigma - 1) = xi_prediction;
    weights[n_sigma - 1] = lambda / (n + lambda);

    Eigen::MatrixXd sigma_points_out(12, n_sigma);

    // Compute the transformed sigma points

    for (int i = 0; i < n_sigma; i++)
    {
        Eigen::VectorXd eta = sigma_points.col(i).head(6);
        Eigen::VectorXd nu = sigma_points.col(i).subVector(6, 11);

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
            -m * (y_g * q + z_g * r), m * (y_g * p + w), m * (z_g * p - v), 0.0, I_z * r - I_yz * q - I_xz * p, I_xy * p - I_yz * r - I_y * q,
            m * (x_g * q - w), -m * (z_g * r + x_g * p), m * (z_g * q + u), I_yz * q + I_xz * p - I_z, 0.0, -I_xz * r - I_xy * q + I_x * p,
            m * (x_g * r + v), -m * (y_g * r - u), -m * (x_g * p + y_g * q), -I_yz * r - I_xy * p + I_y * q, I_xy * q + I_xz * r - I_x * p, 0.0;

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

        D << A_x * abs(u) * u, A_x * abs(v) * v, A_x * abs(w) * w, A_x * abs(p) * p, A_x * abs(q) * q, A_x * abs(r) * r;
        A_y *abs(u) * u, A_y *abs(v) * v, A_y *abs(w) * w, A_y *abs(p) * p, A_y *abs(q) * q, A_y *abs(r) * r;
        A_z *abs(u) * u, A_z *abs(v) * v, A_z *abs(w) * w, A_z *abs(p) * p, A_z *abs(q) * q, A_z *abs(r) * r;
        A_p *abs(u) * u, A_p *abs(v) * v, A_p *abs(w) * w, A_p *abs(p) * p, A_p *abs(q) * q, A_p *abs(r) * r;
        A_q *abs(u) * u, A_q *abs(v) * v, A_q *abs(w) * w, A_q *abs(p) * p, A_q *abs(q) * q, A_q *abs(r) * r;
        A_r *abs(u) * u, A_r *abs(v) * v, A_r *abs(w) * w, A_r *abs(p) * p, A_r *abs(q) * q, A_r *abs(r) * r;

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

        Eigen::Matrix<double, 6, 6> eta_k1;
        eta_k1 = dt * (Jacobian * nu) + eta;

        double rem;

        // wrapToPi(eta_k1(phi));
        rem = std::fmod(eta_k1(3) + M_PI, 2 * M_PI);
        if (rem < 0)
        {
            rem += 2 * M_PI;
        }
        eta_k1(3) = rem - M_PI;

        // wrapToPi(eta_k1(theta));
        rem = std::fmod(eta_k1(4) + M_PI, 2 * M_PI);
        if (rem < 0)
        {
            rem += 2 * M_PI;
        }
        eta_k1(4) = rem - M_PI;

        // wrapToPi(eta_k1(psi));
        rem = std::fmod(eta_k1(5) + M_PI, 2 * M_PI);
        if (rem < 0)
        {
            rem += 2 * M_PI;
        }
        eta_k1(5) = rem - M_PI;

        sigma_points_out.col(i) << eta_k1, nu_k1;
    }

    // Compute the transformed mean

    Eigen::VectorXd xi_out(xi_prediction.size()).setZero();

    for (int i = 1; i < n_sigma; i++)
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
        x_phi += cos(sigma_point_out(3, i)) * weights[i];
        x_theta += cos(sigma_point_out(4, i)) * weights[i];
        x_psi += cos(sigma_point_out(5, i)) * weights[i];
        y_phi += sin(sigma_point_out(3, i)) * weights[i];
        y_theta += sin(sigma_point_out(4, i)) * weights[i];
        y_psi += sin(sigma_point_out(5, i)) * weights[i];
    }

    xi_out(3) = atan2(y_phi, x_phi);
    xi_out(4) = atan2(y_theta, x_theta);
    xi_out(5) = atan2(y_psi, x_psi);

    // Per la costruzione dei momenti di ordine secondo vegono si usa un peso diverso solo per l'elemento w0
    weights[n_sigma - 1] = weights[n_sigma - 1] + (1 - alpha * alpha + beta);

    // Compute the transformed covariance
    Eigen::MatrixXd SigmaX_out(xi_prediction.size(), xi_prediction.size()).setZero();
    for (int i = 1; i < n_sigma; i++)
    {
        SigmaX_out += weights[i] * (sigma_point_out.col(i) - xi_out) * (sigma_point_out.col(i) - xi_out).transpose();
    }

    // Initialize the cross-covariance matrix
    Eigen::MatrixXd SigmaXY_out(xi_k.size(), xi_k.size()).setZero();

    return UnscentedOutput(xi_out, SigmaX_out, SigmaXY_out);
}

UnscentedOutput UnscentedTransform_Correction(Eigen::VectorXd xi_k, Eigen::MatrixXd P_kk, Eigen::VectorXd valid)
{

    // Definiamo i dati secondo le misure che abbiamo

    Eigen::VectorXd xi_correction;

    Eigen::VectorXd var_used;

    for (int i = 0; i < 14; i++)
    {
        if (valid(i) == 1)
        {
            if(var_used.size() < 1)
            {
                var_used << var_sensors(i);
            }
            else{
            var_used.conservativeResize(var_used.size() + 1);
            var_used(var_used.size() - 1) = var_sensors(i);
            }
        }
    }

    Eigen::MatrixXd R(var_used.size(), var_used.size());
    R.diagonal() = var_used;

    Eigen::VectorXd zeros(var_used.size()).setZero();
    xi_correction << xi_k, zeros;

    int n = xi_correction.size();

    Eigen::MatrixXd Zeros(xi_k.size(), var_used.size()).setZero();

    Eigen::MatrixXd Sigma(n, n);
    Sigma << P_kk, Zeros,
        Zeros, R;

    // Cholesky decomposition
    int n_sigma = 2 * n + 1;
    int alpha = 1;
    double kappa = 0.0;
    double beta = 2.0;
    double lambda = alpha * alpha * (n + kappa) - n;

    // Cholesky decomposition
    Eigen::MatrixXd GAMMA(n, n); // compute the Cholesky decomposition of Sigma

    GAMMA = Sigma.llt().matrixL();

    // Create sigma points
    Eigen::MatrixXd sigma_points(n, n_sigma);

    double weights[n_sigma];

    for (int i = 0; i < n; i++)
    {
        sigma_points.col(i) = xi_correction + sqrt(n + lambda) * GAMMA.col(i);
        sigma_points.col(i + n) = xi_correction - sqrt(n + lambda) * GAMMA.col(i);
        weights[i] = 1 / (2 * (n + lambda));
    }

    sigma_points.col(n_sigma - 1) = xi_correction;
    weights[n_sigma - 1] = lambda / (n + lambda);

    // Pesco solo i valori dello stato predetto associati a misure valide
    // in modo tale da poter calcolare l'innovazione in maniera corretta

    Eigen::VectorXd sigma_points_out(var_used.size(),n_sigma);

    for (int i = 0; i < n_sigma; i++)
    {
        int j = 0;

        for (int k = 0; k < valid.size(); k++)
        {
            if (valid(k) == 1)    // Controllo la validità della misura ricevuta
            {
                if(k < 2)  // Se è una misura di posizione del GPS k=j
                {
                    sigma_points_out(j,i) = sigma_points(k,i);
                }
                else    // Altrimenti k = j + 2
                {
                    sigma_points_out(j,i) = sigma_points(k-2,i);

                }
                j++;
            }
        }
    }


    // Calcoliamo la misura data dai valori predetti come media pesata dei sigma points

    Eigen::VectorXd z_out(var_used.size()).setZero();

    for (int i = 1; i < n_sigma; i++)
    {
        z_out += sigma_points_out.col(i) * weights[i];
    }

    // Calcolo la media pesata dei sigma points per le variabili angolari

    if(valid(5) == 1){
        
        int id_phi = valid.head(5).sum();

        double x_phi = 0.0;
        double x_theta = 0.0;
        double x_psi = 0.0;
        double y_phi = 0.0;
        double y_theta = 0.0;
        double y_psi = 0.0;

        for(int i = 0; i < n_sigma; i++)
        {
            x_phi += cos(sigma_points(3,i)) * weights[i];
            x_theta += cos(sigma_points(4,i)) * weights[i];
            x_psi += cos(sigma_points(5,i)) * weights[i];
            y_phi += sin(sigma_points(3,i)) * weights[i];
            y_theta += sin(sigma_points(4,i)) * weights[i];
            y_psi += sin(sigma_points(5,i)) * weights[i];
        }

        z_out(id_phi) = atan2(y_phi,x_phi);
        z_out(id_phi + 1) = atan2(y_theta,x_theta);
        z_out(id_phi + 2) = atan2(y_psi,x_psi);
    }

    // Calcolo la matrice di covarianza pesata dei sigma points

    Eigen::MatrixXd Pzz_out(var_used.size(),var_used.size()).setZero();
    weights[n_sigma - 1] = weights[n_sigma - 1] + (1 - alpha * alpha + beta);

    for (int i = 1; i < n_sigma; i++)
    {
        Pzz_out += weights[i] * (sigma_points_out.col(i) - z_out) * (sigma_points_out.col(i) - z_out).transpose();
    }

    // Calcolo la matrice di covarianza incrociata

    Eigen::MatrixXd Pxz_out(xi_k.size(),var_used.size()).setZero();

    for (int i = 1; i < n_sigma; i++)
    {
        Pxz_out += weights[i] * (sigma_points.col(i) - xi_correction) * (sigma_points_out.col(i) - z_out).transpose();
    }

    return UnscentedOutput(z_out,Pzz_out,Pxz_out);
}

// Callback function for the subscriber
void GPSCallback(const progetto_robotica::Floats::ConstPtr &msg)
{
    if (msg->data[0] != msg->data[0] || msg->valid == 0)
    {
        x_GPS = 0.0;
        y_GPS = 0.0;
        valid_GPS = 0;
    }
    else
    {
        x_GPS = msg->data[0];
        y_GPS = msg->data[1];
        valid_GPS = 1;
    }
}

void ScannerCallback(const progetto_robotica::Floats::ConstPtr &msg)
{
    if (msg->data[0] != msg->data[0] || msg->valid == 0)
    {
        x_scanner = 0.0;
        y_scanner = 0.0;
        valid_scanner = 0;
    }
    else
    {
        x_scanner = msg->data[0];
        y_scanner = msg->data[1];
        valid_scanner = 1;
    }
}

void IMUCallback(const progetto_robotica::Floats::ConstPtr &msg)
{
    if (msg->data[0] != msg->data[0] || msg->valid == 0)
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
        valid_IMU = 1;
    }
}

void DVLCallback(const progetto_robotica::Floats::ConstPtr &msg)
{
    if (msg->data[0] != msg->data[0] || msg->valid == 0)
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
        valid_DVL = 1;
    }
}

void depthSensorCallback(const progetto_robotica::Floats::ConstPtr &msg)
{
    if (msg->data[0] != msg->data[0] || msg->valid == 0)
    {
        z_depth_sensor = 0.0;
        valid_depth_sensor = 0;
    }
    else
    {
        z_depth_sensor = msg->data[0];
        valid_depth_sensor = 1;
    }
}

void tau_callback(const progetto_robotica::Floats::ConstPtr &msg)
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
        tau_r = msg->data[4];
    }
}

int main(int argc, char **argv)
{
    // Initialize the ROS system and become a node.
    ros::init(argc, argv, "navigation_flt");

    // Create a ROS node handle
    ros::NodeHandle n;

    double freq = 1000;
    double dt = 1 / freq;

    // Set the loop rate
    ros::Rate loop_rate(freq);

    // Create a publisher object
    ros::Publisher est_state_pub = n.advertise<progetto_robotica::Floats>("est_state_topic", 1000);

    // Create subscriber objects

    ros::Subscriber tau_sub = n.subscribe("tau_topic", 1000, tau_callback);

    ros::Subscriber GPS_sub = n.subscribe("GPS_topic", 1000, GPSCallback);

    ros::Subscriber scanner_sub = n.subscribe("scanner_topic", 1000, ScannerCallback);

    ros::Subscriber IMU_sub = n.subscribe("IMU_topic", 1000, IMUCallback);

    ros::Subscriber DVL_sub = n.subscribe("DVL_topic", 1000, DVLCallback);

    ros::Subscriber depth_sub = n.subscribe("depth_sensor_topic", 1000, depthSensorCallback);

    while (ros::ok())
    {

        //////Kalman Filter///////

        ///////////////////////////////////////////////////////////////////////
        ///////////////////////////// PREDICTION //////////////////////////////
        ///////////////////////////////////////////////////////////////////////

        // VETTORE DI FORZE E MOMENTI
        Eigen::Matrix<double, 6, 1> tau;
        tau << tau_u, tau_v, tau_w, tau_p, tau_q, tau_r;

        // VETTORE DI STATO
        Eigen::Matrix<double, 6, 1> xi_curr;
        xi_curr << curr_x, curr_y, curr_z, curr_phi, curr_theta, curr_psi;

        // Predizione
        UnscentedOutput Prediction_out = UnscentedTransform_Prediction(xi_curr, tau, P_curr);

        Eigen::Matrix<double, 6, 1> xi_pred = Prediction_out.getX();
        Eigen::Matrix<double, 6, 6> P_pred = Prediction_out.getSigmaX();

        ///////////////////////////////////////////////////////////////////////
        ///////////////////////////// CORRECTION //////////////////////////////
        ///////////////////////////////////////////////////////////////////////

        // VETTORE DI MISURA
        Eigen::VectorXd z;
        z << x_GPS, y_GPS, x_scanner, y_scanner, z_depth_sensor, phi_IMU, theta_IMU, psi_IMU, u_DVL, v_DVL, w_DVL, p_IMU, q_IMU, r_IMU;

        Eigen::VectorXi valid;
        valid << valid_GPS, valid_GPS, valid_scanner, valid_scanner, valid_depth_sensor, valid_IMU, valid_IMU, valid_IMU, valid_DVL, valid_DVL, valid_DVL, valid_IMU, valid_IMU, valid_IMU;

        // Pesco le misure valide
        Eigen::VectorXd z_valid;
        for(int i = 0; i < valid.size(); i++)
        {
            if(valid(i) == 1)
            {
                if(z_valid.size() < 1)
                {
                    z_valid << z(i);
                }
                else
                {
                    z_valid.conservativeResize(z_valid.size() + 1);
                    z_valid(z_valid.size() - 1) = z(i);
                }
            }
        }
        
        
        // Correzione
        UnscentedOutput Correction_out = UnscentedTransform_Correction(xi_pred, P_pred, valid);

        Eigen::VectorXd z_esteem = Correction_out.getX();
        Eigen::MatrixXd S_k = Correction_out.getSigmaX();
        Eigen::MatrixXd Pxz = Correction_out.getSigmaXY().block(0, 0, 12, 12);

        // Calcolo il guadagno di Kalman
        Eigen::MatrixXd K = Pxz * S_k.inverse();

        // Calcolo la stima corretta
        Eigen::VectorXd xi_corr = xi_pred + K * (z_valid - z_esteem);

        // Calcolo la matrice di covarianza corretta
        Eigen::MatrixXd P_corr = P_pred - K * S_k * K.transpose();

        xi_curr = xi_corr;
        P_curr = P_corr;

        ///////////////////////////////////////////////////////////////////////
        ///////////////////////////// PUBLISHING //////////////////////////////
        ///////////////////////////////////////////////////////////////////////

        // Create a message object
        progetto_robotica::Floats msg;

        // Set the message data
        msg.data = [xi_curr(0), xi_curr(1), xi_curr(2), xi_curr(3), xi_curr(4), xi_curr(5), xi_curr(6), xi_curr(7), xi_curr(8), xi_curr(9), xi_curr(10), xi_curr(11)]

        // Publish the message
        est_state_pub.publish(msg);

        int valid_GPS = 0;
        int valid_scanner = 0;
        int valid_IMU = 0;
        int valid_DVL = 0;
        int valid_depth_sensor = 0;

        // Let ROS handle all incoming messages in a callback function
        ros::spinOnce();

        // Sleep for the remaining time to hit our 10Hz target
        loop_rate.sleep();
    }

    return 0;
}