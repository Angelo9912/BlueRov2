#include <iostream>
#include <eigen3/Eigen/Dense> // for eigen matrix
#include <math.h>
#include <vector>
#include "UnscentedOutput.h"

UnscentedOutput UnscentedTransform_Prediction(Eigen::VectorXd xi_k, Eigen::MatrixXd Sigma)
{

    // Il vettore utilizzato effettivamente nella trasformata
    Eigen::VectorXd xi_prediction;
    xi_prediction << xi_k, 0, 0, 0, 0, 0, 0; // 6 = dim(Q) zeri

    int n = xi_prediction.size();
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

    for (int i = 0; i < n; i++)
    {
        sigma_points.col(i) = xi_prediction + sqrt(n + lambda) * GAMMA.col(i);
        sigma_points.col(i + n) = xi_prediction - sqrt(n + lambda) * GAMMA.col(i);
    }

    sigma_points.col(n_sigma - 1) = xi_prediction;


    // Compute the transformed sigma points

    for (int i = 0; i < n_sigma; i++)
    {
        Eigen::VectorXd eta = sigma_points.col(i).head(6);
        Eigen::VectorXd nu = sigma_points.col(i).tail(6);

        double phi = eta(3);
        double theta = eta(4);
        double psi = eta(5);

        double u = nu(0);
        double v = nu(1);
        double w = nu(2);
        double p = nu(3);
        double q = nu(4);
        double r = nu(5);

        
        Eigen::VectorXd nu_dot = M.inverse() * (tau - M * nu);

        sigma_points.col(i) << eta + eta_dot * dt, nu + nu_dot * dt;
    }



    // Compute the transformed mean
    Eigen::VectorXd x_out = sigma_points_out * weights[0];
    for (int i = 1; i < n_sigma; i++)
    {
        x_out += sigma_points_out.col(i) * weights[i];
    }

    // Compute the transformed covariance
    Eigen::MatrixXd SigmaX_out = weights[0] * (sigma_points_out.col(0) - x_out) * (sigma_points_out.col(0) - x_out).transpose();
    for (int i = 1; i < n_sigma; i++)
    {
        SigmaX_out += weights[i] * (sigma_points_out.col(i) - x_out) * (sigma_points_out.col(i) - x_out).transpose();
    }

    // Compute the transformed cross-covariance
    Eigen::MatrixXd SigmaXY_out = weights[0] * (sigma_points.col(0) - x) * (sigma_points_out.col(0) - x_out).transpose();
    for (int i = 1; i < n_sigma; i++)
    {
        SigmaXY_out += weights[i] * (sigma_points.col(i) - x) * (sigma_points_out.col(i) - x_out).transpose();
    }

    return UnscentedOutput(x_out, SigmaX_out, SigmaXY_out);
}
