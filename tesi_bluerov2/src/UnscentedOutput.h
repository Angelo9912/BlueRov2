#include <iostream>
#include <eigen3/Eigen/Dense>         // for eigen matrix
#include <math.h>
#include <vector>

class UnscentedOutput
{
    private:

    Eigen::VectorXd x;
    Eigen::MatrixXd SigmaX;
    Eigen::MatrixXd SigmaXY;

    public:

    UnscentedOutput(Eigen::VectorXd x, int n_x, Eigen::MatrixXd SigmaX, int row_sigma_x, int col_sigma_x, Eigen::MatrixXd SigmaXY)
    {   
        Eigen::VectorXd x(n_x);
        this->x = x;
        this->SigmaX = SigmaX;
        this->SigmaXY = SigmaXY;
    }

    Eigen::VectorXd getX()
    {
        return this->x;
    }

    Eigen::MatrixXd getSigmaX()
    {
        return this->SigmaX;
    }

    Eigen::MatrixXd getSigmaXY()
    {
        return this->SigmaXY;
    }
};