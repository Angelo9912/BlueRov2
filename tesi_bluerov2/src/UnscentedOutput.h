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

    UnscentedOutput(Eigen::VectorXd x, Eigen::MatrixXd SigmaX, Eigen::MatrixXd SigmaXY)
    {   
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