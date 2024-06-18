#include <PostureKF/LKalmanFilter.hpp>
#include <control_frame/BHR_ControlFrame.h>
#include <math.h>
_BHR_CF_BEGIN

lee::LKalmanFilter<2,1,1> FootForceEstimator[2];

void Frame::initFootForceEstimator()
{
    double Ts = this->Ts;
    Eigen::Matrix<double,2,1> ProcessNoise{0.001,0.005};
    Eigen::Matrix<double,1,1> MeasureNoise{0.002};
    Eigen::Matrix<double,2,2> A;  A<<1, Ts, 0, 1;
    Eigen::Matrix<double,2,1> B{Ts*Ts*0.5, Ts};
    Eigen::Matrix<double,1,2> C{1, 0};
    for(int i=0;i<2;i++)
    {
        FootForceEstimator[i].init({0,0}, ProcessNoise, MeasureNoise, A, B, C);
    }
}

void Frame::estFootForce()
{
    for(int i=0;i<2;i++)
    {
        this->EstFootForce[0][i] = this->DataInput.FSFootForceL[i];
        this->EstFootForce[1][i] = this->DataInput.FSFootForceR[i];
    }
    FootForceEstimator[0].predict(Eigen::Matrix<double,1,1>::Zero());
    FootForceEstimator[1].predict(Eigen::Matrix<double,1,1>::Zero());
    FootForceEstimator[0].update(Eigen::Matrix<double,1,1>{this->DataInput.FSFootForceL[2]});
    FootForceEstimator[1].update(Eigen::Matrix<double,1,1>{this->DataInput.FSFootForceR[2]});
    // this->EstFootForce[0][2] = __max(FootForceEstimator[0].getX()(0),0.0);
    // this->EstFootForce[1][2] = __max(FootForceEstimator[1].getX()(0),0.0);
    this->EstFootForce[0][2] = std::max(FootForceEstimator[0].getX()(0),0.0);
    this->EstFootForce[1][2] = std::max(FootForceEstimator[1].getX()(0),0.0);
}

_BHR_CF_End