// Kalman Filter 
// A hpp header for convenient application
// July 22, 2021, Qingqing Li <hexb66@bit.edu.cn>
#pragma once
// #include <Eigen/LEigenTool.h>
#include <Eigen/Dense>
#include <iostream>

namespace lee
{
    using namespace Eigen;

    template<const int STATE_NUM, const int INPUT_NUM, const int MEASURE_NUM>
    class LKalmanFilter
    {
    public:
        LKalmanFilter():
            StateNum(STATE_NUM),
            InputNum(INPUT_NUM),
            MeasureNum(MEASURE_NUM)
        {
        
        };

        void init(
            const Eigen::Matrix<double,STATE_NUM,  1> &x0,
            const Eigen::Matrix<double,STATE_NUM,  1> &noise_process, 
            const Eigen::Matrix<double,MEASURE_NUM,1> &noise_measure,
            const Eigen::Matrix<double,STATE_NUM,   STATE_NUM>  &A,
            const Eigen::Matrix<double,STATE_NUM,   INPUT_NUM>  &B,
            const Eigen::Matrix<double,MEASURE_NUM, STATE_NUM>  &C
            )
        {
            this->X = x0;
            this->NoiseProcess = noise_process;
            this->NoiseMeasure = noise_measure;
            this->Q = this->NoiseProcess*this->NoiseProcess.transpose();
            this->R = this->NoiseMeasure*this->NoiseMeasure.transpose();
            this->A = A;
            this->B = B;
            this->C = C;
            this->P.setIdentity();
            this->I.setIdentity();
        };

        void predict(const Eigen::Matrix<double,INPUT_NUM,1> &_U)
        {
            this->U = _U;
            X_pre=A*X+B*U;
            P_pre=A*P*A.transpose()+Q;
        };

        void update(const Eigen::Matrix<double,MEASURE_NUM,1> &_Y)
        {
            this->Y = _Y;
            K=P_pre*C.transpose()*(C*P_pre*C.transpose()+R).inverse();
            X=X_pre+K*(Y-C*X_pre);
            P=(I-K*C)*P_pre;
        };

        inline auto &getX(){return this->X;};

    private:
        int StateNum, InputNum, MeasureNum;

        Matrix<double,STATE_NUM,1> X_pre, X, NoiseProcess;
        Matrix<double,STATE_NUM,STATE_NUM> A, Q, P, P_pre, I;
        Matrix<double,STATE_NUM,INPUT_NUM> B;
        Matrix<double,MEASURE_NUM,1> Y, NoiseMeasure;
        Matrix<double,MEASURE_NUM,STATE_NUM> C;
        Matrix<double,MEASURE_NUM,MEASURE_NUM> R; 
        Matrix<double,STATE_NUM,MEASURE_NUM> K;
        Matrix<double,INPUT_NUM,1> U;
    };
}