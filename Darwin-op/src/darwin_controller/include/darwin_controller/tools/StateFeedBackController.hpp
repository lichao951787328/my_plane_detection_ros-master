#pragma once
#include "Base.h"
#include "Math.hpp"

_THESIS_TOOL_BEGIN
namespace state_fbc{

template<int STATE_NUM, int CONTROL_NUM>
class Controller
{
public:
    Controller()
    {
        this->X.setZero();
        this->U.setZero();
        this->K.setZero();
        this->Limit.Max.setConstant( INFINITY);
        this->Limit.Min.setConstant(-INFINITY);
    };
    math::Vec<STATE_NUM> X;
    math::Vec<CONTROL_NUM> U;
    Eigen::Matrix<double, CONTROL_NUM, STATE_NUM> K;
    struct {math::Vec<STATE_NUM> Max, Min;} Limit;

    void setK(const double _K[CONTROL_NUM][STATE_NUM])
    {
        for(int i=0;i<CONTROL_NUM;i++)
        {
            for(int j=0;j<STATE_NUM;j++)
            {
                this->K(i,j) = _K[i][j];
            }
        }
    };

    void calU()
    {
        this->U = -this->K*this->X;
    }

    void limitState()
    {
        this->X = this->X.cwiseMax(this->Limit.Min);
        this->X = this->X.cwiseMin(this->Limit.Max);
    }
};

} _THESIS_TOOL_END