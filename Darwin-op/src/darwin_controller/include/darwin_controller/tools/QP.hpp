#pragma once
#include "Base.h"
#include <LEigenQP_CPP/Core.hpp>

_THESIS_TOOL_BEGIN namespace qp{

// constexpr int N = 10;
template<int N>
class Solver:public lee::eigen_qp::LEigenQP<N>
{
protected:

public:
    int CostSeq, IeqConsSeq, EqConsSeq;
    Eigen::Matrix<double,N,1> LastResult;
    int SolveFlag;

    Solver():CostSeq(0), IeqConsSeq(0), EqConsSeq(0), SolveFlag(0)
    {   
        this->x.setZero();
        this->LastResult.setZero();
    };

    void startConfig()
    {
        this->CostSeq = 0;
        this->IeqConsSeq = 0;
        this->EqConsSeq = 0;
    };

    template<typename TypeA, typename TypeB> 
    void addCost(TypeA &_A, TypeB &_b, const double *_Weight, const bool &_CheckFlag=false)
    {
        if(_CheckFlag)
        {
            int NewSize = this->CostSeq + (int)_A.rows();
            if (this->A.rows() < NewSize)
            {
                // std::cout
                //     << "Resize cost, before: "
                //     << this->A.rows() << ", " << this->A.cols() <<", "
                //     << std::endl;
                this->A.conservativeResize(NewSize, N);
                this->b.conservativeResize(NewSize, 1);
                // std::cout
                //     << "After resize: "
                //     << this->A.rows() << ", " << this->A.cols() <<", "
                //     << std::endl;
            }
        }
        for(int i=0;i<_A.rows();i++,this->CostSeq++)
        {
            this->A.row(this->CostSeq) = _A.row(i)*sqrt(fabs(_Weight[i]));
            this->b(this->CostSeq) = _b(i)*sqrt(fabs(_Weight[i]));
        }
    };

    template<typename TypeA, typename TypeB>
    void addIeqConstraint(TypeA &_A, TypeB &_b, const bool &_CheckFlag=false)
    {
        if(_CheckFlag)
        {
            int NewSize = this->IeqConsSeq + (int)_A.rows();
            if (this->Ai.rows() < NewSize)
            {
                this->Ai.conservativeResize(NewSize, N);
                this->bi.conservativeResize(NewSize, 1);
            }
        }
        for(int i=0;i<_A.rows();i++,this->IeqConsSeq++)
        {
            this->Ai.row(this->IeqConsSeq) = _A.row(i);
            this->bi(this->IeqConsSeq) = _b(i);
        }
    };

    template<typename TypeA, typename TypeB>
    void addEqConstraint(TypeA &_A, TypeB &_b, const bool &_CheckFlag=false)
    {
        if(_CheckFlag)
        {
            int NewSize = this->EqConsSeq + (int)_A.rows();
            if (this->Ae.rows() < NewSize)
            {
                this->Ae.conservativeResize(NewSize, N);
                this->be.conservativeResize(NewSize, 1);
            }
        }
        for(int i=0;i<_A.rows();i++,this->EqConsSeq++)
        {
            this->Ae.row(this->EqConsSeq) = _A.row(i);
            this->be(this->EqConsSeq) = _b(i);
        }
    };

    void endConfig(const bool &_CheckFlag=false)
    {
        if(_CheckFlag) this->setProblem(N, this->EqConsSeq, this->IeqConsSeq, this->CostSeq);
    };

    void run()
    {
        this->updateHf();
        this->SolveFlag = this->getResult();
        if(!this->SolveFlag || this->x.hasNaN())
        {
            this->x = this->LastResult;
            this->SolveFlag = -1;
        }
        else
        {
            this->LastResult = this->x;
        }  
    };
};

}_THESIS_TOOL_END