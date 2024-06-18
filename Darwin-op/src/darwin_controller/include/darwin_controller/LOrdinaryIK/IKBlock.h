// IKBlock.h
#pragma once
#include <LBlocks/LBlocks.hpp>
#include <LBlocks/LLog.hpp>
#include <Eigen/EigenKinematics.hh>

namespace lee
{
    namespace thesis
    {
        namespace IK
        {
            namespace EigenK = lee::eigen_kinematics;

            class Input
            {
            public:
                double *pBodyPosition;
                double *pBodyPosture;
                double *pLFootPosition;
                double *pLFootPosture;
                double *pRFootPosition;
                double *pRFootPosture;
            };

            class Output
            {
            public:
                double *LegJointL;
                double *LegJointR;
            };

            class Block:public blocks::LBlock<Input, Output>
            {
            protected:
                Eigen::MatT W_T_Body, W_T_LFoot, W_T_RFoot;
                Eigen::MatT W_T_LHip, W_T_RHip, W_T_LAnkle, W_T_RAnkle;
                Eigen::MatT B_T_LHip, B_T_RHip, F_T_Ankle;
                Eigen::MatT H_T_Ankle;
                double RefJoint[6];
                blocks::LLog<double> Logger;
            public:
                Block();
                int init();
                int run();
                int print();
                int log();
                int clear();
            };
        }
    }//namespace thesis
}//namespace lee