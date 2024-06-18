#pragma once
#include <LBlocks/LBlocks.hpp>

// TODO world2local is only for the walk(foot contact with ground)
// need to be modified to achieve hand/wrist contact

// The local frame is at the location of body (the middle point of two hips), and its yaw 
namespace waw{namespace world2local{

class Input
{
public:
    const double *W_BodyPos;
    const double *W_BodyAng;
    const double *W_BodyLinearVel;
    const double *W_BodyAngularVel;
    const double *W_BodyLinearAcc;

    const double *W_FootPosL;
    const double *W_FootAngL;
    
    const double *W_FootPosR;
    const double *W_FootAngR;

    const double *W_ZMP;
};

class Output
{
public:
    double L_BodyPos[3];
    double L_BodyAng[3];
    double L_BodyLinearVel [3];
    double L_BodyAngularVel[3];
    double L_BodyLinearAcc [3];
    double L_FootPosL[3];
    double L_FootAngL[3];
    double L_FootPosR[3];
    double L_FootAngR[3];
    double L_ZMP[3];
};

class Block: public lee::blocks::LBlock<Input, Output>
{
public:
    Block();
    int init();
    int run();

protected:

};  

}}