
#include <NetPath/FootPolygon.h>
#include <cmath>
#include <BHRRobotKinematicParam/BHRRobotParameters.h>
using waw::ljh::BHRConfig;
//Modified For BHR8P1
const double ForwardLength   = BHRConfig.FootFord; //0.154;
const double BackwardLength  = BHRConfig.FootBack; //0.071;
const double WideWidth       = BHRConfig.FootOuter; //0.05;
const double NarrowWidth     = BHRConfig.FootInner;// 0.05;
/**
 * Input:
 *  CenterPose2D: FootCenter Param X(m),Y(m),Yaw(rad)
 *  StepFlag: 0L 1R 
 * Output:
 *  Vertex of X (1*4) in World Frame
 *  Vertex of Y (1*4) in World Frame
 * 
 *  Local Frame of Foot
 * 
 *         / \ Y (Left)
 *          |  
 * 3——————————————————0
 * |        |_________|___\ X (Forward)
 * |                  |   / 
 * 2——————————————————1
 */
void getFootVertex(const double CenterPos[2], const double &Yaw, const bool &IsLeft, double VertexX[4], double VertexY[4])
{    
    double MidVertexX[4] = {0.0};
    double MidVertexY[4] = {0.0};
    // load coordinates in local frame of foot
    MidVertexX[0] = ForwardLength;
    MidVertexX[1] = MidVertexX[0];
    MidVertexX[2] =-BackwardLength;
    MidVertexX[3] = MidVertexX[2];

    if(!IsLeft) //step is Right
    {
        MidVertexY[0] =  NarrowWidth;
        MidVertexY[1] = -WideWidth;
        
    }
    else //step is Left
    {
        MidVertexY[0] =  WideWidth;
        MidVertexY[1] = -NarrowWidth;
    }

    MidVertexY[2] = MidVertexY[1];
    MidVertexY[3] = MidVertexY[0];

    // GetRotate
    for(int i=0;i<4;i++)
    {
        VertexX[i] = CenterPos[0] + cos(Yaw) * MidVertexX[i] - sin(Yaw) * MidVertexY[i];
        VertexY[i] = CenterPos[1] + sin(Yaw) * MidVertexX[i] + cos(Yaw) * MidVertexY[i];
    }
}

 