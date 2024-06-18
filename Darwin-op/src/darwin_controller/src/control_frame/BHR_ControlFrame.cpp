#include <control_frame/BHR_ControlFrame.h>
#include <iostream>
#include <tools/ImuRevision.h>
#include <BHRRobotKinematicParam/BHRRobotParameters.h>
#include <LSimpleRoboticsMath/Biped.hh>
#include <unistd.h>
using waw::ljh::BHRConfig;


_BHR_CF_BEGIN

#  define FRAME_ERROR_STREAM(x)     std::cerr << x << std::endl
#  define FRAME_WARN_STREAM(x)      std::cerr << x << std::endl
#  define FRAME_INFO_STREAM(x)      std::cout << x << std::endl
#  define FRAME_PRINT_LINE_STREAM    std::cout<<"--------------------------------------------"<<std::endl


Frame::Frame(double _Ts):lee::blocks::LBlock<Input,Output>()
{
    this->Time = 0.0;
    this->Ts   = _Ts;
    this->ControlStartFlag = false;

    for(int i=0;i<3;i++) this->LocalRealBodyOmega[i] = 0.0;

    FRAME_INFO_STREAM("Create Block: YouYi_ControlFrame");
    // system("pause");
    // pause();
}

Frame::Frame(const Frame & other):lee::blocks::LBlock<Input,Output>(other)
{
    this->Time = other.Time;
    this->Ts   = other.Ts;
    this->ControlStartFlag = other.ControlStartFlag;
    this->pLogger = other.pLogger;
    for (size_t i = 0; i < 3; i++)
    {
        this->RevisedAng[i] = other.RevisedAng[i];
        this->LocalRealBodyOmega[i] = other.LocalRealBodyOmega[i];
    }
    for (size_t i = 0; i < 2; i++)
    {
        this->RealBodyZMP[i] = other.RealBodyZMP[i];
        for (size_t j = 0; j < 3; j++)
        {
            this->RefFootForce[i][j] = other.RefFootForce[i][j];
            this->RefFootTorque[i][j] = other.RefFootTorque[i][j];
            this->EstFootForce[i][j] = other.EstFootForce[i][j];
        }
    }
}

Frame & Frame::operator=(const Frame & other)
{
    if (this == &other)
    {
        return *this;
    }
    
    lee::blocks::LBlock<Input,Output>::operator=(other);
    this->Time = other.Time;
    this->Ts   = other.Ts;
    this->ControlStartFlag = other.ControlStartFlag;
    this->pLogger = other.pLogger;
    for (size_t i = 0; i < 3; i++)
    {
        this->RevisedAng[i] = other.RevisedAng[i];
        this->LocalRealBodyOmega[i] = other.LocalRealBodyOmega[i];
    }
    for (size_t i = 0; i < 2; i++)
    {
        this->RealBodyZMP[i] = other.RealBodyZMP[i];
        for (size_t j = 0; j < 3; j++)
        {
            this->RefFootForce[i][j] = other.RefFootForce[i][j];
            this->RefFootTorque[i][j] = other.RefFootTorque[i][j];
            this->EstFootForce[i][j] = other.EstFootForce[i][j];
        }
    }
    return *this;
}

// basic walk version
int Frame::init()
{
    // Default Setting is Position Control
    this->ControlStateFlag = ljh::tools::ControlType::PositionControl;
    for(auto i:this->SubBlockList) i->init();
    this->initFootForceEstimator();
    FRAME_INFO_STREAM("Init YouYi_ControlFrame: ");
    return 0;
    //auto pBlockKF      = this->addBlock<lee::posture_kf::Block>();
    //auto pBlockDCM     = this->addBlock<lee::DCM_WalkPlanner::Block>();
}

int Frame::run()
{
    this->Time+=this->Ts;
    #ifdef COPPELIA_SIM
        getPostureFromCopp(this->DataInput.RealAng, this->RevisedAng);
    #else
    // Done 修改此函数适应YouYi
        getPostureFromBHR(this->DataInput.RealAng, this->RevisedAng);
    #endif
    this->RevisedAng[2] = 0.0;
    this->calRealBodyZMP(this->RevisedAng);
    
    if(*this->DataInput.PressKey == '1')
    {
        this->ControlStartFlag = true;
        std::cout<<"control flag is true"<<std::endl;
    }
    else
        std::cout<<"control flag is false"<<std::endl;
        
    
    return 0;
}

int Frame::print()
{
    std::cout<<"[Time] "<<this->Time<<" s, ";
    std::cout<<"[ControlStartFlag] "<<this->ControlStartFlag<<", ";
    std::cout<<"[Revised Posture] "<< this->RevisedAng[0]*57.3 <<", "<< this->RevisedAng[1]*57.3<<std::endl;
    std::cout<<"[ControlStateFlag] ";
    if(this->ControlStateFlag == ljh::tools::ControlType::PositionControl)
        std::cout<<" PositionControl"<<std::endl;
    else if(this->ControlStateFlag == ljh::tools::ControlType::TorqueControl)
        std::cout<<" TorqueControl"<<std::endl;
    else
        std::cout<<" StandStill"<<std::endl;

    for(auto i:this->SubBlockList) 
    {
        FRAME_PRINT_LINE_STREAM;
        i->print();
    }
    
    return 0;
}


void Frame::calRefForce(const int &_SupFlag)
{
    // Modified For BHR-9P
    // double weight = 540.0*1.2;
    std::cout<<"RobotWeight: "<<BHRConfig.RobotWeight<<std::endl;
    double weight = BHRConfig.RobotWeight * 1.1; // vip
   
    for(int i=0;i<2;i++)
    {
        for(int j=0;j<3;j++)
        {
            this->RefFootForce[i][j] = 0.0;
            this->RefFootTorque[i][j] = 0.0;
        }
    }
    using namespace lee::math::biped;
    switch (_SupFlag)
    {
    case DS_SUP: // DSP
        this->RefFootForce[0][2] = 0.5*weight;
        this->RefFootForce[1][2] = 0.5*weight;
        break;
    case _LEFT__: // Left
        this->RefFootForce[0][2] = weight;
        this->RefFootForce[1][2] = 0.0;
        break;
    case _RIGHT_: // Right
        this->RefFootForce[0][2] = 0.0;
        this->RefFootForce[1][2] = weight;
        break;
    default:
        break;
    }
    std::cout<<"this->RefFootForce[0][2]: "<<this->RefFootForce[0][2]<<std::endl;
    std::cout<<"this->RefFootForce[1][2]: "<<this->RefFootForce[1][2]<<std::endl;
}







_BHR_CF_End