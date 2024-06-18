#include <control_frame/BHR_ControlFrame.h>
#include <LBlocks/LLog.hpp>
#include <matplotlibcpp.h>
#include <algorithm>
#include <glog/logging.h>
namespace plt = matplotlibcpp;

_BHR_CF_BEGIN

int Frame::log()
{
    // this->pLogger->startLog();
    LOG(INFO)<<"frame log: ";
    pLogger->addLog(this->Time, "Time");

    pLogger->addLog(this->DataInput.RealLegJointL[0], "RealLegJointL1");
    pLogger->addLog(this->DataInput.RealLegJointL[1], "RealLegJointL2");
    pLogger->addLog(this->DataInput.RealLegJointL[2], "RealLegJointL3");
    pLogger->addLog(this->DataInput.RealLegJointL[3], "RealLegJointL4");
    pLogger->addLog(this->DataInput.RealLegJointL[4], "RealLegJointL5");
    pLogger->addLog(this->DataInput.RealLegJointL[5], "RealLegJointL6");

    pLogger->addLog(this->DataInput.RealLegJointR[0], "RealLegJointR1");
    pLogger->addLog(this->DataInput.RealLegJointR[1], "RealLegJointR2");
    pLogger->addLog(this->DataInput.RealLegJointR[2], "RealLegJointR3");
    pLogger->addLog(this->DataInput.RealLegJointR[3], "RealLegJointR4");
    pLogger->addLog(this->DataInput.RealLegJointR[4], "RealLegJointR5");
    pLogger->addLog(this->DataInput.RealLegJointR[5], "RealLegJointR6");
    
    pLogger->addLog(this->RevisedAng[0]*57.3, "IMU_Roll");
    pLogger->addLog(this->RevisedAng[1]*57.3, "IMU_Pitch");

    pLogger->addLog(this->DataInput.FSFootForceL[0], "ForceFootLX");
    pLogger->addLog(this->DataInput.FSFootForceL[1], "ForceFootLY");
    pLogger->addLog(this->DataInput.FSFootForceL[2], "ForceFootLZ");
    pLogger->addLog(this->DataInput.FSFootTorqueL[0], "TorqueFootLX");
    pLogger->addLog(this->DataInput.FSFootTorqueL[1], "TorqueFootLY");
    pLogger->addLog(this->DataInput.FSFootTorqueL[2], "TorqueFootLZ");

    pLogger->addLog(this->DataInput.FSFootForceR[0], "ForceFootRX");
    pLogger->addLog(this->DataInput.FSFootForceR[1], "ForceFootRY");
    pLogger->addLog(this->DataInput.FSFootForceR[2], "ForceFootRZ");
    pLogger->addLog(this->DataInput.FSFootTorqueR[0], "TorqueFootRX");
    pLogger->addLog(this->DataInput.FSFootTorqueR[1], "TorqueFootRY");
    pLogger->addLog(this->DataInput.FSFootTorqueR[2], "TorqueFootRZ");

    pLogger->addLog(this->RealBodyZMP[0], "RealBodyZMPX");
    pLogger->addLog(this->RealBodyZMP[1], "RealBodyZMPY");

    for(auto i:this->SubBlockList) i->log();
    
    return 0;
}

void Frame::plotData(const char *_Str)
{
    plt::named_plot(_Str, this->pLogger->getData("Time"), this->pLogger->getData(_Str), ".-");
}

int Frame::clear()
{   
    LOG(INFO)<<"enter frame:";
    for(auto i:this->SubBlockList) i->clear();
    #ifdef SIMULATION
        LOG(INFO)<<"end log";
        if(this->pLogger->getNameList().size() <=0) 
        {
            std::cout<<"There is no data saved!"<<std::endl;
            return 0;
        }
        std::cout<<"Save data.dat, "<<pLogger->getNameList().size()<<" x "<<pLogger->getDataList()[0].size()<<" numbers";
        pLogger->saveLog("data.dat");
        std::cout<<" have been saved."<<std::endl;
    #endif
    return 1;
}


_BHR_CF_End