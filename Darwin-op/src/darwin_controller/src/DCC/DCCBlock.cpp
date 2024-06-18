#include <Compliance/DCC/DCCBlock.h>
#include <Compliance/DCC/Dcc_lib/Base/dcc_con_base.h>
#include <Compliance/DCC/Dcc_lib/Base/dcc_Mat.h>
#include <Compliance/DCC/DccCon_lib/DCC_RunningCon.h>
#include <iostream>
#include <BHRRobotKinematicParam/BHRRobotParameters.h>
#include <DHdCon.hpp> // dccrec
#include <glog/logging.h>
// #define __DConNew // dccrec
 
using waw::ljh::BHRConfig;
using waw::ljh::BHRResetPos;
extern "C" dccSpacial MdzmpConVal;
extern "C" double MpitLimit[2];
extern "C" double MrolLimit[2];

extern "C" dccRotational Mstab ;
extern "C" dccRotational Mmdzmp;
extern "C" dccRotational Mpend ;
extern "C" dccRotational Mwheel;
extern "C" dccRotational Mfeet ;
extern "C" dccJoints stJoints; 

extern "C" dccPositional dBaseErr;

int init_flag = 1;
#ifdef __DConNew
_D_USING_DCON
st_RobotConfig st7p2Config = {
	/*Tc*/			0.004, 
	/*Mass*/		69.0,
	/*UpperMass*/	14.0,
	/*UpperIner*/	0.18592,
	/*Zc*/			0.63,
	/*AnkWidth*/	0.2,
	/*FootGeom*/	{ 0.085, 0.07, 0.06, 0.075 } //{ 0.115, 0.08, 0.05, 0.065 }
};
double dFBCom = 0.95 * 1.45, dFBvCom = 2.4 * 1.25;
double dSoftFootGain = 0.4 * 1.2; 
double dSoftFootGainz = 0.5 * 1.25; 
double dHardSpringGain = 2.0; 
double dHardSpringGainz = 2.0; 
double dDampFootGainz = 2.5; // 1.5
// double dFBCom = 1.35, dFBvCom = 4.6;
// double dSoftFootGain = 0.6; 
// double dSoftFootGainz = 2.0; 
// double dHardSpringGain = 0.2; 
// double dHardSpringGainz = 0.5;
st_DHdConGains st7p2Gains = {
	/*Vip*/			1.0,
					{ 200.0, 60.0, 25.0 },
					{ 1.5 * dFBCom * st7p2Config.Mass * __Gravity, dFBvCom * 30.0 },
					{ 50.0, 10.0, 50.0, 10.0 },
					{ dSoftFootGainz * 0.025 * 0.8, dHardSpringGainz * 100.0, dDampFootGainz * 80.0 * 1.5, 3.0, 0.003, 0.00007, 0.000048, dSoftFootGain * 0.06, dHardSpringGain * 2.0 },
					{ 0.005, 100.0 },
					{ 0.8049, 2.0 }
};
st_Filters st7p2Filter = {
	/*zmp*/			0.002,
	/*imu*/			0.004, 
	/*Gro*/			0.01, 
	/*frc*/			0.0002,
	/*trq*/			0.0002,
	/*amp*/			0.5
};
st_DHdConIO stDHdConIo;
c_DHdCon cDHdCon7p2(&stDHdConIo, &st7p2Config, &st7p2Gains, &st7p2Filter);
#endif

namespace compliance{namespace dcc{
	
    int Block::init()
    {
		#ifdef __DConNew
		cDHdCon7p2.Init(
			/*PosCon*/	0, 
			/*MdZMP*/	0, 
			/*FbFft*/	0, // ToDo 加手还行不？
			/*GRFC*/	0, 
			/*Comp*/	0,
			/*ArmSwi*/	0);
		#else
        dTLagZMP = 0.002; // 0.004
		dTLagIMU = 0.004; // 0.004 important!! dont change!!
		// dTLagdIMU = 0.01; // 0.01 important!! dont change!!
		dTLagdIMU = 0.004; // 0.01 important!! dont change!!
		dTLagFrc = 0.0002; // 0.0001
		dTLagTrq = 0.0002; // 0.0001
		fnvStateInit(&stStatePG);
		fnvStateInit(&stStateSens);
		fnvStateInit(&stStateConVal);
		fnvStateInit(&stStateRef);
		fnvStateInit(&stStateCmd);
		
		stStateCmd.Ankle.B.Rfoot.pos.x = BHRConfig.HipWidth/2.0;//0.10;//0.08;
		stStateCmd.Ankle.B.Lfoot.pos.x = -BHRConfig.HipWidth/2.0;//-0.08;
		for(int i=0;i<6;i++)
		{
			this->DataOutput.dArmLq[i] = 0.0;
			this->DataOutput.dArmRq[i] = 0.0;
		}
		
		#endif
		this->GUIFlag = ljh::tools::GUIStateFlag::GUI_OFF;
        return 0;
    }

    int Block::run()
    {
		#ifdef __DConNew
		if(*this->DataInput.pControlStartFlag == true) 
			cDHdCon7p2.On();
		for(int i = 0; i < 2; i++)  {
			stDHdConIo.IMUSen[i] = this->DataInput.dBaseRotSens[i];
			stDHdConIo.ZMPSen_B[i] = this->DataInput.dZMPbSens[i];
		}
		for(int i = 0; i < 3; i++) {
			stDHdConIo.LFTSen_B[i] 		= this->DataInput.dLFootFSens[i];
			stDHdConIo.RFTSen_B[i] 		= this->DataInput.dRFootFSens[i];
			stDHdConIo.LFTSen_B[i + 3] 	= this->DataInput.dLFootTSens[i];
			stDHdConIo.RFTSen_B[i + 3] 	= this->DataInput.dRFootTSens[i];
			stDHdConIo.LFTPG_B[i]		= this->DataInput.dLFootFPG[i];
			stDHdConIo.RFTPG_B[i]		= this->DataInput.dRFootFPG[i];
			stDHdConIo.LFTPG_B[i + 3]	= this->DataInput.dLFootTPG[i];
			stDHdConIo.RFTPG_B[i + 3]	= this->DataInput.dRFootTPG[i];
			stDHdConIo.BasePG[i] 		= this->DataInput.dBasePosPG[i];
			stDHdConIo.BasePG[i + 3] 	= this->DataInput.dBaseRotPG[i];
			stDHdConIo.LAnkPG_W[i] 		= this->DataInput.dLAnkPosPG[i];
			stDHdConIo.RAnkPG_W[i] 		= this->DataInput.dRAnkPosPG[i];
			stDHdConIo.LAnkPG_W[i + 3] 	= this->DataInput.dLAnkRotPG[i];
			stDHdConIo.RAnkPG_W[i + 3] 	= this->DataInput.dRAnkRotPG[i];
		}		
		for(int i = 0; i < 6; i++) {
			stDHdConIo.LegQ[i] = this->DataInput.dLegLq[i];
			stDHdConIo.LegQ[i + 6] = this->DataInput.dLegRq[i];
		}
		stDHdConIo.ArmQ[0] =  BHRResetPos.getLeft_Arm1IniPos();;
		stDHdConIo.ArmQ[1] =  BHRResetPos.getRightArm1IniPos();;
		switch (this->DataInput.nSupleg[0])
		{
		case 2:
			stDHdConIo.SupSignal = DSup;
			break;
		case 1:
			stDHdConIo.SupSignal = RSup;
			break;
		case 0:
			stDHdConIo.SupSignal = LSup;
			break;
		case 'N':
			stDHdConIo.SupSignal = Fly;
			break;
		default:
			break;
		}
		if(stDHdConIo.SupSignal == DSup) {
			stDHdConIo.LFTPG_B[_rl] = 0.0;
			stDHdConIo.LFTPG_B[_pt] = 0.0;
			stDHdConIo.RFTPG_B[_rl] = 0.0;
			stDHdConIo.RFTPG_B[_pt] = 0.0;
		}
		cDHdCon7p2.Loop();
		for(int i = 0; i < 3; i++) {
			this->DataOutput.dBasePosCmd[i] = stDHdConIo.BaseCmd[i];
			this->DataOutput.dBaseRotCmd[i] = stDHdConIo.BaseCmd[i + 3];
			this->DataOutput.dLAnkPosCmd[i] = stDHdConIo.LAnkCmd_W[i];
			this->DataOutput.dRAnkPosCmd[i] = stDHdConIo.RAnkCmd_W[i];
			this->DataOutput.dLAnkRotCmd[i] = stDHdConIo.LAnkCmd_W[i + 3];
			this->DataOutput.dRAnkRotCmd[i] = stDHdConIo.RAnkCmd_W[i + 3];
		}
		this->DataOutput.dArmLq[0] = stDHdConIo.ArmQcmd[0];
		this->DataOutput.dArmRq[0] = stDHdConIo.ArmQcmd[1];
		#else
		// std::cout<<"in dcc control frame:"<<std::endl;
		
		
        fnvGetStatePG();	// -> stStatePG
		fnvGetStateSens();	// -> stStateSens
		// std::cout<<"DCC FLAG "<<*this->DataInput.pControlStartFlag<<std::endl;
		// *this->DataInput.pControlStartFlag = true;
		if(*this->DataInput.pControlStartFlag == true) {
			printf("Control is actived!\n");
			fnvGetConVal();		// -> stStateConVal
		}
		fnvAddConVal();		// -> stStateCmd
		this->DataOutput.dArmLq[0] = -stJoints.La; 
		this->DataOutput.dArmRq[0] = -stJoints.Ra;  
		stStateConVal.Base.pos.x = 0.0;
		stStateConVal.Base.pos.y = 0.0;
		stStateConVal.Base.pos.z = 0.0;
		#endif
		this->DataOutput.dArmLq[1] = BHRResetPos.getLeft_Arm2IniPos();  
		this->DataOutput.dArmLq[2] = 0.0;  
		this->DataOutput.dArmLq[3] = BHRResetPos.getLeft_Arm4IniPos();  
		this->DataOutput.dArmLq[4] = 0.0;  
		this->DataOutput.dArmLq[5] = 0.0;  
		this->DataOutput.dArmRq[1] = BHRResetPos.getRightArm2IniPos();  
		this->DataOutput.dArmRq[2] = 0.0;  
		this->DataOutput.dArmRq[3] = BHRResetPos.getRightArm4IniPos();  
		this->DataOutput.dArmRq[4] = 0.0;  
		this->DataOutput.dArmRq[5] = 0.0;  
		// LOG(INFO)<<"DCC block:";
		// std::cout<<"left foot force: "<<this->DataInput.dLFootFSens[0]<<" "<<this->DataInput.dLFootFSens[1]<<" "<<this->DataInput.dLFootFSens[2]<<std::endl;
		// std::cout<<"left foot torque: "<<this->DataInput.dLFootTSens[0]<<" "<<this->DataInput.dLFootTSens[1]<<" "<<this->DataInput.dLFootTSens[2]<<std::endl;

		// std::cout<<"right foot force: "<<this->DataInput.dRFootFSens[0]<<" "<<this->DataInput.dRFootFSens[1]<<" "<<this->DataInput.dRFootFSens[2]<<std::endl;
		// std::cout<<"right foot torque: "<<this->DataInput.dRFootTSens[0]<<" "<<this->DataInput.dRFootTSens[1]<<" "<<this->DataInput.dRFootTSens[2]<<std::endl;

		// std::cout<<"output: "<<std::endl;
		// std::cout<<"dBasePosCmd"<<std::endl;
		// for (size_t i = 0; i < 3; i++)
		// {
		// 	std::cout<<this->DataOutput.dBasePosCmd[i]<<" ";
		// }
		// std::cout<<std::endl;
		// std::cout<<"dBaseRotCmd"<<std::endl;
		// for (size_t i = 0; i < 3; i++)
		// {
		// 	std::cout<<this->DataOutput.dBaseRotCmd[i]<<" ";
		// }
		// std::cout<<std::endl;
		// std::cout<<"dLAnkPosCmd"<<std::endl;
		// for (size_t i = 0; i < 3; i++)
		// {
		// 	std::cout<<this->DataOutput.dLAnkPosCmd[i]<<" ";
		// }
		// std::cout<<std::endl;
		// std::cout<<"dLAnkRotCmd"<<std::endl;
		// for (size_t i = 0; i < 3; i++)
		// {
		// 	std::cout<<this->DataOutput.dLAnkRotCmd[i]<<" ";
		// }
		// std::cout<<std::endl;
		// std::cout<<"dRAnkPosCmd"<<std::endl;
		// for (size_t i = 0; i < 3; i++)
		// {
		// 	std::cout<<this->DataOutput.dRAnkPosCmd[i]<<" ";
		// }
		// std::cout<<std::endl;
		// std::cout<<"dRAnkRotCmd"<<std::endl;
		// for (size_t i = 0; i < 3; i++)
		// {
		// 	std::cout<<this->DataOutput.dRAnkRotCmd[i]<<" ";
		// }
		// std::cout<<std::endl;
        return 0;
    }

	#ifndef __DConNew
    void Block::fnvStateInit(dccRobotState * stStateName){
        for (int i = 0; i < 9; i++) {
			*(&stStateName->Ankle.B.Rfoot.pos.x + i) = -0.0;
			*(&stStateName->Ankle.B.Lfoot.pos.x + i) = 0.0;
			*(&stStateName->Ankle.B.Rfoot.rot.pit + i) = 0.0;
			*(&stStateName->Ankle.B.Lfoot.rot.pit + i) = 0.0;
			*(&stStateName->Ankle.W.Rfoot.pos.x + i) = 0.0;
			*(&stStateName->Ankle.W.Lfoot.pos.x + i) = 0.0;
			*(&stStateName->Ankle.W.Rfoot.rot.pit + i) = 0.0;
			*(&stStateName->Ankle.W.Lfoot.rot.pit + i) = 0.0;
			*(&stStateName->Base.pos.x + i) = 0.0;
			*(&stStateName->Base.rot.pit + i) = 0.0;
			*(&stStateName->ZMP.B.x + i) = 0.0;
			*(&stStateName->ZMP.W.x + i) = 0.0;
		}
		for (int i = 0; i < 6; i++) {
			*(&stStateName->FootFT.Rfoot.fx + i) = 0.0;
			*(&stStateName->FootFT.Lfoot.fx + i) = 0.0;
		}
		stStateName->SupLeg = 0;
		stStateName->FootFT.Fsum = 0.0;
    }

	double Lq3_old, Lq4_old, Rq3_old, Rq4_old; 
	int nIfFirst = 2;
    void Block::fnvGetStatePG(){
		// dccarm ======================================
		stJoints.Lq[2] = this->DataInput.dLegLq[2]; 
		stJoints.Lq[3] = this->DataInput.dLegLq[3]; 
		stJoints.Rq[2] = this->DataInput.dLegRq[2]; 
		stJoints.Rq[3] = this->DataInput.dLegRq[3]; 
		if(nIfFirst < 2) {
			Lq3_old = stJoints.Lq[2];
			Lq4_old = stJoints.Lq[3];
			Rq3_old = stJoints.Rq[2];
			Rq4_old = stJoints.Rq[3];
			nIfFirst--;
		}
		stJoints.Ldq[2] = (stJoints.Lq[2] - Lq3_old) / __ControlT;
		stJoints.Ldq[3] = (stJoints.Lq[3] - Lq4_old) / __ControlT;
		stJoints.Rdq[2] = (stJoints.Rq[2] - Rq3_old) / __ControlT;
		stJoints.Rdq[3] = (stJoints.Rq[3] - Rq4_old) / __ControlT;
		Lq3_old = stJoints.Lq[2];
		Lq4_old = stJoints.Lq[3];
		Rq3_old = stJoints.Rq[2];
		Rq4_old = stJoints.Rq[3];
		// dccarm ======================================
        stStatePG.ZMP.W.x = -this->DataInput.dZMPwPG[1];
		stStatePG.ZMP.W.y = this->DataInput.dZMPwPG[0];
		stStatePG.Base.pos.dx = -this->DataInput.dBasedPosPG[1];
		stStatePG.Base.pos.dy = this->DataInput.dBasedPosPG[0];
		stStatePG.Base.pos.x = -this->DataInput.dBasePosPG[1];
		stStatePG.Base.pos.y = this->DataInput.dBasePosPG[0];
		stStatePG.Base.pos.z = this->DataInput.dBasePosPG[2];
		stStatePG.ZMP.B.x = -this->DataInput.dZMPbPG[1];
		stStatePG.ZMP.B.y = this->DataInput.dZMPbPG[0];
		stStatePG.Base.rot.dpit = -this->DataInput.dBasedRotPG[1];
		stStatePG.Base.rot.drol  = this->DataInput.dBasedRotPG[0];
		stStatePG.Base.rot.pit  = -this->DataInput.dBaseRotPG[1];
		stStatePG.Base.rot.rol   = this->DataInput.dBaseRotPG[0];
		stStateRef.Base.rot.dpit = stStatePG.Base.rot.dpit;
		stStateRef.Base.rot.drol = stStatePG.Base.rot.drol;
		stStateRef.Base.rot.pit  = stStatePG.Base.rot.pit ;
		stStateRef.Base.rot.rol  = stStatePG.Base.rot.rol ;
		stStatePG.Ankle.W.Rfoot.pos.x = -this->DataInput.dRAnkPosPG[1];
		stStatePG.Ankle.W.Rfoot.pos.y = this->DataInput.dRAnkPosPG[0];
		stStatePG.Ankle.W.Rfoot.pos.z = this->DataInput.dRAnkPosPG[2];
		stStatePG.Ankle.W.Rfoot.rot.pit = -this->DataInput.dRAnkRotPG[1];
		stStatePG.Ankle.W.Rfoot.rot.rol = this->DataInput.dRAnkRotPG[0];
		stStatePG.Ankle.W.Lfoot.pos.x = -this->DataInput.dLAnkPosPG[1];
		stStatePG.Ankle.W.Lfoot.pos.y = this->DataInput.dLAnkPosPG[0];
		stStatePG.Ankle.W.Lfoot.pos.z = this->DataInput.dLAnkPosPG[2];
		stStatePG.Ankle.W.Lfoot.rot.pit = -this->DataInput.dLAnkRotPG[1];
		stStatePG.Ankle.W.Lfoot.rot.rol = this->DataInput.dLAnkRotPG[0];
		stStatePG.Ankle.B.Rfoot.pos.x = stStatePG.Ankle.W.Rfoot.pos.x - stStatePG.Base.pos.x;
		stStatePG.Ankle.B.Rfoot.pos.y = stStatePG.Ankle.W.Rfoot.pos.y - stStatePG.Base.pos.y;
		stStatePG.Ankle.B.Rfoot.pos.z = stStatePG.Ankle.W.Rfoot.pos.z - stStatePG.Base.pos.z;
		stStatePG.Ankle.B.Lfoot.pos.x = stStatePG.Ankle.W.Lfoot.pos.x - stStatePG.Base.pos.x;
		stStatePG.Ankle.B.Lfoot.pos.y = stStatePG.Ankle.W.Lfoot.pos.y - stStatePG.Base.pos.y;
		stStatePG.Ankle.B.Lfoot.pos.z = stStatePG.Ankle.W.Lfoot.pos.z - stStatePG.Base.pos.z;
		stStatePG.Base.pos.ddz = this->DataInput.dBaseddPosPG[2];
		stStatePG.FootFT.Rfoot.tx =  -this->DataInput.dRFootTPG[1];
		stStatePG.FootFT.Rfoot.ty =  this->DataInput.dRFootTPG[0];
		stStatePG.FootFT.Rfoot.fz = this->DataInput.dRFootFPG[2];
		// printf
		printf("stStatePG.Rfoot.fz:....%f\n", stStatePG.FootFT.Rfoot.fz);
		stStatePG.FootFT.Lfoot.tx = -this->DataInput.dLFootTPG[1];
		stStatePG.FootFT.Lfoot.ty = this->DataInput.dLFootTPG[0];
		stStatePG.FootFT.Lfoot.fz = this->DataInput.dLFootFPG[2];
        switch (this->DataInput.nSupleg[0])
		{
		case 2:
			stStatePG.SupLeg = 0;
			break;
		case 1:
			stStatePG.SupLeg = 1;
			break;
		case 0:
			stStatePG.SupLeg = 2;
			break;
		case 'N':
			stStatePG.SupLeg = 3;
			break;
		default:
			break;
		}
    }

    void Block::fnvGetStateSens(){
		double dTorqueAmp = 0.5;
        if (init_flag == 1){
			init_flag = 0;
			stStateSens.Base.rot.pit = -this->DataInput.dBaseRotSens[1];
			stStateSens.Base.rot.rol = this->DataInput.dBaseRotSens[0];
		}
		stStateSens.ZMP.W.x = fndFilterTimeLag(stStateSens.ZMP.W.x, -this->DataInput.dZMPwSens[1], __ControlT, dTLagZMP);
		stStateSens.ZMP.W.y = fndFilterTimeLag(stStateSens.ZMP.W.y, this->DataInput.dZMPwSens[0], __ControlT, dTLagZMP);
		stStateSens.ZMP.B.x = fndFilterTimeLag(stStateSens.ZMP.B.x, -this->DataInput.dZMPbSens[1], __ControlT, dTLagZMP);
		stStateSens.ZMP.B.y = fndFilterTimeLag(stStateSens.ZMP.B.y, this->DataInput.dZMPbSens[0], __ControlT, dTLagZMP);
		stStateSens.Base.rot.dpit = fndFilterTimeLag(stStateSens.Base.rot.dpit, (-this->DataInput.dBaseRotSens[1] - stStateSens.Base.rot.pit) / __ControlT, __ControlT, dTLagdIMU);
		stStateSens.Base.rot.drol = fndFilterTimeLag(stStateSens.Base.rot.drol, (this->DataInput.dBaseRotSens[0] - stStateSens.Base.rot.rol) / __ControlT, __ControlT, dTLagdIMU);
		stStateSens.Base.rot.pit = fndFilterTimeLag(stStateSens.Base.rot.pit, -this->DataInput.dBaseRotSens[1], __ControlT, dTLagIMU);
		stStateSens.Base.rot.rol = fndFilterTimeLag(stStateSens.Base.rot.rol, this->DataInput.dBaseRotSens[0], __ControlT, dTLagIMU); 
		stStateSens.FootFT.Lfoot.fx = fndFilterTimeLag(stStateSens.FootFT.Lfoot.fx, -this->DataInput.dLFootFSens[1], __ControlT, dTLagFrc);
		stStateSens.FootFT.Lfoot.fy = fndFilterTimeLag(stStateSens.FootFT.Lfoot.fy, this->DataInput.dLFootFSens[0], __ControlT, dTLagFrc);
		stStateSens.FootFT.Lfoot.fz = fndFilterTimeLag(stStateSens.FootFT.Lfoot.fz, this->DataInput.dLFootFSens[2], __ControlT, dTLagFrc);
		stStateSens.FootFT.Rfoot.fx = fndFilterTimeLag(stStateSens.FootFT.Rfoot.fx, -this->DataInput.dRFootFSens[1], __ControlT, dTLagFrc);
		stStateSens.FootFT.Rfoot.fy = fndFilterTimeLag(stStateSens.FootFT.Rfoot.fy, this->DataInput.dRFootFSens[0], __ControlT, dTLagFrc);
		stStateSens.FootFT.Rfoot.fz = fndFilterTimeLag(stStateSens.FootFT.Rfoot.fz, this->DataInput.dRFootFSens[2], __ControlT, dTLagFrc);
		// print
		printf("stStateSens.Lfoot.fz:....%f\n", stStateSens.FootFT.Rfoot.fz);
		stStateSens.FootFT.Lfoot.tx = fndFilterTimeLag(stStateSens.FootFT.Lfoot.tx, -dTorqueAmp * this->DataInput.dLFootTSens[1], __ControlT, dTLagTrq);
		stStateSens.FootFT.Lfoot.ty = fndFilterTimeLag(stStateSens.FootFT.Lfoot.ty, dTorqueAmp * this->DataInput.dLFootTSens[0], __ControlT, dTLagTrq);
		stStateSens.FootFT.Lfoot.tz = fndFilterTimeLag(stStateSens.FootFT.Lfoot.tz, this->DataInput.dLFootTSens[2], __ControlT, dTLagTrq);
		stStateSens.FootFT.Rfoot.tx = fndFilterTimeLag(stStateSens.FootFT.Rfoot.tx, -dTorqueAmp * this->DataInput.dRFootTSens[1], __ControlT, dTLagTrq);
		stStateSens.FootFT.Rfoot.ty = fndFilterTimeLag(stStateSens.FootFT.Rfoot.ty, dTorqueAmp * this->DataInput.dRFootTSens[0], __ControlT, dTLagTrq);
		stStateSens.FootFT.Rfoot.tz = fndFilterTimeLag(stStateSens.FootFT.Rfoot.tz, this->DataInput.dRFootTSens[2], __ControlT, dTLagTrq);
		stStateSens.FootFT.Fsum = stStateSens.FootFT.Lfoot.fz + stStateSens.FootFT.Rfoot.fz;
    }

    void Block::fnvGetConVal(){
        fnvStateInit(&stStateConVal);
		#ifdef USE_DCC_METHOD
			fnvDccRunCon();
		#endif
		#ifdef USE_HONDA_METHOD
			fnvRunConHonda();
		#endif
    }

    void Block::fnvAddConVal(){
        for (int i = 0; i < 3; i++) {
			*(&stStateCmd.Base.pos.x + i) = *(&stStatePG.Base.pos.x + i) + *(&stStateConVal.Base.pos.x + i);
			*(&stStateCmd.Base.rot.pit + i) = *(&stStatePG.Base.rot.pit + i) + *(&stStateConVal.Base.rot.pit + i);
		}
		for (int i = 0; i < 3; i++) {
			*(&stStateCmd.Ankle.W.Rfoot.pos.x + i) = *(&stStatePG.Ankle.W.Rfoot.pos.x + i) + *(&stStateConVal.Ankle.B.Rfoot.pos.x + i);
			*(&stStateCmd.Ankle.W.Lfoot.pos.x + i) = *(&stStatePG.Ankle.W.Lfoot.pos.x + i) + *(&stStateConVal.Ankle.B.Lfoot.pos.x + i);
			
			*(&stStateCmd.Ankle.W.Rfoot.rot.pit + i) = *(&stStatePG.Ankle.W.Rfoot.rot.pit + i) + *(&stStateConVal.Ankle.B.Rfoot.rot.pit + i);
			*(&stStateCmd.Ankle.W.Lfoot.rot.pit + i) = *(&stStatePG.Ankle.W.Lfoot.rot.pit + i) + *(&stStateConVal.Ankle.B.Lfoot.rot.pit + i);
			
			*(&stStateCmd.Ankle.B.Rfoot.pos.x + i) = *(&stStatePG.Ankle.B.Rfoot.pos.x + i) + *(&stStateConVal.Ankle.B.Rfoot.pos.x + i);
			*(&stStateCmd.Ankle.B.Lfoot.pos.x + i) = *(&stStatePG.Ankle.B.Lfoot.pos.x + i) + *(&stStateConVal.Ankle.B.Lfoot.pos.x + i);
		}
		// dccarm ======================================
		this->DataOutput.dArmLq[0] = -stJoints.La;  
		this->DataOutput.dArmLq[1] = DccD2R(7.0);  
		this->DataOutput.dArmLq[2] = 0.0;  
		this->DataOutput.dArmLq[3] = DccD2R(-30.0);  
		this->DataOutput.dArmLq[4] = 0.0;  
		this->DataOutput.dArmLq[5] = 0.0;  
		this->DataOutput.dArmRq[0] = -stJoints.Ra; 
		this->DataOutput.dArmRq[1] = DccD2R(-7.0);  
		this->DataOutput.dArmRq[2] = 0.0;  
		this->DataOutput.dArmRq[3] = -DccD2R(30.0);  
		this->DataOutput.dArmRq[4] = 0.0;  
		this->DataOutput.dArmRq[5] = 0.0;  
		// dccarm ======================================
		this->DataOutput.dBasePosCmd[1] = -stStateCmd.Base.pos.x;
		this->DataOutput.dBasePosCmd[0] = stStateCmd.Base.pos.y;
		this->DataOutput.dBasePosCmd[2] = stStateCmd.Base.pos.z;
		this->DataOutput.dBaseRotCmd[1] = -stStateCmd.Base.rot.pit;
		this->DataOutput.dBaseRotCmd[0] = stStateCmd.Base.rot.rol;
		this->DataOutput.dRAnkPosCmd[1] = -stStateCmd.Ankle.W.Rfoot.pos.x;
		this->DataOutput.dRAnkPosCmd[0] = stStateCmd.Ankle.W.Rfoot.pos.y;
		this->DataOutput.dRAnkPosCmd[2] = stStateCmd.Ankle.W.Rfoot.pos.z;
		this->DataOutput.dRAnkRotCmd[1] = -stStateCmd.Ankle.W.Rfoot.rot.pit;
		this->DataOutput.dRAnkRotCmd[0] = stStateCmd.Ankle.W.Rfoot.rot.rol;
		this->DataOutput.dLAnkPosCmd[1] = -stStateCmd.Ankle.W.Lfoot.pos.x;
		this->DataOutput.dLAnkPosCmd[0] = stStateCmd.Ankle.W.Lfoot.pos.y;
		this->DataOutput.dLAnkPosCmd[2] = stStateCmd.Ankle.W.Lfoot.pos.z;
		this->DataOutput.dLAnkRotCmd[1] = -stStateCmd.Ankle.W.Lfoot.rot.pit;
		this->DataOutput.dLAnkRotCmd[0] = stStateCmd.Ankle.W.Lfoot.rot.rol;
		printf("InDCC ankez %lf\n", stStateCmd.Ankle.W.Rfoot.pos.z);

		// this->DataOutput.dBaseRotCmd[2] = this->DataInput.dBasedRotPG[2];
		this->DataOutput.dLAnkRotCmd[2] = this->DataInput.dLAnkRotPG[2];
		this->DataOutput.dRAnkRotCmd[2] = this->DataInput.dRAnkRotPG[2];
    }
	#endif

    int Block::clear()
    {
        return 0;
    }

    int Block::log()
    {
		// std::string strAxis[3] = {"X", "Y", "Z"};
		auto p = this->DataInput.pLogger;
		auto &Out = this->DataOutput;
		
		#ifdef __DConNew
        p->addLog(cDHdCon7p2.GetPGData().Fft.R.B[__z] , "Fz_r_pg");
        p->addLog(cDHdCon7p2.GetRefData().Fft.R.B[__z], "Fz_r_ref");	
		p->addLog(cDHdCon7p2.GetSenData().Fft.R.B[__z], "Fz_r_sens");		
		p->addLog(cDHdCon7p2.GetCoVData().Ank.R.B[__z], "z_r_conval");		

		p->addLog(cDHdCon7p2.GetPGData().Fft.L.B[__z] , "Fz_l_pg");		
		p->addLog(cDHdCon7p2.GetRefData().Fft.L.B[__z], "Fz_l_ref");		
		p->addLog(cDHdCon7p2.GetSenData().Fft.L.B[__z], "Fz_l_sens");		
		p->addLog(cDHdCon7p2.GetCoVData().Ank.L.B[__z], "z_l_conval");	
		
		p->addLog(cDHdCon7p2.GetPGData().Fft.R.B[_rl] , "Trol_r_pg");	
		p->addLog(cDHdCon7p2.GetRefData().Fft.R.B[_rl], "Trol_r_ref");		
		p->addLog(cDHdCon7p2.GetSenData().Fft.R.B[_rl], "Trol_r_sens");		
		p->addLog(cDHdCon7p2.GetCoVData().Ank.R.B[_rl], "rol_r_conval");	
		p->addLog(dConR[_rl] , 							"rol_r_conval");		
			
		p->addLog(cDHdCon7p2.GetPGData().Fft.L.B[_rl] , "Trol_l_pg");	
		p->addLog(cDHdCon7p2.GetRefData().Fft.L.B[_rl], "Trol_l_ref");		
		p->addLog(cDHdCon7p2.GetSenData().Fft.L.B[_rl], "Trol_l_sens");		
		p->addLog(cDHdCon7p2.GetCoVData().Ank.L.B[_rl], "rol_l_conval");		
		p->addLog(dConL[_rl] , 							"rol_l_conval");	
			
		p->addLog(cDHdCon7p2.GetPGData().Fft.R.B[_pt] , "Tpit_r_pg");	
		p->addLog(cDHdCon7p2.GetRefData().Fft.R.B[_pt], "Tpit_r_ref");		
		p->addLog(cDHdCon7p2.GetSenData().Fft.R.B[_pt], "Tpit_r_sens");		
		p->addLog(cDHdCon7p2.GetCoVData().Ank.R.B[_pt], "pit_r_conval");	
		p->addLog(dConR[_pt] , 							"pit_r_conval");	
			
		p->addLog(cDHdCon7p2.GetPGData().Fft.L.B[_pt] , "Tpit_l_pg");	
		p->addLog(cDHdCon7p2.GetRefData().Fft.L.B[_pt], "Tpit_l_ref");		
		p->addLog(cDHdCon7p2.GetSenData().Fft.L.B[_pt], "Tpit_l_sens");		
		p->addLog(cDHdCon7p2.GetCoVData().Ank.L.B[_pt], "pit_l_conval");	
		p->addLog(dConL[_pt] , 							"pit_l_conval");		

		p->addLog(cDHdCon7p2.GetSenData().Base[_pt]	, 	"sen_pit");	
		p->addLog(cDHdCon7p2.GetSenData().Base[_rl]	, 	"sen_rol");	
		p->addLog(cDHdCon7p2.GetSenData().dBase[_pt] , 	"sen_dpit");	
		p->addLog(cDHdCon7p2.GetSenData().dBase[_rl] , 	"sen_drol");	
        p->addLog(cDHdCon7p2.GetCoVData().Base[_pt]	, 	"Con_pit");	
        p->addLog(cDHdCon7p2.GetCoVData().dBase[_pt] , 	"Con_dpit");	

        p->addLog(cDHdCon7p2.GetErrData().Base[__x] , 	"ComErr_x");	
		p->addLog(cDHdCon7p2.GetErrData().Base[__y] , 	"ComErr_y");	

		p->addLog(cDHdCon7p2.GetMStabLimitData()[2]	, 	"MLimit_pit");
		p->addLog(cDHdCon7p2.GetMStabLimitData()[3]	, 	"MLimit_pit");
		p->addLog(cDHdCon7p2.GetMStabData()[_pt] , 		"Mstab_pit");	
		p->addLog(cDHdCon7p2.GetMFeetData()[_pt] , 		"Mfeet_pit");	
		p->addLog(cDHdCon7p2.GetMdZMPData()[_pt] , 		"Mmdzmp_pit");	
		p->addLog(cDHdCon7p2.GetMPendData()[_pt] , 		"Mpend_pit");	
		p->addLog(cDHdCon7p2.GetMWheelData()[_pt] , 	"Mwheel_pit");	
		p->addLog(cDHdCon7p2.GetMStabLimitData()[0], 	"MLimit_rol");
		p->addLog(cDHdCon7p2.GetMStabLimitData()[1], 	"MLimit_rol");
		p->addLog(cDHdCon7p2.GetMStabData()[_rl] , 		"Mstab_rol");	
		p->addLog(cDHdCon7p2.GetMFeetData()[_rl] , 		"Mfeet_rol");	
		p->addLog(cDHdCon7p2.GetMdZMPData()[_rl] , 		"Mmdzmp_rol");	
		p->addLog(cDHdCon7p2.GetMPendData()[_rl] , 		"Mpend_rol");	
		p->addLog(cDHdCon7p2.GetMWheelData()[_rl] , 	"Mwheel_rol");	

        p->addLog(cDHdCon7p2.GetSenData().ZMP[__x] , 	"ZMP_x");	
        p->addLog(cDHdCon7p2.GetSenData().ZMP[__y] , 	"ZMP_y");	

        p->addLog(dPr[0] , 								"kfr_L");
        p->addLog(dPr[1] , 								"kfr_R");
        p->addLog(dPr[2] , 								"kpr_L");
        p->addLog(dPr[3] , 								"kpr_R");
        
        p->addLog(cDHdCon7p2.GetPGData().Ank.L.W[__x] , "Ank_Lx");
        p->addLog(cDHdCon7p2.GetPGData().Ank.R.W[__x] , "Ank_Rx");
        p->addLog(cDHdCon7p2.GetPGData().Ank.L.W[__y] , "Ank_Ly");
        p->addLog(cDHdCon7p2.GetPGData().Ank.R.W[__y] , "Ank_Ry");
        p->addLog(cDHdCon7p2.GetPGData().Ank.L.W[__z] , "Ank_Lz");
        p->addLog(cDHdCon7p2.GetPGData().Ank.R.W[__z] , "Ank_Rz");

        p->addLog(cDHdCon7p2.GetPGData().LegQ.L[2] ,	"LegQL3");
		p->addLog(cDHdCon7p2.GetPGData().LegQ.R[2] ,	"LegQR3");
        p->addLog(cDHdCon7p2.GetPGData().LegdQ.L[2] ,	"LegdQL3");
		p->addLog(cDHdCon7p2.GetPGData().LegdQ.R[2] ,	"LegdQR3");
        p->addLog(cDHdCon7p2.GetPGData().LegQ.L[3] ,	"LegQL4");
		p->addLog(cDHdCon7p2.GetPGData().LegQ.R[3] ,	"LegQR4");
        p->addLog(cDHdCon7p2.GetPGData().LegdQ.L[3] ,	"LegdQL4");
		p->addLog(cDHdCon7p2.GetPGData().LegdQ.R[3] ,	"LegdQR4");
        p->addLog(stDHdConIo.ArmQcmd[0] , 					"La");
		p->addLog(stDHdConIo.ArmQcmd[1] , 					"Ra");
        
        p->addLog(stDHdConIo.SupSignal , 				"SupSig");
		#else 
		p->addLog(Out.dBasePosCmd[0], "ModBodyPosX" );
		p->addLog(Out.dBasePosCmd[1], "ModBodyPosY" );
		p->addLog(Out.dBasePosCmd[2], "ModBodyPosZ" );
		p->addLog(Out.dBaseRotCmd[0], "ModBodyAngX" );
		p->addLog(Out.dBaseRotCmd[1], "ModBodyAngY" );
		p->addLog(Out.dBaseRotCmd[2], "ModBodyAngZ" );
		p->addLog(Out.dLAnkPosCmd[0], "ModFootPosLX");
		p->addLog(Out.dLAnkPosCmd[1], "ModFootPosLY");
		p->addLog(Out.dLAnkPosCmd[2], "ModFootPosLZ");
		p->addLog(Out.dLAnkRotCmd[0], "ModFootAngLX");
		p->addLog(Out.dLAnkRotCmd[1], "ModFootAngLY");
		p->addLog(Out.dLAnkRotCmd[2], "ModFootAngLZ");
		p->addLog(Out.dRAnkPosCmd[0], "ModFootPosRX");
		p->addLog(Out.dRAnkPosCmd[1], "ModFootPosRY");
		p->addLog(Out.dRAnkPosCmd[2], "ModFootPosRZ");
		p->addLog(Out.dRAnkRotCmd[0], "ModFootAngRX");
		p->addLog(Out.dRAnkRotCmd[1], "ModFootAngRY");
		p->addLog(Out.dRAnkRotCmd[2], "ModFootAngRZ");
		p->addLog( stStateRef.FootFT.Rfoot.tx, "RefRpit");
		p->addLog(stStateSens.FootFT.Rfoot.tx, "SensRpit");
		p->addLog( stStateRef.FootFT.Lfoot.tx, "RefLpit");
		p->addLog(stStateSens.FootFT.Lfoot.tx, "SensLpit");
		p->addLog( stStateRef.FootFT.Rfoot.ty, "RefRrol");
		p->addLog(stStateSens.FootFT.Rfoot.ty, "SensRrol");
		p->addLog( stStateRef.FootFT.Lfoot.ty, "RefLrol");
		p->addLog(stStateSens.FootFT.Lfoot.ty, "SensLrol");
		p->addLog( stStateRef.FootFT.Rfoot.fz, "RefRZ");
		p->addLog(stStateSens.FootFT.Rfoot.fz, "SensRZ");
		p->addLog( stStateRef.FootFT.Lfoot.fz, "RefLZ");
		p->addLog(stStateSens.FootFT.Lfoot.fz, "SensLZ");

		p->addLog(stStateConVal.Ankle.B.Rfoot.rot.pit, "ConRPitch");
		p->addLog(stStateConVal.Ankle.B.Rfoot.rot.rol, "ConRRoll");
		
		p->addLog(dBaseErr.x, "CoMErrX");
		p->addLog(dBaseErr.y, "CoMErrY");
		
		p->addLog(MdzmpConVal.pos.x, "MdZMPx");
		p->addLog(MdzmpConVal.pos.y, "MdZMPy");
		p->addLog(MdzmpConVal.rot.pit, "MdZMPpit");
		p->addLog(MdzmpConVal.rot.rol, "MdZMProl");
		
		p->addLog(MpitLimit[0], "MpitLimit_");
		p->addLog(MpitLimit[1], "MpitLimit");
		p->addLog(MrolLimit[0], "MrolLimit_");
		p->addLog(MrolLimit[1], "MrolLimit");

		p->addLog(Mstab.pit, "Mstab_pit");
		p->addLog(Mmdzmp.pit, "Mmdzmp_pit");
		p->addLog(Mpend.pit, "Mpend_pit");
		p->addLog(Mwheel.pit, "Mwheel_pit");
		p->addLog(Mfeet.pit, "Mfeet_pit");
		p->addLog(Mstab.rol, "Mstab_rol");
		p->addLog(Mmdzmp.rol, "Mmdzmp_rol");
		p->addLog(Mpend.rol, "Mpend_rol");
		p->addLog(Mwheel.rol, "Mwheel_rol");
		p->addLog(Mfeet.rol, "Mfeet_rol");
		
		// dccarm ======================================
		p->addLog(stJoints.Lq[2], "Lq3");
		p->addLog(stJoints.Rq[2], "Rq3");
		p->addLog(stJoints.Lq[3], "Lq4");
		p->addLog(stJoints.Rq[3], "Rq4");
		p->addLog(stJoints.Ldq[2], "Ldq3");
		p->addLog(stJoints.Rdq[2], "Rdq3");
		p->addLog(stJoints.Ldq[3], "Ldq4");
		p->addLog(stJoints.Rdq[3], "Rdq4");
		p->addLog(stJoints.La, 	   "La");
		p->addLog(stJoints.Ra, 	   "Ra");
		// dccarm ======================================
		#endif

        return 0;
    }

    int Block::print()
    {
		using namespace ljh::tools;
		switch (this->GUIFlag)
		{
		case GUIStateFlag::GUI_OFF :
			std::cout<<"[DCC Control]" <<std::endl
			<< "Torque Pitch ConVal : " <<  stStateConVal.Ankle.B.Lfoot.rot.pit <<", "<<  stStateConVal.Ankle.B.Rfoot.rot.pit << std::endl
			// << "Torque Pitch Sens: " << stStateSens.FootFT.Lfoot.tx <<", "<< stStateSens.FootFT.Rfoot.tx
			<< std::endl; 
			break;
		case GUIStateFlag::GUI_ON :
			ImGui::Begin("DCC Control");
			if (ImGui::BeginTable("DCC-Control", 2))
        	{
        	    ImU32 row_bg_color_header = ImGui::GetColorU32(ImVec4(((float)51)/255, ((float)102)/255, ((float)0.)/255, 0.65f));
        	    ImU32 row_bg_color = ImGui::GetColorU32(ImVec4(((float)0)/255, ((float)102)/255, ((float)51)/255, 0.65f));
        	    //ImU32 row_bg_color_1 = ImGui::GetColorU32(ImVec4(((float)0.)/255, ((float)51)/255, ((float)102)/255, 0.65f));
        	    ImGui::TableNextRow();
        	    ImGui::TableSetBgColor(ImGuiTableBgTarget_RowBg0 + 1, row_bg_color_header);
        	    ImGui::TableNextColumn();
        	    ImGui::Text("Left Torque Pitch");
        	    ImGui::TableNextColumn();
        	    ImGui::Text("Right Torque Pitch");

        	    ImGui::TableNextRow();
        	    ImGui::TableSetBgColor(ImGuiTableBgTarget_RowBg0 + 1, row_bg_color);
        	    ImGui::TableNextColumn();
        	    ImGui::Text("%.3f",stStateConVal.Ankle.B.Lfoot.rot.pit);
        	    ImGui::TableNextColumn();
        	    ImGui::Text("%.3f",stStateConVal.Ankle.B.Rfoot.rot.pit);

				ImGui::TableNextRow();
        	    ImGui::TableSetBgColor(ImGuiTableBgTarget_RowBg0 + 1, row_bg_color_header);
        	    ImGui::TableNextColumn();
        	    ImGui::Text("Left Torque Roll");
        	    ImGui::TableNextColumn();
        	    ImGui::Text("Right Torque Roll");

        	    ImGui::TableNextRow();
        	    ImGui::TableSetBgColor(ImGuiTableBgTarget_RowBg0 + 1, row_bg_color);
        	    ImGui::TableNextColumn();
        	    ImGui::Text("%.3f",stStateConVal.Ankle.B.Lfoot.rot.rol);
        	    ImGui::TableNextColumn();
        	    ImGui::Text("%.3f",stStateConVal.Ankle.B.Rfoot.rot.rol);

				ImGui::TableNextRow();
        	    ImGui::TableSetBgColor(ImGuiTableBgTarget_RowBg0 + 1, row_bg_color_header);
        	    ImGui::TableNextColumn();
        	    ImGui::Text("Control Start Flag");
				ImGui::TableNextColumn();
        	    ImGui::Text("RotSen0");
        	    
				ImGui::TableNextRow();
        	    ImGui::TableSetBgColor(ImGuiTableBgTarget_RowBg0 + 1, row_bg_color);
        	    ImGui::TableNextColumn();
        	    ImGui::Text("%d",*this->getInput().pControlStartFlag);
				ImGui::TableNextColumn();
        	    ImGui::Text("%.3f",this->getInput().dBaseRotSens[0]);

				ImGui::TableNextRow();
        	    ImGui::TableSetBgColor(ImGuiTableBgTarget_RowBg0 + 1, row_bg_color_header);
        	    ImGui::TableNextColumn();
        	    ImGui::Text("RotSen1");
				ImGui::TableNextColumn();
        	    ImGui::Text("RotSen2");

				ImGui::TableNextRow();
        	    ImGui::TableSetBgColor(ImGuiTableBgTarget_RowBg0 + 1, row_bg_color);
        	    ImGui::TableNextColumn();
        	    ImGui::Text("%.3f",this->getInput().dBaseRotSens[1]);
				ImGui::TableNextColumn();
        	    ImGui::Text("%.3f",this->getInput().dBaseRotSens[2]);

				ImGui::EndTable();

				// static lee::gui::LRtPlot Plot("", true, 5.0);
				// if(Plot.begin({-1,-1}))
				// {
				// 	Plot.add(ImGui::GetTime(), stStateSens.ZMP.B.x, "x");
				// 	Plot.add(ImGui::GetTime(), stStateSens.ZMP.B.y, "y");
				// 	Plot.end();
				// }

				static lee::gui::LRtPlot Plot2("", true, 5.0);
				if(Plot2.begin({-1,-1}))
				{
					Plot2.add(ImGui::GetTime(), stStateSens.Base.rot.pit, "pit");
					Plot2.add(ImGui::GetTime(), stStateSens.Base.rot.rol, "rol");
					Plot2.end();
				}

			}
			ImGui::End();
			break;
		default:
			break;
		}
		
        return 0;
    }
}
}