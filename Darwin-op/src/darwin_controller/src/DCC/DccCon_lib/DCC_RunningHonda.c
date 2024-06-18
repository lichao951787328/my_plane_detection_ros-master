// For running project of BHR7 testing Honda frame
// 20210708 bit

#ifndef DCC_RUNNINGHONDA_C
#define DCC_RUNNINGHONDA_C
#include <Compliance/DCC/DccCon_lib/DCC_RunningCon.h>
#include <stdio.h>
// #include <KalmanFilterVision\KalmanFilterVision.h>

double FBMAmp = 1.0; // 1.2;
double dZcHonda = 0.63;
dccRotational Mstab = { 0.0 };
dccRotational Mmdzmp = { 0.0 };
dccRotational Mpend = { 0.0 };
dccRotational Mwheel = { 0.0 };
dccRotational Mfeet = { 0.0 };
double MpitLimit[2] = { 0.0 };
double MrolLimit[2] = { 0.0 };
dccSpacial MdzmpConVal = { 0.0 };
dccForceSensor FootFTstab = { 0.0 };
dccForceSensor FootFTstabTrigered = { 0.0 };
dccForceSensor FootFTpg = { 0.0 };
dccAnkleTwoFrame GRFConVal = { 0.0 };
dccAnkleTwoFrame CompConVal = { 0.0 };
dccAnkleTwoFrame StepConVal = { 0.0 };
// coupled
dccFootFT DelFootFT = { 0.0 };

extern dccSupPoly stSupPoly_B;
double dReHonda[100] = { 0.0 };
double dFzAmp = 1.0; //1.25 stand good
double dTimeLagFz = 0.002;
int nRek = 0;
// stepcon new
int nRSupFlag[2] = { 0 }, nLSupFlag[2] = { 0 }; // [0] new, [1] old
int nKRTakeOff = 0, nKLTakeOff = 0;
double dDelT = 0.0;
dccPositional dSumDelCom = { 0.0 }, dDelCom = { 0.0 }, dDelComT0 = { 0.0 }, dDelComTe = { 0.0 }, dDelCp = { 0.0 };
dccAnkle_tra dDelAnkCpTra = { 0.0 };
double dStepLenNow; // new added
// newnew =========================================================================================================
dccPositional dStepBkConValR = { 0.0 }, dStepBkConValL = { 0.0 }, dStepReR = { 0.0 }, dStepReL = { 0.0 };
dccPositional dStepLast = { 0.0 };
// ================================================================================================================
#define __IUpperHonda __MUpper * 0.4 * 0.4 * 0.083
enum supsignal {
	DouSup, RightSup, LeftSup, Fly
};

void fnvCalMstabLimit(double * dFootGeom_in) { 
	// cal MLimit for feet
	double dMarginFlag = 0.0;
	#ifdef USE_SUPPOSCON
		dMarginFlag = 1.0;
	#endif
	double dMargin[3] = { /*forw*/dMarginFlag * 0.015, /*back*/dMarginFlag * 0.01, /*side*/dMarginFlag * 0.01 };
	double dSupAnkPos[2] = { 0.0 }; // Left, right
	MpitLimit[0] = (-dFootGeom_in[1] + dMargin[1]) * __MRobot * __Gravity; // -
	MpitLimit[1] = (dFootGeom_in[0] - dMargin[0]) * __MRobot * __Gravity; // +
	if (stStatePG.SupLeg == RightSup) dSupAnkPos[0] = dSupAnkPos[1] = stStatePG.Ankle.B.Rfoot.pos.x;
	else if (stStatePG.SupLeg == LeftSup) dSupAnkPos[0] = dSupAnkPos[1] = stStatePG.Ankle.B.Lfoot.pos.x;
	else if (stStatePG.SupLeg == DouSup) {
		dSupAnkPos[0] = stStatePG.Ankle.B.Lfoot.pos.x;
		dSupAnkPos[1] = stStatePG.Ankle.B.Rfoot.pos.x;
	}
	else {
		dSupAnkPos[0] = stSupPoly_B.left;
		dSupAnkPos[1] = stSupPoly_B.righ;
	}
	MrolLimit[0] = (dSupAnkPos[1] - stSupPoly_B.righ - dMargin[2]) * __MRobot * __Gravity; // -
	MrolLimit[1] = (dSupAnkPos[0] - stSupPoly_B.left + dMargin[2]) * __MRobot * __Gravity; // +
	for (int i = 0; i < 2; i++, nRek++) { 
		dReHonda[nRek] = MpitLimit[i];
	}
	for (int i = 0; i < 2; i++, nRek++) {
		dReHonda[nRek] = MrolLimit[i];
	}
	// FrameAddLogsC(789, "Mlimit_pit",  &MpitLimit[0], 1);
	// FrameAddLogsC(789, "Mlimit_pit",  &MpitLimit[1], 1);
	// FrameAddLogsC(789, "Mlimit_rol",  &MrolLimit[0], 1);
	// FrameAddLogsC(789, "Mlimit_rol",  &MrolLimit[1], 1);
}

dccPositional dBaseErr = { 0.0 };
void fnvCalMstab(double dParas[4]) { // Kp_pit, Kd_pit, Kp_rol, Kd_rol
	double Kp_pit = dParas[0], Kd_pit = dParas[1], Kp_rol = dParas[2], Kd_rol = dParas[3];
	double dBaseErr_xTemp, dBaseErr_yTemp;
	// ============================================== Get State =======================================================
	// Traditional
	dBaseErr_xTemp = dZcHonda * sin(stStateSens.Base.rot.rol - stStateRef.Base.rot.rol - stStateConVal.Base.rot.rol);
	dBaseErr_yTemp = -dZcHonda * sin(stStateSens.Base.rot.pit - stStateRef.Base.rot.pit - stStateConVal.Base.rot.pit);
	dBaseErr.dx = dZcHonda * sin(stStateSens.Base.rot.drol - stStateRef.Base.rot.drol - stStateConVal.Base.rot.drol);
	dBaseErr.dy = -dZcHonda * sin(stStateSens.Base.rot.dpit - stStateRef.Base.rot.dpit - stStateConVal.Base.rot.dpit);
	// K-Vision
	// dBaseErr_xTemp = -dZcHonda / 1.15 * getPosHatY();
	// dBaseErr_yTemp = dZcHonda / 1.15 * getPosHatX();
	// dBaseErr.dx = -dZcHonda / 1.15 * getVelHatY();
	// dBaseErr.dy = dZcHonda / 1.15 * getVelHatX();
	// ================================================================================================================
	dBaseErr.x = dBaseErr_xTemp;
	dBaseErr.y = dBaseErr_yTemp;
	// cal Mstab from fbcon of dBaseErr
	Mstab.pit = Kp_pit * dBaseErr.y + Kd_pit * dBaseErr.dy;
	Mstab.rol = FBMAmp * (-Kp_rol * dBaseErr.x - Kd_rol * dBaseErr.dx);
	for (int i = 0; i < 2; i++, nRek++) {
		dReHonda[nRek] = *(&dBaseErr.x + i);
	}
	for (int i = 0; i < 2; i++, nRek++) {
		dReHonda[nRek] = *(&dBaseErr.dx + i);
	}
	for (int i = 0; i < 2; i++, nRek++) {
		dReHonda[nRek] = *(&Mstab.pit + i);
	}
	// FrameAddLogsC(788, "com_err_x",  &dBaseErr.x, 1);
	// FrameAddLogsC(788, "com_err_y",  &dBaseErr.y, 1);
	// FrameAddLogsC(788, "com_err_dx",  &dBaseErr.dx, 1);
	// FrameAddLogsC(788, "com_err_dy",  &dBaseErr.dy, 1);
	// FrameAddLogsC(788, "pit_ref",  &stStateRef.Base.rot.pit, 1);
	// FrameAddLogsC(788, "pit_sens",  &stStateSens.Base.rot.pit, 1);
	// FrameAddLogsC(788, "rol_ref",  &stStateRef.Base.rot.rol, 1);
	// FrameAddLogsC(788, "rol_sens",  &stStateSens.Base.rot.rol, 1);
}

void fnvMdzmp(double dParas[4], double dLagTsplit) { // Kp_pos, Kd_pos, Kp_rot, Kd_rot
	double Kp_pos = dParas[0], Kd_pos = dParas[1], Kp_rot = dParas[2], Kd_rot = dParas[3];
	double dLimit_pos[6] = { -0.03, 0.03, -10.0, 10.0, -2500.0, 2500.0 };
	double dLimit_rot[6] = { -0.03 * 6.0, 0.03 * 6.0, -10.0 * 6.0, 10.0 * 6.0, -2500.0 * 6.0, 2500.0 * 6.0 };
	// cal Mfeet & Mmdzmp from Mstab
	Mfeet.pit = fndAddLimit(Mstab.pit, 0.0, &Mmdzmp.pit, MpitLimit);
	Mfeet.rol = fndAddLimit(Mstab.rol, 0.0, &Mmdzmp.rol, MrolLimit);
	FootFTstab.tx = Mfeet.pit;
	FootFTstab.ty = Mfeet.rol;
	FootFTstab.fz = Mfeet.rol / __AnkleWidth;
	// cal Mpend & Mwheel by spliting Mmdzmp
	Mpend.pit = fndFilterTimeLag(Mpend.pit, Mmdzmp.pit, __ControlT, dLagTsplit);
	Mpend.rol = fndFilterTimeLag(Mpend.rol, Mmdzmp.rol, __ControlT, dLagTsplit);
	Mwheel.pit = Mmdzmp.pit - Mpend.pit;
	Mwheel.rol = Mmdzmp.rol - Mpend.rol;
	// cal MdzmpConVal & fbcon
	MdzmpConVal.pos.ddx = -Mpend.rol / __MUpper / dZcHonda - Kp_pos * MdzmpConVal.pos.x - Kd_pos * MdzmpConVal.pos.dx;
	MdzmpConVal.pos.ddy = Mpend.pit / __MUpper / dZcHonda - Kp_pos * MdzmpConVal.pos.y - Kd_pos * MdzmpConVal.pos.dy;
	MdzmpConVal.rot.ddpit = -Mwheel.pit / __IUpperHonda - Kp_rot * MdzmpConVal.rot.pit - Kd_rot * MdzmpConVal.rot.dpit;
	MdzmpConVal.rot.ddrol = -Mwheel.rol / __IUpperHonda - Kp_rot * MdzmpConVal.rot.rol - Kd_rot * MdzmpConVal.rot.drol;
	fnvIntegLimit(&MdzmpConVal.pos.x, &MdzmpConVal.pos.dx, MdzmpConVal.pos.ddx, dLimit_pos, __ControlT);
	fnvIntegLimit(&MdzmpConVal.pos.y, &MdzmpConVal.pos.dy, MdzmpConVal.pos.ddy, dLimit_pos, __ControlT);
	fnvIntegLimit(&MdzmpConVal.rot.pit, &MdzmpConVal.rot.dpit, MdzmpConVal.rot.ddpit, dLimit_rot, __ControlT);
	fnvIntegLimit(&MdzmpConVal.rot.rol, &MdzmpConVal.rot.drol, MdzmpConVal.rot.ddrol, dLimit_rot, __ControlT);
#ifdef USE_MDZMP_HONDA
	stStateConVal.Base.pos.x = MdzmpConVal.pos.x;
	stStateConVal.Base.pos.y = MdzmpConVal.pos.y;
	stStateRef.Base.rot.pit += 1.0 * MdzmpConVal.rot.pit;
	stStateRef.Base.rot.rol += 1.0 * MdzmpConVal.rot.rol;
#endif
	for (int i = 0; i < 2; i++, nRek++) {
		dReHonda[nRek] = *(&Mfeet.pit + i);
	}
	for (int i = 0; i < 2; i++, nRek++) {
		dReHonda[nRek] = *(&Mmdzmp.pit + i);
	}
	for (int i = 0; i < 2; i++, nRek++) {
		dReHonda[nRek] = *(&Mpend.pit + i);
	}
	for (int i = 0; i < 2; i++, nRek++) {
		dReHonda[nRek] = *(&Mwheel.pit + i);
	}
	for (int i = 0; i < 2; i++, nRek++) {
		dReHonda[nRek] = *(&MdzmpConVal.pos.x + i);
	}
	for (int i = 0; i < 2; i++, nRek++) {
		dReHonda[nRek] = *(&MdzmpConVal.rot.pit + i);
	}
	// FrameAddLogsC(789, "Mstab_pit",  &Mstab.pit, 1);
	// FrameAddLogsC(789, "Mfeet_pit",  &Mstab.pit, 1);
	// FrameAddLogsC(789, "Mmzmp_pit",  &Mmdzmp.pit, 1);
	// FrameAddLogsC(789, "Mpend_pit",  &Mpend.pit, 1);
	// FrameAddLogsC(789, "Mwhel_pit",  &Mwheel.pit, 1);
	// FrameAddLogsC(789, "Mstab_rol",  &Mstab.rol, 1);
	// FrameAddLogsC(789, "Mfeet_rol",  &Mstab.rol, 1);
	// FrameAddLogsC(789, "Mmzmp_rol",  &Mmdzmp.rol, 1);
	// FrameAddLogsC(789, "Mpend_rol",  &Mpend.rol, 1);
	// FrameAddLogsC(789, "Mwhel_rol",  &Mwheel.rol, 1);
	// FrameAddLogsC(790, "Base_x",  &stStateConVal.Base.pos.x, 1);
	// FrameAddLogsC(790, "Base_y",  &stStateConVal.Base.pos.y, 1);
	// FrameAddLogsC(790, "Base_pit",  &stStateConVal.Base.rot.pit, 1);
	// FrameAddLogsC(790, "Base_rol",  &stStateConVal.Base.rot.rol, 1);	
}

void fnvAddFBMoment(double dFzMin, double dGrtBias[2]) { //dcc 20210728 加fz
	double Limit_alpha[2] = { 0.0, 1.0 };
	double xmidpoint_pg = (stStatePG.ZMP.B.x*stStateCmd.Ankle.B.Lfoot.pos.x*stStateCmd.Ankle.B.Lfoot.pos.x - 2 * stStatePG.ZMP.B.x*stStateCmd.Ankle.B.Lfoot.pos.x*stStateCmd.Ankle.B.Rfoot.pos.x - stStateCmd.Ankle.B.Lfoot.pos.x*stStateCmd.Ankle.B.Lfoot.pos.y*stStateCmd.Ankle.B.Rfoot.pos.y + stStatePG.ZMP.B.y*stStateCmd.Ankle.B.Lfoot.pos.x*stStateCmd.Ankle.B.Lfoot.pos.y + stStateCmd.Ankle.B.Lfoot.pos.x*stStateCmd.Ankle.B.Rfoot.pos.y*stStateCmd.Ankle.B.Rfoot.pos.y - stStatePG.ZMP.B.y*stStateCmd.Ankle.B.Lfoot.pos.x*stStateCmd.Ankle.B.Rfoot.pos.y + stStatePG.ZMP.B.x*stStateCmd.Ankle.B.Rfoot.pos.x*stStateCmd.Ankle.B.Rfoot.pos.x + stStateCmd.Ankle.B.Rfoot.pos.x*stStateCmd.Ankle.B.Lfoot.pos.y*stStateCmd.Ankle.B.Lfoot.pos.y - stStateCmd.Ankle.B.Rfoot.pos.x*stStateCmd.Ankle.B.Lfoot.pos.y*stStateCmd.Ankle.B.Rfoot.pos.y - stStatePG.ZMP.B.y*stStateCmd.Ankle.B.Rfoot.pos.x*stStateCmd.Ankle.B.Lfoot.pos.y + stStatePG.ZMP.B.y*stStateCmd.Ankle.B.Rfoot.pos.x*stStateCmd.Ankle.B.Rfoot.pos.y) / (stStateCmd.Ankle.B.Lfoot.pos.x*stStateCmd.Ankle.B.Lfoot.pos.x - 2 * stStateCmd.Ankle.B.Lfoot.pos.x*stStateCmd.Ankle.B.Rfoot.pos.x + stStateCmd.Ankle.B.Rfoot.pos.x*stStateCmd.Ankle.B.Rfoot.pos.x + stStateCmd.Ankle.B.Lfoot.pos.y*stStateCmd.Ankle.B.Lfoot.pos.y - 2 * stStateCmd.Ankle.B.Lfoot.pos.y*stStateCmd.Ankle.B.Rfoot.pos.y + stStateCmd.Ankle.B.Rfoot.pos.y*stStateCmd.Ankle.B.Rfoot.pos.y);
	double alpha_pg = fndLimit((xmidpoint_pg - stStateCmd.Ankle.B.Lfoot.pos.x) / (stStateCmd.Ankle.B.Rfoot.pos.x - stStateCmd.Ankle.B.Lfoot.pos.x), Limit_alpha); // force propotion to right foot
	double xmidpoint_fb = (stStateSens.ZMP.B.x*stStateCmd.Ankle.B.Lfoot.pos.x*stStateCmd.Ankle.B.Lfoot.pos.x - 2 * stStateSens.ZMP.B.x*stStateCmd.Ankle.B.Lfoot.pos.x*stStateCmd.Ankle.B.Rfoot.pos.x - stStateCmd.Ankle.B.Lfoot.pos.x*stStateCmd.Ankle.B.Lfoot.pos.y*stStateCmd.Ankle.B.Rfoot.pos.y + stStateSens.ZMP.B.y*stStateCmd.Ankle.B.Lfoot.pos.x*stStateCmd.Ankle.B.Lfoot.pos.y + stStateCmd.Ankle.B.Lfoot.pos.x*stStateCmd.Ankle.B.Rfoot.pos.y*stStateCmd.Ankle.B.Rfoot.pos.y - stStateSens.ZMP.B.y*stStateCmd.Ankle.B.Lfoot.pos.x*stStateCmd.Ankle.B.Rfoot.pos.y + stStateSens.ZMP.B.x*stStateCmd.Ankle.B.Rfoot.pos.x*stStateCmd.Ankle.B.Rfoot.pos.x + stStateCmd.Ankle.B.Rfoot.pos.x*stStateCmd.Ankle.B.Lfoot.pos.y*stStateCmd.Ankle.B.Lfoot.pos.y - stStateCmd.Ankle.B.Rfoot.pos.x*stStateCmd.Ankle.B.Lfoot.pos.y*stStateCmd.Ankle.B.Rfoot.pos.y - stStateSens.ZMP.B.y*stStateCmd.Ankle.B.Rfoot.pos.x*stStateCmd.Ankle.B.Lfoot.pos.y + stStateSens.ZMP.B.y*stStateCmd.Ankle.B.Rfoot.pos.x*stStateCmd.Ankle.B.Rfoot.pos.y) / (stStateCmd.Ankle.B.Lfoot.pos.x*stStateCmd.Ankle.B.Lfoot.pos.x - 2 * stStateCmd.Ankle.B.Lfoot.pos.x*stStateCmd.Ankle.B.Rfoot.pos.x + stStateCmd.Ankle.B.Rfoot.pos.x*stStateCmd.Ankle.B.Rfoot.pos.x + stStateCmd.Ankle.B.Lfoot.pos.y*stStateCmd.Ankle.B.Lfoot.pos.y - 2 * stStateCmd.Ankle.B.Lfoot.pos.y*stStateCmd.Ankle.B.Rfoot.pos.y + stStateCmd.Ankle.B.Rfoot.pos.y*stStateCmd.Ankle.B.Rfoot.pos.y);
	double alpha_fb = fndLimit((xmidpoint_fb - stStateCmd.Ankle.B.Lfoot.pos.x) / (stStateCmd.Ankle.B.Rfoot.pos.x - stStateCmd.Ankle.B.Lfoot.pos.x), Limit_alpha); // force propotion to right foot
	double dFbFlag = 0.0;
#ifdef USE_FBMOMENT_HONDA
	dFbFlag = 1.0;
#endif
	for (int i = 0; i < 6; i++) {
		*(&stStateRef.FootFT.Rfoot.fx + i) = 0.0;
		*(&stStateRef.FootFT.Lfoot.fx + i) = 0.0;
	}
	stStateRef.FootFT.Rfoot.fz = stStatePG.FootFT.Rfoot.fz; // alpha_pg * __MRobot * (__Gravity + stStatePG.Base.pos.ddz);
	stStateRef.FootFT.Lfoot.fz = stStatePG.FootFT.Lfoot.fz; // (1 - alpha_pg) * __MRobot * (__Gravity + stStatePG.Base.pos.ddz);

	if (stStateSens.FootFT.Fsum >= dFzMin) { // touch down
		if (stStatePG.SupLeg == DouSup) FootFTstabTrigered.fz = fndFilterTimeLag(FootFTstabTrigered.fz, dFzAmp * FootFTstab.fz, __ControlT, dTimeLagFz);
		else FootFTstabTrigered.fz = fndFilterTimeLag(FootFTstabTrigered.fz, 0.0, __ControlT, dTimeLagFz);
		stStateRef.FootFT.Rfoot.fz -= FootFTstabTrigered.fz; // feedback z
		stStateRef.FootFT.Lfoot.fz += FootFTstabTrigered.fz; // feedback z
		if (stStatePG.SupLeg == DouSup) {
			if (stStateSens.FootFT.Rfoot.fz >= 0.5 * dFzMin) {
				stStateRef.FootFT.Rfoot.tx = stStatePG.FootFT.Rfoot.tx + dFbFlag * alpha_fb * FootFTstab.tx + 0.5 * dGrtBias[0];
				stStateRef.FootFT.Rfoot.ty = stStatePG.FootFT.Rfoot.ty + dFbFlag * alpha_fb * FootFTstab.ty;
			}
			if (stStateSens.FootFT.Lfoot.fz >= 0.5 * dFzMin) {
				stStateRef.FootFT.Lfoot.tx = stStatePG.FootFT.Lfoot.tx + dFbFlag * (1 - alpha_fb) * FootFTstab.tx + 0.5 * dGrtBias[0];
				stStateRef.FootFT.Lfoot.ty = stStatePG.FootFT.Lfoot.ty + dFbFlag * (1 - alpha_fb) * FootFTstab.ty;
			}
		}
		else if (stStatePG.SupLeg == RightSup) {
			if (stStateSens.FootFT.Rfoot.fz >= 0.5 * dFzMin) {
				stStateRef.FootFT.Rfoot.tx = stStatePG.FootFT.Rfoot.tx + dFbFlag * alpha_fb * FootFTstab.tx + dGrtBias[0];
				stStateRef.FootFT.Rfoot.ty = stStatePG.FootFT.Rfoot.ty + dFbFlag * alpha_fb * FootFTstab.ty + dGrtBias[1];
			}
		}
		else if (stStatePG.SupLeg == LeftSup) {
			if (stStateSens.FootFT.Lfoot.fz >= 0.5 * dFzMin) {
				stStateRef.FootFT.Lfoot.tx = stStatePG.FootFT.Lfoot.tx + dFbFlag * (1 - alpha_fb) * FootFTstab.tx + dGrtBias[0];
				stStateRef.FootFT.Lfoot.ty = stStatePG.FootFT.Lfoot.ty + dFbFlag * (1 - alpha_fb) * FootFTstab.ty - dGrtBias[1];
			}
		}
		else { // fly
			stStateRef.FootFT.Rfoot.fz = 0.0;
			stStateRef.FootFT.Lfoot.fz = 0.0;
		}
	}
	for (int i = 0; i < 6; i++, nRek++) {
		dReHonda[nRek] = *(&stStateRef.FootFT.Rfoot.fx + i);
	}
	for (int i = 0; i < 6; i++, nRek++) {
		dReHonda[nRek] = *(&stStateRef.FootFT.Lfoot.fx + i);
	}
	for (int i = 0; i < 6; i++, nRek++) {
		dReHonda[nRek] = *(&stStateSens.FootFT.Rfoot.fx + i);
	}
	for (int i = 0; i < 6; i++, nRek++) {
		dReHonda[nRek] = *(&stStateSens.FootFT.Lfoot.fx + i);
	}
}

void fnvGRFC(double dParas[9]) { // Kf_z, Kp_z, Kd_z, Kc_z, Kdf_z, Kdfc_z, Kf_r, Kp_r
	double Kf_z = dParas[0], Kp_z = dParas[1], Kd_z = dParas[2], Kvc_z = dParas[3], Kac_z = dParas[4], Kdf_z = dParas[5], Kdfc_z = dParas[6], Kf_r = dParas[7], Kp_r = dParas[8];
	double dLimit_z[4] = { -0.0 * 0.02, 0.04, -10.0, 10.0 }, dLimit_z2nd[6] = { -0.0 * 0.02, 0.04, -10.0, 10.0, -2500.0, 2500.0 }, dLimit_r[4] = { -DccD2R(15.0), DccD2R(15.0), -10.0 * 6.0, 10.0 * 6.0 };
	double dThresh_z[2] = { -10.0, 10.0 }, dThresh_r[2] = { -1.0, 1.0};
#ifdef USE_2ND_ORDER
	dccFootFT dDelFootFT = { 0.0 };
	dDelFootFT.Rfoot.fz = ((stStateSens.FootFT.Rfoot.fz - stStateRef.FootFT.Rfoot.fz) - DelFootFT.Rfoot.fz) / __ControlT;
	dDelFootFT.Lfoot.fz = ((stStateSens.FootFT.Lfoot.fz - stStateRef.FootFT.Lfoot.fz) - DelFootFT.Lfoot.fz) / __ControlT;
	GRFConVal.B.Rfoot.pos.ddz = Kf_z * fndThreshold((stStateSens.FootFT.Rfoot.fz - stStateRef.FootFT.Rfoot.fz), dThresh_z) - Kp_z * GRFConVal.B.Rfoot.pos.z - Kd_z * GRFConVal.B.Rfoot.pos.dz + Kvc_z * GRFConVal.B.Lfoot.pos.dz + Kac_z * GRFConVal.B.Lfoot.pos.ddz + Kdf_z * dDelFootFT.Rfoot.fz + Kdfc_z * dDelFootFT.Lfoot.fz;
	GRFConVal.B.Lfoot.pos.ddz = Kf_z * fndThreshold((stStateSens.FootFT.Lfoot.fz - stStateRef.FootFT.Lfoot.fz), dThresh_z) - Kp_z * GRFConVal.B.Lfoot.pos.z - Kd_z * GRFConVal.B.Lfoot.pos.dz + Kvc_z * GRFConVal.B.Rfoot.pos.dz + Kac_z * GRFConVal.B.Rfoot.pos.ddz + Kdf_z * dDelFootFT.Lfoot.fz + Kdfc_z * dDelFootFT.Rfoot.fz;
	DelFootFT.Rfoot.fz = stStateSens.FootFT.Rfoot.fz - stStateRef.FootFT.Rfoot.fz;
	DelFootFT.Lfoot.fz = stStateSens.FootFT.Lfoot.fz - stStateRef.FootFT.Lfoot.fz;
	printf("DelFootFT.Lfoot.fz:....%f\n", DelFootFT.Lfoot.fz);
#endif
#ifdef USE_1ST_ORDER
	GRFConVal.B.Rfoot.pos.dz = Kf_z * fndThreshold((stStateSens.FootFT.Rfoot.fz - stStateRef.FootFT.Rfoot.fz), dThresh_z) - Kp_z * GRFConVal.B.Rfoot.pos.z; // one order
	GRFConVal.B.Lfoot.pos.dz = Kf_z * fndThreshold((stStateSens.FootFT.Lfoot.fz - stStateRef.FootFT.Lfoot.fz), dThresh_z) - Kp_z * GRFConVal.B.Lfoot.pos.z; // one order
#endif
	GRFConVal.B.Rfoot.rot.dpit = Kf_r * fndThreshold((stStateSens.FootFT.Rfoot.tx - stStateRef.FootFT.Rfoot.tx), dThresh_r) - Kp_r * GRFConVal.B.Rfoot.rot.pit;
	GRFConVal.B.Lfoot.rot.dpit = Kf_r * fndThreshold((stStateSens.FootFT.Lfoot.tx - stStateRef.FootFT.Lfoot.tx), dThresh_r) - Kp_r * GRFConVal.B.Lfoot.rot.pit;
	GRFConVal.B.Rfoot.rot.drol = Kf_r * fndThreshold((stStateSens.FootFT.Rfoot.ty - stStateRef.FootFT.Rfoot.ty), dThresh_r) - Kp_r * GRFConVal.B.Rfoot.rot.rol;
	GRFConVal.B.Lfoot.rot.drol = Kf_r * fndThreshold((stStateSens.FootFT.Lfoot.ty - stStateRef.FootFT.Lfoot.ty), dThresh_r) - Kp_r * GRFConVal.B.Lfoot.rot.rol;
#ifdef USE_2ND_ORDER
	fnvIntegLimit(&GRFConVal.B.Rfoot.pos.z, &GRFConVal.B.Rfoot.pos.dz, GRFConVal.B.Rfoot.pos.ddz, dLimit_z2nd, __ControlT); // second order
	fnvIntegLimit(&GRFConVal.B.Lfoot.pos.z, &GRFConVal.B.Lfoot.pos.dz, GRFConVal.B.Lfoot.pos.ddz, dLimit_z2nd, __ControlT); // second order
#endif
#ifdef USE_1ST_ORDER
	fnvVeloLimit(&GRFConVal.B.Rfoot.pos.z, GRFConVal.B.Rfoot.pos.dz, dLimit_z, __ControlT);	// one order
	fnvVeloLimit(&GRFConVal.B.Lfoot.pos.z, GRFConVal.B.Lfoot.pos.dz, dLimit_z, __ControlT); // one order
#endif
	fnvVeloLimit(&GRFConVal.B.Rfoot.rot.pit, GRFConVal.B.Rfoot.rot.dpit, dLimit_r, __ControlT);
	fnvVeloLimit(&GRFConVal.B.Lfoot.rot.pit, GRFConVal.B.Lfoot.rot.dpit, dLimit_r, __ControlT);
	fnvVeloLimit(&GRFConVal.B.Rfoot.rot.rol, GRFConVal.B.Rfoot.rot.drol, dLimit_r, __ControlT);
	fnvVeloLimit(&GRFConVal.B.Lfoot.rot.rol, GRFConVal.B.Lfoot.rot.drol, dLimit_r, __ControlT);
#ifdef USE_POSTURESTABLIZE_HONDA
	stStateConVal.Ankle.B.Rfoot.pos.z = GRFConVal.B.Rfoot.pos.z;
	stStateConVal.Ankle.B.Lfoot.pos.z = GRFConVal.B.Lfoot.pos.z;
	stStateConVal.Ankle.B.Rfoot.rot.pit = GRFConVal.B.Rfoot.rot.pit;
	stStateConVal.Ankle.B.Lfoot.rot.pit = GRFConVal.B.Lfoot.rot.pit;
	stStateConVal.Ankle.B.Rfoot.rot.rol = GRFConVal.B.Rfoot.rot.rol;
	stStateConVal.Ankle.B.Lfoot.rot.rol = GRFConVal.B.Lfoot.rot.rol;
#endif
	for (int i = 0; i < 1; i++, nRek++) {
		dReHonda[nRek] = *(&GRFConVal.B.Rfoot.pos.z + i);
	}
	for (int i = 0; i < 1; i++, nRek++) {
		dReHonda[nRek] = *(&GRFConVal.B.Lfoot.pos.z + i);
	}
	for (int i = 0; i < 2; i++, nRek++) {
		dReHonda[nRek] = *(&GRFConVal.B.Rfoot.rot.pit + i);
	}
	for (int i = 0; i < 2; i++, nRek++) {
		dReHonda[nRek] = *(&GRFConVal.B.Lfoot.rot.pit + i);
	}
	// z
	// FrameAddLogsC(799, "Fz_r_ref",  &stStateRef.FootFT.Rfoot.fz, 1);
	// FrameAddLogsC(799, "Fz_r_sens",  &stStateSens.FootFT.Rfoot.fz, 1);
	// FrameAddLogsC(799, "z_r_conval",  &stStateConVal.Ankle.B.Rfoot.pos.z, 1);
	// FrameAddLogsC(799, "Fz_l_ref",  &stStateRef.FootFT.Lfoot.fz, 1);
	// FrameAddLogsC(799, "Fz_l_sens",  &stStateSens.FootFT.Lfoot.fz, 1);
	// FrameAddLogsC(799, "z_l_conval",  &stStateConVal.Ankle.B.Lfoot.pos.z, 1);
	// // rol
	// FrameAddLogsC(799, "Trol_r_ref",  &stStateRef.FootFT.Rfoot.ty, 1);
	// FrameAddLogsC(799, "Trol_r_sens",  &stStateSens.FootFT.Rfoot.ty, 1);
	// FrameAddLogsC(799, "rol_r_conval",  &stStateConVal.Ankle.B.Rfoot.rot.rol, 1);
	// FrameAddLogsC(799, "Trol_l_ref",  &stStateRef.FootFT.Lfoot.ty, 1);
	// FrameAddLogsC(799, "Trol_l_sens",  &stStateSens.FootFT.Lfoot.ty, 1);
	// FrameAddLogsC(799, "rol_l_conval",  &stStateConVal.Ankle.B.Lfoot.rot.rol, 1);
	// // pit
	// FrameAddLogsC(799, "Tpit_r_ref",  &stStateRef.FootFT.Rfoot.tx, 1);
	// FrameAddLogsC(799, "Tpit_r_sens",  &stStateSens.FootFT.Rfoot.tx, 1);
	// FrameAddLogsC(799, "pit_r_conval",  &stStateConVal.Ankle.B.Rfoot.rot.pit, 1);
	// FrameAddLogsC(799, "Tpit_l_ref",  &stStateRef.FootFT.Lfoot.tx, 1);
	// FrameAddLogsC(799, "Tpit_l_sens",  &stStateSens.FootFT.Lfoot.tx, 1);
	// FrameAddLogsC(799, "pit_l_conval",  &stStateConVal.Ankle.B.Lfoot.rot.pit, 1);
}

void fnvCompliance(double dParas[2]) {
	double Kf = dParas[0], Kp = dParas[1];
	double dLimit[4] = { -DccD2R(15.0), DccD2R(15.0), -10.0 * 6.0, 10.0 * 6.0 };
	double dThresh[4] = { -0.0, 0.0 };
	CompConVal.B.Rfoot.rot.dpit = Kf * fndThreshold((stStateSens.FootFT.Rfoot.tx - 0.0 * stStateRef.FootFT.Rfoot.tx), dThresh) - Kp * CompConVal.B.Rfoot.rot.pit;
	CompConVal.B.Lfoot.rot.dpit = Kf * fndThreshold((stStateSens.FootFT.Lfoot.tx - 0.0 * stStateRef.FootFT.Lfoot.tx), dThresh) - Kp * CompConVal.B.Lfoot.rot.pit;
	CompConVal.B.Rfoot.rot.drol = Kf * fndThreshold((stStateSens.FootFT.Rfoot.ty - 0.0 * stStateRef.FootFT.Rfoot.ty), dThresh) - Kp * CompConVal.B.Rfoot.rot.rol;
	CompConVal.B.Lfoot.rot.drol = Kf * fndThreshold((stStateSens.FootFT.Lfoot.ty - 0.0 * stStateRef.FootFT.Lfoot.ty), dThresh) - Kp * CompConVal.B.Lfoot.rot.rol;
	fnvVeloLimit(&CompConVal.B.Rfoot.rot.pit, CompConVal.B.Rfoot.rot.dpit, dLimit, __ControlT);
	fnvVeloLimit(&CompConVal.B.Lfoot.rot.pit, CompConVal.B.Lfoot.rot.dpit, dLimit, __ControlT);
	fnvVeloLimit(&CompConVal.B.Rfoot.rot.rol, CompConVal.B.Rfoot.rot.drol, dLimit, __ControlT);
	fnvVeloLimit(&CompConVal.B.Lfoot.rot.rol, CompConVal.B.Lfoot.rot.drol, dLimit, __ControlT);
#ifdef USE_COMPLIANCE_HONDA
	stStateConVal.Ankle.B.Rfoot.rot.pit += CompConVal.B.Rfoot.rot.pit;
	stStateConVal.Ankle.B.Lfoot.rot.pit += CompConVal.B.Lfoot.rot.pit;
	stStateConVal.Ankle.B.Rfoot.rot.rol += CompConVal.B.Rfoot.rot.rol;
	stStateConVal.Ankle.B.Lfoot.rot.rol += CompConVal.B.Lfoot.rot.rol;
#endif
}

void fnvStepConNew(double dTswiTsup[2], double dStepLen) {
	// #define USE_SPLINE_BK
	#define USE_FBCON_BK
	// double dZRTempRe, dZLTempRe;
	double dAmpx = 0.0, dAmpy = 0.8, dAmpz = 0.8, dSpeedAmp = 1.0; //dAmpx = 0.5, dAmpy = 0.2, dAmpz = 1.0, dSpeedAmp = 0.8; 
	double dCpLimit[4] = { /*x*/ -0.05, 0.05, /*y*/ -0.05, 0.05 };
	double dLimitRAnkX[2] = { 0.0, dCpLimit[1] }, dLimitLAnkX[2] = { dCpLimit[0], 0.0 };
	double dLimitAnkZ[2] = { -0.02, 0.05 }; // new added
	double dThesh[4] = { /*x*/ -0.006, 0.006, /*y*/ -0.01 * 1.5, 0.01 * 1.5 };
	double dTpit[4][4], dTrol[4][4], dTtemp[4][4], dPosAxis[3] = { 0.0 }, dPosStepTemp[4], dPosStepW[4];
	int nRTakeOffFlag = 0, nLTakeOffFlag = 0;
	// newnew =======================================================================================================================
	double dFmin = 0.1 * __MRobot * __Gravity; 
	double dKx[2] = { 25.0, 8.0 }, dKy[2] = { 25.0, 8.0 }, dKz[2] = { 25.0, 8.0 };
	double dLimitsTemp[6] = { -0.05, 0.05, -100.0, 100.0, -1000.0, 1000.0 };
	dccPositional dLastAmp = { 0.0 }; 
	dLastAmp.x = 0.5, dLastAmp.y = 0.5;
	// ==============================================================================================================================
	dccPositional dBaseErr = { 0.0 };
	dccRotational dPostErr = { 0.0 };
	dccRotational dDelPost = { 0.0 };
	// prep err
	for (int i = 0; i < 2; i++) *(&dPostErr.pit + i) = *(&stStateSens.Base.rot.pit + i) - *(&stStateRef.Base.rot.pit + i);
	dBaseErr.x = dZcHonda * sin(dPostErr.rol - stStateConVal.Base.rot.rol);
	dBaseErr.y = dZcHonda * sin(-dPostErr.pit - -stStateConVal.Base.rot.pit);
	// check sup
	if (stStatePG.SupLeg == 1 || stStatePG.SupLeg == 0) nRSupFlag[0] = 1;
	else nRSupFlag[0] = 0;
	if (stStatePG.SupLeg == 2 || stStatePG.SupLeg == 0) nLSupFlag[0] = 1;
	else nLSupFlag[0] = 0;
	if (nRSupFlag[1] == 1 && nRSupFlag[0] == 0) nRTakeOffFlag = 1, nKRTakeOff = 0;
	if (nLSupFlag[1] == 1 && nLSupFlag[0] == 0) nLTakeOffFlag = 1, nKLTakeOff = 0;
	// normal circle
	nKRTakeOff++, nKLTakeOff++;
	for (int i = 0; i < 2; i++) *(&dSumDelCom.x + i) = *(&dSumDelCom.x + i) + *(&dBaseErr.x + i) * __ControlT; // sum del com
	if (nRTakeOffFlag == 1) { // the timming that right foot takeoff
		fnvClearPosition(&dStepBkConValR); // newnew
		// finally end!!
		dDelT = ((double)nKLTakeOff) * __ControlT; // cal del t
		dDelComTe = dBaseErr;
		for (int i = 0; i < 2; i++) {
			*(&dDelCom.x + i) = *(&dSumDelCom.x + i) / dDelT; // cal del com
			*(&dDelCom.dx + i) = (*(&dDelComTe.x + i) - *(&dDelComT0.x + i)) / dDelT; // cal del vcom
			*(&dDelCp.x + i) = fndLimit(fndThreshold(/*pos*/*(&dDelCom.x + i) + /*vel*/dSpeedAmp / sqrt(__Gravity / dZcHonda) * *(&dDelCom.dx + i), dThesh + 2 * i), dCpLimit + 2 * i) - /*last*/*(&dLastAmp.x + i) * *(&dStepLast.x + i); // cal cp // newnew
			*(&dStepLast.x + i) = *(&dDelCp.x + i); // newnew
		}
		// new added
		dDelPost.pit = -dDelCom.y / dZcHonda, dDelPost.rol = dDelCom.x / dZcHonda; // cal delposture
		fnvObtainTransMat3(dTpit, 'x', dDelPost.pit, dPosAxis);
		fnvObtainTransMat3(dTrol, 'y', dDelPost.rol, dPosAxis);
		dcc_fnvMatMet(dTpit, dTrol, 4, 4, 4, '*', dTtemp);
		dPosStepTemp[0] = __AnkleWidth + dDelCp.x, dPosStepTemp[1] = dStepLen + dDelCp.y, dPosStepTemp[2] = 0.0, dPosStepTemp[3] = 1.0; // get step pos in left foot frame
		dcc_fnvMatMet(dTtemp, dPosStepTemp, 4, 4, 1, '*', dPosStepW); // cal step pos in world frame
		dDelCp.z = fndLimit(-dPosStepW[2], dLimitAnkZ); // cal step z adj
		// new added
		// give cp to the spline for right foot
		#ifdef USE_SPLINE_BK
		dDelAnkCpTra.Rfoot.pos.x[0] = dDelAnkCpTra.Rfoot.pos.y[0] = dDelAnkCpTra.Rfoot.pos.z[0] = 0.0; 
		fnvEzSpline(dDelAnkCpTra.Rfoot.pos.x, 1000, 0.0, dTswiTsup[0], dDelCp.x, __ControlT);
		fnvEzSpline(dDelAnkCpTra.Rfoot.pos.y, 1000, 0.0, dTswiTsup[0], dDelCp.y, __ControlT);
		fnvEzSpline(dDelAnkCpTra.Rfoot.pos.z, 1000, 0.0, dTswiTsup[0], dDelCp.z, __ControlT); // new added
		// spline a recover for right foot
		fnvEzSpline(dDelAnkCpTra.Rfoot.pos.x, 1000, dTswiTsup[0], dTswiTsup[0] + dTswiTsup[1], 0.0, __ControlT);
		fnvEzSpline(dDelAnkCpTra.Rfoot.pos.y, 1000, dTswiTsup[0], dTswiTsup[0] + dTswiTsup[1], 0.0, __ControlT);
		fnvEzSpline(dDelAnkCpTra.Rfoot.pos.z, 1000, dTswiTsup[0], dTswiTsup[0] + dTswiTsup[1], 0.0, __ControlT); // new added
		#endif
		#ifdef USE_FBCON_BK
		fnvFifthSpline(dDelAnkCpTra.Rfoot.pos.x, __MaxKprog, dStepReR.x, dStepReR.dx, dStepReR.ddx, 0.0, dDelCp.x, 0.0, 0.0, dTswiTsup[0], __ControlT, 'T');
		fnvFifthSpline(dDelAnkCpTra.Rfoot.pos.y, __MaxKprog, dStepReR.y, dStepReR.dy, dStepReR.ddy, 0.0, dDelCp.y, 0.0, 0.0, dTswiTsup[0], __ControlT, 'T');
		fnvFifthSpline(dDelAnkCpTra.Rfoot.pos.z, __MaxKprog, dStepReR.z, dStepReR.dz, dStepReR.ddz, 0.0, dDelCp.z, 0.0, 0.0, dTswiTsup[0], __ControlT, 'T');
		#endif
		// a new start!!
		dSumDelCom.x = dSumDelCom.y = 0.0; // clear sum
		dDelComT0 = dBaseErr;
	}
	if (nLTakeOffFlag == 1) { // the timming that left foot takeoff
		fnvClearPosition(&dStepBkConValL); // newnew
		// finally end!!
		dDelT = ((double)nKRTakeOff) * __ControlT; // cal del t
		dDelComTe = dBaseErr;
		for (int i = 0; i < 2; i++) {
			*(&dDelCom.x + i) = *(&dSumDelCom.x + i) / dDelT; // cal del com
			*(&dDelCom.dx + i) = (*(&dDelComTe.x + i) - *(&dDelComT0.x + i)) / dDelT; // cal del vcom
			*(&dDelCp.x + i) = fndLimit(fndThreshold(/*pos*/*(&dDelCom.x + i) + /*vel*/dSpeedAmp / sqrt(__Gravity / dZcHonda) * *(&dDelCom.dx + i), dThesh + 2 * i), dCpLimit + 2 * i) - /*last*/*(&dLastAmp.x + i) * *(&dStepLast.x + i); // cal cp // newnew
			*(&dStepLast.x + i) = *(&dDelCp.x + i); // newnew
		}
		// new added
		dDelPost.pit = -dDelCom.y / dZcHonda, dDelPost.rol = dDelCom.x / dZcHonda; // cal delposture
		fnvObtainTransMat3(dTpit, 'x', dDelPost.pit, dPosAxis);
		fnvObtainTransMat3(dTrol, 'y', dDelPost.rol, dPosAxis);
		dcc_fnvMatMet(dTpit, dTrol, 4, 4, 4, '*', dTtemp);
		dPosStepTemp[0] = -__AnkleWidth + dDelCp.x, dPosStepTemp[1] = dStepLen + dDelCp.y, dPosStepTemp[2] = 0.0, dPosStepTemp[3] = 1.0; // get step pos in left foot frame
		dcc_fnvMatMet(dTtemp, dPosStepTemp, 4, 4, 1, '*', dPosStepW); // cal step pos in world frame
		dDelCp.z = fndLimit(-dPosStepW[2], dLimitAnkZ); // cal step z adj
		// new added
		// give cp to the spline for left foot
		#ifdef USE_SPLINE_BK
		dDelAnkCpTra.Lfoot.pos.x[0] = dDelAnkCpTra.Lfoot.pos.y[0] = dDelAnkCpTra.Lfoot.pos.z[0] = 0.0;
		fnvEzSpline(dDelAnkCpTra.Lfoot.pos.x, 1000, 0.0, dTswiTsup[0], dDelCp.x, __ControlT);
		fnvEzSpline(dDelAnkCpTra.Lfoot.pos.y, 1000, 0.0, dTswiTsup[0], dDelCp.y, __ControlT);
		fnvEzSpline(dDelAnkCpTra.Lfoot.pos.z, 1000, 0.0, dTswiTsup[0], dDelCp.z, __ControlT); // new added
		// spline a recover for left foot
		fnvEzSpline(dDelAnkCpTra.Lfoot.pos.x, 1000, dTswiTsup[0], dTswiTsup[0] + dTswiTsup[1], 0.0, __ControlT);
		fnvEzSpline(dDelAnkCpTra.Lfoot.pos.y, 1000, dTswiTsup[0], dTswiTsup[0] + dTswiTsup[1], 0.0, __ControlT);
		fnvEzSpline(dDelAnkCpTra.Lfoot.pos.z, 1000, dTswiTsup[0], dTswiTsup[0] + dTswiTsup[1], 0.0, __ControlT); // new added
		#endif
		#ifdef USE_FBCON_BK
		fnvFifthSpline(dDelAnkCpTra.Lfoot.pos.x, __MaxKprog, dStepReL.x, dStepReL.dx, dStepReL.ddx, 0.0, dDelCp.x, 0.0, 0.0, dTswiTsup[0], __ControlT, 'T');
		fnvFifthSpline(dDelAnkCpTra.Lfoot.pos.y, __MaxKprog, dStepReL.y, dStepReL.dy, dStepReL.ddy, 0.0, dDelCp.y, 0.0, 0.0, dTswiTsup[0], __ControlT, 'T');
		fnvFifthSpline(dDelAnkCpTra.Lfoot.pos.z, __MaxKprog, dStepReL.z, dStepReL.dz, dStepReL.ddz, 0.0, dDelCp.z, 0.0, 0.0, dTswiTsup[0], __ControlT, 'T');
		#endif
		// a new start!!
		dSumDelCom.x = dSumDelCom.y = 0.0; // clear sum
		dDelComT0 = dBaseErr;
	}
	#ifdef USE_FBCON_BK
	// newnew cal bk conval
	if(nRSupFlag[0] == 1 && stStateSens.FootFT.Rfoot.fz > dFmin) dStepBkConValR.ddx = -dKx[0] * (dDelAnkCpTra.Rfoot.pos.x[nKRTakeOff] + dStepBkConValR.x) - dKx[1] * dStepBkConValR.dx, dStepBkConValR.ddy = -dKy[0] * (dDelAnkCpTra.Rfoot.pos.y[nKRTakeOff] + dStepBkConValR.y) - dKy[1] * dStepBkConValR.dy, dStepBkConValR.ddz = -dKz[0] * (dDelAnkCpTra.Rfoot.pos.z[nKRTakeOff] + dStepBkConValR.z) - dKz[1] * dStepBkConValR.dz; // R sup
	else dStepBkConValR.ddx = -dKx[1] * dStepBkConValR.dx, dStepBkConValR.ddy = -dKy[1] * dStepBkConValR.dy, dStepBkConValR.ddz = -dKz[1] * dStepBkConValR.dz; // R smash off
	if(nLSupFlag[0] == 1 && stStateSens.FootFT.Lfoot.fz > dFmin) dStepBkConValL.ddx = -dKx[0] * (dDelAnkCpTra.Lfoot.pos.x[nKRTakeOff] + dStepBkConValL.x) - dKx[1] * dStepBkConValL.dx, dStepBkConValL.ddy = -dKy[0] * (dDelAnkCpTra.Lfoot.pos.y[nKRTakeOff] + dStepBkConValL.y) - dKy[1] * dStepBkConValL.dy, dStepBkConValL.ddz = -dKz[0] * (dDelAnkCpTra.Lfoot.pos.z[nKRTakeOff] + dStepBkConValL.z) - dKz[1] * dStepBkConValL.dz; // L sup
	else dStepBkConValL.ddx = -dKx[1] * dStepBkConValL.dx, dStepBkConValL.ddy = -dKy[1] * dStepBkConValL.dy, dStepBkConValL.ddz = -dKz[1] * dStepBkConValL.dz; // L smash off
	// interg bk conval
	fnvIntegLimit(&dStepBkConValR.x, &dStepBkConValR.dx, dStepBkConValR.ddx, dLimitsTemp, __ControlT);
	fnvIntegLimit(&dStepBkConValR.y, &dStepBkConValR.dy, dStepBkConValR.ddy, dLimitsTemp, __ControlT);
	fnvIntegLimit(&dStepBkConValR.z, &dStepBkConValR.dz, dStepBkConValR.ddz, dLimitsTemp, __ControlT);
	fnvIntegLimit(&dStepBkConValL.x, &dStepBkConValL.dx, dStepBkConValL.ddx, dLimitsTemp, __ControlT);
	fnvIntegLimit(&dStepBkConValL.y, &dStepBkConValL.dy, dStepBkConValL.ddy, dLimitsTemp, __ControlT);
	fnvIntegLimit(&dStepBkConValL.z, &dStepBkConValL.dz, dStepBkConValL.ddz, dLimitsTemp, __ControlT);
	// re for spline
	dStepReR.x = dDelAnkCpTra.Rfoot.pos.x[nKRTakeOff] + dStepBkConValR.x, dStepReR.y = dDelAnkCpTra.Rfoot.pos.y[nKRTakeOff] + dStepBkConValR.y, dStepReR.z = dDelAnkCpTra.Rfoot.pos.z[nKRTakeOff] + dStepBkConValR.z;
	dStepReR.dx = dStepBkConValR.dx, dStepReR.dy = dStepBkConValR.dy, dStepReR.dz = dStepBkConValR.dz;
	dStepReR.ddx = dStepBkConValR.ddx, dStepReR.ddy = dStepBkConValR.ddy, dStepReR.ddz = dStepBkConValR.ddz;
	dStepReL.x = dDelAnkCpTra.Lfoot.pos.x[nKRTakeOff] + dStepBkConValL.x, dStepReL.y = dDelAnkCpTra.Lfoot.pos.y[nKRTakeOff] + dStepBkConValL.y, dStepReL.z = dDelAnkCpTra.Lfoot.pos.z[nKRTakeOff] + dStepBkConValL.z;
	dStepReL.dx = dStepBkConValL.dx, dStepReL.dy = dStepBkConValL.dy, dStepReL.dz = dStepBkConValL.dz;
	dStepReL.ddx = dStepBkConValL.ddx, dStepReL.ddy = dStepBkConValL.ddy, dStepReL.ddz = dStepBkConValL.ddz;
	#endif
	// give spline val to ank
#ifdef USE_STEPCON_NEW
	stStateConVal.Ankle.B.Rfoot.pos.x += dAmpx * fndLimit(dDelAnkCpTra.Rfoot.pos.x[nKRTakeOff] + dStepBkConValR.x, dLimitRAnkX), stStateConVal.Ankle.B.Rfoot.pos.y += dAmpy * (dDelAnkCpTra.Rfoot.pos.y[nKRTakeOff] + dStepBkConValR.y);
	stStateConVal.Ankle.B.Lfoot.pos.x += dAmpx * fndLimit(dDelAnkCpTra.Lfoot.pos.x[nKLTakeOff] + dStepBkConValL.x, dLimitLAnkX), stStateConVal.Ankle.B.Lfoot.pos.y += dAmpy * (dDelAnkCpTra.Lfoot.pos.y[nKLTakeOff] + dStepBkConValL.y);
	dZRTempRe = dAmpz * (dDelAnkCpTra.Rfoot.pos.z[nKRTakeOff] + dStepBkConValR.z), dZLTempRe = dAmpz * (dDelAnkCpTra.Lfoot.pos.z[nKLTakeOff] + dStepBkConValL.z);
#ifdef USE_STEPCON_Z
	stStateConVal.Ankle.B.Rfoot.pos.z += dZRTempRe,	stStateConVal.Ankle.B.Lfoot.pos.z += dZLTempRe;
#endif
#endif
	// refresh supflag
	nRSupFlag[1] = nRSupFlag[0];
	nLSupFlag[1] = nLSupFlag[0];
	// FrameAddLogsC(987, "dDelCom.x",  &dDelCom.x, 1);
	// FrameAddLogsC(987, "dDelCom.dx",  &dDelCom.dx, 1);
	// FrameAddLogsC(987, "dDelCp.x",  &dDelCp.x, 1);
	// FrameAddLogsC(987, "dDelCom.y",  &dDelCom.y, 1);
	// FrameAddLogsC(987, "dDelCom.dy",  &dDelCom.dy, 1);
	// FrameAddLogsC(987, "dDelCp.y",  &dDelCp.y, 1);
	// FrameAddLogsC(987, "rankx",  &stStateConVal.Ankle.B.Rfoot.pos.x, 1);
	// FrameAddLogsC(987, "ranky",  &stStateConVal.Ankle.B.Rfoot.pos.y, 1); 
	// FrameAddLogsC(987, "rankz",  &dZRTempRe, 1);
	// FrameAddLogsC(987, "lankx",  &stStateConVal.Ankle.B.Lfoot.pos.x, 1);
	// FrameAddLogsC(987, "lanky",  &stStateConVal.Ankle.B.Lfoot.pos.y, 1);
	// FrameAddLogsC(987, "lankz",  &dZLTempRe, 1);
	double dRTakeOffFlag = (double)nRTakeOffFlag, dLTakeOffFlag = (double)nLTakeOffFlag, dKRTakeOff = (double)nKRTakeOff, dKLTakeOff = (double)nKLTakeOff;
	// FrameAddLogsC(987, "nRTakeOffFlag",  &dRTakeOffFlag, 1);
	// FrameAddLogsC(987, "nLTakeOffFlag",  &dLTakeOffFlag, 1);
	// FrameAddLogsC(987, "nKRTakeOff",  &dKRTakeOff, 1);
	// FrameAddLogsC(987, "nKLTakeOff",  &dKLTakeOff, 1);
	// FrameAddLogsC(987, "dFbConValR",  &dStepBkConValR.y, 1);
	// FrameAddLogsC(987, "dFbConValL",  &dStepBkConValL.y, 1);

}

// double dFBCom = 0.95, dFBvCom = 2.4;
// double dFBCom = 0.8, dFBvCom = 2.8;
// double dFBCom = 0.7, dFBvCom = 3.0;
// double dFBCom = 0.66, dFBvCom = 3.2;
// double dFBCom = 0.95 * 1.2, dFBvCom = 2.4 * 0.8; // first good without poscon
double dFBCom = 0.95 * 1.5, dFBvCom = 2.4 * 1.2;
void fnvRunConHonda() {
	// #ifdef USE_SUPPOSCON
	// 	// dFBCom = 1.0;
	// 	// dFBvCom = 1.6 * 1.3;
	// #endif
	// double dSoftFootGain = 0.4; 
	// double dSoftFootGainz = 0.55; 
	// double dHardSpringGain = 1.0; // 10.0;
	double dSoftFootGain = 0.4; // 0.52; // 0.55; // 0.6 // 0.5
	double dSoftFootGainz = 0.5; // 0.9; // 0.8 // 0.45
	double dDampFootGainz = 2.0; // 1.5
	double dHardSpringGain = 2.5; // 0.4; // 0.5 // 0.5
	// modified by LJH 20230420
	// { /*forw*/0.085, /*back*/0.060, /*iner*/0.050, /*outer*/0.050 };
	// { /*forw*/0.085, /*back*/0.07, /*iner*/0.055, /*outer*/0.07 };
	double dFootGeom_in[4] = { /*forw*/0.085, /*back*/0.07, /*iner*/0.06, /*outer*/0.075 }; //{ /*forw*/0.076, /*back*/0.06, /*iner*/0.045, /*outer*/0.06 }; // { /*forw*/0.13, /*back*/0.095, /*iner*/0.05, /*outer*/0.075 };
	double dParas_CalMstab[4] = { /*Kp_pit*/1.5 * dFBCom * __MRobot * __Gravity, /*Kd_pit*/dFBvCom * 30.0, /*Kp_rol*/1.5 * dFBCom * __MRobot * __Gravity, /*Kd_rol*/dFBvCom * 1.5 * 30.0 }; // important
	double dParas_Mdzmp[4] = { /*Kp_pos*/50.0, /*Kd_pos*/10.0, /*Kp_rot*/50.0, /*Kd_rot*/10.0 };
	double dParas_GRFC[9] = { /*Kf_z*/dSoftFootGainz * 0.025 * 0.8, /*Kp_z*/dHardSpringGain * 100.0, /*Kd_z*/80.0 * dDampFootGainz/*chz strait knee*/, /*Kvc_z*/ 0.6 * 5.0, /*Kac_z*/ 0.6 * 0.005, /*Kdf_z*/ 0.35 * 0.0002, /*Kdfc_z*/ 0.4 * 0.00012, /*Kf_r*/dSoftFootGain * 0.06, /*Kp_r*/dHardSpringGain * 2.0  }; 
	double dGrtBias[2] = { /*dGrtBias_pit*/-0.0 * 2.0, /*dGrtBias_pit*/0.0}; 
	double dParasCompliance[2] = { /*Kf*/ 0.002, /*Kp*/ 104.0 };
	double dTswiTsup[2] = { /*Tswi*/ 0.3045, /*Tsup*/ 0.2755 }; // 0.29
	// { /*Tswi*/ 0.3045, /*Tsup*/ 0.2755 }; // 0.28
	// { /*Tswi*/ 0.3045, /*Tsup*/ 0.2755 }; // 0.29
	// { /*Tswi*/ 0.315, /*Tsup*/ 0.285 }; // 0.3 
	// { /*Tswi*/ 0.285, /*Tsup*/ 0.315 }; // 0.3 
	// { /*Tswi*/ 0.304, /*Tsup*/ 0.336 }; // 0.32

	nRek = 0;
 	fnvGetSupPoly(dFootGeom_in);
	fnvCalMstabLimit(dFootGeom_in);
	fnvCalMstab(dParas_CalMstab);
	fnvMdzmp(dParas_Mdzmp, /*dLagTsplit*/0.1);
	fnvAddFBMoment(/*dFzMin*/0.1 * __MRobot * __Gravity, dGrtBias);
	fnvGRFC(dParas_GRFC);
	fnvCompliance(dParasCompliance);
	fnvStepConNew(dTswiTsup, dStepLenNow);
}
#endif