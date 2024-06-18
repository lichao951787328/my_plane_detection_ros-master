// For running project of BHR7
// 20210111 bit
#ifndef DCC_RunningCon_C
#define DCC_RunningCon_C
// #include "DCC_RunningCon.h"
// #include "..\..\ChzFrameCpp2C.h"
#include <stdio.h>
// #include "Resistant_Compliance.h"
#include <Compliance/DCC/DccCon_lib/DCC_RunningCon.h>
#include <Compliance/DCC/DccCon_lib/Resistant_Compliance.h>
#include <BHRRobotKinematicParam/BHRRobotParametersC.h>

// dcc_fuzzy
#define FuzzyDispFlag 0

// convals
dccPositional stLimpConVal = { 0.0 };
dccSpacial stMZmpConVal = { 0.0 }; // rec dcc
double dMback_rol = 0.0;
double dMback_pit = 0.0;
dccFootFT stLimpAddiTrq = { 0.0 };
dccAnkle stGrfConVal = { 0.0 };
dccPositional stTpcConVal = { 0.0 };
dccRotational stSupPosConVal = { 0.0 };
dccPositional stSurPassedZmp = { 0.0 }; // rec dcc

// suppoly // rec dcc
dccSupPoly stSupPoly_B = { 0.0 };
dccSupPoly stSupPoly_W = { 0.0 };

// globals
enum supsignal {
	DouSup, RightSup, LeftSup, Fly
};
double dZc = 0.55;
double dFTouchDown = 0.1 * __MRobot * __Gravity;
double dFUpToFly  = 0.05 * __MRobot * __Gravity;
double dTLagLIPM = 0.00005;
double dTLagTrqRef = 0.00002;
double dTLagFrcRef = 0.00002;
double dTLagConValRot = 0.00001;
double dTLagConValPos = 0.00001;
#define __IUpper 14.0 * 0.4 * 0.4 * 0.083

int nKCheckTimeLag = 0;

double dStand = 0.0;
double dStandNow = 0.0;

// rec dcc
void fnvGetSupPoly(double dFootGeom_in[4]) {
	if (stStatePG.SupLeg == RightSup) {
		stSupPoly_B.forw = stStateCmd.Ankle.B.Rfoot.pos.y + dFootGeom_in[0];
		stSupPoly_B.back = stStateCmd.Ankle.B.Rfoot.pos.y - dFootGeom_in[1];
		stSupPoly_B.left = stStateCmd.Ankle.B.Rfoot.pos.x - dFootGeom_in[2];
		stSupPoly_B.righ = stStateCmd.Ankle.B.Rfoot.pos.x + dFootGeom_in[3];
		stSupPoly_W.forw = stStateCmd.Ankle.W.Rfoot.pos.y + dFootGeom_in[0];
		stSupPoly_W.back = stStateCmd.Ankle.W.Rfoot.pos.y - dFootGeom_in[1];
		stSupPoly_W.left = stStateCmd.Ankle.W.Rfoot.pos.x - dFootGeom_in[2];
		stSupPoly_W.righ = stStateCmd.Ankle.W.Rfoot.pos.x + dFootGeom_in[3];
	}
	else if (stStatePG.SupLeg == LeftSup) {
		stSupPoly_B.forw = stStateCmd.Ankle.B.Lfoot.pos.y + dFootGeom_in[0];
		stSupPoly_B.back = stStateCmd.Ankle.B.Lfoot.pos.y - dFootGeom_in[1];
		stSupPoly_B.left = stStateCmd.Ankle.B.Lfoot.pos.x - dFootGeom_in[3];
		stSupPoly_B.righ = stStateCmd.Ankle.B.Lfoot.pos.x + dFootGeom_in[2];
		stSupPoly_W.forw = stStateCmd.Ankle.W.Lfoot.pos.y + dFootGeom_in[0];
		stSupPoly_W.back = stStateCmd.Ankle.W.Lfoot.pos.y - dFootGeom_in[1];
		stSupPoly_W.left = stStateCmd.Ankle.W.Lfoot.pos.x - dFootGeom_in[3];
		stSupPoly_W.righ = stStateCmd.Ankle.W.Lfoot.pos.x + dFootGeom_in[2];
	}
	else if (stStatePG.SupLeg == DouSup) {
		if (stStateCmd.Ankle.B.Rfoot.pos.y >= stStateCmd.Ankle.B.Lfoot.pos.y) { // R forward
			stSupPoly_B.forw = stStateCmd.Ankle.B.Rfoot.pos.y + dFootGeom_in[0];
			stSupPoly_B.back = stStateCmd.Ankle.B.Lfoot.pos.y - dFootGeom_in[1];
			stSupPoly_W.forw = stStateCmd.Ankle.W.Rfoot.pos.y + dFootGeom_in[0];
			stSupPoly_W.back = stStateCmd.Ankle.W.Lfoot.pos.y - dFootGeom_in[1];
		}
		else { // L forward
			stSupPoly_B.forw = stStateCmd.Ankle.B.Lfoot.pos.y + dFootGeom_in[0];
			stSupPoly_B.back = stStateCmd.Ankle.B.Rfoot.pos.y - dFootGeom_in[1];
			stSupPoly_W.forw = stStateCmd.Ankle.W.Lfoot.pos.y + dFootGeom_in[0];
			stSupPoly_W.back = stStateCmd.Ankle.W.Rfoot.pos.y - dFootGeom_in[1];
		}
		stSupPoly_B.left = stStateCmd.Ankle.B.Lfoot.pos.x - dFootGeom_in[3];
		stSupPoly_B.righ = stStateCmd.Ankle.B.Rfoot.pos.x + dFootGeom_in[3];
		stSupPoly_W.left = stStateCmd.Ankle.W.Lfoot.pos.x - dFootGeom_in[3];
		stSupPoly_W.righ = stStateCmd.Ankle.W.Rfoot.pos.x + dFootGeom_in[3];
	}
	else {
		for (int i = 0; i < 4; i++) {
			*(&stSupPoly_B.left + i) = 0.0;
			*(&stSupPoly_W.left + i) = 0.0;
		}
	}
	// re
	for (int i = 0; i < 4; i++) {
		dSupPoly[i] = *(&stSupPoly_B.forw + i);
		dSupPoly[i + 4] = *(&stSupPoly_W.forw + i);
	}	
	// FrameAddLogsC(345, "SupPoly_forw",  &stSupPoly_W.forw, 1);
	// FrameAddLogsC(345, "SupPoly_back",  &stSupPoly_W.back, 1);
	// FrameAddLogsC(345, "SupPoly_left",  &stSupPoly_W.left, 1);
	// FrameAddLogsC(345, "SupPoly_righ",  &stSupPoly_W.righ, 1);
}

/** LIPM controller
* InputVal: paras[d6], limits[d4], #Base_rot_Sens, Base_drot_Sens, Base_rot_Ref, Base_drot_Ref, TpcConVal_old, ZMP_B_Sens, ZMP_B_PG 
* OutoutVal: #ZMP_B_Ref
*/
// rec dcc
void fnvLipmCon(double dParasLipm_in[6], double dLimitsLipm_in[4]) {
	double kp_x = dParasLipm_in[0];	double kp_y = dParasLipm_in[0 + 3];
	double kv_x = dParasLipm_in[1];	double kv_y = dParasLipm_in[1 + 3];
	double kz_x = dParasLipm_in[2];	double kz_y = dParasLipm_in[2 + 3];
	double LIPMLimit_x[2] = { stSupPoly_W.left + dLimitsLipm_in[0], stSupPoly_W.righ - dLimitsLipm_in[1] };
	double LIPMLimit_y[2] = { stSupPoly_W.back + dLimitsLipm_in[2], stSupPoly_W.forw - dLimitsLipm_in[3] };
	// cal posture
	double delta_pit = stStateSens.Base.rot.pit - stStateRef.Base.rot.pit - stStateConVal.Base.rot.pit;
	double delta_rol = stStateSens.Base.rot.rol - stStateRef.Base.rot.rol - stStateConVal.Base.rot.rol;
	double delta_dpit = stStateSens.Base.rot.dpit - stStateRef.Base.rot.dpit;
	double delta_drol = stStateSens.Base.rot.drol - stStateRef.Base.rot.drol;
	// cal state
	double delta_cx = stTpcConVal.x + dZc * sin(delta_rol);
	double delta_cy = stTpcConVal.y - dZc * sin(delta_pit);
	double delta_vx = stTpcConVal.dx + dZc * sin(delta_drol);
	double delta_vy = stTpcConVal.dy - dZc * sin(delta_dpit);
#ifdef USE_LIPM
	// cal ConVal
	// stLimpConVal.x = kp_x * delta_cx + kv_x * delta_vx + kz_x * (stStateSens.ZMP.W.x - stStatePG.ZMP.W.x);
	// stLimpConVal.y = kp_y * delta_cy + kv_y * delta_vy + kz_y * (stStateSens.ZMP.W.y - stStatePG.ZMP.W.y);
	stLimpConVal.x = fndAddLimit(-dMback_rol * __IUpper / __MRobot / __Gravity + kp_x * delta_cx + kv_x * delta_vx + kz_x * (stStateSens.ZMP.W.x - stStatePG.ZMP.W.x), stStatePG.ZMP.W.x, &stSurPassedZmp.x, LIPMLimit_x);
	stLimpConVal.y = fndAddLimit(dMback_pit * __IUpper / __MRobot / __Gravity + kp_y * delta_cy + kv_y * delta_vy + kz_y * (stStateSens.ZMP.W.y - stStatePG.ZMP.W.y), stStatePG.ZMP.W.y, &stSurPassedZmp.y, LIPMLimit_y);
	// StateRef update
	stStateRef.ZMP.W.x = stStatePG.ZMP.W.x + stLimpConVal.x;
	stStateRef.ZMP.W.y = stStatePG.ZMP.W.y + stLimpConVal.y;
#endif
	// re
	dLipmRe[0] = delta_cx;
	dLipmRe[1] = delta_vx;
	dLipmRe[2] = delta_cy;
	dLipmRe[3] = delta_vy;
	for (int i = 0; i < 2; i++) {
		dLipmRe[4 + 4 * i] = *(&stLimpConVal.x + i);
		dLipmRe[5 + 4 * i] = *(&stStateRef.ZMP.W.x + i);
		dLipmRe[6 + 4 * i] = *(&stStateSens.ZMP.W.x + i);
		dLipmRe[7 + 4 * i] = *(&stStatePG.ZMP.W.x + i);
	}
	// FrameAddLogsC(345, "ZMP_PG_x",  &stStatePG.ZMP.W.x, 1);
	// FrameAddLogsC(345, "ZMP_PG_y",  &stStatePG.ZMP.W.y, 1);
	// FrameAddLogsC(345, "ZMP_Ref_x",  &stStateRef.ZMP.W.x, 1);
	// FrameAddLogsC(345, "ZMP_Ref_y",  &stStateRef.ZMP.W.y, 1);
	// FrameAddLogsC(345, "ZMP_Rel_x",  &stStateSens.ZMP.W.x, 1);
	// FrameAddLogsC(345, "ZMP_Rel_y",  &stStateSens.ZMP.W.y, 1);
}

void fnvModelZmpCon(double dParasMZmp_in[6], double dLimitMZmp_in[2], char cMethodFlag) {
	double micro_x = dParasMZmp_in[0], micro_y = dParasMZmp_in[3];
	double kp_x = dParasMZmp_in[1], kd_x = dParasMZmp_in[2], kp_y = dParasMZmp_in[4], kd_y = dParasMZmp_in[5];
	double dLimit_x[6] = { -dLimitMZmp_in[0], dLimitMZmp_in[0], -10.0, 10.0, -100.0, 100.0 };
	double dLimit_y[6] = { -dLimitMZmp_in[1], dLimitMZmp_in[1], -10.0, 10.0, -100.0, 100.0 };
	// double dPosBaseTarget_x, dPosBaseTarget_y;
	double kcom_x = 50.0, kcom_y = 50.0;
	double dLimit_cx[6] = { -0.03, 0.03, -50.0, 50.0, -500.0, 500.0 };
	double dLimit_cy[6] = { -0.2, 0.2, -50.0, 50.0, -500.0, 500.0 };
	if (cMethodFlag == 'a') { // cal acc method
		stMZmpConVal.pos.ddx = (micro_x * stSurPassedZmp.x * __MRobot * __Gravity) / __MRobot / dZc - kp_x * stMZmpConVal.pos.x - kd_x * stMZmpConVal.pos.dx;
		stMZmpConVal.pos.ddy = (micro_y * stSurPassedZmp.y * __MRobot * __Gravity) / __MRobot / dZc - kp_y * stMZmpConVal.pos.y - kd_y * stMZmpConVal.pos.dy;
		// update
		fnvIntegLimit(&stMZmpConVal.pos.x, &stMZmpConVal.pos.dx, stMZmpConVal.pos.ddx, dLimit_x, __ControlT);
		fnvIntegLimit(&stMZmpConVal.pos.y, &stMZmpConVal.pos.dy, stMZmpConVal.pos.ddy, dLimit_y, __ControlT);
#ifdef USE_MODELZMP
		stStateConVal.Base.pos.x += stMZmpConVal.pos.x;
		stStateConVal.Base.pos.y += stMZmpConVal.pos.y;
		dMZmpRe[2] = stMZmpConVal.pos.x;
		dMZmpRe[3] = stMZmpConVal.pos.y;
#endif
	}
	else {
		// if (fabs(stSurPassedZmp.x) > 1e-6) dPosBaseTarget_x = -0.1 * dZc * sin(stStateSens.Base.rot.rol);
		// else dPosBaseTarget_x = 0.0;
		// if (fabs(stSurPassedZmp.y) > 1e-6) dPosBaseTarget_y = 0.1 * dZc * sin(stStateSens.Base.rot.pit);
		// else dPosBaseTarget_y = 0.0;
		// stMZmpConVal.pos.ddx = kcom_x * (dPosBaseTarget_x - stMZmpConVal.pos.x) - 10.0 * stMZmpConVal.pos.dx;
		// stMZmpConVal.pos.ddy = kcom_y * (dPosBaseTarget_y - stMZmpConVal.pos.y) - 10.0 * stMZmpConVal.pos.dy;
		dMback_rol = - kp_x * stMZmpConVal.rot.rol - kd_x * stMZmpConVal.rot.drol;
		dMback_pit = - kp_y * stMZmpConVal.rot.pit - kd_y * stMZmpConVal.rot.dpit;
		stMZmpConVal.rot.ddrol = -(-micro_x * stSurPassedZmp.x * __Gravity + stMZmpConVal.pos.ddx * dZc) * __MRobot / __IUpper + dMback_rol;
		stMZmpConVal.rot.ddpit =  (-micro_y * stSurPassedZmp.y * __Gravity + stMZmpConVal.pos.ddy * dZc) * __MRobot / __IUpper + dMback_pit;
		// update
		// fnvIntegLimit(&stMZmpConVal.pos.x, &stMZmpConVal.pos.dx, stMZmpConVal.pos.ddx, dLimit_cx, __ControlT);
		// fnvIntegLimit(&stMZmpConVal.pos.y, &stMZmpConVal.pos.dy, stMZmpConVal.pos.ddy, dLimit_cy, __ControlT);
		fnvIntegLimit(&stMZmpConVal.rot.rol, &stMZmpConVal.rot.drol, stMZmpConVal.rot.ddrol, dLimit_x, __ControlT);
		fnvIntegLimit(&stMZmpConVal.rot.pit, &stMZmpConVal.rot.dpit, stMZmpConVal.rot.ddpit, dLimit_y, __ControlT);
		stMZmpConVal.pos.x = -stMZmpConVal.rot.rol * 0.2;
		stMZmpConVal.pos.y = stMZmpConVal.rot.pit * 0.2;
		stMZmpConVal.pos.z = -0.5 * sqrt(stMZmpConVal.pos.x * stMZmpConVal.pos.x + stMZmpConVal.pos.y * stMZmpConVal.pos.y);
#ifdef USE_MODELZMP
		stStateConVal.Base.pos.x += 0.4 * stMZmpConVal.pos.x;
		stStateConVal.Base.pos.y += 0.4 * stMZmpConVal.pos.y;
		stStateConVal.Base.pos.z += 0.4 * stMZmpConVal.pos.z;
		stStateRef.Base.rot.rol += 1.0 * stMZmpConVal.rot.rol;
		stStateRef.Base.rot.pit += 1.0 * stMZmpConVal.rot.pit;
		dMZmpRe[2] = stMZmpConVal.pos.x;
		dMZmpRe[3] = stMZmpConVal.pos.y;
		dMZmpRe[4] = stMZmpConVal.pos.z;
		dMZmpRe[5] = stMZmpConVal.rot.pit * 57.3;
		dMZmpRe[6] = stMZmpConVal.rot.rol * 57.3;
#endif
	}

	// re
	dMZmpRe[0] = stSurPassedZmp.x;
	dMZmpRe[1] = stSurPassedZmp.y;
	// FrameAddLogsC(34543, "Sur_zmp_x",  &stSurPassedZmp.x, 1);
	// FrameAddLogsC(34543, "Sur_zmp_y",  &stSurPassedZmp.y, 1);
	// FrameAddLogsC(34543, "Mzmp_x",  &stMZmpConVal.pos.x, 1);
	// FrameAddLogsC(34543, "Mzmp_y",  &stMZmpConVal.pos.y, 1);
	// FrameAddLogsC(34543, "Mzmp_z",  &stMZmpConVal.pos.z, 1);
	// FrameAddLogsC(34543, "Mzmp_pit",  &dMZmpRe[5], 1);
	// FrameAddLogsC(34543, "Mzmp_rol",  &dMZmpRe[6], 1);
}


/** LIPM Additorque phases check & torque distribution
* InputVal: paras[d2], limits[d4], #ZMP_B_Ref, ZMP_B_Sens, SupSignal, FootFT_Sens, Ankle_B_Cmd_Old
* OutputVal: #LipmAddiTrq
*/
void fnvLipmAddiTrq(double dParasAddiTrq_in[2], double dLimitAddiTrq_in[4]) {
	double Trq_x_limit[2] = { dLimitAddiTrq_in[0], dLimitAddiTrq_in[1] }; // pit
	double Trq_y_limit[2] = { dLimitAddiTrq_in[2], dLimitAddiTrq_in[3] }; // rol
	double micro_px = dParasAddiTrq_in[0]; double micro_py = dParasAddiTrq_in[1];
	double xmidpoint = 0.0;
	double alpha = 0.5;
	double Limit_alpha[2] = { 0.0, 1.0 };
#ifdef USE_ADDITRQ
	// cal delzmp
	// double delta_px = stLimpConVal.x; // wayaya
	// double delta_py = stLimpConVal.y; // wayaya
	// double delta_px = stStateRef.ZMP.B.x - micro_px * stStateSens.ZMP.B.x; // NONO
	// double delta_py = stStateRef.ZMP.B.y - micro_py * stStateSens.ZMP.B.y; // NONO
	// double delta_px = stStateRef.ZMP.W.x - stStatePG.ZMP.W.x;//micro_px * stStateSens.ZMP.W.x;
	// double delta_py = stStateRef.ZMP.W.y - stStatePG.ZMP.W.y;//micro_py * stStateSens.ZMP.W.y;
	double delta_px = stStateRef.ZMP.W.x - micro_px * stStateSens.ZMP.W.x;
	double delta_py = stStateRef.ZMP.W.y - micro_py * stStateSens.ZMP.W.y;

	// // FrameAddLogsC(456, "ZMPRef", &stStateRef.ZMP.B.x, 2);
	// // FrameAddLogsC(456, "ZMPRel", &stStateSens.ZMP.B.x, 2);
	// FrameAddLogsC(456, "ZMPRef", &stStateRef.ZMP.W.x, 2);
	// FrameAddLogsC(456, "ZMPRel", &stStateSens.ZMP.W.x, 2);
	// FrameAddLogsC(456, "del_px", &delta_px, 1);
	// FrameAddLogsC(456, "del_py", &delta_py, 1);
	// printf("%lf, %lf,,,,,", stStateRef.ZMP.B.x, stStateRef.ZMP.B.y);
	// printf("%lf, %lf,,,,,\n", stStateSens.ZMP.B.x, stStateSens.ZMP.B.y);
	int RealSupFlag = Fly;
	// phases check
	if (stStateSens.FootFT.Fsum > dFTouchDown) { // touch down
		if (stStatePG.SupLeg == RightSup) { // r sup
			if (stStateSens.FootFT.Rfoot.fz > 0.5 * dFTouchDown) { // r touch down
				RealSupFlag = RightSup;
			}
			else { // fly
				RealSupFlag = Fly;
			}
		}
		else if (stStatePG.SupLeg == LeftSup) { // l sup
			if (stStateSens.FootFT.Lfoot.fz > 0.5 * dFTouchDown) { // l touch down
				RealSupFlag = LeftSup;
			}
			else { // fly
				RealSupFlag = Fly;
			}
		}
		else if (stStatePG.SupLeg == DouSup) { // dou sup
			if ((stStateSens.FootFT.Rfoot.fz > 0.5 * dFTouchDown) && (stStateSens.FootFT.Lfoot.fz > 0.5 * dFTouchDown)) { // dou touch down
				RealSupFlag = DouSup;
			}
			else if ((stStateSens.FootFT.Rfoot.fz > 0.5 * dFTouchDown) && (stStateSens.FootFT.Lfoot.fz <= 0.5 * dFTouchDown)) { // r touch down
				RealSupFlag = RightSup;
			}
			else if ((stStateSens.FootFT.Rfoot.fz <= 0.5 * dFTouchDown) && (stStateSens.FootFT.Lfoot.fz > 0.5 * dFTouchDown)) { // l touch down
				RealSupFlag = LeftSup;
			}
			else { // fly
				RealSupFlag = Fly;
			}
		}
		else { // fly phase
			RealSupFlag = Fly;
		}
	}
	else { // no touch down
		RealSupFlag = Fly;
	}
	// distribution
	if (RealSupFlag == RightSup) {
		stLimpAddiTrq.Rfoot.ty = fndLimit(-delta_px * __MRobot * __Gravity, Trq_y_limit);
		stLimpAddiTrq.Rfoot.tx = fndLimit(delta_py * __MRobot * __Gravity, Trq_x_limit);
		stLimpAddiTrq.Rfoot.fz = 0.0;
		stLimpAddiTrq.Lfoot.ty = 0.0;
		stLimpAddiTrq.Lfoot.tx = 0.0;
		stLimpAddiTrq.Lfoot.fz = 0.0;
		//printf("RightSup\n");
	}
	else if (RealSupFlag == LeftSup) {
		stLimpAddiTrq.Rfoot.ty = 0.0;
		stLimpAddiTrq.Rfoot.tx = 0.0;
		stLimpAddiTrq.Rfoot.fy = 0.0;
		stLimpAddiTrq.Lfoot.ty = fndLimit(-delta_px * __MRobot * __Gravity, Trq_y_limit);
		stLimpAddiTrq.Lfoot.tx = fndLimit(delta_py * __MRobot * __Gravity, Trq_x_limit);
		stLimpAddiTrq.Lfoot.fz = 0.0;
		//printf("LeftSup\n");
	}
	else if (RealSupFlag == DouSup) {
		xmidpoint = (stStateSens.ZMP.B.x*stStateCmd.Ankle.B.Lfoot.pos.x*stStateCmd.Ankle.B.Lfoot.pos.x - 2 * stStateSens.ZMP.B.x*stStateCmd.Ankle.B.Lfoot.pos.x*stStateCmd.Ankle.B.Rfoot.pos.x - stStateCmd.Ankle.B.Lfoot.pos.x*stStateCmd.Ankle.B.Lfoot.pos.y*stStateCmd.Ankle.B.Rfoot.pos.y + stStateSens.ZMP.B.y*stStateCmd.Ankle.B.Lfoot.pos.x*stStateCmd.Ankle.B.Lfoot.pos.y + stStateCmd.Ankle.B.Lfoot.pos.x*stStateCmd.Ankle.B.Rfoot.pos.y*stStateCmd.Ankle.B.Rfoot.pos.y - stStateSens.ZMP.B.y*stStateCmd.Ankle.B.Lfoot.pos.x*stStateCmd.Ankle.B.Rfoot.pos.y + stStateSens.ZMP.B.x*stStateCmd.Ankle.B.Rfoot.pos.x*stStateCmd.Ankle.B.Rfoot.pos.x + stStateCmd.Ankle.B.Rfoot.pos.x*stStateCmd.Ankle.B.Lfoot.pos.y*stStateCmd.Ankle.B.Lfoot.pos.y - stStateCmd.Ankle.B.Rfoot.pos.x*stStateCmd.Ankle.B.Lfoot.pos.y*stStateCmd.Ankle.B.Rfoot.pos.y - stStateSens.ZMP.B.y*stStateCmd.Ankle.B.Rfoot.pos.x*stStateCmd.Ankle.B.Lfoot.pos.y + stStateSens.ZMP.B.y*stStateCmd.Ankle.B.Rfoot.pos.x*stStateCmd.Ankle.B.Rfoot.pos.y) / (stStateCmd.Ankle.B.Lfoot.pos.x*stStateCmd.Ankle.B.Lfoot.pos.x - 2 * stStateCmd.Ankle.B.Lfoot.pos.x*stStateCmd.Ankle.B.Rfoot.pos.x + stStateCmd.Ankle.B.Rfoot.pos.x*stStateCmd.Ankle.B.Rfoot.pos.x + stStateCmd.Ankle.B.Lfoot.pos.y*stStateCmd.Ankle.B.Lfoot.pos.y - 2 * stStateCmd.Ankle.B.Lfoot.pos.y*stStateCmd.Ankle.B.Rfoot.pos.y + stStateCmd.Ankle.B.Rfoot.pos.y*stStateCmd.Ankle.B.Rfoot.pos.y);
		alpha = fndLimit((xmidpoint - stStateCmd.Ankle.B.Lfoot.pos.x) / (stStateCmd.Ankle.B.Rfoot.pos.x - stStateCmd.Ankle.B.Lfoot.pos.x), Limit_alpha); // force propotion to right foot
		stLimpAddiTrq.Rfoot.ty = (alpha) * fndLimit(-delta_px * __MRobot * __Gravity, Trq_y_limit);
		stLimpAddiTrq.Rfoot.tx = (alpha) * fndLimit(delta_py * __MRobot * __Gravity, Trq_x_limit);
		stLimpAddiTrq.Rfoot.fz = 0.0;
		stLimpAddiTrq.Lfoot.ty = (1 - alpha) * fndLimit(-delta_px * __MRobot * __Gravity, Trq_y_limit);
		stLimpAddiTrq.Lfoot.tx = (1 - alpha) * fndLimit(delta_py * __MRobot * __Gravity, Trq_x_limit);
		stLimpAddiTrq.Lfoot.fz = 0.0;
		//printf("DouSup\n");
		//printf("alpha = %f\n", alpha);
	}
	else if (RealSupFlag == Fly) {
		for (int i = 0; i < 3; i++) {
			*(&stLimpAddiTrq.Rfoot.fz + i) = 0.0;
			*(&stLimpAddiTrq.Lfoot.fz + i) = 0.0;
		}
		//printf("Fly\n");
	}
	else {
		printf("Error in fnsLipmAddiTrq !!!\n");
		for (int i = 0; i < 3; i++) {
			*(&stLimpAddiTrq.Rfoot.fz + i) = 0.0;
			*(&stLimpAddiTrq.Lfoot.fz + i) = 0.0;
		}
	}
#endif
	// re
	for (int i = 0; i < 3; i++) {
		dAddiTrqRe[i] = *(&stLimpAddiTrq.Rfoot.fz + i);
		dAddiTrqRe[i + 3] = *(&stLimpAddiTrq.Lfoot.fz + i);
	}
	dAddiTrqRe[6] = xmidpoint;
	dAddiTrqRe[7] = alpha;
}

/** Cal Ref FootFT 
* InputVal: dAdditrqPit, #FootFT_PG, LipmAddiTrq, FootFT_Sens, SupSignal, ZMP_B_Ref, Ankle_B_Cmd_old
* OutputVal: #FootFT_Ref
*/
void fnvCalFootFTRef(double dAdditrq[3]) {
	int RealSupFlag = Fly;
	double xmidpoint = 0.0;
	double alpha = 0.5;
	double Limit_alpha[2] = { 0.0, 1.0 }; 
	// phases ckeck
	if (stStatePG.SupLeg == RightSup) { // r sup
		RealSupFlag = RightSup;
		if (stStateSens.FootFT.Fsum < dFUpToFly) RealSupFlag = Fly;
		//printf("RightSup\n");
	}
	else if (stStatePG.SupLeg == LeftSup) { // l sup
		RealSupFlag = LeftSup;
		if (stStateSens.FootFT.Fsum < dFUpToFly) RealSupFlag = Fly;
		//printf("LeftSup\n");
	}
	else if (stStatePG.SupLeg == DouSup) { // dou sup
		RealSupFlag = DouSup;
		if (stStateSens.FootFT.Fsum < dFUpToFly) RealSupFlag = Fly;
		//printf("DouSup\n");
	}
	else { // fly
		RealSupFlag = Fly;
		//printf("Fly\n");
	}
	// distribution
	if (RealSupFlag == RightSup) {
		stStateRef.FootFT.Rfoot.tx = fndFilterTimeLag(stStateRef.FootFT.Rfoot.tx, 0.0 * stStatePG.FootFT.Rfoot.tx + stLimpAddiTrq.Rfoot.tx + dAdditrq[0], __ControlT, dTLagTrqRef);
		stStateRef.FootFT.Rfoot.ty = fndFilterTimeLag(stStateRef.FootFT.Rfoot.ty, 0.0 * stStatePG.FootFT.Rfoot.ty + stLimpAddiTrq.Rfoot.ty + dAdditrq[1], __ControlT, dTLagTrqRef);
		stStateRef.FootFT.Lfoot.tx = 0.0;
		stStateRef.FootFT.Lfoot.ty = 0.0;
		
		// stStateRef.FootFT.Rfoot.fz = stStatePG.FootFT.Rfoot.fz;
		// stStateRef.FootFT.Lfoot.fz = stStatePG.FootFT.Lfoot.fz;
		stStateRef.FootFT.Rfoot.fz = fndFilterTimeLag(stStateRef.FootFT.Rfoot.fz, __MRobot * (__Gravity + 1.0 * stStatePG.Base.pos.ddz), __ControlT, dTLagFrcRef);
		stStateRef.FootFT.Lfoot.fz = 0.0;
	}
	else if (RealSupFlag == LeftSup) {
		stStateRef.FootFT.Rfoot.tx = 0.0;
		stStateRef.FootFT.Rfoot.ty = 0.0;
		stStateRef.FootFT.Lfoot.tx = fndFilterTimeLag(stStateRef.FootFT.Lfoot.tx, 0.0 * stStatePG.FootFT.Lfoot.tx + stLimpAddiTrq.Lfoot.tx + dAdditrq[0], __ControlT, dTLagTrqRef);
		stStateRef.FootFT.Lfoot.ty = fndFilterTimeLag(stStateRef.FootFT.Lfoot.ty, 0.0 * stStatePG.FootFT.Lfoot.ty + stLimpAddiTrq.Lfoot.ty + dAdditrq[2], __ControlT, dTLagTrqRef);
		
		// stStateRef.FootFT.Rfoot.fz = stStatePG.FootFT.Rfoot.fz;
		// stStateRef.FootFT.Lfoot.fz = stStatePG.FootFT.Lfoot.fz;		
		stStateRef.FootFT.Rfoot.fz = 0.0;
		stStateRef.FootFT.Lfoot.fz = fndFilterTimeLag(stStateRef.FootFT.Lfoot.fz, __MRobot * (__Gravity + 1.0 * stStatePG.Base.pos.ddz), __ControlT, dTLagFrcRef);
	}
	else if (RealSupFlag == DouSup) {
		xmidpoint = (stStatePG.ZMP.B.x*stStateCmd.Ankle.B.Lfoot.pos.x*stStateCmd.Ankle.B.Lfoot.pos.x - 2 * stStatePG.ZMP.B.x*stStateCmd.Ankle.B.Lfoot.pos.x*stStateCmd.Ankle.B.Rfoot.pos.x - stStateCmd.Ankle.B.Lfoot.pos.x*stStateCmd.Ankle.B.Lfoot.pos.y*stStateCmd.Ankle.B.Rfoot.pos.y + stStatePG.ZMP.B.y*stStateCmd.Ankle.B.Lfoot.pos.x*stStateCmd.Ankle.B.Lfoot.pos.y + stStateCmd.Ankle.B.Lfoot.pos.x*stStateCmd.Ankle.B.Rfoot.pos.y*stStateCmd.Ankle.B.Rfoot.pos.y - stStatePG.ZMP.B.y*stStateCmd.Ankle.B.Lfoot.pos.x*stStateCmd.Ankle.B.Rfoot.pos.y + stStatePG.ZMP.B.x*stStateCmd.Ankle.B.Rfoot.pos.x*stStateCmd.Ankle.B.Rfoot.pos.x + stStateCmd.Ankle.B.Rfoot.pos.x*stStateCmd.Ankle.B.Lfoot.pos.y*stStateCmd.Ankle.B.Lfoot.pos.y - stStateCmd.Ankle.B.Rfoot.pos.x*stStateCmd.Ankle.B.Lfoot.pos.y*stStateCmd.Ankle.B.Rfoot.pos.y - stStatePG.ZMP.B.y*stStateCmd.Ankle.B.Rfoot.pos.x*stStateCmd.Ankle.B.Lfoot.pos.y + stStatePG.ZMP.B.y*stStateCmd.Ankle.B.Rfoot.pos.x*stStateCmd.Ankle.B.Rfoot.pos.y) / (stStateCmd.Ankle.B.Lfoot.pos.x*stStateCmd.Ankle.B.Lfoot.pos.x - 2 * stStateCmd.Ankle.B.Lfoot.pos.x*stStateCmd.Ankle.B.Rfoot.pos.x + stStateCmd.Ankle.B.Rfoot.pos.x*stStateCmd.Ankle.B.Rfoot.pos.x + stStateCmd.Ankle.B.Lfoot.pos.y*stStateCmd.Ankle.B.Lfoot.pos.y - 2 * stStateCmd.Ankle.B.Lfoot.pos.y*stStateCmd.Ankle.B.Rfoot.pos.y + stStateCmd.Ankle.B.Rfoot.pos.y*stStateCmd.Ankle.B.Rfoot.pos.y);
		alpha = fndLimit((xmidpoint - stStateCmd.Ankle.B.Lfoot.pos.x) / (stStateCmd.Ankle.B.Rfoot.pos.x - stStateCmd.Ankle.B.Lfoot.pos.x), Limit_alpha); // force propotion to right foot
		//alpha = fndLimit((stStatePG.ZMP.B.x - stStatePG.Ankle.B.Lfoot.pos.x) / (stStatePG.Ankle.B.Rfoot.pos.x - stStatePG.Ankle.B.Lfoot.pos.x), Limit_alpha);
		stStateRef.FootFT.Rfoot.tx = fndFilterTimeLag(stStateRef.FootFT.Rfoot.tx, 0.0 * stStatePG.FootFT.Rfoot.tx + alpha * dAdditrq[0] + stLimpAddiTrq.Rfoot.tx, __ControlT, dTLagTrqRef);
		stStateRef.FootFT.Rfoot.ty = fndFilterTimeLag(stStateRef.FootFT.Rfoot.ty, 0.0 * stStatePG.FootFT.Rfoot.ty + stLimpAddiTrq.Rfoot.ty, __ControlT, dTLagTrqRef);
		stStateRef.FootFT.Lfoot.tx = fndFilterTimeLag(stStateRef.FootFT.Lfoot.tx, 0.0 * stStatePG.FootFT.Lfoot.tx + (1 - alpha) * dAdditrq[0] + stLimpAddiTrq.Lfoot.tx, __ControlT, dTLagTrqRef);
		stStateRef.FootFT.Lfoot.ty = fndFilterTimeLag(stStateRef.FootFT.Lfoot.ty, 0.0 * stStatePG.FootFT.Lfoot.ty + stLimpAddiTrq.Lfoot.ty, __ControlT, dTLagTrqRef);
		
		// stStateRef.FootFT.Rfoot.fz = stStatePG.FootFT.Rfoot.fz;
		// stStateRef.FootFT.Lfoot.fz = stStatePG.FootFT.Lfoot.fz;
		stStateRef.FootFT.Rfoot.fz = fndFilterTimeLag(stStateRef.FootFT.Rfoot.fz, (alpha) * __MRobot * (__Gravity + 1.0 * stStatePG.Base.pos.ddz), __ControlT, dTLagFrcRef);
		stStateRef.FootFT.Lfoot.fz = fndFilterTimeLag(stStateRef.FootFT.Lfoot.fz, (1 - alpha) * __MRobot * (__Gravity + 1.0 * stStatePG.Base.pos.ddz), __ControlT, dTLagFrcRef);
		//printf("alpha = %f\n", alpha);
	}
	else if (RealSupFlag == Fly) {
		for (int i = 0; i < 3; i++) {
			*(&stStateRef.FootFT.Rfoot.fz + i) = 0.0;
			*(&stStateRef.FootFT.Lfoot.fz + i) = 0.0;
		}
	}
	else {
		printf("Error in fnsLipmAddiTrq !!!\n");
		for (int i = 0; i < 3; i++) {
			*(&stStateRef.FootFT.Rfoot.fz + i) = 0.0;
			*(&stStateRef.FootFT.Lfoot.fz + i) = 0.0;
		}
	}
	// re
	dFootFTRefRe[6] = xmidpoint;
	dFootFTRefRe[7] = alpha;
	nSupSig = stStatePG.SupLeg;
}

/** Ground reaction force control
* InputVal: Paras[d6], Paras[d2], Limits[d9], dLimitAddiTrq[d4], TreshHolds[d6], dVarStiffLat_T, #GrfConVal_old, FootFT_Ref, FootFT_Sens
* OutputVal:
*/

double kf_delF[4] = { 0.0 }; // Rpit, Lpit, Rrol, Lrol
double kp_delF[4] = { 0.0 }; // Rpit, Lpit, Rrol, Lrol
double kz_delF[2] = { 0.0 }; // Rz, Lz
int nInitFlag = 1;

void fnvGrfCon(double dParasGrfC[7], double dParasVarStiff[2], double dLimitsGrfC[7], double dLimitAddiTrq[4], double dThreshGrfC[6], double dVarStiffLat_T) {
	double kp_pitch = dParasGrfC[0]; double kd_pitch = dParasGrfC[1];
	double kp_roll  = dParasGrfC[2]; double kd_roll  = dParasGrfC[3];
	double kf_zctrl = dParasGrfC[4]; double kp_zctrl = dParasGrfC[5]; double kd_zctrl = dParasGrfC[6];
	double limit_pitch[4] = { -dLimitsGrfC[0], dLimitsGrfC[0], -dLimitsGrfC[1], dLimitsGrfC[1] };
	double limit_roll[4] = { -dLimitsGrfC[2], dLimitsGrfC[2], -dLimitsGrfC[3], dLimitsGrfC[3] };
	double limit_zctrl[6] = { dLimitsGrfC[4], dLimitsGrfC[5], -dLimitsGrfC[6], dLimitsGrfC[6], -100.0, 100.0 };
	double thresh_pitch[2] = { dThreshGrfC[0], dThreshGrfC[1] };
	double thresh_roll[2] = { dThreshGrfC[2], dThreshGrfC[3] };
	double thresh_zctrl[2] = { dThreshGrfC[4], dThreshGrfC[5] };
	// var stiff
#ifdef USE_VARSTIFF
	if (nInitFlag == 1) {
		nInitFlag = 0;
		for (int i = 0; i < 2; i++) {
			kf_delF[i] = kp_pitch;
			kp_delF[i] = kd_pitch;
			kf_delF[i + 2] = kp_roll;
			kp_delF[i + 2] = kd_roll;
		}
	}
	double k_kf[2] = { 0.0 }, b_kf[2] = { 0.0 }, k_kp[2] = { 0.0 }, b_kp[2] = { 0.0 }; // pit, rol
	double Zu = dParasVarStiff[0], Zd = dParasVarStiff[1];
	double delFmax[2] = { dLimitAddiTrq[1], dLimitAddiTrq[3] }; // pit, rol
	double LimitPit[2] = { dLimitAddiTrq[0], dLimitAddiTrq[1] }, LimitRol[2] = { dLimitAddiTrq[2], dLimitAddiTrq[3] };
	double delF[4] = { fndLimit(fabs(fndThreshold((stStateSens.FootFT.Rfoot.tx - stStateRef.FootFT.Rfoot.tx), thresh_pitch)), LimitPit), fndLimit(fabs(fndThreshold((stStateSens.FootFT.Lfoot.tx - stStateRef.FootFT.Lfoot.tx), thresh_pitch)), LimitPit), fndLimit(fabs(fndThreshold((stStateSens.FootFT.Rfoot.ty - stStateRef.FootFT.Rfoot.ty), thresh_roll)), LimitRol), fndLimit(fabs(fndThreshold((stStateSens.FootFT.Lfoot.ty - stStateRef.FootFT.Lfoot.ty), thresh_roll)), LimitRol)}; // Rpit, Lpit, Rrol, Lrol
	for (int i = 0; i < 2; i++) {
		k_kf[i] = (dParasGrfC[2 * i] * (Zd * Zu - 1)) / (Zd * delFmax[i]);
		b_kf[i] = dParasGrfC[2 * i] / Zd;
		k_kp[i] = -(dParasGrfC[2 * i + 1] * (Zd * Zu - 1)) / (Zd * delFmax[i]);
		b_kp[i] = Zu * dParasGrfC[2 * i + 1];
	}
	for (int i = 0; i < 2; i++) {
		kf_delF[i] = fndFilterTimeLag(kf_delF[i], k_kf[0] * delF[i] + b_kf[0], __ControlT, dVarStiffLat_T);
		kp_delF[i] = fndFilterTimeLag(kp_delF[i], k_kp[0] * delF[i] + b_kp[0], __ControlT, dVarStiffLat_T);
		kf_delF[i + 2] = fndFilterTimeLag(kf_delF[i + 2], k_kf[1] * delF[i + 2] + b_kf[1], __ControlT, dVarStiffLat_T);
		kp_delF[i + 2] = fndFilterTimeLag(kp_delF[i + 2], k_kp[1] * delF[i + 2] + b_kp[1], __ControlT, dVarStiffLat_T);
	}
#else
	for (int i = 0; i < 2; i++) {
		kf_delF[i] = kp_pitch;
		kp_delF[i] = kd_pitch;
		kf_delF[i + 2] = kp_roll;
		kp_delF[i + 2] = kd_roll;
	}
#endif
#ifdef USE_DCCFUZZY
	double Limit_Fz[2] = { 0.0, 1000.0 };
	double delF_z[2] = { fndLimit(fabs(fndThreshold((stStateSens.FootFT.Rfoot.fz - stStateRef.FootFT.Rfoot.fz), thresh_zctrl)), Limit_Fz), fndLimit(fabs(fndThreshold((stStateSens.FootFT.Lfoot.fz - stStateRef.FootFT.Lfoot.fz), thresh_zctrl)), Limit_Fz) };
	double H_step_z[2] = { stStatePG.Ankle.W.Rfoot.pos.z - __AnkleHeight, stStatePG.Ankle.W.Lfoot.pos.z - __AnkleHeight };
	for (int i = 0; i < 2; i++) {
		kz_delF[i] = fndCalFuzzyConVal(delF_z[i], H_step_z[i], FuzzyDispFlag); // control cycle: delF, H_step
	}
	// FrameAddLogsC(115, "Fuzzy_r",  &kz_delF[0], 1);
	// FrameAddLogsC(115, "Fuzzy_l",  &kz_delF[1], 1);
#else
	for (int i = 0; i < 2; i++) {
		kz_delF[i] = kf_zctrl;
	}
#endif
	// cal pit rol
	stGrfConVal.Rfoot.rot.dpit = fndFilterTimeLag(stGrfConVal.Rfoot.rot.dpit, kf_delF[0] * fndThreshold((stStateSens.FootFT.Rfoot.tx - stStateRef.FootFT.Rfoot.tx), thresh_pitch) - kp_delF[0] * stGrfConVal.Rfoot.rot.pit, __ControlT, dTLagConValRot);
	stGrfConVal.Rfoot.rot.drol = fndFilterTimeLag(stGrfConVal.Rfoot.rot.drol, kf_delF[2] * fndThreshold((stStateSens.FootFT.Rfoot.ty - stStateRef.FootFT.Rfoot.ty), thresh_roll ) - kp_delF[2] * stGrfConVal.Rfoot.rot.rol, __ControlT, dTLagConValRot);
	stGrfConVal.Lfoot.rot.dpit = fndFilterTimeLag(stGrfConVal.Lfoot.rot.dpit, kf_delF[1] * fndThreshold((stStateSens.FootFT.Lfoot.tx - stStateRef.FootFT.Lfoot.tx), thresh_pitch) - kp_delF[1] * stGrfConVal.Lfoot.rot.pit, __ControlT, dTLagConValRot);
	stGrfConVal.Lfoot.rot.drol = fndFilterTimeLag(stGrfConVal.Lfoot.rot.drol, kf_delF[3] * fndThreshold((stStateSens.FootFT.Lfoot.ty - stStateRef.FootFT.Lfoot.ty), thresh_roll ) - kp_delF[3] * stGrfConVal.Lfoot.rot.rol, __ControlT, dTLagConValRot);
	// cal z
	stGrfConVal.Rfoot.pos.ddz = fndFilterTimeLag(stGrfConVal.Rfoot.pos.ddz, kz_delF[0] * fndThreshold((stStateSens.FootFT.Rfoot.fz - stStateRef.FootFT.Rfoot.fz), thresh_zctrl) - kp_zctrl * stGrfConVal.Rfoot.pos.z - kd_zctrl * stGrfConVal.Rfoot.pos.dz, __ControlT, dTLagConValPos);
	stGrfConVal.Lfoot.pos.ddz = fndFilterTimeLag(stGrfConVal.Lfoot.pos.ddz, kz_delF[1] * fndThreshold((stStateSens.FootFT.Lfoot.fz - stStateRef.FootFT.Lfoot.fz), thresh_zctrl) - kp_zctrl * stGrfConVal.Lfoot.pos.z - kd_zctrl * stGrfConVal.Lfoot.pos.dz, __ControlT, dTLagConValPos);
	// update
	fnvVeloLimit(&stGrfConVal.Rfoot.rot.pit, stGrfConVal.Rfoot.rot.dpit, limit_pitch, __ControlT);
	fnvVeloLimit(&stGrfConVal.Rfoot.rot.rol, stGrfConVal.Rfoot.rot.drol, limit_roll , __ControlT);
	fnvVeloLimit(&stGrfConVal.Lfoot.rot.pit, stGrfConVal.Lfoot.rot.dpit, limit_pitch, __ControlT);
	fnvVeloLimit(&stGrfConVal.Lfoot.rot.rol, stGrfConVal.Lfoot.rot.drol, limit_roll , __ControlT);
	fnvIntegLimit(&stGrfConVal.Rfoot.pos.z, &stGrfConVal.Rfoot.pos.dz, stGrfConVal.Rfoot.pos.ddz, limit_zctrl, __ControlT);
	fnvIntegLimit(&stGrfConVal.Lfoot.pos.z, &stGrfConVal.Lfoot.pos.dz, stGrfConVal.Lfoot.pos.ddz, limit_zctrl, __ControlT);
#ifdef USE_GRFC
	stStateConVal.Ankle.B.Rfoot.rot.pit = 1.0 * stGrfConVal.Rfoot.rot.pit;
	stStateConVal.Ankle.B.Rfoot.rot.rol = 1.0 * stGrfConVal.Rfoot.rot.rol;
	stStateConVal.Ankle.B.Rfoot.pos.z   = 1.0 * stGrfConVal.Rfoot.pos.z;
	stStateConVal.Ankle.B.Lfoot.rot.pit = 1.0 * stGrfConVal.Lfoot.rot.pit;
	stStateConVal.Ankle.B.Lfoot.rot.rol = 1.0 * stGrfConVal.Lfoot.rot.rol;
	stStateConVal.Ankle.B.Lfoot.pos.z   = 1.0 * stGrfConVal.Lfoot.pos.z;
#endif
	// re
	for (int i = 0; i < 3; i++) {
		dFootFTRelRe[i] = *(&stStateSens.FootFT.Rfoot.fz + i);
		dFootFTRelRe[i + 3] = *(&stStateSens.FootFT.Lfoot.fz + i);
		dFootFTRefRe[i] = *(&stStateRef.FootFT.Rfoot.fz + i);
		dFootFTRefRe[i + 3] = *(&stStateRef.FootFT.Lfoot.fz + i);
	}
	dGrfConValRe[0] = stGrfConVal.Rfoot.pos.z; 
	dGrfConValRe[1] = stGrfConVal.Rfoot.rot.pit; 
	dGrfConValRe[2] = stGrfConVal.Rfoot.rot.rol;
	dGrfConValRe[3] = stGrfConVal.Lfoot.pos.z;
	dGrfConValRe[4] = stGrfConVal.Lfoot.rot.pit;
	dGrfConValRe[5] = stGrfConVal.Lfoot.rot.rol;
	
	// FrameAddLogsC(567, "RTxRef", &stStateRef.FootFT.Rfoot.tx, 1);
	// FrameAddLogsC(567, "RTxRel", &stStateSens.FootFT.Rfoot.tx, 1);
	// FrameAddLogsC(567, "RxCon",  &stGrfConVal.Rfoot.rot.pit, 1);
	// FrameAddLogsC(567, "RTyRef", &stStateRef.FootFT.Rfoot.ty, 1);
	// FrameAddLogsC(567, "RTyRel", &stStateSens.FootFT.Rfoot.ty, 1);
	// FrameAddLogsC(567, "RyCon",  &stGrfConVal.Rfoot.rot.rol, 1);
	// FrameAddLogsC(567, "RFzRef", &stStateRef.FootFT.Rfoot.fz, 1);
	// FrameAddLogsC(567, "RFzRel", &stStateSens.FootFT.Rfoot.fz, 1);
	// FrameAddLogsC(567, "RzCon",  &stGrfConVal.Rfoot.pos.z, 1);
	
	// FrameAddLogsC(567, "LTxRef", &stStateRef.FootFT.Lfoot.tx, 1);
	// FrameAddLogsC(567, "LTxRel", &stStateSens.FootFT.Lfoot.tx, 1);
	// FrameAddLogsC(567, "LxCon",  &stGrfConVal.Lfoot.rot.pit, 1);
	// FrameAddLogsC(567, "LTyRef", &stStateRef.FootFT.Lfoot.ty, 1);
	// FrameAddLogsC(567, "LTyRel", &stStateSens.FootFT.Lfoot.ty, 1);
	// FrameAddLogsC(567, "LyCon",  &stGrfConVal.Lfoot.rot.rol, 1);
	// FrameAddLogsC(567, "LFzRef", &stStateRef.FootFT.Lfoot.fz, 1);
	// FrameAddLogsC(567, "LFzRel", &stStateSens.FootFT.Lfoot.fz, 1);
	// FrameAddLogsC(567, "LzCon",  &stGrfConVal.Lfoot.pos.z, 1);
	for (int i = 0; i < 4; i++) {
		dKfKpRe[i] = kf_delF[i];
		dKfKpRe[i + 4] = kp_delF[i];
	}
}

/** TPC controller
InputVal: Bias[d2], TpcConVal_old[dccPositional], paras[d6], limits[d6], #ZMP_B_Ref, ZMP_B_Sens, FootFT_Sens
OutputVal: TpcConVal
*/
void fnvTpcCon(double dBias[2], double dParasTpc_in[6], double dLimitsTpc_in[6]) {
	double kz_x = dParasTpc_in[0];	double kz_y = dParasTpc_in[0 + 3];
	double kp_x = dParasTpc_in[1];	double kp_y = dParasTpc_in[1 + 3];
	double kd_x = dParasTpc_in[2];  double kd_y = dParasTpc_in[2 + 3];
	double limit_x[6] = { -dLimitsTpc_in[0], dLimitsTpc_in[0], -dLimitsTpc_in[1], dLimitsTpc_in[1], -dLimitsTpc_in[2], dLimitsTpc_in[2] };
	double limit_y[6] = { -dLimitsTpc_in[3], dLimitsTpc_in[3], -dLimitsTpc_in[4], dLimitsTpc_in[4], -dLimitsTpc_in[5], dLimitsTpc_in[5] };
	// fly protection: back to zero point
	if (stStateSens.FootFT.Fsum < dFUpToFly) kz_x = 0.0, kz_y = 0.0; 
	// cal acc
	// stTpcConVal.ddx = kz_x * (stStateSens.ZMP.B.x + dBias[0] - stStateRef.ZMP.B.x) - kp_x * stTpcConVal.x - kd_x * stTpcConVal.dx;
	// stTpcConVal.ddy = kz_y * (stStateSens.ZMP.B.y + dBias[1] - stStateRef.ZMP.B.y) - kp_y * stTpcConVal.y - kd_y * stTpcConVal.dy;
	stTpcConVal.ddx = kz_x * (stStateSens.ZMP.B.x + dBias[0] - stStatePG.ZMP.B.x) - kp_x * stTpcConVal.x - kd_x * stTpcConVal.dx;
	stTpcConVal.ddy = kz_y * (stStateSens.ZMP.B.y + dBias[1] - stStatePG.ZMP.B.y) - kp_y * stTpcConVal.y - kd_y * stTpcConVal.dy;
	// update
	fnvIntegLimit(&stTpcConVal.x, &stTpcConVal.dx, stTpcConVal.ddx, limit_x, __ControlT);
	fnvIntegLimit(&stTpcConVal.y, &stTpcConVal.dy, stTpcConVal.ddy, limit_y, __ControlT);
#ifdef USE_TPC
	stStateConVal.Base.pos.x = 1.0 * stTpcConVal.x;
	stStateConVal.Base.pos.y = stTpcConVal.y;
#endif
	// re
	// FrameAddLogsC(345, "TPCx", &stTpcConVal.x, 1);
	// FrameAddLogsC(345, "TPCy", &stTpcConVal.y, 1);
	dTpcRe[0] = stStateConVal.Base.pos.x;
	dTpcRe[1] = stStateConVal.Base.pos.y;
}

/** Support phase posture controller
InputVal: Paras[d6], Limits[d4], #Base_rot_Rel, Base_rot_Sens
OutputVal: #Base_rot_Ref, Base_drot_Ref 
*/
void fnvSupPosCon(double dParasSupPos_in[6], double dLimitSupPos_in[4]) {
	double k_pitch  = dParasSupPos_in[0]; double k_roll  = dParasSupPos_in[0 + 3];
	double kp_pitch = dParasSupPos_in[1]; double kp_roll = dParasSupPos_in[1 + 3];
	double kd_pitch = dParasSupPos_in[2]; double kd_roll = dParasSupPos_in[2 + 3];
	double limit_pitch[6] = { -dLimitSupPos_in[0], dLimitSupPos_in[0], -dLimitSupPos_in[1], dLimitSupPos_in[1], -50.0, 50.0};
	double limit_roll[6]  = { -dLimitSupPos_in[2], dLimitSupPos_in[2], -dLimitSupPos_in[3], dLimitSupPos_in[3], -50.0, 50.0};
	if (stStateSens.FootFT.Fsum < dFUpToFly) k_pitch = 0.0, k_roll = 0.0; // fly
	// cal acc
	// stSupPosConVal.ddpit = k_pitch * (stStatePG.Base.rot.pit - stStateSens.Base.rot.pit) - kp_pitch * stSupPosConVal.pit - kd_pitch * stSupPosConVal.dpit;
	// stSupPosConVal.ddrol = k_roll  * (stStatePG.Base.rot.rol - stStateSens.Base.rot.rol) - kp_roll  * stSupPosConVal.rol - kd_roll  * stSupPosConVal.drol;
	stSupPosConVal.ddpit = k_pitch * (stStateRef.Base.rot.pit - stStateSens.Base.rot.pit) - kp_pitch * stSupPosConVal.pit - kd_pitch * stSupPosConVal.dpit;
	stSupPosConVal.ddrol = k_roll  * (stStateRef.Base.rot.rol - stStateSens.Base.rot.rol) - kp_roll  * stSupPosConVal.rol - kd_roll  * stSupPosConVal.drol;
	// update
	fnvIntegLimit(&stSupPosConVal.pit, &stSupPosConVal.dpit, stSupPosConVal.ddpit, limit_pitch, __ControlT);
	fnvIntegLimit(&stSupPosConVal.rol, &stSupPosConVal.drol, stSupPosConVal.ddrol, limit_roll , __ControlT);
#ifdef USE_SUPPOSCON
	for (int i = 0; i < 2; i++) {
		// *(&stStateRef.Base.rot.pit + i) = *(&stStatePG.Base.rot.pit + i) + *(&stSupPosConVal.pit + i);
		*(&stStateConVal.Base.rot.pit + i) = *(&stSupPosConVal.pit + i);
	}
#endif
	//re
	dSupPosp[0] = stSupPosConVal.pit;
	dSupPosp[1] = stSupPosConVal.rol;
	dSupPosp[2] = stStateRef.Base.rot.pit;
	dSupPosp[3] = stStateSens.Base.rot.pit;
	dSupPosp[4] = stStateRef.Base.rot.rol;
	dSupPosp[5] = stStateSens.Base.rot.rol;
	
	// FrameAddLogsC(567, "PitPG" , &stStatePG.Base.rot.pit, 2);
	// FrameAddLogsC(567, "PitRef", &stStateRef.Base.rot.pit, 2);
	// FrameAddLogsC(567, "PitRel", &stStateSens.Base.rot.pit, 2);
}

void fnvArmSwi(double dParasArmSwi_in[2]) { // armswi
	double K_L = dParasArmSwi_in[0];
	double K_P = 1.0 - K_L;
	double v_M = dParasArmSwi_in[1];
	double dql_arm[3], dqr_arm[3]; // L, P, L + P
	dql_arm[0] = 2 * __AnkleWidth / (__ShouderWidth * __LArm * (__MArm + v_M)) * ((0.5 * __MThigh + __MShank + __MFoot) * __LThigh * stJoints.Ldq[2] + (0.5 * __MShank + __MFoot) * __LShank * stJoints.Ldq[3]);
	dqr_arm[0] = 2 * __AnkleWidth / (__ShouderWidth * __LArm * (__MArm + v_M)) * ((0.5 * __MThigh + __MShank + __MFoot) * __LThigh * stJoints.Rdq[2] + (0.5 * __MShank + __MFoot) * __LShank * stJoints.Rdq[3]);
	dqr_arm[1] = -2 / (__LArm * (__MArm + v_M)) * ((0.5 * __MThigh + __MShank + __MFoot) * __LThigh * stJoints.Ldq[2] + (0.5 * __MShank + __MFoot) * __LShank * stJoints.Ldq[3]);
	dql_arm[1] = -2 / (__LArm * (__MArm + v_M)) * ((0.5 * __MThigh + __MShank + __MFoot) * __LThigh * stJoints.Rdq[2] + (0.5 * __MShank + __MFoot) * __LShank * stJoints.Rdq[3]);
	dql_arm[2] = K_L * dql_arm[0] + K_P * dql_arm[1];
	dqr_arm[2] = K_L * dqr_arm[0] + K_P * dqr_arm[1];
#ifdef USE_ARMSWING
	stJoints.La = stJoints.La + dql_arm[2] * __ControlT;
	stJoints.Ra = stJoints.Ra + dqr_arm[2] * __ControlT;
#endif
	// re
	dArm[0] = stJoints.Ra;
	dArm[1] = stJoints.La;
}

double F_Ext_Late[6] = { 0.0 }; // check five times, so we need six mark point 
double F_Ext_Sagi[6] = { 0.0 };
double F_Vir_Late = 0.0;	double Tau_Vir_Late = 0.0;
double F_Vir_Sagi = 0.0;	double Tau_Vir_Sagi = 0.0;
double F_mpc_Sagi = 0.0;
double F_mpc_Late = 0.0;
int Count_Check_Release_x = 0;
int Count_Check_Release_y = 0;

RC_comp_onedirection RC_Comp_Late;
RC_comp_onedirection RC_Comp_Sagi;
RC_comp RC_Comp_Cont;

RC_comp_onedirection RC_Lateral(RC_comp_onedirection comp_late, double zmp_x_ref, double zmp_x_rel, double Z_c, double T_filter, double K_RC[4], double T_lag[2], double K_VMM[6], double micro[2], double T_start_walk)
{
	double F_ext;				double Tau_ext;
	double F_resi;				double Tau_resi;
	double K_1_e = K_RC[0];		double K_1_r = K_RC[2];
	double K_2_e = K_RC[1];		double K_2_r = K_RC[3];
	double K_p_e = K_VMM[0];	double K_p_r = K_VMM[3];
	double K_d_e = K_VMM[1];	double K_d_r = K_VMM[4];
	double K_m_e = K_VMM[2];	double K_m_r = K_VMM[5];
	double T_lag_e = T_lag[0];	double T_lag_r = T_lag[1];
	double RC_micro = micro[0]; double MPC_micro = 1.0; //micro[1];
	double Tau_micro = 1.0;
	double T_rls = 0.5; // cancel resist force in T_rls seconds after release the external force
	
	// F -------------------------------------------------------------------------------------------------
	F_ext = 1.0 * (0.0 * __MRobot * comp_late.ddel + __MRobot * __Gravity / Z_c * (zmp_x_rel - zmp_x_ref - 0.0 * comp_late.el));
	if (F_ext >  80) F_ext = 80;
	if (F_ext < -80) F_ext = -80;
	if (Count_Check_Release_x++ == T_rls * 0.2 / CONTROL_T) // continues decline of F_ext for five times, then release is judged
	{
		Count_Check_Release_x = 0;
		for (int i = 5; i > 0; i--)
		{
			F_Ext_Late[i] = F_Ext_Late[i - 1];
		}
	}
	F_Ext_Late[0] = (F_ext + T_filter / CONTROL_T * F_Ext_Late[0]) / (1 + T_filter / CONTROL_T);
	F_ext = F_Ext_Late[0];

	F_resi = -K_1_e * F_ext - K_2_e * comp_late.el;
	F_Vir_Late = (F_resi + T_lag_e / CONTROL_T * F_Vir_Late) / (1 + T_lag_e / CONTROL_T);
	F_resi = F_Vir_Late;

	#ifdef STANDING_PUSH_CHECK
		if ((abs(F_Ext_Late[5]) > abs(F_Ext_Late[4])) && (abs(F_Ext_Late[4]) > abs(F_Ext_Late[3])) && (abs(F_Ext_Late[3]) > abs(F_Ext_Late[2])) && (abs(F_Ext_Late[2]) > abs(F_Ext_Late[1])) && (abs(F_Ext_Late[1]) > abs(F_Ext_Late[0])))
		{
			F_resi = -F_ext; // if release, stop resisting
			F_Vir_Late = F_resi;
		}
	#endif

	//comp_late.dde = (F_mpc_x - K_p_e * comp_late.el - K_d_e * comp_late.del) / K_m_e;
	//comp_late.dde = (RC_micro * (F_ext + F_resi) - K_p_e * comp_late.el - K_d_e * comp_late.del) / K_m_e;
	//comp_late.dde = (RC_micro * (F_ext + F_resi) + /*MPC_micro * F_mpc_x*/ - K_p_e * comp_late.el - K_d_e * comp_late.del) / K_m_e;
	// comp_late.dde = (RC_micro * (F_ext + F_resi) + 1.0 * MPC_micro * F_MPC.x - K_p_e * comp_late.el - K_d_e * comp_late.del) / K_m_e;
	// comp_late.de = comp_late.del + comp_late.dde * CONTROL_T;
	// if (comp_late.de >  0.1) comp_late.de = 0.1;
	// if (comp_late.de < -0.1) comp_late.de = -0.1;
	// comp_late.e = comp_late.el + comp_late.de * CONTROL_T;
	// if (comp_late.e >  0.07) comp_late.e = 0.07;
	// if (comp_late.e < -0.07) comp_late.e = -0.07;

	// comp_late.ddel = comp_late.dde;
	// comp_late.del = comp_late.de;
	// comp_late.el = comp_late.e;
	
	comp_late.e = (RC_micro * (F_ext + F_resi) / K_p_e  + (K_d_e / CONTROL_T / K_p_e + K_m_e / CONTROL_T / CONTROL_T / K_p_e) * comp_late.el + K_m_e / CONTROL_T / K_p_e * comp_late.del) / (1 + K_d_e / CONTROL_T / K_p_e + K_m_e / CONTROL_T / CONTROL_T / K_p_e);
	if (comp_late.e >  0.07) comp_late.e =  0.07;
	if (comp_late.e < -0.07) comp_late.e = -0.07;
	comp_late.de = (comp_late.e - comp_late.el) / CONTROL_T;
	comp_late.dde = (comp_late.de - comp_late.del) / CONTROL_T;
	comp_late.ddel = comp_late.dde;
	comp_late.del = comp_late.de;
	comp_late.el = comp_late.e;

	// Tau -----------------------------------------------------------------------------------------------
	Tau_ext = F_ext * 0.3 * Z_c;

	Tau_resi = -K_1_r * Tau_ext - K_2_r * comp_late.rl;
	Tau_Vir_Late = (Tau_resi + T_lag_r / CONTROL_T * Tau_Vir_Late) / (1 + T_lag_r / CONTROL_T);
	Tau_resi = Tau_Vir_Late;

	// comp_late.ddr = (Tau_micro * (Tau_ext + Tau_resi) - K_p_r * comp_late.rl - K_d_r * comp_late.drl) / K_m_r;
	// comp_late.dr = comp_late.drl + comp_late.ddr * CONTROL_T;
	// if (comp_late.dr >  20 / 57.3) comp_late.dr = 20 / 57.3;
	// if (comp_late.dr < -20 / 57.3) comp_late.dr = -20 / 57.3;
	// comp_late.r = comp_late.rl + comp_late.dr * CONTROL_T;
	// if (comp_late.r >  8 / 57.3) comp_late.r = 8 / 57.3;
	// if (comp_late.r < -8 / 57.3) comp_late.r = -8 / 57.3;

	// comp_late.ddrl = comp_late.ddr;
	// comp_late.drl = comp_late.dr;
	// comp_late.rl = comp_late.r;

	comp_late.r = 1.0 * ((RC_micro * (Tau_ext + Tau_resi) / K_p_r + (K_d_r / CONTROL_T / K_p_r + K_m_r / CONTROL_T / CONTROL_T / K_p_r) * comp_late.rl + K_m_r / CONTROL_T / K_p_r * comp_late.drl) / (1 + K_d_r / CONTROL_T / K_p_r + K_m_r / CONTROL_T / CONTROL_T / K_p_r));
	if (comp_late.r >  12 / 57.3) comp_late.r =  12 / 57.3;
	if (comp_late.r < -12 / 57.3) comp_late.r = -12 / 57.3;
	comp_late.dr = (comp_late.r - comp_late.rl) / CONTROL_T;
	comp_late.ddr = (comp_late.dr - comp_late.drl) / CONTROL_T;
	comp_late.ddrl = comp_late.ddr;
	comp_late.drl = comp_late.dr;
	comp_late.rl = comp_late.r;

	return comp_late;
}

RC_comp_onedirection RC_Sagitta(RC_comp_onedirection comp_sagi, double zmp_y_ref, double zmp_y_rel, double Z_c, double T_filter, double K_RC[4], double T_lag[2], double K_VMM[6], double micro[2], double T_start_walk)
{
	double F_ext;				double Tau_ext;
	double F_resi;				double Tau_resi;
	double K_1_e = K_RC[0];		double K_1_r = K_RC[2];
	double K_2_e = K_RC[1];		double K_2_r = K_RC[3];
	double K_p_e = K_VMM[0];	double K_p_r = K_VMM[3];
	double K_d_e = K_VMM[1];	double K_d_r = K_VMM[4];
	double K_m_e = K_VMM[2];	double K_m_r = K_VMM[5];
	double T_lag_e = T_lag[0];	double T_lag_r = T_lag[1];
	double RC_micro = micro[0]; double MPC_micro = micro[1];
	double Tau_micro = 1.0;
	double T_rls = 0.5; // cancel resist force in T_rls seconds after release the external force

	// double Ref_Leg_Joint[3][7];

	// F ------------------------------------------------------t-------------------------------------------
	// F_ext = 0.0 * __MRobot * comp_sagi.ddel + __MRobot * __Gravity / Z_c * (zmp_y_rel - zmp_y_ref - 0.0 * comp_sagi.el) + 0.3 * (-2.0 + 24.5 * 0.5 * (Ref_Leg_Joint[1][4] + Ref_Leg_Joint[2][4]));
	F_ext = 0.0;
	if (F_ext >  80) F_ext = 80;
	if (F_ext < -80) F_ext = -80;
	if (Count_Check_Release_y++ == T_rls * 0.2 / CONTROL_T) // continues decline of F_ext for five times, then release is judged
	{
		Count_Check_Release_y = 0;
		for (int i = 5; i > 0; i--)
		{
			F_Ext_Sagi[i] = F_Ext_Sagi[i - 1];
		}
	}
	F_Ext_Sagi[0] = (F_ext + T_filter / CONTROL_T * F_Ext_Sagi[0]) / (1 + T_filter / CONTROL_T);
	F_ext = F_Ext_Sagi[0];

	F_resi = -K_1_e * F_ext - K_2_e * comp_sagi.el;
	F_Vir_Sagi = (F_resi + T_lag_e / CONTROL_T * F_Vir_Sagi) / (1 + T_lag_e / CONTROL_T);
	F_resi = F_Vir_Sagi;

	#ifdef STANDING_PUSH_CHECK
		if ((abs(F_Ext_Sagi[5]) > abs(F_Ext_Sagi[4])) && (abs(F_Ext_Sagi[4]) > abs(F_Ext_Sagi[3])) && (abs(F_Ext_Sagi[3]) > abs(F_Ext_Sagi[2])) && (abs(F_Ext_Sagi[2]) > abs(F_Ext_Sagi[1])) && (abs(F_Ext_Sagi[1]) > abs(F_Ext_Sagi[0])))
		{
			F_resi = -F_ext; // if release, stop resisting
			F_Vir_Sagi = F_resi;
		}
	#endif

	//comp_sagi.dde = (F_mpc_y - K_p_e * comp_sagi.el - K_d_e * comp_sagi.del) / K_m_e;
	//comp_sagi.dde = (RC_micro * (F_ext + F_resi)  - K_p_e * comp_sagi.el - K_d_e * comp_sagi.del) / K_m_e;
	//comp_sagi.dde = (RC_micro * (F_ext + F_resi) + /*MPC_micro * F_mpc_y*/ - K_p_e * comp_sagi.el - K_d_e * comp_sagi.del) / K_m_e;
	//comp_sagi.dde = (RC_micro * (F_ext + F_resi) + 1.0 * MPC_micro * F_MPC.y - K_p_e * comp_sagi.el - K_d_e * comp_sagi.del) / K_m_e;
	// comp_sagi.de = comp_sagi.del + comp_sagi.dde * CONTROL_T;
	// if (comp_sagi.de >  0.1) comp_sagi.de = 0.1;
	// if (comp_sagi.de < -0.1) comp_sagi.de = -0.1;
	// comp_sagi.e = comp_sagi.el + comp_sagi.de * CONTROL_T;
	// if (comp_sagi.e >  0.05) comp_sagi.e = 0.05;
	// if (comp_sagi.e < -0.05) comp_sagi.e = -0.05;

	// comp_sagi.ddel = comp_sagi.dde;
	// comp_sagi.del = comp_sagi.de;
	// comp_sagi.el = comp_sagi.e;
	
	comp_sagi.e = (RC_micro * (F_ext + F_resi) / K_p_e  + (K_d_e / CONTROL_T / K_p_e + K_m_e / CONTROL_T / CONTROL_T / K_p_e) * comp_sagi.el + K_m_e / CONTROL_T / K_p_e * comp_sagi.del) / (1 + K_d_e / CONTROL_T / K_p_e + K_m_e / CONTROL_T / CONTROL_T / K_p_e);
	if (comp_sagi.e >  0.1) comp_sagi.e =  0.1;
	if (comp_sagi.e < -0.1) comp_sagi.e = -0.1;
	comp_sagi.de = (comp_sagi.e - comp_sagi.el) / CONTROL_T;
	comp_sagi.dde = (comp_sagi.de - comp_sagi.del) / CONTROL_T;
	comp_sagi.ddel = comp_sagi.dde;
	comp_sagi.del = comp_sagi.de;
	comp_sagi.el = comp_sagi.e;

	// Tau -----------------------------------------------------------------------------------------------
	Tau_ext = F_ext * 0.3 * Z_c;

	Tau_resi = -K_1_r * Tau_ext - K_2_r * comp_sagi.rl;
	Tau_Vir_Sagi = (Tau_resi + T_lag_r / CONTROL_T * Tau_Vir_Sagi) / (1 + T_lag_r / CONTROL_T);
	Tau_resi = Tau_Vir_Sagi;

	// comp_sagi.ddr = (Tau_micro * (Tau_ext + Tau_resi) - K_p_r * comp_sagi.rl - K_d_r * comp_sagi.drl) / K_m_r;
	// comp_sagi.dr = comp_sagi.drl + comp_sagi.ddr * CONTROL_T;
	// if (comp_sagi.dr >  20 / 57.3) comp_sagi.dr = 20 / 57.3;
	// if (comp_sagi.dr < -20 / 57.3) comp_sagi.dr = -20 / 57.3;
	// comp_sagi.r = comp_sagi.rl + comp_sagi.dr * CONTROL_T;
	// if (comp_sagi.r >  10 / 57.3) comp_sagi.r =  10 / 57.3;
	// if (comp_sagi.r < -10 / 57.3) comp_sagi.r = -10 / 57.3;

	// comp_sagi.ddrl = comp_sagi.ddr;
	// comp_sagi.drl = comp_sagi.dr;
	// comp_sagi.rl = comp_sagi.r;
	
	comp_sagi.r = 1.0 * ((RC_micro * (Tau_ext + Tau_resi) / K_p_r + (K_d_r / CONTROL_T / K_p_r + K_m_r / CONTROL_T / CONTROL_T / K_p_r) * comp_sagi.rl + K_m_r / CONTROL_T / K_p_r * comp_sagi.drl) / (1 + K_d_r / CONTROL_T / K_p_r + K_m_r / CONTROL_T / CONTROL_T / K_p_r));
	if (comp_sagi.r >  20 / 57.3) comp_sagi.r =  20 / 57.3;
	if (comp_sagi.r < -20 / 57.3) comp_sagi.r = -20 / 57.3;
	comp_sagi.dr = (comp_sagi.r - comp_sagi.rl) / CONTROL_T;
	comp_sagi.ddr = (comp_sagi.dr - comp_sagi.drl) / CONTROL_T;
	comp_sagi.ddrl = comp_sagi.ddr;
	comp_sagi.drl = comp_sagi.dr;
	comp_sagi.rl = comp_sagi.r;

	return comp_sagi;
}

void VMM_RC_controller(double Z_c, int nStateFlag)
{
	double T_start_walk = 4.0;
	double mpc_micro_x[2] = { 0.4, 0.01 * 0.4 }; // { RC_micro, MPC_micro } { 1.0, 1.0 * 0.065 };	 // { 1.0, 1.0 } for standing on rotating slope
	double mpc_micro_y[2] = { 0.4, 0.01 * 0.4 }; // { RC_micro, MPC_micro } { 1.0, 1.0 * 0.1   };	     // { 1.0, 1.0 } for standing on rotating slope	
	double T_filter_late = 0.10; // Time lag of input Force for lateral
	double T_filter_sagi = 0.10; // Time lag of input Force for sagitta
	double K_RC_late[4] = { 1.1 * 1.60, 50.0, 1.3 * 1.50, 30.0 }; // {K_1_e, K_2_e, K_1_r, K_2_r} for lateral  { 1.6 * 1.20, 120.0, 2.2 * 1.17, 100.0 }; // for standing on rotating slope
	double K_RC_sagi[4] = { 1.2, 50.0, 1.1, 30.0 }; // {K_1_e, K_2_e, K_1_r, K_2_r} for sagitta
	double T_lag_sagi[2] = { 1.6, 1.0 }; // {T_lag_e, T_lag_r} for lateral
	double T_lag_late[2] = { 1.6, 0.8 }; // {T_lag_e, T_lag_r} for saggita
	double K_VMM_late[6] = { 1.7 * 620.0, 0.4 * 290.0, 2.0, 1.7 * 70.0, 0.4 * 13.0, 0.0086 }; //500.0, 250.0, 2.0, 60.0, 12.0, 0.0086//720.0, 320.0, 2.0, 85.0, 15.0, 0.0086// {K_p_e, K_d_e, K_m_e, K_p_r, K_d_r, K_m_r} for lateral
	
	if(nStateFlag == 0){ // stand still
		K_VMM_late[0] = 1.6 * 1.7 * 620.0;
		K_VMM_late[1] = 0.6 * 0.4 * 290.0;
		K_VMM_late[2] = 2.0;
		K_VMM_late[3] = 1.6 * 1.7 * 70.0;
		K_VMM_late[4] =	0.6 * 0.4 * 13.0;
		K_VMM_late[5] = 0.0086;
	}
	else if(nStateFlag == 1){ // walk
		K_VMM_late[0] = 1.7 * 620.0;
		K_VMM_late[1] = 0.4 * 290.0;
		K_VMM_late[2] = 2.0;
		K_VMM_late[3] = 1.7 * 70.0;
		K_VMM_late[4] =	0.4 * 13.0;
		K_VMM_late[5] = 0.0086;
	}
	
	double K_VMM_sagi[6] = { 1.2 * 720.0, 1.2 * 400.0, 2.0, 1.2 * 50.0, 1.2 * 9.0, 0.0086 }; //500.0, 250.0, 2.0, 60.0, 12.0, 0.0086//720.0, 320.0, 2.0, 85.0, 15.0, 0.0086// {K_p_e, K_d_e, K_m_e, K_p_r, K_d_r, K_m_r} for sagitta
	double K_model[3] = { 260.0, 120.0, 22.0 };
	double mech_paras[2] = { 1.0, 22.0};
	double R = 1e-7;
	double T_pre = 1.0;
	double rol = 1e-2;
	
	double zmp_x_ref = stStatePG.ZMP.W.x;
	double zmp_y_ref = stStatePG.ZMP.W.y;

	double zmp_x_rel = stStateSens.ZMP.W.x;
	double zmp_y_rel = stStateSens.ZMP.W.y;

	RC_Comp_Late = RC_Lateral(RC_Comp_Late, zmp_x_ref, zmp_x_rel, Z_c, T_filter_late, K_RC_late, T_lag_late, K_VMM_late, mpc_micro_x, T_start_walk);
	RC_Comp_Sagi = RC_Sagitta(RC_Comp_Sagi, zmp_y_ref, zmp_y_rel, Z_c, T_filter_sagi, K_RC_sagi, T_lag_sagi, K_VMM_sagi, mpc_micro_y, T_start_walk);
	RC_Comp_Cont.late = RC_Comp_Late;
	RC_Comp_Cont.sagi = RC_Comp_Sagi;
	
	#ifdef USE_RC
		stStateConVal.Base.pos.x 	+= RC_Comp_Cont.late.e;
		stStateConVal.Base.rot.rol 	+= RC_Comp_Cont.late.r;
		stStateConVal.Base.pos.y 	+= RC_Comp_Cont.sagi.e;
		stStateConVal.Base.rot.pit 	-= RC_Comp_Cont.sagi.r;
	#endif
}

void fnvDccRunCon() {
	// paras FootGeom
	// rec dcc
	double dFootGeom[4] = { /*forw*/BHRConfigC.FootFord, /*back*/BHRConfigC.FootBack, /*iner*/BHRConfigC.FootInner, /*outer*/BHRConfigC.FootOuter };
	// paras Limp
	double dParasLipm[6] = { /*kp_x, kv_x, kz_x*/1.56, 0.34, 0.01, /*kp_y, kv_y, kz_y*/1.56, 0.34, 0.01 };
	double dLimitsLipm[4] = { /*x*/-0.0, 0.0, /*y*/-0.0, 0.0 };
	// paras MZmp
	char   cMethodMZmp = 'p';
	double dParasMZmp[6] = { /*micro_x, kp_x, kd_x*/120.0, 60.0, 10.0, /*micro_y, kp_y, kd_y*/120.0, 60.0, 10.0 };
	double dLimitMZmp[2] = { /*x*/DccD2R(5.0), /*y*/DccD2R(10.0) };
	// paras AddiTrq
	double dParasAddiTrq[2] = { /*micro_x*/0.38/* * 1.7*/, /*micro_y*/0.35/* * 1.2*/ }; // smaller wilder
	double dLimitAddiTrq[4] = { /*trq_x*/-40.0 + 5.0, 55.0 - 5.0, /*trq_y*/-34.0 + 4.0, 34.0 - 4.0 };
	// paras GrfCon
	double dAdditrq[3] = { /*pit*/-8.5, /*rol_r*/0.0, /*rol_l*/0.0 };
	double dParasGrfC[7] = { /*pit*/0.02, 2.0, /*rol*/0.02, 2.0, /*zctrl*/0.004, 250.0, 80.0 };
	double dParasVarStiff[2] = { /*Zu*/2.0, /*Zd*/2.5 };
	double dLimitsGrfC[7] = { /*pit*/DccD2R(20.0), 10.0, /*rol*/DccD2R(15.0), 10.0, /*zctrl*/-0.01, 0.02, 10.0 };
	double dThreshGrfC[6] = { /*pit*/-0.0, 0.0, /*rol*/-0.0, 0.0, /*zctrl*/-10.0, 10.0 };
	double dVarStiffLat_T = 0.04;
	// paras Tpc
	double dBias[2] = { /*x*/0.0, /*y*/-0.0 };
	double dParasTpc[6] = { /*kz_x, kp_x, kv_x*/26.340529, 68.782720, 25.445165, /*kz_y, kp_y, kv_y*/32.98669, 58.31472, 21.17373 };
	double dLimitsTpc[6] = { /*x*/0.04, 10.0, 50.0, /*y*/0.04, 10.0, 50.0 };
	// paras SupPos
	// double dParasSupPos[6] = { /*kpit, kp, kd*/1.0 * 85.0, 30.0, 20.0, /*krol, kp, kd*/1.0 * 80.0, 30.0, 20.0 };
	// double dParasSupPos[6] = { /*kpit, kp, kd*/1.0 * 120.0, 50.0, 20.0, /*krol, kp, kd*/1.0 * 120.0, 50.0, 20.0 };
	double dParasSupPos[6] = { /*kpit, kp, kd*/1.0 * 200.0, 60.0, 25.0, /*krol, kp, kd*/1.0 * 200.0, 60.0, 25.0 };
	double dLimitSupPos[4] = { /*pit*/DccD2R(45.0), 100.0, /*rol*/DccD2R(45.0), 100.0 };
	// paras ArmSwi
	double dParasArmSwi[2] = { /*K_L*/ 0.8049, /*VritualMassForArm*/ 20.0 };
	// controllers
	fnvGetSupPoly(dFootGeom);								// -> SupPoly rec dcc
	fnvLipmCon(dParasLipm, dLimitsLipm);					// -> ZMP_Ref 
	fnvModelZmpCon(dParasMZmp, dLimitMZmp, cMethodMZmp);	// -> ModelZmp rec dcc
	fnvLipmAddiTrq(dParasAddiTrq, dLimitAddiTrq);			// -> LipmAddiTrq
	fnvCalFootFTRef(dAdditrq);								// -> FootFT_Ref
	fnvGrfCon(dParasGrfC, dParasVarStiff, dLimitsGrfC, dLimitAddiTrq, dThreshGrfC, dVarStiffLat_T);	// -> AnkleConVal
	fnvTpcCon(dBias, dParasTpc, dLimitsTpc);				// -> TpcConVal
	fnvSupPosCon(dParasSupPos, dLimitSupPos);				// -> SupPosConVal
	fnvArmSwi(dParasArmSwi);								// -> ArmSwi
	int nStateFlag = 0;
	if(dStandNow >= -1e-8 && dStandNow < dStand) nStateFlag = 1;
	VMM_RC_controller(1.0, nStateFlag);
}
 
#endif 