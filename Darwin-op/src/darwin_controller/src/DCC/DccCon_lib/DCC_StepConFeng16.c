// 20220211 bit first code
// two biggest differences against feng16:
// 1. feng16 generate CoM trajectory online according to the re-planned foot print (ZMP); our CoM trajectory can not be rectified.
// 2. feng16 rule all the position in world frame; we rule these position in local frame.
#ifndef DCC_STEPCONFENG_C
#define DCC_STEPCONFENG_C
// #include "DCC_RunningCon.h"
// #include "DCC_StepCOnFeng16.h"
// #include "..\..\ChzFrameCpp2C.h"
#include <Compliance/DCC/DccCon_lib/DCC_RunningCon.h>
#include <Compliance/DCC/DccCon_lib/DCC_StepConFeng16.h>

dccAnkle StepFengConVal = { 0.0 };
dccPositional dRFootRef = { 0.0 }, dLFootRef = { 0.0 };  
double dZFeng16 = 0.63;
double dReFeng[6] = { 0.0 };

// first trial: using cp theory, check CoM & CoM speed error at current and cal cp, then tracking with pd
void fnvStepConCurCpNextStep(double dKTrack[2], double dKResto[2]){
    double dAmp[4] = { /*x*/1.0, /*dx*/0.2, /*y*/1.0, /*dy*/0.2 };
    double dOme = sqrt(__Gravity / dZFeng16);
    double dThresh[4] = { /*x*/-0.0 * 0.04, 0.0 * 0.04, /*y*/-0.0 * 0.04, 0.0 * 0.06 };
    double dStartTrackTime = 0.05;
    double dKR[2] = { 0.0 }, dKL[2] = { 0.0 };
    double dLimitRx[6] = { 0.0, 0.04, -50.0, 50.0, -500.0, 500.0 }, dLimitLx[6] = { -0.04, 0.0, -50.0, 50.0, -500.0, 500.0 }, dLimity[6] = { -0.04, 0.04, -50.0, 50.0, -500.0, 500.0 };
    dccPositional dBaseErr = { 0.0 };
    dccPositional dCpCur = { 0.0 };
    // cal cp
	dBaseErr.x = dZFeng16 * sin(stStateSens.Base.rot.rol - stStateRef.Base.rot.rol - stStateConVal.Base.rot.rol);
	dBaseErr.y = -dZFeng16 * sin(stStateSens.Base.rot.pit - stStateRef.Base.rot.pit - stStateConVal.Base.rot.pit);
	dBaseErr.dx = dZFeng16 * sin(stStateSens.Base.rot.drol - stStateRef.Base.rot.drol - stStateConVal.Base.rot.drol);
	dBaseErr.dy = -dZFeng16 * sin(stStateSens.Base.rot.dpit - stStateRef.Base.rot.dpit - stStateConVal.Base.rot.dpit);
    for(int i = 0; i < 2; i++) *(&dCpCur.x + i) = fndThreshold(dAmp[2 * i] * *(&dBaseErr.x + i) + dAmp[1 + 2 * i] * *(&dBaseErr.dx + i) / dOme, (&dThresh[0] + 2 * i)) ; // cal cpcur with threshold
    // check supleg: set zero in dou sup & sin sup, keep unchanged in fly & swi
    if (stStatePG.SupLeg == 1) { // r sup
        dRFootRef.x = 0.0, dRFootRef.y = 0.0; // set r
        dLFootRef.x = dCpCur.x, dLFootRef.y = dCpCur.y; // set l
        for(int i = 0; i < 2; i++) dKR[i] = dKResto[i], dKL[i] = dKTrack[i];
    }
    else if(stStatePG.SupLeg == 2) { // l sup
        dRFootRef.x = dCpCur.x, dRFootRef.y = dCpCur.y; // set r
        dLFootRef.x = 0.0, dLFootRef.y = 0.0; // set l
        for(int i = 0; i < 2; i++) dKR[i] = dKTrack[i], dKL[i] = dKResto[i];
    }
    else{ // dou sup & fly
        for(int i = 0; i < 2; i++) dKR[i] = dKResto[i], dKL[i] = dKResto[i];
    }
    // tracking
    for(int i = 0; i < 2; i++){
        *(&StepFengConVal.Rfoot.pos.ddx + i) = dKR[0] * (*(&dRFootRef.x + i) - *(&StepFengConVal.Rfoot.pos.x + i)) - dKR[1] * *(&StepFengConVal.Rfoot.pos.dx + i);
        *(&StepFengConVal.Lfoot.pos.ddx + i) = dKL[0] * (*(&dLFootRef.x + i) - *(&StepFengConVal.Lfoot.pos.x + i)) - dKL[1] * *(&StepFengConVal.Lfoot.pos.dx + i);
    }
    fnvIntegLimit(&StepFengConVal.Rfoot.pos.x, &StepFengConVal.Rfoot.pos.dx, StepFengConVal.Rfoot.pos.ddx, dLimitRx, __ControlT);
    fnvIntegLimit(&StepFengConVal.Rfoot.pos.y, &StepFengConVal.Rfoot.pos.dy, StepFengConVal.Rfoot.pos.ddy, dLimity, __ControlT);
    fnvIntegLimit(&StepFengConVal.Lfoot.pos.x, &StepFengConVal.Lfoot.pos.dx, StepFengConVal.Lfoot.pos.ddx, dLimitLx, __ControlT);
    fnvIntegLimit(&StepFengConVal.Lfoot.pos.y, &StepFengConVal.Lfoot.pos.dy, StepFengConVal.Lfoot.pos.ddy, dLimity, __ControlT);
    #ifdef USE_STEPCON_CPCUR
        stStateConVal.Ankle.B.Rfoot.pos.x += StepFengConVal.Rfoot.pos.x, stStateConVal.Ankle.B.Rfoot.pos.y += StepFengConVal.Rfoot.pos.y;
        stStateConVal.Ankle.B.Lfoot.pos.x += StepFengConVal.Lfoot.pos.x, stStateConVal.Ankle.B.Lfoot.pos.y += StepFengConVal.Lfoot.pos.y;
    #endif
    // FrameAddLogsC(1024, "stepfeng_rx",  &StepFengConVal.Rfoot.pos.x, 1);
    // FrameAddLogsC(1024, "stepfeng_lx",  &StepFengConVal.Lfoot.pos.x, 1);
    // FrameAddLogsC(1024, "stepfeng_ry",  &StepFengConVal.Rfoot.pos.y, 1);
    // FrameAddLogsC(1024, "stepfeng_ly",  &StepFengConVal.Lfoot.pos.y, 1);
    //re
    dReFeng[0] = StepFengConVal.Rfoot.pos.x;
    dReFeng[1] = StepFengConVal.Lfoot.pos.x;
    dReFeng[2] = StepFengConVal.Rfoot.pos.y;
    dReFeng[3] = StepFengConVal.Lfoot.pos.y;
    dReFeng[4] = dCpCur.x;
    dReFeng[5] = dCpCur.y;
}



#endif