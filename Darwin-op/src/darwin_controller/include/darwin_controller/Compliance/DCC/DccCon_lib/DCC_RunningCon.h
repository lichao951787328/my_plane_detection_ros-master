// 20210111 bit
#pragma once
#ifndef DCC_RunningCon_H
#define DCC_RunningCon_H
#ifdef DCC_RunningCon_C
#define Extern
#else
#define Extern extern
#endif
// recording
Extern double dSupPoly[8]; // { B:forw, back, left, righ, W:forw, back, left, righ }
// SupSig
Extern int nSupSig;
// Limp
Extern double dLipmRe[12]; // { delta_cx, delta_vx, delta_cy, delta_vy, lipm_px, px_ref, px_sens, px_pg, lipm_py, py_ref, py_sens, py_pg }
// MZmp // rec dcc
Extern double dMZmpRe[7]; // { sur_px, sur_py, md_cx, md_cy }
// AddiTrq Lipm
Extern double dAddiTrqRe[8]; // { frz, trx, try, flz, tlx, tly, xmid, alpha }
// CalFootFTRef
Extern double dFootFTRefRe[8]; // { frz, trx, try, flz, tlx, tly, xmid, alpha }
// GrfCon
Extern double dFootFTRelRe[6]; // { frz, trx, try, flz, tlx, tly  }
Extern double dGrfConValRe[6]; // { prz, rrx, rry, plz, rlx, rly }
Extern double dKfKpRe[8]; // { kf:Rpit, Lpit, Rrol, Lrol, kp:Rpit, Lpit, Rrol, Lrol }
// Tpc
Extern double dTpcRe[2]; // { tpc_cx, tpc_cy }
// SupPos
Extern double dSupPosp[2]; // { pit, rol }
// ArmSwi
Extern double dArm[2]; // { Ra, La }
// temp
Extern double TempDataRe[20];
#undef Extern

#include "DCC_ConFrame.h"
#include "Dcc_lib/Base/dcc_FuzzyCon.h"
#include "Dcc_lib/Base/dcc_con_base.h"

// ================== General ==================
#define USE_ARMSWING
#define USE_DYBIAS

// ================== DCC Method ==================
#define USE_DCC_METHOD
#ifdef USE_DCC_METHOD
// #define USE_LIPM
// #define USE_MODELZMP // dcc rec
// #define USE_ADDITRQ
// #define USE_GRFC
// // #define CKECK_TIMELAG_OF_GRFC
// #define USE_VARSTIFF
// #define USE_DCCFUZZY
// #define USE_SUPPOSCON
#endif

// ================== Honda Method ==================
#define USE_HONDA_METHOD
#ifdef USE_HONDA_METHOD
    // #define USE_MDZMP_HONDA
    #define USE_FBMOMENT_HONDA
    #define USE_POSTURESTABLIZE_HONDA
    #define USE_2ND_ORDER
    
    // #define USE_COMPLIANCE_HONDA
    // #define USE_STEPCON_NEW // not so good
    // #define USE_STEPCON_Z // not so good
    // #define USE_STEPCON_CPCUR
#endif

// ================== RC Method ==================
// #define USE_TPC
// #define USE_RC

void fnvGetSupPoly(double dFootGeom_in[4]);
void fnvDccRunCon();
void fnvRunConHonda();
void fnvStepConCurCpNextStep(double dKTrack[2], double dKResto[2]);

#endif