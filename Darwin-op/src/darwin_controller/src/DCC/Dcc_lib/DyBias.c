#ifndef DEFINE_DYBIAS_C
#define DEFINE_DYBIAS_C
// #include "DyBias.h"
// #include "Base/dcc_con_base.h"
#include <Compliance/DCC/Dcc_lib/Base/DyBias.h>
#include <Compliance/DCC/Dcc_lib/Base/dcc_con_base.h>

void fnvInitDyBiasP1(double dPoint1[2]) {
	fnvEzSpline(dSeDyBias, __MaxProgNDyBias, 0.0, dPoint1[0], dPoint1[1], __CONTROLT);
}

void fnvInitDyBiasP2(double dPoint1[2], double dPoint2[2]) {
	fnvEzSpline(dSeDyBias, __MaxProgNDyBias, 0.0, dPoint1[0], dPoint1[1], __CONTROLT);
	fnvEzSpline(dSeDyBias, __MaxProgNDyBias, dPoint1[0], dPoint2[0], dPoint2[1], __CONTROLT);
}

void fnvInitDyBiasP3(double dPoint1[2], double dPoint2[2], double dPoint3[2]) {
	fnvEzSpline(dSeDyBias, __MaxProgNDyBias, 0.0, dPoint1[0], dPoint1[1], __CONTROLT);
	fnvEzSpline(dSeDyBias, __MaxProgNDyBias, dPoint1[0], dPoint2[0], dPoint2[1], __CONTROLT);
	fnvEzSpline(dSeDyBias, __MaxProgNDyBias, dPoint2[0], dPoint3[0], dPoint3[1], __CONTROLT);
}

void fnvInitDyBiasP4(double dPoint1[2], double dPoint2[2], double dPoint3[2], double dPoint4[2]) {
	fnvEzSpline(dSeDyBias, __MaxProgNDyBias, 0.0, dPoint1[0], dPoint1[1], __CONTROLT);
	fnvEzSpline(dSeDyBias, __MaxProgNDyBias, dPoint1[0], dPoint2[0], dPoint2[1], __CONTROLT);
	fnvEzSpline(dSeDyBias, __MaxProgNDyBias, dPoint2[0], dPoint3[0], dPoint3[1], __CONTROLT);
	fnvEzSpline(dSeDyBias, __MaxProgNDyBias, dPoint3[0], dPoint4[0], dPoint4[1], __CONTROLT);
}

double fndGetDyBiasVal(int nProgK) {
	return dSeDyBias[nProgK];
}

#endif