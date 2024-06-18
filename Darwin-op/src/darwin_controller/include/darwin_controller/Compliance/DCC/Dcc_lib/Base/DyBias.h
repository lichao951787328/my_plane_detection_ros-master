#ifndef DEFINE_DYBIAS_H
#define DEFINE_DYBIAS_H
#ifdef DEFINE_DYBIAS_C
#define Extern 
#else 
#define Extern extern
#endif

#define __CONTROLT 0.004
#define __MaxProgNDyBias 500000
Extern double dSeDyBias[__MaxProgNDyBias];

#undef Extern

void fnvInitDyBiasP1(double dPoint1[2]);
void fnvInitDyBiasP2(double dPoint1[2], double dPoint2[2]);
void fnvInitDyBiasP3(double dPoint1[2], double dPoint2[2], double dPoint3[2]);
void fnvInitDyBiasP4(double dPoint1[2], double dPoint2[2], double dPoint3[2], double dPoint4[2]);
double fndGetDyBiasVal(int nProgK);


#endif

