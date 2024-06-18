// !ImuRevision.h
#pragma once
#ifdef __cplusplus
extern "C"{
#endif
void getPostureFromCopp(const double * _CoppPosture, double * _Posture);
void getPostureFromBHR(const double * _BHR7P3Posture, double * _Posture);
#ifdef __cplusplus
}
#endif
