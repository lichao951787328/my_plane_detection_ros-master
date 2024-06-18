#pragma once
#include "Base.h"

_THESIS_TOOL_BEGIN

constexpr struct {int MS=5;double S=0.005;}SysControlTime_5MS;
constexpr struct {int MS=1;double S=0.001;}SysControlTime_1MS;
constexpr struct {int MS=2;double S=0.002;}SysControlTime_2MS;
constexpr struct {int MS=4;double S=0.004;}SysControlTime_4MS;

constexpr double HipWidth   =   0.16;
constexpr double ThighLen   =   0.33;
constexpr double CrusLen    =   0.32;
constexpr double FootHeight =   0.112;
constexpr double RobotWeight=   55.0*9.8+10.0;

_THESIS_TOOL_END