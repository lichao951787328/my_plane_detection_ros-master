// QPFD/Base.h
// QP based force distribution
// lee, hexb66@bit.edu.cn
// Apr. 18, 2022
#pragma once
#define _L_QPFD_BEGIN namespace lee{namespace qpfd{
#define _L_QPFD_END }}
#include <LEigenQP.hpp>

_L_QPFD_BEGIN

enum VARIABLE_INDEX
{
    LFX, LFY, LFZ, RFX, RFY, RFZ, LTX, LTY, LTZ, RTX, RTY, RTZ, VAR_NUM
};

_L_QPFD_END