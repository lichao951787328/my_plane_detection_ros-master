#pragma once
#include "Base.h"
#include <iomanip>

#define PRECISION(X) std::setprecision(X)
#define ALIGN_R(X) std::setiosflags(std::ios::right)<<std::setw(X)
#define ALIGN_L(X) std::setiosflags(std::ios::left)<<std::setw(X)
#define SPLIT_LINE "-----------------------------------------------------------------------------"

#define D_PRECISION std::setprecision(4)
#define D_ALIGN_R ALIGN_R(8)
#define D_ALIGN_L ALIGN_L(8)