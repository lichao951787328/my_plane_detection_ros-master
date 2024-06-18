#pragma once
#include <BHR7P_Thesis/include/model_parameter.h>
#include "LDynamicsEigen.hpp"

_L_DYN_BEGIN
    
class LDynBHR7PEigen:public LDynamicsEigen<dof,noc>{};

_L_DYN_END