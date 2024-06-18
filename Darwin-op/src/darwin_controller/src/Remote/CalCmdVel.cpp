#include <Remote/Block.h>
#include <math.h>
extern "C" {
    #include <LSimpleRoboticsMath/leeKinematics.h>
}
#include <iostream>
_REMOTE_BEGIN

void Block::calCmdVel()
{
    switch (*this->DataInput.PressKey)
    {
    case 'w':
        // std::cout<<"cmd-vel add 0.015: "<<CmdVel.Linear[0] + 0.015<<std::endl;
        this->CmdVel.Linear[0] = 0.05;
        break;    
    case 'W':
        this->CmdVel.Linear[0] = 0.08;
        break;
    case 's':
        this->CmdVel.Linear[0] = -0.05;
        break;
    case 'a':
        this->CmdVel.Linear[1] = 0.03;
        break;
    case 'd':
        this->CmdVel.Linear[1] = -0.03;
        break;
    case 'j':
        this->CmdVel.Angular = _RAD(10);
        break; 
    case 'J':
        this->CmdVel.Angular = _RAD(10.0);
        break;
    case 'k':
        this->CmdVel.Angular = _RAD(0);
        break;
    case 'l':
        this->CmdVel.Angular = _RAD(-10);
        break;
    case 'L':
        this->CmdVel.Angular = _RAD(-10.0);
        break;
    case 'q':
        this->setCmdVel({0.0,0.0,0.0});
        break;
    default:
        break;
    }

    this->CmdVel.Linear[0] = std::max(this->CmdVel.Linear[0], -0.20);
    this->CmdVel.Linear[0] = std::min(this->CmdVel.Linear[0],  0.56);
    // this->CmdVel.Linear[0] = __max(this->CmdVel.Linear[0], -0.02);
    // this->CmdVel.Linear[0] = __min(this->CmdVel.Linear[0],  0.02);
    // std::cout<<"cmd-vel final: "<<CmdVel.Linear[0] <<std::endl;
    this->CmdVel.Linear[1] = std::max(this->CmdVel.Linear[1], -0.10);
    this->CmdVel.Linear[1] = std::min(this->CmdVel.Linear[1],  0.10);

    this->CmdVel.Angular = std::max(this->CmdVel.Angular, -_RAD(10.0));
    this->CmdVel.Angular = std::min(this->CmdVel.Angular,  _RAD(10.0));
    // this->CmdVel.Angular = __max(this->CmdVel.Angular, -_RAD(0.5));
    // this->CmdVel.Angular = __min(this->CmdVel.Angular,  _RAD(0.5));
}

_REMOTE_END