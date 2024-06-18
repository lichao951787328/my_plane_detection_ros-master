#include <tools/Integrator.h>
#include <iostream>

_INTEGRATOR_BEGIN

Block::Block(const double &_Ts, const char *_Name)
{
    this->Ts = _Ts;
    this->BlockName = _Name;
    this->BlockName.append(" Integrator");
    std::cout<<"Create Block: "<<this->BlockName<<std::endl;
}

int Block::init()
{
    std::cout<<"Init Block: "<<this->BlockName<<std::endl;
    return 0;
}

int Block::run()
{
    this->DataOutput.State.Acc.setZero();
    for(auto &i:this->DataInput.AccList)
    {
        this->DataOutput.State.Acc += Vec3{i};
    }
    this->DataOutput.State.Vel += this->DataOutput.State.Acc*this->Ts;
    this->DataOutput.State.Pos += this->DataOutput.State.Vel*this->Ts;
    return 0;
}

int Block::print()
{
    return 0;
}

_INTEGRATOR_END