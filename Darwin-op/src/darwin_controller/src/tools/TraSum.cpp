#include <tools/TraSum.h>
#include <iostream>
#include <tools/AlignCout.h>

_TRA_SUM_BEGIN
Block::Block()
{
    this->BlockName = "TraSum";
    this->pExtLogger = NULL;
    this->InList.push_back(&this->DataInput.BodyPos );
    this->InList.push_back(&this->DataInput.FootPosL);
    this->InList.push_back(&this->DataInput.FootPosR);
    this->InList.push_back(&this->DataInput.BodyAng );
    this->InList.push_back(&this->DataInput.FootAngL);
    this->InList.push_back(&this->DataInput.FootAngR);
    this->OutList.push_back(&this->DataOutput.BodyPos );
    this->OutList.push_back(&this->DataOutput.FootPosL);
    this->OutList.push_back(&this->DataOutput.FootPosR);
    this->OutList.push_back(&this->DataOutput.BodyAng );
    this->OutList.push_back(&this->DataOutput.FootAngL);
    this->OutList.push_back(&this->DataOutput.FootAngR);

    for(auto i:this->OutList) i->setZero();

    std::cout<<"Create Block: "<<this->BlockName<<std::endl;
}

int Block::init()
{
    std::cout<<"Init Block: "<<this->BlockName<<std::endl;
    return 0;
}

int Block::run()
{
    for(int i=0;i<6;i++)
    {
        this->OutList[i]->setZero();
        for(int j=0;j<this->InList[i]->size();j++)
        {
            *this->OutList[i] += Vec3((*this->InList[i])[j]);
        }
    }
    return 0;
}

int Block::print()
{
    // std::cout<<"[ "<<this->BlockName<<" ]"<<std::endl
    // << "Body :" << this->DataOutput.BodyPos.transpose()
    // << std::endl
    // << SPLIT_LINE 
    // << std::endl;
    return 0;
}

int Block::log()
{
    if(this->pExtLogger==NULL)
        return -1;
    auto pLog = this->pExtLogger;
    auto logPosture = [&](const double *pos, const double *ang, const std::string &name){
        pLog->addLog(pos[0], (name+"Pos_X").c_str());
        pLog->addLog(pos[1], (name+"Pos_Y").c_str());
        pLog->addLog(pos[2], (name+"Pos_Z").c_str());
        pLog->addLog(ang[0], (name+"AngRad_X").c_str());
        pLog->addLog(ang[1], (name+"AngRad_Y").c_str());
        pLog->addLog(ang[2], (name+"AngRad_Z").c_str());
    };
    logPosture(this->DataOutput.BodyPos.data(), this->DataOutput.BodyAng.data(), "TraSum_Body");
    logPosture(this->DataOutput.FootPosL.data(), this->DataOutput.FootAngL.data(), "TraSum_FootL");
    logPosture(this->DataOutput.FootPosR.data(), this->DataOutput.FootAngR.data(), "TraSum_FootR");
    return 0;
}

_TRA_SUM_END