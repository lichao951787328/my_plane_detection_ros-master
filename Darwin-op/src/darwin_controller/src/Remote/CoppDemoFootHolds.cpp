#include <LSimpleRoboticsMath/Biped.hh>
#include <deque>
extern"C"{
    #include<LSimpleRoboticsMath/leeKinematics.h>
}
std::deque<lee::math::biped::FootholdType> CoppDemoFootholdList;

auto &getCoppDemoFootholdList()
{
    using namespace lee::math::biped;
    if(CoppDemoFootholdList.size() == 0)
    {
        double StepTime = 0.8;
        CoppDemoFootholdList.push_back(FootholdType(_RIGHT_, StepTime, {0.0, -0.08, +1.0000e-02}, {0, 0, _RAD(0.0)}));
        CoppDemoFootholdList.push_back(FootholdType(_LEFT__, StepTime, {+4.0000e-01, 0.125, +1.0000e-02}, {0, 0, _RAD(0.0)}));
        CoppDemoFootholdList.push_back(FootholdType(_RIGHT_, StepTime, {+6.2500e-01, -2.2500e-01, +5.5000e-02}, {_RAD(10), 0, _RAD(-30.0)}));
        CoppDemoFootholdList.push_back(FootholdType(_LEFT__, StepTime, {+9.5000e-01, -7.5000e-02, +2.0000e-02}, {0, _RAD(10), _RAD(-60.0)}));
        CoppDemoFootholdList.push_back(FootholdType(_RIGHT_, StepTime, {+9.5000e-01, -5.0000e-01, +1.0000e-02}, {0, 0, _RAD(-80.0)}));
        CoppDemoFootholdList.push_back(FootholdType(_LEFT__, StepTime, {+9.5000e-01 + 0.23, -5.0000e-01, +0.0000e-02}, {0, 0, _RAD(-70.0)}));
        CoppDemoFootholdList.push_back(FootholdType(_RIGHT_, StepTime, {+9.5000e-01, -5.0000e-01 - 0.36, 0}, {0, 0, _RAD(-90.0)}));
        CoppDemoFootholdList.push_back(FootholdType(_LEFT__, StepTime, {+9.5000e-01 + 0.23, -5.0000e-01 - 0.4, 0}, {0, 0, _RAD(-90.0)}));
        CoppDemoFootholdList.push_back(FootholdType(_RIGHT_, StepTime, {+9.5000e-01, -5.0000e-01 - 0.6, 0}, {0, 0, _RAD(-90.0)}));
        CoppDemoFootholdList.push_back(FootholdType(_LEFT__, StepTime, {+9.5000e-01 + 0.16, -5.0000e-01 - 0.9, 0}, {0, 0, _RAD(-90.0)}));
    }
    return CoppDemoFootholdList;
}