// TODO 待添加GUI显示函数
#include <QPFD/Block.h>
#define USE_LGUI

#ifdef USE_LGUI
#include <LGui/LGui.h>
#endif

// ....
_L_QPFD_BEGIN

void Block::printLGui()
{
#ifdef USE_LGUI

ImGui::Begin("QPFD Torque");

ImGui::SliderFloat("Smooth Weight", &this->Params.SmoothWeight, 1e-5, 1e-2);
ImGui::SliderFloat("Contact Weight", &this->Params.ContactWeight, 1e-5, 1e-1);
ImGui::SliderFloat("No Contact Weight", &this->Params.NoContactWeight, 1e0, 1e5);

static int TorqueIndex = 0;
ImGui::SliderInt("Torque Index", &TorqueIndex, 0, 2);
static float TorqueTime = 5.0;
ImGui::SliderFloat("Time", &TorqueTime, 0.1, 10.0);

static lee::gui::LRtPlot Plot("", true, TorqueTime);
if(Plot.begin({-1,-1}))
{
    Plot.add(ImGui::GetTime(), this->DataOutput.LocalRefFootTorqueL[TorqueIndex], "Torque L");
    Plot.add(ImGui::GetTime(), this->DataOutput.LocalRefFootTorqueR[TorqueIndex], "Torque R");
    Plot.add(ImGui::GetTime(), this->RefAcc[TorqueIndex+3], "Acc");
    Plot.end();
}
ImGui::End();

ImGui::Begin("QPFD Force");
static int ForceIndex = 0;
ImGui::SliderInt("Force Index", &ForceIndex, 0, 2);
static lee::gui::LRtPlot Plot2("", true, 5.0);
if(Plot2.begin({-1,-1}))
{
    Plot2.add(ImGui::GetTime(), this->DataOutput.LocalRefFootForceL[ForceIndex], "Force L");
    Plot2.add(ImGui::GetTime(), this->DataOutput.LocalRefFootForceR[ForceIndex], "Force R");
    Plot2.end();
}
ImGui::End();

#endif
}

_L_QPFD_END