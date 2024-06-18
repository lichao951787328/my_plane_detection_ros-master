#include <iostream>
#include <deque>
#include <string>
#include <LGui/LGui.h>

_L_GUI_BEGIN

LRtPlot::LRtPlot(const char *name, const bool &scrolling_on, const double &history)
{
    this->History = history;
    this->Index = 0;
    this->ScrollingOn = scrolling_on;
    this->Name = name;
}

bool LRtPlot::begin(const ImVec2 &size, const ImPlotFlags flags)
{
    this->Index = 0;
    return ImPlot::BeginPlot(this->Name.c_str(), size, flags);
}

void LRtPlot::add(const double &x, const double &y, const char *name)
{
    if(Index>=DataList.size())
    {
        DataList.push_back(ImPlot::ScrollingBuffer());
    }
    auto &buf = DataList[Index];
    buf.AddPoint((float)x, (float)y);
    if ((Index == 0) && (ScrollingOn))
    {
        ImPlot::SetupAxisLimits(ImAxis_X1, x, x - History, ImGuiCond_Always);
    }
    ImPlot::PlotLine(name, &buf.Data[0].x, &buf.Data[0].y, buf.Data.size(),0,buf.Offset, 2*sizeof(float));
    Index++;
}

void LRtPlot::end()
{
    ImPlot::EndPlot();
}


_L_GUI_END