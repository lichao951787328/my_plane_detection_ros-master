#pragma once
#include <Windows.h>

namespace lee{namespace tools{

class CWinClock
{
protected:
    double Frequency;
    LARGE_INTEGER   TimeCount, TimeBase;
    double TimeMs;
public:

    inline CWinClock()
    {
        LARGE_INTEGER fs;
        QueryPerformanceFrequency(&fs);
        QueryPerformanceCounter(&TimeBase);
        this->Frequency = (double)fs.QuadPart;
    };
    inline auto updateTimeBase(){QueryPerformanceCounter(&TimeBase);};
    inline auto getCurrentTimeMs()
    {
        QueryPerformanceCounter(&this->TimeCount);
        return (double)((this->TimeCount.QuadPart-this->TimeBase.QuadPart)/this->Frequency)*1000.0;
    };
    inline auto tic()
    {
        this->TimeMs = this->getCurrentTimeMs();
    };
    inline auto toc()
    {
        return this->getCurrentTimeMs()-this->TimeMs;
    }
};

}}