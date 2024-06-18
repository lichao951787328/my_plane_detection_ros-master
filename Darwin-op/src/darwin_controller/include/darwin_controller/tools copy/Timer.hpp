#pragma once
#include <Windows.h>
#include "Base.h"
_THESIS_TOOL_BEGIN

namespace timer_internal{
    static thread_local LARGE_INTEGER t1,t2;
}

inline double getFS()
{
    static long long FS = 0;
    if (FS == 0)
    {
        LARGE_INTEGER fs;
        QueryPerformanceFrequency(&fs);
        FS = fs.QuadPart;
    }
    return (double)FS;
}

inline void tic()
{
    QueryPerformanceCounter(&timer_internal::t1);
}

inline double toc()
{
    QueryPerformanceCounter(&timer_internal::t2);
    return (double)(timer_internal::t2.QuadPart - timer_internal::t1.QuadPart) / getFS() * 1000.0;
}

_THESIS_TOOL_END