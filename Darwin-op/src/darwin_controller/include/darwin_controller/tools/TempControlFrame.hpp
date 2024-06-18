#pragma once
#include <LBlocks/LBlocks.hpp>

namespace lee{namespace tools{namespace temp_control_frame{
    class Block:public lee::blocks::LBlock<int,int>
    {
    public:
        int run(){return 0;};
    };
}}}