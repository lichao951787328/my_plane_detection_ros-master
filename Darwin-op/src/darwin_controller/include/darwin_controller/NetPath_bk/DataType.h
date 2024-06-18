#pragma once
struct SendType
{
    char str[2];
    double BodyAng[3];
    double BodyPos[3];
};

struct PathType
{
    bool IsLeft;
    double Pos[3];
    double Yaw;
};

constexpr int MAX_STEP = 100;
template<const int N=MAX_STEP>
struct RecvType
{
    int Num;
    PathType Data[N];
};
