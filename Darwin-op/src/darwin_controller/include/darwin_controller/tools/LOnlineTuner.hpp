// LOnlineTuner.hpp
// Online parameters tunning tools
// lee, <hexb66@bit.edu.cn>
#pragma once
#include <iostream>
#include <string.h>
#include <vector>
#include <iomanip>
#include <windows.h>
#include <memory>
#include "Base.h"

_THESIS_TOOL_BEGIN namespace tuner{

class ParameterBase
{
protected:
    std::string Name;
    void *pParameter;

public:
    virtual void add() = 0;
    virtual void sub() = 0;
    virtual void print() = 0;
    inline void setName(const char *_Name)
    {
        this->Name = std::string(_Name);
    };
    inline void printName()
    {
        // std::cout << "[" << this->Name.c_str() << "]";
        std::cout << this->Name.c_str();
    };
};

template <typename _TYPE>
class Parameter : public ParameterBase
{
protected:
    _TYPE *pParameter;

public:
    Parameter(_TYPE &_Parameter, const char *_Name)
    {
        this->init(_Parameter, _Name);
    };
    void init(_TYPE &_Parameter, const char *_Name)
    {
        this->pParameter = &_Parameter;
        this->setName(_Name);
    };
};

class LOnlineTuner
{
protected:
    std::vector<std::shared_ptr<ParameterBase>> List;
    int Index;
    int KeyParameterAdd;
    int KeyParameterSub;
    int KeyMoveFront;
    int KeyMoveBack;

    HANDLE ConsoleOutputHandle;
    WORD OriginalConsoleAttributes;
    WORD HighlightConsoleAttributes;

public:
    LOnlineTuner() : Index(-1), HighlightConsoleAttributes(FOREGROUND_BLUE + FOREGROUND_GREEN + FOREGROUND_RED + BACKGROUND_BLUE)
    {
        this->KeyParameterAdd = '=';
        this->KeyParameterSub = '-';
        this->KeyMoveFront = '.';
        this->KeyMoveBack = ',';
        this->ConsoleOutputHandle = GetStdHandle(STD_OUTPUT_HANDLE);
        CONSOLE_SCREEN_BUFFER_INFO _temp_info;
        GetConsoleScreenBufferInfo(GetStdHandle(STD_OUTPUT_HANDLE), &_temp_info);
        this->OriginalConsoleAttributes = _temp_info.wAttributes;
    };

    template <class CLASS_TYPE, typename _TYPE>
    void addParameters(_TYPE &_Parameter, const char *_Name)
    {
        this->List.push_back(std::make_shared<CLASS_TYPE>(_Parameter, _Name));
    };

    template <class CLASS_TYPE>
    auto addParameters(CLASS_TYPE &_Parameter)
    {
        this->List.push_back(std::make_shared<CLASS_TYPE>(_Parameter));
        return this->getParameter<CLASS_TYPE>((int)this->List.size() - 1);
    };

    void run(const int &_Key)
    {
        if (_Key == this->KeyParameterAdd || _Key == this->KeyParameterSub)
        {
            this->operateParameter(_Key);
        }
        else if (_Key == this->KeyMoveFront || _Key == this->KeyMoveBack)
        {
            this->operateIndex(_Key);
        }
    };

    void printParameter()
    {
        std::cout << std::endl
                    << "--------------Parameters Tuner--------------" << std::endl;
        for (int i = 0; i < this->List.size(); i++)
        {
            if (i == this->Index)
                this->highlight();
            this->List[i]->printName();
            this->List[i]->print();
            if (i == this->Index)
                this->recover();
        }
        std::cout << "                                   ";
        std::cout << std::endl;
    };

    int &getIndex() { return this->Index; };
    template <class _TYPE>
    auto getParameter(int i)
    {
        return std::dynamic_pointer_cast<_TYPE>(this->List[i]);
    }

protected:
    void operateParameter(const int &_Key)
    {
        if (this->Index == -1 || this->Index >= this->List.size())
        {
            return;
        }
        if (_Key == this->KeyParameterAdd)
        {
            this->List[this->Index]->add();
        }
        else if (_Key == this->KeyParameterSub)
        {
            this->List[this->Index]->sub();
        }
    };
    void operateIndex(const int &_Key)
    {
        if (_Key == this->KeyMoveFront)
            this->Index++;
        else if (_Key == this->KeyMoveBack)
            this->Index--;

        if (this->Index >= (int)this->List.size())
            this->Index = -1;
        if (this->Index < -1)
            this->Index = (int)this->List.size() - 1;
    };
    void highlight()
    {
        SetConsoleTextAttribute(this->ConsoleOutputHandle, this->HighlightConsoleAttributes);
    };
    void recover()
    {
        SetConsoleTextAttribute(this->ConsoleOutputHandle, this->OriginalConsoleAttributes);
    };
};

class DoubleParameter: public Parameter<double>
{
private:
    double Delta, MaxValue, MinValue;

public:
    DoubleParameter(double &_Param, const char *_Name, double _Delta = 0.001)
        : Parameter<double>(_Param, _Name),
            Delta(_Delta), MaxValue(INFINITY), MinValue(-INFINITY){};
    void add()
    {
        auto &_data = *this->pParameter;
        _data = min(this->MaxValue, _data + this->Delta);
    };
    void sub()
    {
        auto &_data = *this->pParameter;
        _data = max(this->MinValue, _data - this->Delta);
    };
    void print()
    {
        std::cout << " " << *this->pParameter << " ";
    };
    void setLimit(double _MinValue, double _MaxValue)
    {
        this->MinValue = _MinValue;
        this->MaxValue = _MaxValue;
    };
};

class Double2Parameters: public DoubleParameter
{
private:
    double *pParam2;
public:
    Double2Parameters(double &_Param1, double &_Param2, const char *_Name, double _Delta = 0.001)
        : DoubleParameter(_Param1, _Name, _Delta){
        this->pParam2 = &_Param2;
    };
    void add(){
        this->DoubleParameter::add();
        *this->pParam2 = *this->pParameter;
    };
    void sub(){
        this->DoubleParameter::sub();
        *this->pParam2 = *this->pParameter;
    };
    void print(){
        this->DoubleParameter::print();
    };
};

} _THESIS_TOOL_END