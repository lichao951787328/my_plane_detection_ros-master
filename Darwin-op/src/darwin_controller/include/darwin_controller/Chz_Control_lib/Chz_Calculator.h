#pragma once
#ifndef CHZ_CONTROL_LIB_CHZ_CALCULATOR_H
#define CHZ_CONTROL_LIB_CHZ_CALCULATOR_H
#include "Chz_Base.h"

#ifdef CHZ_CONTROL_LIB_CHZ_CALCULATOR_CPP
#define Extern 
#else
#define Extern extern
#endif
namespace Chz
{
	const int MaxDiscreteCalNum = 20;
	class Calculator
	{
		ll Factorial[MaxDiscreteCalNum + 1];
		ll Permutation[MaxDiscreteCalNum + 1][MaxDiscreteCalNum + 1];
		ll Combination[MaxDiscreteCalNum + 1][MaxDiscreteCalNum + 1];
	public:
		Calculator();
		double dPow(double a, int b);
		int Pow_minus1(int b);
		double param_abs[1];
		double abs(double a);

		ll Fact(int i);
		ll Perm(int m, int n);
		ll Comb(int m, int n);
	};
	Extern Calculator Calc;
}
#undef Extern
#endif