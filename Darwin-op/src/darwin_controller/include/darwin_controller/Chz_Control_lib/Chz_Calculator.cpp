#ifndef CHZ_CONTROL_LIB_CHZ_CALCULATOR_CPP
#define CHZ_CONTROL_LIB_CHZ_CALCULATOR_CPP
#include "Chz_Calculator.h"

Chz::Calculator::Calculator()
{
	Factorial[0] = 1;
	ChzF(i, 1, MaxDiscreteCalNum) Factorial[i] = Factorial[i - 1] * i;
	ChzF(i, 0, MaxDiscreteCalNum)
		ChzF(j, 0, MaxDiscreteCalNum)
			if(j <= i)
			{
				Permutation[i][j] = Factorial[i] / Factorial[j];
				Combination[i][j] = Factorial[i] / Factorial[j] / Factorial[i - j];
			}
	/* approximation of abs(): 
		fx = 2 / k * ln(1 + e^(kx)) - x - 2 / k * ln(2)
	*/
	double k = 0.001;
	param_abs[0] = k;
}

double Chz::Calculator::dPow(double a, int b)
{
	if (b < 0) return 1.0 / dPow(a, -b);
	if (b == 0) return 1.0;
	double res = 1.0;
	while (b > 1)
	{
		if (b % 2) res *= a, b--;
		a *= a;
		b >>= 1;
	}
	return res * a;
}

int Chz::Calculator::Pow_minus1(int b)
{
	if (b % 2)
		return -1;
	else
		return 1;
}

double Chz::Calculator::abs(double a)
{
	//return a * tanh(a / param_abs[0]);
	return sqrt(a * a + param_abs[0] * param_abs[0]);
}

ll Chz::Calculator::Fact(int i)
{
	return Factorial[i];
}

ll Chz::Calculator::Perm(int m, int n)
{
	return Permutation[m][n];
}

ll Chz::Calculator::Comb(int m, int n)
{
	return Combination[m][n];
}
#endif

