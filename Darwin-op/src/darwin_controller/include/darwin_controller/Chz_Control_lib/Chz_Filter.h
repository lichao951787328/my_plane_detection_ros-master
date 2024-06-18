#pragma once
#include "Chz_Base.h"

namespace Chz
{
	class LowPassFilter5
	{
		double Wn;
		static const int n = 5;
		double a[n + 1], b[n + 1];
		double xin[n + 1], xout[n + 1];
	public:
		void Reset(double wn1);
		void Init(double xin1);
		void Update(double xinnew);
		void GetVal(double& xnow);
		void GetVal(double& xnow, double& xlast);
	};
}