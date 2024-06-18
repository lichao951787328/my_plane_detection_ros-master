#pragma once
#ifndef _CRT_SECURE_NO_WARNINGS
#define _CRT_SECURE_NO_WARNINGS
#endif
#include <cmath>
#include <cstdlib>
#include <algorithm>
#include <iostream>
#include <map>
#include <random>
#include <string>

#include <Eigen/Dense>

#define ChzF(a, b, c) for(int a = (b); a <= (c); a++)
#define mem(a) memset(a, 0, sizeof(a))
typedef long long ll;

namespace Chz
{
	constexpr double inf = 1e20;
	constexpr double eps = 1e-7;
	constexpr double pi = 3.141592653589793;
	constexpr double e = 2.718281828459045;
	constexpr double g = 9.8;
	constexpr double mu = 0.5;
	template<class T, class... Args> void Assign(T* arr, Args... input);
	template<class T> void Assign(T* arr) {}
	template<class T, class... Args> void Assign(T* arr, T var, Args... rest) 
	{
		arr[0] = var;
		Assign(arr + 1, rest...);
	}
}