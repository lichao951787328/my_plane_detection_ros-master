#include "Chz_Spline.h"

#include <algorithm>
#include "Chz_Calculator.h"
//#include <EigenQP/QuadProg.h>

namespace Chz
{
	extern Calculator Calc;
	void GenerateSpline6(double* b, double x0, double v0, double a0, double x2, double v2, double a2, double t2)
	{
		double x0_x2 = x0 - x2;
		double t2v0 = t2 * v0;
		double t2v2 = t2 * v2;
		double t2_2 = t2 * t2, t2_3 = t2_2 * t2, t2_4 = t2_3 * t2, t2_5 = t2_4 * t2;
		double a0t2_2 = a0 * t2_2;
		double a2t2_2 = a2 * t2_2;

		b[0] = x0; b[1] = v0; b[2] = a0 / 2.0;
		b[3] = -(20.0 * x0_x2 + 12.0 * t2v0 + 8.0 * t2v2 + 3.0 * a0t2_2 - a2t2_2) / (2.0 * t2_3);
		b[4] = (30.0 * x0_x2 + 16.0 * t2v0 + 14.0 * t2v2 + 3.0 * a0t2_2 - 2.0 * a2t2_2) / (2.0 * t2_4);
		b[5] = -(12.0 * x0_x2 + 6.0 * t2v0 + 6.0 * t2v2 + a0t2_2 - a2t2_2) / (2.0 * t2_5);
	}
	void GetSpline6(double* x, double* a, double t)
	{
		double ts[6]; ts[0] = 1.0; ChzF(i, 1, 5) ts[i] = ts[i - 1] * t;
		x[0] = x[1] = x[2] = 0.0;
		ChzF(i, 0, 5)
		{
			x[0] += a[i] * ts[i];
			if (i >= 1) x[1] += a[i] * i * ts[i - 1];
			if (i >= 2) x[2] += a[i] * i * (i - 1) * ts[i - 2];
		}
		return;
	}
	void GetSpline6_(double* x, double* a, double t)
	{
		double ts[6]; ts[0] = 1.0; ChzF(i, 1, 5) ts[i] = ts[i - 1] * t;
		*x = 0.0;
		ChzF(i, 0, 5)* x += a[i] * ts[i];
		return;
	}

	void GenerateSpline7(double* a, double x0, double v0, double a0, double t1, double x1, double t2, double x2, double v2, double a2)
	{
		double t1s[7], t2s[7]; t1s[0] = t2s[0] = 1.0;
		ChzF(i, 1, 6) t1s[i] = t1s[i - 1] * t1, t2s[i] = t2s[i - 1] * t2;
		double x2_x0 = x2 - x0;
		double t2v0 = t2 * v0;
		double t2v2 = t2 * v2;
		double a0t2_2 = a0 * t2s[2];
		double a2t2_2 = a2 * t2s[2];
		double a0t2_6t1_2 = a0 * t2s[6] * t1s[2];
		double t2_6v0t1 = t2s[6] * v0 * t1;
		double t2_6x1_x0 = t2s[6] * (x1 - x0);
		double temp1 = a0t2_6t1_2 + 2.0 * t2_6v0t1 - 2.0 * t2_6x1_x0;
		double temp2 = 20.0 * x2_x0 - 12.0 * t2v0 - 8.0 * t2v2 - 3.0 * a0t2_2 + a2t2_2;
		double temp3 = 48.0 * x2_x0 - 30.0 * t2v0 - 18.0 * t2v2 - 8.0 * a0t2_2 + 2.0 * a2t2_2;
		double temp4 = 30.0 * x2_x0 - 20.0 * t2v0 - 10.0 * t2v2 - 6.0 * a0t2_2 + a2t2_2;
		double temp5 = 30.0 * x2_x0 - 16.0 * t2v0 - 14.0 * t2v2 - 3.0 * a0t2_2 + 2.0 * a2t2_2;
		double temp6 = 54.0 * x2_x0 - 30.0 * t2v0 - 24.0 * t2v2 - 6.0 * a0t2_2 + 3.0 * a2t2_2;
		double temp7 = 12.0 * x2_x0 - 6.0 * t2v0 - 6.0 * t2v2 - a0t2_2 + a2t2_2;
		double t1_5t2 = t1s[5] * t2;
		double t1_4t2_2 = t1s[4] * t2s[2];
		double t1_3t2_3 = t1s[3] * t2s[3];
		double denom = 2.0 * t1_3t2_3 * (t1 - t2) * (t1 - t2) * (t1 - t2);
		a[0] = x0; a[1] = v0; a[2] = a0 / 2.0;
		a[3] = temp2 * t1s[6] - temp3 * t1_5t2 + temp4 * t1_4t2_2 + temp1;
		a[4] = -temp5 * t1s[6] + temp6 * t1_5t2 - temp4 * t1_3t2_3 - 3.0 * temp1;
		a[5] = temp7 * t1s[6] - temp6 * t1_4t2_2 + temp3 * t1_3t2_3 + 3.0 * temp1;
		a[6] = -temp7 * t1_5t2 + temp5 * t1_4t2_2 - temp2 * t1_3t2_3 - temp1;
		a[3] /= denom;
		a[4] /= denom * t2;
		a[5] /= denom * t2s[2];
		a[6] /= denom * t2s[3];
		return;
	}
	void GetSpline7(double* x, double* a, double t)
	{
		double ts[7]; ts[0] = 1.0; ChzF(i, 1, 6) ts[i] = ts[i - 1] * t;
		x[0] = x[1] = x[2] = 0.0;
		ChzF(i, 0, 6)
		{
			x[0] += a[i] * ts[i];
			if (i >= 1) x[1] += a[i] * i * ts[i - 1];
			if (i >= 2) x[2] += a[i] * i * (i - 1) * ts[i - 2];
		}
		return;
	}

	/* Generate C^1 piecewise splines
		n + 1 control points: [t_i, x_i] (i = 0 ~ n)
		n segments [t_i, t_i+1] (i = 0 ~ n-1)
		each with three params: x(t) = ai + bi * t + c_i * t^2 (t_i < t < t_i+1) (i = 0 ~ n-1)
		Constraints:
		ai + bi * ti + ci * ti^2 = xi (i = 0 ~ n-1) ------(e1)
		ai + bi * ti+1 + ci * ti+1^2 = xi+1 (i = 0 ~ n-1) ------(e2)
		vi+1 = bi * ti+1 + 2ci * ti+1 = bi+1 * ti+1 + 2ci+1 * ti+1 (i = 0 ~ n-2) ------(e3)
		One last constraint needed:
		Con1. b0 + 2c0 * t0 = v0 --> GeneratePiecewiseSpline()
		Con2. c0 = 0 --> GeneratePiecewiseSpline2()
		Con3. cn-1 = 0

		Calculation process:
		(e2) - (e1): bi + ci(ti + ti+1) = ki ------(e4)
		with: ki = (xi+1 - xi) / (ti+1 - ti)
		Substitute (e4) into (e3), eleminating bi:
		ki - ci(ti + ti+1) + 2ci * ti+1 = ki+1 - ci+1(ti+1 + ti+2) + 2ci+1 * ti+1
		Simplified as:
		ci * Dti + ci+1 * Dti+1 = Dki ------(e5)
		with: Dti = ti+1 - ti, Dki = ki+1 - ki (i = 0 ~ n-2)

		for constraint Con1, taking Con1 into (e4) -> c0 = (k0 - v0) / Dt0;
		for constraint Con2, c0 = 0.
		So: for i = 0 to n - 1
			1. Get bi from (e4), then ai from (e1) or (e2)
			2. If i < n-1, get ci+1 from (e5)
		for constraint Con3, cn-1 = 0, for i = n - 1 downto 0 instead
	*/

	void GeneratePiecewiseSpline(double* _a[3], double* t, double* x, double v0, int n)
	{
		double* a = _a[0], * b = _a[1], * c = _a[2];
		double dt[30], k[30];
		ChzF(i, 0, n - 1) dt[i] = t[i + 1] - t[i], k[i] = (x[i + 1] - x[i]) / dt[i];
		c[0] = (k[0] - v0) / dt[0];
		ChzF(i, 0, n - 2) c[i + 1] = (k[i + 1] - k[i] - c[i] * dt[i]) / dt[i + 1];
		b[0] = v0 - 2.0 * c[0] * t[0];
		ChzF(i, 0, n - 2) b[i + 1] = b[i] - 2.0 * t[i + 1] * (c[i + 1] - c[i]);
		ChzF(i, 0, n - 1) a[i] = x[i] - t[i] * (b[i] + c[i] * t[i]);
	}
	void GeneratePiecewiseSpline2(double* _a[3], double* t, double* x, double v0, int n)
	{
		double* a = _a[0], * b = _a[1], * c = _a[2];
		double dt[30], k[30];
		ChzF(i, 0, n - 1) dt[i] = t[i + 1] - t[i], k[i] = (x[i + 1] - x[i]) / dt[i];
		c[0] = 0.0;
		ChzF(i, 0, n - 2) c[i + 1] = (k[i + 1] - k[i] - c[i] * dt[i]) / dt[i + 1];
		b[0] = v0 - 2.0 * c[0] * t[0];
		ChzF(i, 0, n - 2) b[i + 1] = b[i] - 2.0 * t[i + 1] * (c[i + 1] - c[i]);
		ChzF(i, 0, n - 1) a[i] = x[i] - t[i] * (b[i] + c[i] * t[i]);
	}

	void GetPiecewiseSpline(double x[3], double* _a[3], double* t, int n, double tnow, int resnum)
	{
		double* a = _a[0], * b = _a[1], * c = _a[2];
		tnow = fmin(fmax(tnow, t[0] + eps), t[n] - eps);
		int nseg = 0;
		while(1)
		{
			if (tnow < t[nseg]) break;
			nseg++;
		}
		nseg--;
		x[0] = a[nseg] + b[nseg] * tnow + c[nseg] * tnow * tnow;
		if(resnum == 2 || resnum == 3) x[1] = b[nseg] + 2.0 * c[nseg] * tnow;
		if(resnum == 3) x[2] = c[nseg];
	}

	/*	For n degree Bezier curve, there are n + 1 coefficients, 
		where 6 are defined by boundary value (B(0) dB(0) ddB(0) B(1) dB(1) ddB(1)), 
		which are decided by (a[0 ~ 2], a[(n - 2) ~ n])
		The rest are generated by Bk and tk, where B(tk(i)) = Bk(i) */
	void GenerateSplineBezier(double* a, double* Bk, double* tk, int n)
	{
		Eigen::VectorXd Bk_star(n - 5), as(n - 5);
		Eigen::MatrixXd M(n - 5, n - 5);
		ChzF(i, 0, n - 6)
		{
			Bk_star(i) = Bk[i];
			ChzF(j, 0, 2) Bk_star(i) -= Calc.Comb(n, j) * a[j] * Calc.dPow(tk[i], j) * Calc.dPow(1.0 - tk[i], n - j);
			ChzF(j, n - 2, n) Bk_star(i) -= Calc.Comb(n, j) * a[j] * Calc.dPow(tk[i], j) * Calc.dPow(1.0 - tk[i], n - j);
		}
		ChzF(i, 0, n - 6) ChzF(j, 0, n - 6)
			M(i, j) = Calc.Comb(n, j + 3) * Calc.dPow(tk[i], 3 + j) * Calc.dPow(1 - tk[i], n - 3 - j);
		as = M.inverse() * Bk_star;
		ChzF(i, 3, n - 3) a[i] = as(i - 3);
	}
	void GetSplineBezier(double* x, double* a, int n, double t, char mode)
	{
		double ss[2][30];
		ChzF(i, 0, n) ss[0][i] = Calc.dPow(t, i), ss[1][i] = Calc.dPow(1 - t, i);
		double Cniai[30];
		ChzF(i, 0, n) Cniai[i] = Calc.Comb(n, i) * a[i];
		x[0] = 0.0;
		if (mode == 'a') x[1] = x[2] = 0.0;
		if (mode == 'j') x[1] = x[2] = x[3] = 0.0;
		ChzF(i, 0, n)
		{
			x[0] += Cniai[i] * ss[0][i] * ss[1][n - i];
			if (mode == 'a' || mode == 'j')
			{
				if (i >= 1) x[1] += Cniai[i] * i * ss[0][i - 1] * ss[1][n - i];
				if (i <= n - 1) x[1] -= Cniai[i] * ss[0][i] * (n - i) * ss[1][n - i - 1];

				if (i >= 2) x[2] += Cniai[i] * i * (i - 1) * ss[0][i - 2] * ss[1][n - i];
				if (i >= 1 && i <= n - 1) x[2] -= 2.0 * Cniai[i] * i * ss[0][i - 1] * (n - i) * ss[1][n - i - 1];
				if (i <= n - 2) x[2] += Cniai[i] * ss[0][i] * (n - i) * (n - i - 1) * ss[1][n - i - 2];
			}
			if (mode == 'j')
			{
				if (i >= 3) x[3] += Cniai[i] * i * (i - 1) * (i - 2) * ss[0][i - 3] * ss[1][n - i];
				if (i >= 2 && i <= n - 1) x[3] -= 3.0 * Cniai[i] * i * (i - 1) * ss[0][i - 2] * (n - i) * ss[1][n - i - 1];
				if (i >= 1 && i <= n - 2) x[3] += 3.0 * Cniai[i] * i * ss[0][i - 1] * (n - i) * (n - i - 1) * ss[1][n - i - 2];
				if (i <= n - 3) x[3] -= Cniai[i] * ss[0][i] * (n - i) * (n - i - 1) * (n - i - 2) * ss[1][n - i - 3];
			}
		}
	}
	
	void GetPDParam(double& kp, double& kd, double mp, double me, double ts)
	{
		double wn, xi;
		double lnmp = log(mp);
		xi = sqrt(lnmp * lnmp / (pi * pi + lnmp * lnmp));
		wn = -1.0 / (ts * xi) * log(me * sqrt(1 - xi * xi));
		kp = wn * wn;
		kd = 2.0 * wn * xi;
	}
	void GetPDParam(double& kp, double& kd, double w0, double ksi)
	{
		//上升时间 tr = (pi - arctan(sqrt(1 - ksi^2) / ksi)) / (w0 * sqrt(1 - ksi^2))
		//峰值时间 tp = pi / (w0 * sqrt(1 - ksi^2))
		//超调量 Mp = exp(-pi * ksi / sqrt(1 - ksi^2))
		kp = w0 * w0;
		kd = 2.0 * ksi * w0;
	}
	double GetLimitedValue(double x, double X1, double Y1, double X2, double Y2, double K)
	{
		double sig = 1.0, y;
		if (abs(x) < eps) return x;
		if (x < -eps) sig = -1.0, x *= -1.0;
		if (x < X1) y = Y1 / X1 * x;
		else if (x > X2) y = K * (x - X2) + Y2;
		else
		{
			static double b[7], tempy[3];
			GenerateSpline6(b, Y1, Y1 / X1, 0.0, Y2, K, 0.0, X2 - X1);
			GetSpline6(tempy, b, x - X1);
			y = tempy[0];
		}
		return sig * y;
	}
	void LeastSquareFit(double& k, double& b, int n, double x[], double y[])
	{
		double avex = 0.0, avey = 0.0;
		ChzF(i, 0, n - 1) avex += x[i], avey += y[i];
		avex /= n; avey /= n;
		double fz = 0.0;
		ChzF(i, 0, n - 1) fz += x[i] * y[i];
		fz -= n * avex * avey;
		double fm = 0.0;
		ChzF(i, 0, n - 1) fm += x[i] * x[i];
		fm -= n * avex * avex;
		k = fz / fm;
		b = avey - avex * k;
	}

	void ChzC1Intp::assignt(int _n, double* _t)
	{
		n = _n;
		trange[0] = _t[0]; trange[1] = _t[n]; ttot = trange[1] - trange[0];
		t[0] = 0.0; ChzF(i, 1, n - 1) t[i] = (_t[i] - trange[0]) / ttot; t[n] = 1.0;
		ChzF(i, 0, n - 1) tadd[i] = t[i] + t[i + 1];
		ChzF(i, 0, n - 1) dt[i] = (t[i + 1] - t[i]);
		ChzF(i, 0, n - 2) dtdiv[i] = dt[i] / dt[i + 1];
	}

	void ChzC1Intp::assignx(double* x, int _conmode, double _con)
	{
		conmode = _conmode;
		con = _con;
		ChzF(i, 0, n - 1) k[i] = (x[i + 1] - x[i]) / dt[i];
		if (conmode == 0) c[0] = (k[0] - _con) / dt[0];
		else if (conmode == 1) c[0] = 0.0;
		ChzF(i, 0, n - 1)
		{
			b[i] = k[i] - c[i] * tadd[i];
			a[i] = x[i] - t[i] * (b[i] + c[i] * t[i]);
			if (i != n - 1) c[i + 1] = (k[i + 1] - k[i]) / dt[i + 1] - c[i] * dtdiv[i];
		}
	}

	void ChzC1Intp::assigntx(int _n, double* _t, double* _x, int _conmode, double _con)
	{
		assignt(_n, _t);
		assignx(_x, _conmode, _con);
	}

	void ChzC1Intp::get(double* x, double _t, int resnum)
	{
		_t = (_t - trange[0]) / ttot;
		_t = fmin(fmax(_t, eps), 1.0 - eps);
		int nseg = 0;
		while (1)
		{
			if (_t < t[nseg]) break;
			nseg++;
		}
		nseg--;
		double ct = c[nseg] * _t;
		x[0] = (a[nseg] + _t * (b[nseg] + ct));
		if (resnum == 2 || resnum == 3) x[1] = (b[nseg] + 2.0 * ct);
		if (resnum == 3) x[2] = c[nseg];
	}

	void SplineGenerator::GenerateSubsecSpline(int n1, double xs[], double ys[], double dy0[2], double dyn[2])
	{
		n = n1;
		ChzF(i, 0, n1) x[i] = xs[i];
		int dim = 4 * n1 + 2;
		Param.resize(dim); Param.setZero();
		A.resize(dim, dim); A.setZero();
		b.resize(dim); b.setZero();
		ChzF(k, 0, 1)
		{
			ChzF(i, 0, n1 - 1)
			{
				int l, r;
				if (i == 0) l = 0, r = 4;
				else if (i == n1 - 1) l = 4 * n1 - 3, r = l + 4;
				else l = 4 * i + 1, r = l + 3;

				double temp = 1.0;
				ChzF(j, l, r)
				{
					A(k * n1 + i, j) = temp;
					temp *= xs[i + k];
				}
				b(k * n1 + i) = ys[i + k];
			}
		}

		ChzF(i, 1, n1 - 1)
		{
			int l1, r1, l2, r2;
			if (i == 1) l1 = 0, r1 = l1 + 4;
			else l1 = 4 * i - 3, r1 = l1 + 3;
			if (i == n1 - 1) l2 = 4 * n1 - 3, r2 = l2 + 4;
			else l2 = 4 * i + 1, r2 = l2 + 3;
			//vel
			double temp = 1.0;
			ChzF(j, l1 + 1, r1)
			{
				A(2 * n1 - 1 + i, j) = temp * (j - l1);
				temp *= xs[i];
			}
			temp = 1.0;
			ChzF(j, l2 + 1, r2)
			{
				A(2 * n1 - 1 + i, j) = -temp * (j - l2);
				temp *= xs[i];
			}
			b(2 * n1 - 1 + i) = 0.0;
			//acc
			temp = 1.0;
			ChzF(j, l1 + 2, r1)
			{
				A(3 * n1 - 2 + i, j) = temp * (j - l1) * (j - l1 - 1);
				temp *= xs[i];
			}
			temp = 1.0;
			ChzF(j, l2 + 2, r2)
			{
				A(3 * n1 - 2 + i, j) = -temp * (j - l2) * (j - l2 - 1);
				temp *= xs[i];
			}
			b(3 * n1 - 2 + i) = 0.0;
		}

		int i = 4 * n1 - 3 + 1;
		double temp = 1.0;
		ChzF(j, 1, 4) { A(i, j) = temp * j; temp *= xs[0]; }
		b(i) = dy0[0];
		temp = 1.0;
		ChzF(j, 2, 4) { A(i + 1, j) = temp * j * (j - 1); temp *= xs[0]; }
		b(i + 1) = dy0[1];

		temp = 1.0;
		ChzF(j, 1, 4) { A(i + 2, 4 * n1 - 3 + j) = temp * j; temp *= xs[n1]; }
		b(i + 2) = dyn[0];
		temp = 1.0;
		ChzF(j, 2, 4) { A(i + 3, 4 * n1 - 3 + j) = temp * j * (j - 1); temp *= xs[n1]; }
		b(i + 3) = dyn[1];

		//std::cout << A << std::endl << std::endl;
		//std::cout << b << std::endl;

		Param = A.inverse() * b;

		//std::cout << Param << std::endl;
	}

	void SplineGenerator::GetSubsecSpline(double y[3], double x1)
	{
		int seg = 0;
		ChzF(i, 0, n - 1) if (x1 >= x[i] && x1 < x[i + 1]) seg = i;
		x1 = std::min(std::max(x1, x[0]), x[n]);
		if (x1 > x[n] - eps) seg = n - 1;
		int l, r;
		if (seg == 0) l = 0, r = 4;
		else if (seg == n - 1) l = 4 * n - 3, r = l + 4;
		else l = 4 * seg + 1, r = l + 3;
		double temp = 1.0;
		y[0] = y[1] = y[2] = 0.0;
		ChzF(i, l, r)
		{
			y[0] += temp * Param[i];
			if (i <= r - 1) y[1] += temp * Param[i + 1] * (i - l + 1);
			if (i <= r - 2) y[2] += temp * Param[i + 2] * (i - l + 2) * (i - l + 1);
			temp = temp * x1;
		}
	}

}

