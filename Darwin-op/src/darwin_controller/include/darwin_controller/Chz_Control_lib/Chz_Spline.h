#pragma once
#include "Chz_Base.h"

namespace Chz
{
	void GenerateSpline6(double* b, double x0, double v0, double a0, double x2, double v2, double a2, double t2);
	void GetSpline6(double* x, double* a, double t);
	void GetSpline6_(double* x, double* a, double t);

	void GetSpline7(double* x, double* a, double t);
	void GenerateSpline7(double* a, double x0, double v0, double a0, double t1, double x1, double t2, double x2, double v2, double a2);

	void GeneratePiecewiseSpline(double* _a[3], double* t, double* x, double v0, int n);
	void GetPiecewiseSpline(double x[3], double* _a[3], double* t, int n, double tnow, int resnum = 3);
	void GeneratePiecewiseSpline2(double* _a[3], double* t, double* x, double v0, int n);

	void GenerateSplineBezier(double* a, double* xk, double* tk, int n);
	void GetSplineBezier(double* x, double* a, int n, double t, char mode);

	void GetPDParam(double& kp, double& kd, double mp, double me, double ts);
	void GetPDParam(double& kp, double& kd, double w0, double ksi);
	//y = Y1 / X1 * x (x < X1); y = K * (x - X2) + Y2 (x > X2); y = GetSpline6(*) (X1 <= x <= X2);
	double GetLimitedValue(double x, double X1, double Y1, double X2, double Y2, double K);
	void LeastSquareFit(double& k, double& b, int n, double x[], double y[]);

	constexpr int max_n_intp = 50;
	class ChzC1Intp
	{
	public:
		void assignt(int _n, double* _t);
		void assignx(double* _x, int _conmode, double _con);
		void assigntx(int _n, double* _t, double* _x, int _conmode, double _con);
		void get(double* _x, double _t, int _resnum = 3);
	private:
		int n, conmode;
		double trange[2], ttot;
		double a[max_n_intp], b[max_n_intp], c[max_n_intp];
		double t[max_n_intp], con;
		// used for calculation
		double tadd[max_n_intp]; // tadd[i] = t[i] + t[i + 1]
		double dt[max_n_intp]; // dt[i] = t[i + 1] - t[i]
		double dtdiv[max_n_intp]; // dtdiv[i] = dt[i] / dt[i + 1]
		double k[max_n_intp]; // k[i] = (x[i + 1] - x[i]) / dt[i]
	};


	class SplineGenerator
	{
	public:
		void GetSubsecSpline(double y[3], double x);
		void GenerateSubsecSpline(int n, double xs[], double ys[], double dy0[2], double dyn[2]);
	protected:
	private:
		Eigen::VectorXd Param;
		Eigen::MatrixXd A;
		Eigen::VectorXd b;
		int n;
		double x[100];
	};

}