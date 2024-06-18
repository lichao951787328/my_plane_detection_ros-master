// NumIKBlock.h
#pragma once
#include <LBlocks/LBlocks.hpp>
#include <LBlocks/LLog.hpp>
#include <Eigen/Dense>

namespace chz
{
	namespace NumIK
	{
		constexpr int JOINTNUM = 6;

		class Input
		{
		public:
			double *pBodyPosition;
			double *pBodyPosture;
			double *pLFootPosition;
			double *pLFootPosture;
			double *pRFootPosition;
			double *pRFootPosture;
		};

		class Output
		{
		public:
			double *LegJointL;
			double *LegJointR;
		};

		class NumIKBlock:public lee::blocks::LBlock<Input, Output>
		{
		protected:
			Eigen::Vector3d ps[JOINTNUM + 1], pr[JOINTNUM + 1], ws[JOINTNUM + 1], as[JOINTNUM + 1];
			Eigen::Matrix3d Rs[JOINTNUM + 1];
			char axes[JOINTNUM];
			int jointseq[JOINTNUM];

			Eigen::Vector3d pbod, plft, prft;
			Eigen::Matrix3d Rbod, Rlft, Rrft;
			Eigen::Matrix<double, JOINTNUM, 1> ql, qr;

			//temp values in IK
			Eigen::Matrix<double, 6, 1> We;
			Eigen::Matrix<double, JOINTNUM, 1> Wn_;
			Eigen::Vector3d pnow;
			Eigen::Matrix3d Rnow;
			Eigen::Matrix<double, 6, JOINTNUM> Jacnow;

			lee::blocks::LLog<double> Logger;

			Eigen::Vector3d Getw(char axis, double dth);
			Eigen::Matrix3d GetRot(char axis, double th);
			Eigen::Vector3d rot3omega(Eigen::Matrix3d R);
			Eigen::Matrix<double, 6, 1> calculate_err(
				Eigen::Vector3d p_ref, 
				Eigen::Matrix3d R_ref, 
				Eigen::Vector3d p_rel, 
				Eigen::Matrix3d R_rel);
			void GetFK(char legno, 
				Eigen::Matrix<double, 3, 1>& pout, 
				Eigen::Matrix3d& Rout, 
				Eigen::Matrix<double, 6, JOINTNUM>& Jacout,
				Eigen::Matrix<double, JOINTNUM, 1> qin);
			void GetIK(char legno,
				Eigen::Matrix<double, JOINTNUM, 1>& qout,
				Eigen::Vector3d pin,
				Eigen::Matrix3d Rin,
				Eigen::Matrix<double, JOINTNUM, 1> qin);
			
		public:
			NumIKBlock();
			int init();
			int run();
			int print();
			int log();
			int clear();
		};
	}
}//namespace lee