#include <NumericalIK/NumIKBlock.h>
#include <iostream>
#include <Chz_Control_lib/Chz_Base.h>
// for gao robot

namespace chz
{
	namespace NumIK
	{
		constexpr double widhip = 0.09, lhip = 0.0075, lthigh = 0.33, lshank = 0.33, lank = 0.05;

		NumIKBlock::NumIKBlock()
		{
			std::cout<<"Create Block: Chz Numerical Inverse Kinematics"<<std::endl;
		}
		int NumIKBlock::init()
		{
			ws[0] << 0.0, 0.0, 0.0;
			Rs[0] = Eigen::Matrix3d::Identity();
			as[0] << 0.0, 0.0, 1.0;
			Chz::Assign(axes, 'z', 'y', 'x', 'y', 'y', 'x');
			Chz::Assign(jointseq, 0, 2, 1, 3, 4, 5 );
			ChzF(i, 0, JOINTNUM)
			{
				if (i == 3) pr[i] << 0.0, 0.0, -lthigh;
				else if (i == 4) pr[i] << 0.0, 0.0, -lshank;
				else if (i == 5) pr[i] << 0.0, 0.0, -lank;
				else pr[i] << 0.0, 0.0, 0.0;
			}
			double D2R = acos(-1.0) / 180.0;
			ql << 0.0, 0.0, -12.5 * D2R, 25.0 * D2R, -12.5 * D2R, 0.0;
			qr << 0.0, 0.0, -12.5 * D2R, 25.0 * D2R, -12.5 * D2R, 0.0;

			We = 10.0 * Eigen::Matrix<double, 6, 1>::Ones();
			Wn_= 1e-3 * Eigen::Matrix<double, JOINTNUM, 1>::Ones();
			return 0;
		}
		int NumIKBlock::run()
		{
			pbod << this->DataInput.pBodyPosition [0], this->DataInput.pBodyPosition [1], this->DataInput.pBodyPosition [2];
			plft << this->DataInput.pLFootPosition[0], this->DataInput.pLFootPosition[1], this->DataInput.pLFootPosition[2];
			prft << this->DataInput.pRFootPosition[0], this->DataInput.pRFootPosition[1], this->DataInput.pRFootPosition[2];
			Rbod = GetRot('z', this->DataInput.pBodyPosture [2]) * GetRot('y', this->DataInput.pBodyPosture [1]) * GetRot('x', this->DataInput.pBodyPosture [0]);
			Rlft = GetRot('z', this->DataInput.pLFootPosture[2]) * GetRot('y', this->DataInput.pLFootPosture[1]) * GetRot('x', this->DataInput.pLFootPosture[0]);
			Rrft = GetRot('z', this->DataInput.pRFootPosture[2]) * GetRot('y', this->DataInput.pRFootPosture[1]) * GetRot('x', this->DataInput.pRFootPosture[0]);
			GetIK('L', ql, Rbod.transpose() * (plft - pbod), Rbod.transpose() * Rlft, ql);
			GetIK('R', qr, Rbod.transpose() * (prft - pbod), Rbod.transpose() * Rrft, qr);
			for(int i = 0; i != 6; i++) this->DataOutput.LegJointL[i] = ql(i);
			for(int i = 0; i != 6; i++) this->DataOutput.LegJointR[i] = qr(i);
			return 0;
		}
		int NumIKBlock::print() { return 0; }
		int NumIKBlock::log()
		{
		#ifdef _CHECK_JOINT_LIMIT
			this->Logger.startLog();
			for(int i=0;i<6;i++)
			{
				this->Logger.addLog(this->DataOutput.LegJointL[i], std::string("LegL").append(std::to_string(i+1)).c_str());
				this->Logger.addLog(this->DataOutput.LegJointR[i], std::string("LegR").append(std::to_string(i+1)).c_str());
			}
		#endif
			return 0;
		}
		int NumIKBlock::clear() { return 0; }

		void NumIKBlock::GetFK(char legno, 
			Eigen::Matrix<double, 3, 1>& pout, 
			Eigen::Matrix3d& Rout, 
			Eigen::Matrix<double, 6, JOINTNUM>& Jacout,
			Eigen::Matrix<double, JOINTNUM, 1> qin)
		{
			if (legno == 'l' || legno == 'L') ps[0] << 0.0, widhip, 0.0; else ps[0] << 0.0, -widhip, 0.0;
			for (int i = 1; i != JOINTNUM + 1; i++)
			{
				ws[i] = ws[i - 1] + Rs[i - 1] * Getw(axes[i - 1], qin(jointseq[i - 1]));
				Rs[i] = Rs[i - 1] * GetRot(axes[i - 1], qin(jointseq[i - 1]));
				as[i] = Rs[i - 1] * Getw(axes[i - 1], 1.0);
				ps[i] = ps[i - 1] + Rs[i] * pr[i];
			}
			pout = ps[JOINTNUM];
			Rout = Rs[JOINTNUM];
			for (int i = 0; i != JOINTNUM; i++)
			{
				Jacout.block<3, 1>(0, jointseq[i]) = as[i + 1].cross(ps[JOINTNUM] - ps[i]);
				Jacout.block<3, 1>(3, jointseq[i]) = as[i + 1];
			}
		}
		void NumIKBlock::GetIK(char legno,
			Eigen::Matrix<double, JOINTNUM, 1>& qout,
			Eigen::Vector3d pin,
			Eigen::Matrix3d Rin,
			Eigen::Matrix<double, JOINTNUM, 1> qin)
		{
			qout = qin;
			double E_n_last = 1e10;
			int nn = 100;
			while (nn--)
			{
				GetFK(legno, pnow, Rnow, Jacnow, qout);
				Eigen::Matrix<double, 6, 1> e_n = calculate_err(pin, Rin, pnow, Rnow);
				auto g_n = Jacnow.transpose() * We.asDiagonal() * e_n;
				double E_n = 0.5 * e_n.transpose() * We.asDiagonal() * e_n;
				auto W_n = E_n * Eigen::Matrix<double, JOINTNUM, 1>::Ones() + Wn_;
				auto H_n = Jacnow.transpose() * We.asDiagonal() * Jacnow + Eigen::Matrix<double, JOINTNUM, JOINTNUM>::Identity() * W_n.asDiagonal();
				qout += H_n.inverse() * g_n;
				if (fabs(E_n) < 1e-8 || fabs(E_n - E_n_last) < 1e-8 || E_n_last < E_n)
					break;
				E_n_last = E_n;
			}
			return;
		}
		Eigen::Vector3d NumIKBlock::Getw(char axis, double dth)
		{
			if (axis == 'z') return Eigen::Vector3d(0.0, 0.0, dth);
			if (axis == 'y') return Eigen::Vector3d(0.0, dth, 0.0);
			if (axis == 'x') return Eigen::Vector3d(dth, 0.0, 0.0);
			return Eigen::Vector3d(0.0, 0.0, 0.0);
		}

		Eigen::Matrix3d NumIKBlock::GetRot(char axis, double th)
		{
			Eigen::Matrix3d R;
			switch (axis)
			{
			case 'z':
				R << cos(th), -sin(th), 0.0, sin(th), cos(th), 0.0, 0.0, 0.0, 1.0;
				break;
			case 'y':
				R << cos(th), 0.0, sin(th), 0.0, 1.0, 0.0, -sin(th), 0.0, cos(th);
				break;
			case 'x':
				R << 1.0, 0.0, 0.0, 0.0, cos(th), -sin(th), 0.0, sin(th), cos(th);
				break;
			default:
				R = Eigen::Matrix3d::Identity();
			}
			return R;
		}
		Eigen::Vector3d NumIKBlock::rot3omega(Eigen::Matrix3d R)
		{
			Eigen::Vector3d omega;
			double theta = cos((R.trace() - 1) / 2);
			if (abs(theta) < 1e-6)
				omega.setZero();
			else
			{
				omega << R(2, 1) - R(1, 2), R(0, 2) - R(2, 0), R(1, 0) - R(0, 1);
				omega = theta / (2 * sin(theta)) * omega;
			}
			return omega;
		}

		Eigen::Matrix<double, 6, 1> NumIKBlock::calculate_err(Eigen::Vector3d p_ref, Eigen::Matrix3d R_ref, Eigen::Vector3d p_rel, Eigen::Matrix3d R_rel)
		{
			Eigen::Matrix<double, 6, 1> err;
			err.segment<3>(0) = p_ref - p_rel;
			err.segment<3>(3) = R_rel * rot3omega(R_rel.transpose() * R_ref);
			return err;
		}
	}
}