#include <iostream>
#include <Compliance/CHZ/CHZBlock.h>
#include <LSimpleRoboticsMath/Biped.hh>
#include <Chz_Control_lib/Chz_Spline.h>

namespace compliance{namespace chz{
	constexpr double Control_T = 0.004;
	class ChzFootDown
	{
		friend class ChzFootDowns;
	private:
		double X1, Y1, X2, Y2, K2;
		double Kp_up, Kd_up, zd_up;
		double Kp_down, Kd_down, zd_down;
		double Kp_pause, Kd_pause, zd_pause; int n_pause, n_pause_tot;
		double Kp_return, Kd_return;
		double zcon[3];

		std::string mode;

		void Reset();
		void SetMode(std::string s);
		void Update();
		double Getdelz();
	};
	class ChzFootDowns
	{	
		friend class ChzLandBlock;
		ChzFootDown LCon, RCon;
	public:
		int Reset_nonRT();
		int Reset_RT();
		int Update(double Lz, double LFz, double Rz, double RFz, double tnow, char supleg, char supleglast);
		void GetZCon(double& Lzcon, double& Rzcon);
		ChzLandBlock* pBlock;
	private:
		int flock;
		char Supleg;
	};
	ChzFootDowns ChzFootDownCon;

	class ChzLanding
	{
		friend class ChzLandings;
	private:
		double E0, fmin_mode, fmin_con;
		int Inte_num_max;
		double X1, Y1, X2, Y2, K2;

		double W, Wo;
		double z1, z1_last, z2;
		double zcon_real;
		std::string mode;
		int Inte_num;
		double delz_inte;

		void Reset(double e1, double fmin1, double fmin2, int intenum, double xx1, double yy1, double xx2, double yy2, double kk2);
		void SetMode(std::string s);
		void Update(double f, double z2next);
		double Getdelz();
	};
	class ChzLandings
	{	
		friend class ChzLandBlock;
		ChzLanding LCon[6], RCon[6];
	public:
		int Reset_nonRT();
		int Reset_RT();
		int Update(double Lz, double LFz, double Rz, double RFz, double tnow, char supleg, char supleglast);
		void GetZCon(double& Lzcon, double& Rzcon);
		ChzLandBlock* pBlock;
	private:
		char Supleg;
		int Lobs, Robs;
	};
	ChzLandings ChzLandCon;

    int ChzLandBlock::init()
    {
		static int iffirst = 1;
		if(iffirst) ChzFootDownCon.Reset_nonRT();
		ChzFootDownCon.Reset_RT();
		ChzFootDownCon.pBlock = this;
		if(iffirst) ChzLandCon.Reset_nonRT();
		ChzLandCon.Reset_RT();
		ChzLandCon.pBlock = this;
		this->tnow = 0.0;
		this->Tssp = 0.64;
		this->supleg = this->supleglast = 'D';
		iffirst = 0;
      return 0;
    }
    double Lcon_footdown = 0.0, Rcon_footdown = 0.0;
	double Lcon_landing = 0.0, Rcon_landing = 0.0;
	int ChzLandBlock::run()
    {
		switch(*(this->DataInput.nSupleg))
		{
		case (lee::math::biped::DS_SUP):
			this->supleg = 'D';
			break;
		case (lee::math::biped::L_SUP):
			this->supleg = 'L';
			break;
		case (lee::math::biped::R_SUP):
			this->supleg = 'R';
			break;
		default:
			this->supleg = 'D';
			break;
		}
		if(this->supleg != this->supleglast && this->supleg != 'D') this->tnow = 0.0;
		ChzFootDownCon.Update(this->DataInput.dLAnkPosPG[2], this->DataInput.dLFootFSens[2], 
									 this->DataInput.dRAnkPosPG[2], this->DataInput.dRFootFSens[2],
									 this->tnow, this->supleg, this->supleglast);
		//double Lcon_footdown = 0.0, Rcon_footdown = 0.0;
		ChzFootDownCon.GetZCon(Lcon_footdown, Rcon_footdown);
		ChzLandCon.Update(this->DataInput.dLAnkPosPG[2] + Lcon_footdown, this->DataInput.dLFootFSens[2], 
								this->DataInput.dRAnkPosPG[2] + Rcon_footdown, this->DataInput.dRFootFSens[2],
								this->tnow, this->supleg, this->supleglast);
		//double Lcon_landing = 0.0, Rcon_landing = 0.0;
		ChzLandCon.GetZCon(Lcon_landing, Rcon_landing);
		if(*this->DataInput.pControlStartFlag == true)
		{
			this->DataOutput.L_FootPosL[0] = this->DataInput.dLAnkPosPG[0];
			this->DataOutput.L_FootPosL[1] = this->DataInput.dLAnkPosPG[1];
			this->DataOutput.L_FootPosL[2] = this->DataInput.dLAnkPosPG[2] + Lcon_footdown + Lcon_landing;
			this->DataOutput.L_FootPosR[0] = this->DataInput.dRAnkPosPG[0];
			this->DataOutput.L_FootPosR[1] = this->DataInput.dRAnkPosPG[1];
			this->DataOutput.L_FootPosR[2] = this->DataInput.dRAnkPosPG[2] + Rcon_footdown + Rcon_landing;
		}
		else
		{
			ChzF(i, 0, 2) this->DataOutput.L_FootPosL[i] = this->DataInput.dLAnkPosPG[i];
			ChzF(i, 0, 2) this->DataOutput.L_FootPosR[i] = this->DataInput.dRAnkPosPG[i];
		}
		this->tnow += Control_T;
		this->supleglast = this->supleg;
      return 0;
    }
    int ChzLandBlock::clear()
    {

        return 0;
    }
    int ChzLandBlock::log()
    {
		auto p = this->DataInput.pLogger;
		auto &In = this->DataInput;
		auto &Out = this->DataOutput;
		
		double dsupleg = (ChzFootDownCon.Supleg == 'R'? 0.0: (ChzFootDownCon.Supleg == 'L'? 1.0: 2.0));
		p->addLog(tnow, "tnow");
		p->addLog(dsupleg, "Supleg_Footdown");
		p->addLog(In.dLAnkPosPG[2], "PLAnkz");
		p->addLog(In.dRAnkPosPG[2], "PRAnkz");
		p->addLog(Out.L_FootPosL[2], "CLAnkz1");
		p->addLog(Out.L_FootPosR[2], "CRAnkz1");
		p->addLog(Lcon_footdown, "LFootDown");
		p->addLog(Rcon_footdown, "RFootDown");
		p->addLog(Lcon_landing, "LLanding");
		p->addLog(Rcon_landing, "RLanding");
        return 0;
    }
    int ChzLandBlock::print()
    {
		// std::cout<<"[DCC Control]" <<std::endl
		// 	<< "Torque Pitch ConVal : " <<  stStateConVal.Ankle.B.Lfoot.rot.pit <<", "<<  stStateConVal.Ankle.B.Rfoot.rot.pit << std::endl
		// 	// << "Torque Pitch Sens: " << stStateSens.FootFT.Lfoot.tx <<", "<< stStateSens.FootFT.Rfoot.tx
		// 	<< std::endl; 
		using namespace ljh::tools;
		switch (this->GUIFlag)
		{
		case GUIStateFlag::GUI_OFF :
			std::cout<<"[CHZ Control]"<<std::endl
			<< "Lcon_footdown: " << Lcon_footdown <<", "<<"Rcon_footdown: " << Rcon_footdown <<std::endl 
			<< "Lcon_landing: " << Lcon_landing <<", "<<"Rcon_landing: " << Rcon_landing <<std::endl; 

			break;
		case GUIStateFlag::GUI_ON :
			ImGui::Begin("CHZ Control");
			if (ImGui::BeginTable("CHZ-Control", 2))
        	{
        	    ImU32 row_bg_color_header = ImGui::GetColorU32(ImVec4(((float)51)/255, ((float)102)/255, ((float)0.)/255, 0.65f));
        	    ImU32 row_bg_color = ImGui::GetColorU32(ImVec4(((float)0)/255, ((float)102)/255, ((float)51)/255, 0.65f));
        	    //ImU32 row_bg_color_1 = ImGui::GetColorU32(ImVec4(((float)0.)/255, ((float)51)/255, ((float)102)/255, 0.65f));
        	    ImGui::TableNextRow();
        	    ImGui::TableSetBgColor(ImGuiTableBgTarget_RowBg0 + 1, row_bg_color_header);
        	    ImGui::TableNextColumn();
        	    ImGui::Text("Left footdown");
        	    ImGui::TableNextColumn();
        	    ImGui::Text("Right footdown");

				ImGui::TableNextRow();
        	    ImGui::TableSetBgColor(ImGuiTableBgTarget_RowBg0 + 1, row_bg_color);
        	    ImGui::TableNextColumn();
        	    ImGui::Text("%.3f", Lcon_footdown);
        	    ImGui::TableNextColumn();
        	    ImGui::Text("%.3f", Rcon_footdown);

				ImGui::TableNextRow();
        	    ImGui::TableSetBgColor(ImGuiTableBgTarget_RowBg0 + 1, row_bg_color_header);
        	    ImGui::TableNextColumn();
        	    ImGui::Text("Left landing");
        	    ImGui::TableNextColumn();
        	    ImGui::Text("Right landing");

				ImGui::TableNextRow();
        	    ImGui::TableSetBgColor(ImGuiTableBgTarget_RowBg0 + 1, row_bg_color);
        	    ImGui::TableNextColumn();
        	    ImGui::Text("%.3f", Lcon_landing);
        	    ImGui::TableNextColumn();
        	    ImGui::Text("%.3f", Rcon_landing);

				ImGui::TableNextRow();
        	    ImGui::TableSetBgColor(ImGuiTableBgTarget_RowBg0 + 1, row_bg_color_header);
        	    ImGui::TableNextColumn();
        	    ImGui::Text("Ts-sup");

				ImGui::TableNextRow();
        	    ImGui::TableSetBgColor(ImGuiTableBgTarget_RowBg0 + 1, row_bg_color);
        	    ImGui::TableNextColumn();
        	    ImGui::Text("%.3f", this->Tssp);


				ImGui::EndTable();

			}
			ImGui::End();
			break;
		default:
			break;
		}
        return 0;
    }
	
	void ChzFootDown::Reset()
	{
		zd_down = -0.005; // -0.005
		zd_up = 0.004; // -0.005
		zcon[0] = zcon[1] = 0.0;
		Chz::GetPDParam(Kp_up, Kd_up, 4.0 * Chz::pi, 0.9);
		Chz::GetPDParam(Kp_down, Kd_down, 7.0 * Chz::pi, 0.9);
		Chz::GetPDParam(Kp_pause, Kd_pause, 8.0 * Chz::pi, 0.9);
		n_pause_tot = int(floor(0.08 / Control_T + 1e-6));
		Chz::GetPDParam(Kp_return, Kd_return, 4.0 * Chz::pi, 0.9);
		X1 = 0.012; Y1 = 0.012; X2 = 0.020; Y2 = 0.014; K2 = 0.0;
		mode = "RETURN";
	}
	void ChzFootDown::SetMode(std::string s) 
	{
		if (s == mode) return;
		if (mode == "UP" && s != "DOWN") return;
		if (mode == "DOWN" && s != "PAUSE" && s != "UP") return;
		if (mode == "PAUSE" && s != "RETURN") return;
		if (mode == "RETURN" && s != "UP") return;

		if (s == "PAUSE") zd_pause = zcon[0], n_pause = n_pause_tot;
		mode = s; 
	}
	void ChzFootDown::Update()
	{
		double Kp, Kd, zd;
		if (mode == "RETURN") Kp = Kp_return, Kd = Kd_return, zd = 0.0;
		else if (mode == "UP") Kp = Kp_up, Kd = Kd_up, zd = zd_up;
		else if (mode == "DOWN") Kp = Kp_down, Kd = Kd_down, zd = zd_down;
		else if (mode == "PAUSE") Kp = 0.0, Kd = Kd_pause, zd = zd_pause;
		else Kp = 0.0, Kd = 0.0, zd = 0.0;
		zcon[2] = Kp * (zd - zcon[0]) + Kd * (0.0 - zcon[1]);
		zcon[1] += zcon[2] * Control_T;
		zcon[0] += zcon[1] * Control_T;
		if (mode == "PAUSE")
		{
			n_pause--;
			if (n_pause == 0) SetMode("RETURN");
		}
	}
	double ChzFootDown::Getdelz()
	{
		return Chz::GetLimitedValue(zcon[0], X1, Y1, X2, Y2, K2);
	}

	int ChzFootDowns::Reset_nonRT()
	{
		LCon.Reset();
		RCon.Reset();
		return 1;
	}
	int ChzFootDowns::Reset_RT()
	{
		LCon.zcon[0] = LCon.zcon[1] = 0.0;
		LCon.mode = "RETURN";
		LCon.zcon[0] = LCon.zcon[1] = 0.0;
		LCon.mode = "RETURN";
		flock = 0;
		Supleg = 'D';
		return 1;
	}
	int ChzFootDowns::Update(double Lz, double LFz, double Rz, double RFz, double tnow, char supleg, char supleglast)
	{
		constexpr double Up_time = 0.00;
		constexpr double Down_time = 0.04;
		int Lcon = 0, Rcon = 0;
		if (supleg != 'N' && supleg != 'D') Supleg = supleg;

		if(supleg != 'D' && supleglast == 'D') 
			if(supleg == 'L') LCon.SetMode("PAUSE"), flock = 0; else if(supleg == 'R') RCon.SetMode("PAUSE"), flock = 0;
		// swing foot up
		if (!flock && tnow - Up_time > 0.0) if (supleg == 'L') Rcon = 1; else if (supleg == 'R') Lcon = 1;
		// swing foot down
		if (tnow + Down_time > pBlock->Tssp) if (supleg == 'L') Rcon = 0; else if (supleg == 'R') Lcon = 0;
		// force threshold
		if (tnow + Down_time > 0.75 * pBlock->Tssp)
		{
			if (Supleg == 'L' && RFz > 60.0) RCon.SetMode("PAUSE"), flock = 1;
			else if (Supleg == 'R' && LFz > 60.0) LCon.SetMode("PAUSE"), flock = 1;
		}
		if (Lcon == 0) LCon.SetMode("DOWN");
		else LCon.SetMode("UP");
		if (Rcon == 0) RCon.SetMode("DOWN");
		else RCon.SetMode("UP");
		//Update
		LCon.Update(); RCon.Update();
		return 1;
	}
	void ChzFootDowns::GetZCon(double& Lzcon, double& Rzcon)
	{
		Lzcon = LCon.Getdelz(); Rzcon = RCon.Getdelz();
	}

	void ChzLanding::Reset(double e1, double fmin1, double fmin2, int intenum, double xx1, double yy1, double xx2, double yy2, double kk2)
	{
		E0 = e1; fmin_mode = fmin1; fmin_con = fmin2; Inte_num_max = intenum;
		X1 = xx1; Y1 = yy1; X2 = xx2; Y2 = yy2; K2 = kk2;
		mode = "WAIT";
	}
	void ChzLanding::SetMode(std::string s)
	{
		if (mode == s) return;
		if (s == "WAIT")
		{
			mode = "WAIT";
			W = E0;
			z1 = z2;
			z1_last = z1;
			return;
		}
		if (s == "PO") { mode = "PO"; return; }
		if (s == "PC") { mode = "PC"; return; }
		if (s == "INTE")
		{
			mode = "INTE";
			Inte_num = 0;
			delz_inte = zcon_real;
		}
	}
	void ChzLanding::Update(double f, double z2next)
	{
		f = fmax(f, 0.01);
		if (mode == "WAIT")
		{
			z2 = z2next;
			z1_last = z1;
			z1 = z2;
			zcon_real = 0.0;
			return;
		}
		if (mode == "PO")
		{
			z2 = z2next;
			z1_last = z1;
			z1 = z2;
			zcon_real = 0.0;
			if (f > fmin_mode) SetMode("PC");
			return;
		}
		if (mode == "PC")
		{
			double eps = 1e-6;
			if (f < fmin_con) f = 0.01;
			if (abs(z2next - z2) < eps && abs(z2next) < eps) { SetMode("INTE"); return; }
			double delz1 = z1 - z1_last;
			double delz2_1 = z2next - z2;
			W += f * delz1;
			double Wo_1 = W + f * delz2_1;
			double delz_pc;
			if (Wo_1 < 0) delz_pc = fmin(-Wo_1 / f, 0.4e-3);
			else delz_pc = 0.0;
			double delz1_1 = delz2_1 + delz_pc;
			double z1_next = z1 + delz1_1;
			z1_last = z1;
			z2 = z2next;
			z1 = z1_next;

			zcon_real = Chz::GetLimitedValue(z1 - z2, X1, Y1, X2, Y2, K2);
			return;
		}
		if (mode == "INTE")
		{
			double tempt = 1.0 * Inte_num / (1.0 * Inte_num_max);
			static double a[7], x[3];
			Chz::GenerateSpline6(a, delz_inte, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0);
			Chz::GetSpline6(x, a, tempt);

			z2 = z2next;
			z1_last = z1;
			z1 = x[0] + z2;
			zcon_real = z1 - z2;
			Inte_num++;
			if (Inte_num > Inte_num_max) SetMode("WAIT");
		}
	}
	double ChzLanding::Getdelz()
	{
		return zcon_real;
	}

	int ChzLandings::Reset_nonRT() { return 1; }
	int ChzLandings::Reset_RT()
	{
		const int Inte_time = int(floor(0.2 / Control_T + 1e-6));

		//LCon[0].Reset(0.01, 40.0, 20.0, Inte_time, 4e-3, 4e-3, 7e-3, 6e-3, 0.0);
		//LCon[1].Reset(0.01, 40.0, 20.0, Inte_time, 4e-3, 4e-3, 7e-3, 6e-3, 0.0);
		LCon[2].Reset(0.01, 50.0, 30.0, Inte_time, 6e-3, 6e-3, 10e-3, 9e-3, 0.0);
		//LCon[3].Reset(0.01, 2.5, 2.0, Inte_time, 1e-2, 1e-2, 3e-2, 2.5e-2, 0.0);
		//LCon[4].Reset(0.01, 2.5, 2.0, Inte_time, 1e-2, 1e-2, 3e-2, 2.5e-2, 0.0);
		//LCon[5].Reset(0.01, 2.5, 2.0, Inte_time, 1e-2, 1e-2, 3e-2, 2.5e-2, 0.0);

		//RCon[0].Reset(0.01, 40.0, 20.0, Inte_time, 4e-3, 4e-3, 7e-3, 6e-3, 0.0);
		//RCon[1].Reset(0.01, 40.0, 20.0, Inte_time, 4e-3, 4e-3, 7e-3, 6e-3, 0.0);
		RCon[2].Reset(0.01, 50.0, 30.0, Inte_time, 6e-3, 6e-3, 10e-3, 9e-3, 0.0);
		//RCon[3].Reset(0.01, 2.5, 2.0, Inte_time, 1e-2, 1e-2, 3e-2, 2.5e-2, 0.0);
		//RCon[4].Reset(0.01, 2.5, 2.0, Inte_time, 1e-2, 1e-2, 3e-2, 2.5e-2, 0.0);
		//RCon[5].Reset(0.01, 2.5, 2.0, Inte_time, 1e-2, 1e-2, 3e-2, 2.5e-2, 0.0);

		Lobs = 0, Robs = 0;
		Supleg = 'D';
		return 1;
	}
	int ChzLandings::Update(double Lz, double LFz, double Rz, double RFz, double tnow, char supleg, char supleglast)
	{
		static int Inte_num_L = 0, Inte_num_R = 0;
		const int Inte_time = int(floor(0.08 / Control_T + 1e-6)) + 1;
		const int Obs_time = int(floor(0.12 / Control_T + 1e-6)) + 1;

		double fzl_real = fmin(fmax(LFz, 0.0), 800.0);
		double fzr_real = fmin(fmax(RFz, 0.0), 800.0);

		LCon[2].Update(fzl_real, Lz);
		RCon[2].Update(fzr_real, Rz);
		
		constexpr double Obs_advtime = 0.12;
		int Lobsnew = Lobs, Robsnew = Robs;
		if (1)
		{
			if (supleg != 'N' && supleg != 'D') Supleg = supleg;

			if (pBlock->tnow < Chz::eps) Lobs = Robs = Lobsnew = Robsnew = 0;
			if (pBlock->tnow + Obs_advtime > pBlock->Tssp)
				if (Supleg == 'L') Robsnew = 1; else if (Supleg == 'R') Lobsnew = 1;

			if (Lobsnew == 1 && Lobs == 0) LCon[2].SetMode("PO"), Inte_num_L = Obs_time + Inte_time;
			if (Robsnew == 1 && Robs == 0) RCon[2].SetMode("PO"), Inte_num_R = Obs_time + Inte_time;
			Lobs = Lobsnew; Robs = Robsnew;
			Inte_num_L--;
			Inte_num_R--;
			//Inte
			if (Inte_num_L == 1) LCon[2].SetMode("INTE");
			if (Inte_num_R == 1) RCon[2].SetMode("INTE");
		}
		return 1;
	}
	void ChzLandings::GetZCon(double& Lzcon, double& Rzcon)
	{
		Lzcon = LCon[2].Getdelz();
		Rzcon = RCon[2].Getdelz();
	}
}
}