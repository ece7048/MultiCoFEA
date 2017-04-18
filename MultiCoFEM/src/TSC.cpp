#include "TSC.h"
#include "FEBRunner.h"
#include "INIReader.h"

///// based the position DoF because these are the difficult section in convert of FEM dynamic analysis///////////////

TSC::TSC(void)
{
}

TSC::~TSC(void)
{
}
void TSC::run(Vector fpx, Vector fpy, Vector fpz, Vector fox, Vector foy, Vector foz, Vector spx, Vector spy, Vector spz, Vector sox, Vector soy, Vector soz, Vector time){

	//Vector t = timedistance( time);
	int sizert = time.size();
	double t;
	double prevt;
	double diff[500];

	for (int i = 1; i < sizert; ++i){
		t = time[i];
		prevt = time[i - 1];
		diff[i] = t - prevt;
	}
	Vector tdiff(sizert, diff);
	/////////////////////////////////

	//Vector f = vectordistance(fpx, fpy, fpz, fox, foy, foz, spx, spy, spz, sox, soy, soz);

	int sizer = fpx.size();
	double f[500];
	double f1;
	double prevf1;
	double diff1[500];
	double f2;
	double prevf2;
	double diff2[500];
	double f3;
	double prevf3;
	double diff3[500];
	double f4;
	double prevf4;
	double diff4[500];
	double f5;
	double prevf5;
	double diff5[500];
	double f6;
	double prevf6;
	double diff6[500];
	double f7;
	double prevf7;
	double diff7[500];
	double f8;
	double prevf8;
	double diff8[500];
	double f9;
	double prevf9;
	double diff9[500];
	double f10;
	double prevf10;
	double diff10[500];
	double f11;
	double prevf11;
	double diff11[500];
	double f12;
	double prevf12;
	double diff12[500];
	//Compute the distances value with normilized vector of position found in any time step the one with the bigger step
	diff1[0] = fpx[0];
	diff2[0] = fpy[0];
	diff3[0] = fpz[0];
	diff4[0] = fox[0];
	diff5[0] = foy[0];
	diff6[0] = foz[0];
	diff7[0] = spx[0];
	diff8[0] = spy[0];
	diff9[0] = spz[0];
	diff10[0] = sox[0];
	diff11[0] = soy[0];
	diff12[0] = soz[0];

	double max = 0;
	if (abs(diff1[0]) > abs(diff2[0])){ max = abs(diff1[0]); }
	if (abs(diff1[0]) < abs(diff2[0])){ max = abs(diff2[0]); }
	if (abs(diff3[0]) > max){ max = abs(diff3[0]); }
	if (abs(diff4[0]) > max){ max = abs(diff4[0]); }
	if (abs(diff5[0]) > max){ max = abs(diff5[0]); }
	if (abs(diff6[0]) > max){ max = abs(diff6[0]); }
	if (abs(diff7[0]) > max){ max = abs(diff7[0]); }
	if (abs(diff8[0]) > max){ max = abs(diff8[0]); }
	if (abs(diff9[0]) > max){ max = abs(diff9[0]); }
	if (abs(diff10[0]) > max){ max = abs(diff10[0]); }
	if (abs(diff11[0]) > max){ max = abs(diff11[0]); }
	if (abs(diff12[0]) > max){ max = abs(diff12[0]); }
	f[0] = max;

	for (int i = 2; i < sizer; ++i){
		f1 = fpx[i];
		prevf1 = fpx[i - 1];
		diff1[i - 1] = f1 - prevf1;

		f2 = fpy[i];
		prevf2 = fpy[i - 1];
		diff2[i - 1] = f2 - prevf2;

		f3 = fpz[i];
		prevf3 = fpz[i - 1];
		diff3[i - 1] = f3 - prevf3;

		f4 = fox[i];
		prevf4 = fox[i - 1];
		diff4[i - 1] = f4 - prevf4;

		f5 = foy[i];
		prevf5 = foy[i - 1];
		diff5[i - 1] = f5 - prevf5;

		f6 = foz[i];
		prevf6 = foz[i - 1];
		diff6[i - 1] = f6 - prevf6;

		f7 = spx[i];
		prevf7 = spx[i - 1];
		diff7[i - 1] = f7 - prevf7;

		f8 = spy[i];
		prevf8 = spy[i - 1];
		diff8[i - 1] = f8 - prevf8;

		f9 = spz[i];
		prevf9 = spz[i - 1];
		diff9[i - 1] = f9 - prevf9;

		f10 = sox[i];
		prevf10 = sox[i - 1];
		diff10[i - 1] = f10 - prevf10;

		f11 = soy[i];
		prevf11 = soy[i - 1];
		diff11[i - 1] = f11 - prevf11;

		f12 = soz[i];
		prevf12 = soz[i - 1];
		diff12[i - 1] = f12 - prevf12;
		if (abs(diff1[i - 1]) > abs(diff2[i - 1])){ max = abs(diff1[i - 1]); }
		if (abs(diff1[i - 1]) < abs(diff2[i - 1])){ max = abs(diff2[i - 1]); }
		if (abs(diff3[i - 1]) > max){ max = abs(diff3[i - 1]); }
		if (abs(diff4[i - 1]) > max){ max = abs(diff4[i - 1]); }
		if (abs(diff5[i - 1]) > max){ max = abs(diff5[i - 1]); }
		if (abs(diff6[i - 1]) > max){ max = abs(diff6[i - 1]); }
		if (abs(diff7[i - 1]) > max){ max = abs(diff7[i - 1]); }
		if (abs(diff8[i - 1]) > max){ max = abs(diff8[i - 1]); }
		if (abs(diff9[i - 1]) > max){ max = abs(diff9[i - 1]); }
		if (abs(diff10[i - 1]) > max){ max = abs(diff10[i - 1]); }
		if (abs(diff11[i - 1]) > max){ max = abs(diff11[i - 1]); }
		if (abs(diff12[i - 1]) > max){ max = abs(diff12[i - 1]); }
		f[i - 1] = max;
	}
	Vector fx(sizer, f);
	////////////////////////////////////
	int sizerf = time.size();

	string answer;
	INIReader ini = INIReader(INI_FILE);
	cout << "" << endl;
	cout << "START THE TIMESTEP COUNTER FOR THE RESAMPLE VECTORS" << endl;
	cout << "" << endl;
	cout << "Do you want TSC method ? (y/n)" << endl;
	string answer1 = ini.Get("BASICSETUP", "TSC_method", "");
	cout << answer1 << endl;

	if (answer1 == "y"){
		cout << "1. Give the maximun timesteps you want for the Initial Step Simulation: " << endl;
		double maxtimestep = ini.GetReal("BASICSETUP", "Max_timesteps", 0);
		cout << maxtimestep << endl;
		
		cout << "2. Give the minimum timesteps you want for the Initial Step Simulation: " << endl;
		double mintimestep = ini.GetReal("BASICSETUP", "Min_timesteps", 0);
		cout << mintimestep << endl;
		
		stepdetermine(sizerf, tdiff, fx, maxtimestep, mintimestep);
		cout << "" << endl;

		while (answer != "n"){
			cout << "Do you want the TSC method to reused once again for different results? (y/n)" << endl;
			cin >> answer;
			{if (answer == "y"){
				cout << "1. Give the maximun timesteps you want for the Initial Step Simulation: " << endl;
				int max;
				cin >> max;
				cout << "2. Give the minimum timesteps you want for the Initial Step Simulation: " << endl;
				int min;
				cin >> min;
				stepdetermine(sizerf, tdiff, fx, maxtimestep, mintimestep);
			}
			else if (answer != "y" && answer != "n") { cout << "ERROR...unused answer " << endl; }
			}
		}
	}
	if (answer1 == "n"){
		
		stepswriter();
	}
	cout << "" << endl;
}



void TSC::stepdetermine(int sizer, Vector t, Vector f, int maxtimestep, int mintimestep){
	// the 0 value is for the static the other for dynamic..
	//determine the static step and timestep
	int stepstatic;
	double timestepstatic;
	double dist = maxtimestep - mintimestep;
	double max1s = abs(f[0]);
	double max2s = t[1];
	cout << max2s << endl;
	cout << max1s << endl;
	//linear determine step 
	if (max1s > max2s){
		stepstatic = maxtimestep - (max2s / max1s)*mintimestep * dist * 1000;
		//ensure that you are inside the boundaries
		if (stepstatic < mintimestep){
			for (int start = dist; start > 0; --start) {
				if (stepstatic < mintimestep){ stepstatic = maxtimestep - (max2s / max1s)*mintimestep * start * 1000; cout << start << endl; }
			}
			if (stepstatic < mintimestep){ stepstatic = mintimestep; }
		}
	}
	if (max1s < max2s){
		stepstatic = maxtimestep - (max1s / max2s)*mintimestep * dist * 1000;
		//ensure that you are inside the boundaries
		if (stepstatic < mintimestep){
			for (int start = dist; start > 0; --start) {
				if (stepstatic < mintimestep){ stepstatic = maxtimestep - (max1s / max2s)*mintimestep * start * 1000; cout << start << endl; }
			}
			if (stepstatic < mintimestep){ stepstatic = mintimestep; }
		}
	}
	if (maxtimestep == mintimestep){ stepstatic = maxtimestep; cout << "the maximum and minimum steps are equal so we will take the: " << maxtimestep << " as static step" << endl; } // a way to determine the steps for static...

	
	timestepstatic = max2s / stepstatic;
	char rounding;
	cout << "For the first round of TSC the static steps and numper are:" << endl;
	cout << "timestep for the STATIC analysis are: " << timestepstatic << endl;
	cout << "steps for the STATIC analysis are: " << stepstatic << endl;
	cout << "Do you want a modification of the time step value for more rounding result? (y/n)" << endl;
	cin >> rounding;
	if (rounding=='y'){
	// take only the one accuration number and recompute the steps.
	int try1s = timestepstatic * 10;
	int try2s = timestepstatic * 100;
	int try3s = timestepstatic * 1000;
	int try4s = timestepstatic * 10000;
	int try5s = timestepstatic * 100000;
	int try6s = timestepstatic * 1000000;
	int try7s = timestepstatic * 10000000;
	int try8s = timestepstatic * 100000000;
	int try9s = timestepstatic * 1000000000;
	int try10s = timestepstatic * 10000000000;
	int try11s = timestepstatic * 100000000000;
	int try12s = timestepstatic * 1000000000000;
	int try13s = timestepstatic * 10000000000000;
	int try14s = timestepstatic * 100000000000000;
	if (try1s != 0){
		if ((try2s - try1s * 10) > 5){ timestepstatic = (try1s + 1)*0.1; try2s = 0; try3s = 0; try4s = 0; try5s = 0; try6s = 0; try7s = 0; try8s = 0; try9s = 0; try10s = 0; try11s = 0; try12s = 0; try13s = 0; try14s = 0; }
		else if ((try2s - try1s * 10) < 5){ timestepstatic = (try1s)*0.1; try2s = 0; try3s = 0; try4s = 0; try5s = 0; try6s = 0; try7s = 0; try8s = 0; try9s = 0; try10s = 0; try11s = 0; try12s = 0; try13s = 0; try14s = 0; }
	}
	if (try2s != 0){
		if ((try3s - try2s * 10) > 5){ timestepstatic = (try2s + 1)*0.01;  try3s = 0; try4s = 0; try5s = 0; try6s = 0; try7s = 0; try8s = 0; try9s = 0; try10s = 0; try11s = 0; try12s = 0; try13s = 0; try14s = 0; }
		else if ((try3s - try2s * 10) < 5){ timestepstatic = (try2s)*0.01; try3s = 0; try4s = 0; try5s = 0; try6s = 0; try7s = 0; try8s = 0; try9s = 0; try10s = 0; try11s = 0; try12s = 0; try13s = 0; try14s = 0; }
	}
	if (try3s != 0){
		if ((try4s - try3s * 10) > 5){ timestepstatic = (try3s + 1)*0.001;  try4s = 0; try5s = 0; try6s = 0; try7s = 0; try8s = 0; try9s = 0; try10s = 0; try11s = 0; try12s = 0; try13s = 0; try14s = 0; }
		else if ((try4s - try3s * 10) < 5){ timestepstatic = (try3s)*0.001;  try4s = 0; try5s = 0; try6s = 0; try7s = 0; try8s = 0; try9s = 0; try10s = 0; try11s = 0; try12s = 0; try13s = 0; try14s = 0; }
	}
	if (try4s != 0){
		if ((try5s - try4s * 10) > 5){ timestepstatic = (try4s + 1)*0.0001;  try5s = 0; try6s = 0; try7s = 0; try8s = 0; try9s = 0; try10s = 0; try11s = 0; try12s = 0; try13s = 0; try14s = 0; }
		else if ((try5s - try4s * 10) < 5){ timestepstatic = (try4s)*0.0001;  try5s = 0; try6s = 0; try7s = 0; try8s = 0; try9s = 0; try10s = 0; try11s = 0; try12s = 0; try13s = 0; try14s = 0; }
	}
	if (try5s != 0){
		if ((try6s - try5s * 10) > 5){ timestepstatic = (try5s + 1)*0.00001;  try6s = 0; try7s = 0; try8s = 0; try9s = 0; try10s = 0; try11s = 0; try12s = 0; try13s = 0; try14s = 0; }
		else if ((try6s - try5s * 10) < 5){ timestepstatic = (try5s)*0.00001;  try6s = 0; try7s = 0; try8s = 0; try9s = 0; try10s = 0; try11s = 0; try12s = 0; try13s = 0; try14s = 0; }
	}
	if (try6s != 0){
		if ((try7s - try6s * 10) > 5){ timestepstatic = (try6s + 1)*0.000001;  try7s = 0; try8s = 0; try9s = 0; try10s = 0; try11s = 0; try12s = 0; try13s = 0; try14s = 0; }
		else if ((try7s - try6s * 10) < 5){ timestepstatic = (try6s)*0.000001;  try7s = 0; try8s = 0; try9s = 0; try10s = 0; try11s = 0; try12s = 0; try13s = 0; try14s = 0; }
	}
	if (try7s != 0){
		if ((try8s - try7s * 10) > 5){ timestepstatic = (try7s + 1)*0.0000001;  try8s = 0; try9s = 0; try10s = 0; try11s = 0; try12s = 0; try13s = 0; try14s = 0; }
		else if ((try8s - try7s * 10) < 5){ timestepstatic = (try7s)*0.0000001;  try8s = 0; try9s = 0; try10s = 0; try11s = 0; try12s = 0; try13s = 0; try14s = 0; }
	}
	if (try8s != 0){
		if ((try9s - try8s * 10) > 5){ timestepstatic = (try8s + 1)*0.00000001;  try9s = 0; try10s = 0; try11s = 0; try12s = 0; try13s = 0; try14s = 0; }
		else if ((try9s - try8s * 10) < 5){ timestepstatic = (try8s)*0.00000001;  try9s = 0; try10s = 0; try11s = 0; try12s = 0; try13s = 0; try14s = 0; }
	}
	if (try9s != 0){
		if ((try10s - try9s * 10) > 5){ timestepstatic = (try9s + 1)*0.000000001;  try10s = 0; try11s = 0; try12s = 0; try13s = 0; try14s = 0; }
		else if ((try10s - try9s * 10) < 5){ timestepstatic = (try9s)*0.000000001; try10s = 0; try11s = 0; try12s = 0; try13s = 0; try14s = 0; }
	}
	if (try10s != 0){
		if ((try11s - try10s * 10) > 5){ timestepstatic = (try10s + 1)*0.0000000001; try2s = 0; try3s = 0; try4s = 0; try5s = 0; try6s = 0; try7s = 0; try8s = 0; try9s = 0; try10s = 0; try11s = 0; try12s = 0; try13s = 0; try14s = 0; }
		else if ((try11s - try10s * 10) < 5){ timestepstatic = (try10s)*0.0000000001;  try12s = 0; try13s = 0; try14s = 0; }
	}
	if (try11s != 0){
		if ((try12s - try11s * 10) > 5){ timestepstatic = (try11s + 1)*0.00000000001; try2s = 0; try3s = 0; try4s = 0; try5s = 0; try6s = 0; try7s = 0; try8s = 0; try9s = 0; try10s = 0; try11s = 0; try12s = 0; try13s = 0; try14s = 0; }
		else if ((try12s - try11s * 10) < 5){ timestepstatic = (try11s)*0.00000000001;  try12s = 0; try13s = 0; try14s = 0; }
	}
	if (try12s != 0){
		if ((try13s - try12s * 10) > 5){ timestepstatic = (try12s + 1)*0.000000000001; try13s = 0; try14s = 0; }
		else if ((try13s - try12s * 10) < 5){ timestepstatic = (try12s)*0.000000000001;  try13s = 0; try14s = 0; }
	}
	if (try13s != 0){
		if ((try14s - try13s * 10) > 5){ timestepstatic = (try13s + 1)*0.0000000000001; try14s = 0; }
		else if ((try14s - try13s * 10) < 5){ timestepstatic = (try13s)*0.0000000000001; try14s = 0; }
	}
	if (try14s != 0){ timestepstatic = (try14s)*0.00000000000001; }
	stepstatic = max2s / timestepstatic;
}
	//cout << try1s << try2s << try3s << try4s << try5s << try6s << try7s << try8s << try9s << try10s << try11s << try12s << try13s << try14s << endl;
	//determine dynamic step base the static step

	int stepdynamic;
	double timestepdynamic;
	double max1d = abs(f[1]);
	double max2d = t[1];//the total dynamic time
	double max2dtotal = t[1];
	int sumsteps=0;
	for (int i = 2; i < sizer;++i){
		if (abs(f[i])>max1d){ max1d = abs(f[i]); max2d = t[i]; }
		//max1d = f[i];
		//double deviation = max1d / max1s;
		//int step = (int)(stepstatic*deviation);
		//sumsteps = sumsteps+step;
		//cout << sumsteps << endl;
		max2dtotal = max2dtotal + t[i]; //the total dynamic time
		} 
	double deviation = max1d / max1s;
	int step = (int)(stepstatic*deviation);

	stepdynamic = step;
	timestepdynamic = max2d / stepdynamic;
	// take only the one accuration number and recompute the steps.
	int try1 = timestepdynamic * 10;
	int try2 = timestepdynamic * 100;
	int try3 = timestepdynamic * 1000;
	int try4 = timestepdynamic * 10000;
	int try5 = timestepdynamic * 100000;
	int try6 = timestepdynamic * 1000000;
	int try7 = timestepdynamic * 10000000;
	int try8 = timestepdynamic * 100000000;
	int try9 = timestepdynamic * 1000000000;
	int try10 = timestepdynamic * 10000000000;
	int try11 = timestepdynamic * 100000000000;
	int try12 = timestepdynamic * 1000000000000;
	int try13 = timestepdynamic * 10000000000000;
	int try14 = timestepdynamic * 100000000000000;
	if (try1 != 0){ if ((try2 - try1 * 10) > 5){ timestepdynamic = (try1 + 1)*0.1; try2 = 0; try3 = 0; try4 = 0; try5 = 0; try6 = 0; try7 = 0; try8 = 0; try9 = 0; try10 = 0; try11 = 0; try12 = 0; try13 = 0; try14 = 0; }
		else if((try2 - try1 * 10)< 5){ timestepdynamic = (try1)*0.1; try2 = 0; try3 = 0; try4 = 0; try5 = 0; try6 = 0; try7 = 0; try8 = 0; try9 = 0; try10 = 0; try11 = 0; try12 = 0; try13 = 0; try14 = 0; }
	}
	if (try2 != 0){ if ((try3 - try2 * 10)> 5){ timestepdynamic = (try2 + 1)*0.01;  try3 = 0; try4 = 0; try5 = 0; try6 = 0; try7 = 0; try8 = 0; try9 = 0; try10 = 0; try11 = 0; try12 = 0; try13 = 0; try14 = 0; }
		else if((try3 - try2 * 10)< 5){ timestepdynamic = (try2)*0.01;  try3 = 0; try4 = 0; try5 = 0; try6 = 0; try7 = 0; try8 = 0; try9 = 0; try10 = 0; try11 = 0; try12 = 0; try13 = 0; try14 = 0; }
	}
	if (try3 != 0){ if ((try4 - try3 * 10)> 5){ timestepdynamic = (try3 + 1)*0.001;  try4 = 0; try5 = 0; try6 = 0; try7 = 0; try8 = 0; try9 = 0; try10 = 0; try11 = 0; try12 = 0; try13 = 0; try14 = 0; }
		else if((try4 - try3 * 10)< 5){ timestepdynamic = (try3)*0.001;  try4 = 0; try5 = 0; try6 = 0; try7 = 0; try8 = 0; try9 = 0; try10 = 0; try11 = 0; try12 = 0; try13 = 0; try14 = 0; }
	}
	if (try4 != 0){ if ((try5 - try4 * 10)> 5){ timestepdynamic = (try4 + 1)*0.0001; try5 = 0; try6 = 0; try7 = 0; try8 = 0; try9 = 0; try10 = 0; try11 = 0; try12 = 0; try13 = 0; try14 = 0; }
		else if((try5 - try4 * 10)< 5){ timestepdynamic = (try4)*0.0001; try5 = 0; try6 = 0; try7 = 0; try8 = 0; try9 = 0; try10 = 0; try11 = 0; try12 = 0; try13 = 0; try14 = 0; }
	}
	if (try5 != 0){ if ((try6 - try5 * 10)> 5){ timestepdynamic = (try5 + 1)*0.00001; try6 = 0; try7 = 0; try8 = 0; try9 = 0; try10 = 0; try11 = 0; try12 = 0; try13 = 0; try14 = 0; }
		else if((try6 - try5 * 10)< 5){ timestepdynamic = (try5)*0.00001;  try6 = 0; try7 = 0; try8 = 0; try9 = 0; try10 = 0; try11 = 0; try12 = 0; try13 = 0; try14 = 0; }
	}
	if (try6 != 0){ if ((try7 - try6 * 10)> 5){ timestepdynamic = (try6 + 1)*0.000001;  try7 = 0; try8 = 0; try9 = 0; try10 = 0; try11 = 0; try12 = 0; try13 = 0; try14 = 0; }
		else if((try7 - try6 * 10)< 5){ timestepdynamic = (try6)*0.000001;  try7 = 0; try8 = 0; try9 = 0; try10 = 0; try11 = 0; try12 = 0; try13 = 0; try14 = 0; }
	}
	if (try7 != 0){ if ((try8 - try7 * 10)> 5){ timestepdynamic = (try7 + 1)*0.0000001; try8 = 0; try9 = 0; try10 = 0; try11 = 0; try12 = 0; try13 = 0; try14 = 0; }
		else if((try8 - try7 * 10)< 5){ timestepdynamic = (try7)*0.0000001; try8 = 0; try9 = 0; try10 = 0; try11 = 0; try12 = 0; try13 = 0; try14 = 0; }
	}
	if (try8 != 0){ if ((try9 - try8 * 10)> 5){ timestepdynamic = (try8 + 1)*0.00000001; try9 = 0; try10 = 0; try11 = 0; try12 = 0; try13 = 0; try14 = 0; }
		else if((try9 - try8 * 10)< 5){ timestepdynamic = (try8)*0.00000001;  try9 = 0; try10 = 0; try11 = 0; try12 = 0; try13 = 0; try14 = 0; }
	}
	if (try9 != 0){ if ((try10 - try9 * 10)> 5){ timestepdynamic = (try9 + 1)* 0.000000001;  try10 = 0; try11 = 0; try12 = 0; try13 = 0; try14 = 0; }
		else if((try10 - try9 * 10)< 5){ timestepdynamic = (try9)*0.000000001; try10 = 0; try11 = 0; try12 = 0; try13 = 0; try14 = 0; }
	}
	if (try10 != 0){ if ((try11 - try10 * 10)> 5){ timestepdynamic = (try10 + 1)* 0.0000000001;  try11 = 0; try12 = 0; try13 = 0; try14 = 0; }
		else if((try11 - try10 * 10)< 5){ timestepdynamic = (try10)*0.0000000001;  try11 = 0; try12 = 0; try13 = 0; try14 = 0; }
	}
	if (try11 != 0){ if ((try12 - try11 * 10)> 5){ timestepdynamic = (try11 + 1)* 0.00000000001; try12 = 0; try13 = 0; try14 = 0; }
		else if((try12 - try11 * 10)< 5){ timestepdynamic = (try11)*0.00000000001; try12 = 0; try13 = 0; try14 = 0; }
	}
	if (try12 != 0){ if ((try13 - try12 * 10)> 5){ timestepdynamic = (try12 + 1)*0.000000000001; try13 = 0; try14 = 0; }
		else if((try13 - try12 * 10)< 5){ timestepdynamic = (try12)*0.000000000001;  try13 = 0; try14 = 0; }
	}
	if (try13 != 0){ if ((try14 - try13 * 10)> 5){ timestepdynamic = (try13 + 1)*0.0000000000001; try14 = 0; }
		else if((try14 - try13 * 10)< 5){ timestepdynamic = (try13)*0.0000000000001; try14 = 0; }
	}
	if (try14 != 0){ timestepdynamic = (try14)*0.00000000000001; }
	/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	stepdynamic =  max2dtotal / timestepdynamic;

	//cout << max2d << endl;

	////user output
	cout << "The timestep for the STATIC analysis are: " << timestepstatic << endl;
	cout << "The steps for the STATIC analysis are: " << stepstatic << endl;
	cout << "The timestep for the DYNAMIC analysis are: " << timestepdynamic << endl;
	cout << "The steps for the DYNAMIC analysis are: " << stepdynamic << endl;
	//write in setup file the steps
	
	string residul = "C:/Users/ece/Desktop/Co_SiM_withFEBiocoding/data/";
	ifstream step1(residul + "setup1.ini");
	ofstream st(residul + "setup.ini");
	string line1;	
	int except = 6;
	for (int i = 0; i < 200; i++)
	{
		getline(step1, line1);
		if (except >= 3){
			st << line1 << endl;
		}
		if (except < 3){
			++except;
		}      // static step
				if (line1.compare("TYPE1=solid") == 0)
				{
					st << "TIMESTEPS1=" << stepstatic << endl;
					st << "STEPSIZE1=" << timestepstatic << endl;
					except=1;
				}
				if (line1.compare("PRES_STIF1=0") == 0)
				{
					st << "DTMIN1=" << timestepstatic*0.1 << endl;
					st << "DTMAX1=" << timestepstatic << endl;
					except=1;
				}

				// dynamic step
				if (line1.compare("TYPE2=solid") == 0)
				{
					st << "TIMESTEPS2=" << stepdynamic << endl;
					st << "STEPSIZE2=" << timestepdynamic << endl;
					except=1;
				}
				if (line1.compare("PRES_STIF2=0") == 0)
				{
					st << "DTMIN2=" << timestepdynamic*0.1 << endl;
					st << "DTMAX2=" << timestepdynamic << endl;
					except=1;
				}
			
		}
	
	step1.close();
	st.close();
	//vision in set up file
	string answer1;
	cout << "Do you want to modified the setup file of FEBio time step control? (y/n) " << endl;
	cin >> answer1;
	string file = residul + "setup.ini";
	if (answer1 == "y"){
		HANDLE process = ShellExecute(file);
		if (process != NULL)
		{ // success 
			::WaitForSingleObject(process, INFINITE);
			::CloseHandle(process);
		} // success 
		else
		{
			try
			{
				throw "Could not run the following program: ";
			}
			catch (const char* msg)
			{
				cerr << msg << file << endl;
			}
		}
	}
	if (answer1 == "n"){
		cout << "The programm resume now without modification of FEBio's setup file ... " << endl;
	}
}


void TSC::stepswriter(){
	int timestep, timestep2;
	 double step, step2;
	 INIReader ini = INIReader(INI_FILE);

	cout << "1. Give the  timestep number you want for static step: " << endl;
	cin >> timestep;
	cout << "2. Give the step you want for static step: " << endl;
	cin >> step;
	cout << "3. Give the  timestep number you want for dynamic step: " << endl;
	cin >> timestep2;
	cout << "4. Give the step you want for dynamic step: " << endl;
	cin >> step2;

	char line[100];
	string residul = "C:/Users/ece/Desktop/Co_SiM_withFEBiocoding/data/";
	ifstream step1(residul + "setup1.ini");
	ofstream st(residul + "setup.ini");
	string line1;
	int except=6;
	for (int i = 0; i < 200; i++)
	{
		getline(step1, line1);
		if (except >= 3){
			st << line1 << endl;
		}
		if (except < 3){
			++except; //we write 2 new lines
		}
		if (step1.is_open()){
			// static step
			if (line1.compare("TYPE1=solid") == 0)
			{
				st << "TIMESTEPS1=" << timestep << endl;
				st << "STEPSIZE1=" << step << endl;
				except = 1;
				
			}
			if (line1.compare("PRES_STIF1=0") == 0)
			{
				st << "DTMIN1=" << step*0.1 << endl;
				st << "DTMAX1=" << step << endl;
				except = 1;
			}

			// dynamic step
			if (line1.compare("TYPE2=solid") == 0)
			{
				st << "TIMESTEPS2=" << timestep2 << endl;

				st << "STEPSIZE2=" << step2 << endl;
				except = 1;
			}
			if (line1.compare("PRES_STIF2=0") == 0)
			{
				st << "DTMIN2=" << step2*0.1 << endl;

				st << "DTMAX2=" << step2 << endl;
				except = 1;
			}
		}
	}
	step1.close();
	st.close();

	//vision in set up file
	string answer1;
	cout << "Do you want to modified the setup file of FEBio time step control? (y/n) " << endl;
	cin >> answer1;
	string file = residul+"setup.ini";
	if (answer1 == "y"){
		HANDLE process = ShellExecute(file);
		if (process != NULL)
		{ // success 
			::WaitForSingleObject(process, INFINITE);
			::CloseHandle(process);
		} // success 
		else
		{
			try
			{
				throw "Could not run the following program: ";
			}
			catch (const char* msg)
			{
				cerr << msg << file << endl;
			}
		}
	}
	if (answer1 == "n"){
		cout << "The programm resume now without modification of FEBio's setup file ... " << endl;
	}
}


HANDLE TSC::ShellExecute( string name)
{
	HANDLE hProcess = NULL;
	SHELLEXECUTEINFO shellInfo;
	::ZeroMemory(&shellInfo, sizeof(shellInfo));
	shellInfo.cbSize = sizeof(shellInfo);
	shellInfo.fMask = SEE_MASK_FLAG_NO_UI | SEE_MASK_NOCLOSEPROCESS;
	shellInfo.lpVerb ="edit";
	shellInfo.lpFile = name.c_str();
	shellInfo.nShow = 10;
	if (::ShellExecuteEx(&shellInfo))
	{ // success 
		hProcess = shellInfo.hProcess;
	} // success 
	return hProcess;
}