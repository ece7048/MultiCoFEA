#include "Num_Of_Partition.h"
#include "ReSampler.h"

 Num_Of_Partition::Num_Of_Partition(void){
}
 Num_Of_Partition::~Num_Of_Partition(void)
 {
 }

Vector Num_Of_Partition::start(int number){

	INIReader ini = INIReader(INI_FILE);
	ofstream valuenon;
	string resultDir = BASE_DIR + ini.Get("PATH", "RESULT_DIR", "");
	string resultDir2 = BASE_DIR + ini.Get("PATH", "RESULT_DIR2", "");
	double t0 = ini.GetReal("BODYFORCES", "START_TIME", 0);
	double tf = ini.GetReal("BODYFORCES", "END_TIME", 0);
	string stateFile = BASE_DIR + ini.Get("BODYFORCES", "STATE", "");
	// the first three are the rotation dof and the other three the translation
	string joint1 = ini.Get("BODYFORCES", "JOINT1", "");
	string joint2 = ini.Get("BODYFORCES", "JOINT2", "");
	string joint3 = ini.Get("BODYFORCES", "JOINT3", "");
	string joint4 = ini.Get("BODYFORCES", "JOINT4", "");
	string joint5 = ini.Get("BODYFORCES", "JOINT5", "");
	string joint6 = ini.Get("BODYFORCES", "JOINT6", "");
	//ofstream valueno;

	Array<double> tim;
	Array<double> j1;
	Array<double> j2;
	Array<double> j3;
	Array<double> j4;
	Array<double> j5;
	Array<double> j6;

	Storage  pos(stateFile);
	pos.getTimeColumn(tim);
	//system("pause");

	if (joint1 != "0"){ pos.getDataColumn(joint1, j1); }
	if (joint2 != "0"){ pos.getDataColumn(joint2, j2); }
	if (joint3 != "0"){ pos.getDataColumn(joint3, j3); //cout << "ibihjjjjjjjjjjjjj" << endl;
}
	if (joint4 != "0"){ pos.getDataColumn(joint4, j4); }
	if (joint5 != "0"){ pos.getDataColumn(joint5, j5); }
	if (joint6 != "0"){ pos.getDataColumn(joint6, j6); }

	/////////////////////found the max value of rotation and the max of translation//////////
	int endend = tim.size();
	double max1=0;
	
	double max2=0;
	
	double max3=0;
	
	double max4=0;
	
	double max5=0;
	
	double max6=0;
	
	for (int o = 0; o < endend; ++o){
		if (joint1 != "0"){
			if (max1 < abs(j1[o])){
				max1 = abs(j1[o]);
	
			}
		}
		if (joint2 != "0"){
			if (max2 < abs(j2[o])){
				max2 = abs(j2[o]);
				}
		}
		if (joint3 != "0"){
			if (max3 < abs(j3[o])){
				max3 = abs(j3[o]);

				}
		}
		if (joint4 != "0"){
			if (max4 < abs(j4[o])){
				max4 = abs(j4[o]);
				}
		}
		if (joint5 != "0"){
			if (max5 < abs(j5[o])){
				max5 = abs(j5[o]);
				}
		}
		if (joint6 != "0"){
			if (max6 < abs(j6[o])){
				max6 = abs(j6[o]);
			}
		}
	}
	if (max1 > max2 && max1>max3){ joint2 = "0"; joint3 = "0"; }
	if (max2 > max1 && max2>max3){ joint1 = "0"; joint3 = "0"; }
	if (max3 > max2 && max3 >max1){ joint2 = "0"; joint1 = "0"; //cout << "ibihjjjjjjjjjjjjj" << endl;
	}
	if (max4 > max6 && max4>max5){ joint6 = "0"; joint5 = "0"; }
	if (max5 > max4 && max5>max6){ joint4 = "0"; joint6 = "0"; }
	if (max6 > max5 && max6 >max4){ joint5 = "0"; joint4 = "0"; }

	Vector Tim(endend, 1);
	for (int o = 0; o < endend; ++o){
		Tim[o] = tim[o];
	}
	
	Vector jj1;
		Vector jj2;
		Vector jj3;
		Vector jj4;
		Vector jj5;
		Vector jj6;
		Vector jj1t;
		Vector jj2t;
		Vector jj3t;
		Vector jj4t;
		Vector jj5t;
		Vector jj6t;
		Vector J1(endend, 1);
		Vector J2(endend, 1);
		Vector J3(endend, 1);
		Vector J4(endend, 1);
		Vector J5(endend, 1);
		Vector J6(endend, 1);
	ReSampler det;
	if (joint1 != "0"){
		for (int o = 0; o < endend; ++o){
			J1[o] = j1[o];}
		 jj1 = det.detector(1, endend, Tim, J1);
		 jj1t = det.detector(0, endend, Tim, J1);
	}
	if (joint2 != "0"){
		for (int o = 0; o < endend; ++o){
			J2[o] = j2[o];
		}  jj2 = det.detector(1, endend, Tim, J2);
		jj2t = det.detector(0, endend, Tim, J2);
	}
	if (joint3 != "0"){
		for (int o = 0; o < endend; ++o){
			J3[o] = j3[o];
		}  jj3 = det.detector(1, endend, Tim, J3);
		jj3t = det.detector(0, endend, Tim, J3);
	}
	if (joint4 != "0"){
		for (int o = 0; o < endend; ++o){
			J4[o] = j4[o];
		}  jj4 = det.detector(1, endend, Tim, J4);
		jj4t = det.detector(0, endend, Tim, J4);
	}
	if (joint5 != "0"){
		for (int o = 0; o < endend; ++o){
			J5[o] = j5[o];
		}  jj5 = det.detector(1, endend, Tim, J5);
		jj5t = det.detector(0, endend, Tim, J5);
	}
	if (joint6 != "0"){
		for (int o = 0; o < endend; ++o){
			J6[o] = j6[o];
		}  jj6 = det.detector(1, endend, Tim, J6);
		jj6t = det.detector(0, endend, Tim, J6);
	}
	
	Vector start;
	Vector start2;
	Vector startt;
	Vector start2t;
	if (joint1 != "0"){
		start = jj1;
		startt = jj1t;
	}
	if (joint2 != "0"){
		start = jj2;
		startt = jj2t;
	}
	if (joint3 != "0"){
		start = jj3;
		startt = jj3t;
		//cout << "ibihjjjjjjjjjjjjj" << endl;
	}
	if (joint4 != "0"){
		start2 = jj4;
		start2t = jj4t;
	}
	if (joint5 != "0"){
		start2 = jj5;
		start2t = jj5t;
	}
	if (joint6 != "0"){
		start2 = jj6;
		start2t = jj6t;
	}
	
	int stdd=1;
	int stddd=2;
	if (joint6 == "0"&& joint5 == "0"&& joint4 == "0"){ stddd= 0; }
	if (joint1 == "0"&& joint2 == "0"&& joint3 == "0"){ stdd = 0; }
	int check=0;
	int check2=0;
	if (stddd == 0 && stdd != 0){check = start.size();}
	if (stddd != 0 && stdd == 0){check = start2.size(); }
	if (stddd != 0 && stdd != 0){ check = start2.size();  check2 = start.size(); }

	Vector startg(number + 1, 1);
	Vector start2g(number + 1, 1);
	Vector starttg(number + 1, 1);
	Vector start2tg(number + 1, 1);

	//////////////////////////CHECK THE TWO DOF R_T AFTER THE DETECTOR IF THEY ARE OK WITH THE NUMBER FO INTRVALS IF I NOT CHANGE THEM////////////////////////////////

	if (check > number + 1 || check2 > number + 1){

		Vector startg2(number + 1, 1);
		Vector start2g2(number + 1, 1);
		Vector starttg2(number + 1, 1);
		Vector start2tg2(number + 1, 1);

		//////////////////////first case//////////////////////////
		if (stddd == 0 && stdd != 0 || (stddd != 0 && stdd != 0)){
			//////////////////////find minimum values////////////////
			cout << "Result of rotation DOF detector: " << endl;
			cout << start << endl;
			cout << startt << endl;
			int ox = number + 1;
			Vector min1(ox);
			Vector tim1(ox);
			int stor1;
			for (int inter = 1; inter < number; ++inter){
				min1[inter] = abs(start[1]);
				tim1[inter] = startt[1];
				stor1=1;
				//cout << inter << endl;
				for (int o = 1; o < start.size() - 1; ++o){
					//cout <<o << endl;
					if (min1[inter] > abs(start[o])){
						min1[inter] = abs(start[o]);
						tim1[inter] = startt[o];
						stor1 = o;
						//cout << o << endl;
					}

				}
				startg2[inter] = start[stor1];
				starttg2[inter] = tim1[inter];
				start[stor1] = 100000000;
			}
			startg[0] = start[0];
			starttg[0] = startt[0];
			startg[number] = start[start.size() - 1];
			starttg[number] = startt[start.size() - 1];
			startg2[0] = start[0];
			starttg2[0] = startt[0];
			startg2[number] = start[start.size() - 1];
			starttg2[number] = startt[start.size() - 1];
			//cout << startg2 << endl;
			//cout << starttg2 << endl;
			/////////////////////in correct time order/////////////////
			int oxx = number ;
			Vector minx1(oxx);
			Vector timx1(oxx);
			int stor1d;
			for (int interx = 1; interx < number ; ++interx){
				minx1[interx] = abs(starttg2[1]);
				timx1[interx] = startg2[1];
				stor1d=1;

				for (int oc = 1; oc < oxx; ++oc){

					if (minx1[interx] > abs(starttg2[oc])){
						minx1[interx] = abs(starttg2[oc]);
						timx1[interx] = startg2[oc];
						stor1d = oc;
					}

				}
				starttg[interx] = minx1[interx];
				startg[interx] = timx1[interx];
				starttg2[stor1d] = 100000000;
			}
			cout << "Change result of rotation DOF detector: " << endl;
			cout << startg << endl;
			cout << starttg << endl;
		}
		//////////////////////////second case///////////////////////////////
		if (stddd != 0 && stdd == 0 || stddd != 0 && stdd != 0){
			/////////////////////find minimum values////////////////
			cout << "Result of translation DOF detector: " << endl;
			cout << start2 << endl;
			cout << start2t << endl;
			int ox = number + 1;
			Vector min2(ox);
			Vector tim2(ox);
			int stor2;
			for (int inter = 1; inter < number; ++inter){
				min2[inter] = abs(start2[1]);
				tim2[inter] = start2t[1];
				 stor2=1;

				for (int o = 1; o < start2.size() - 1; ++o){

					if (min2[inter] > abs(start2[o])){
						min2[inter] = abs(start2[o]);
						tim2[inter] = start2t[o];
						stor2 = o;
					}

				}
				start2g2[inter] = start2[stor2];
				start2tg2[inter] = tim2[inter];
				start2[stor2] = 100000000;
			}
			start2g[0] = start2[0];
			start2tg[0] = start2t[0];
			start2g[number] = start2[start2.size() - 1];
			start2tg[number] = start2t[start2.size() - 1];
			start2g2[0] = start2[0];
			start2tg2[0] = start2t[0];
			start2g2[number] = start2[start2.size() - 1];
			start2tg2[number] = start2t[start2.size() - 1];
			/////////////////////in correct time order/////////////////
			int oxxw = number ;
			Vector minx2(oxxw);
			Vector timx2(oxxw);
			int stor2s;
			for (int interxw = 1; interxw < number; ++interxw){
				minx2[interxw] = abs(start2tg2[1]);
				timx2[interxw] = start2g2[1];
				 stor2s=1;

				for (int ocw = 1; ocw < oxxw; ++ocw){

					if (minx2[interxw] > abs(start2tg2[ocw])){
						minx2[interxw] = abs(start2tg2[ocw]);
						timx2[interxw] = start2g2[ocw];
						stor2s = ocw;
					}

				}
				start2tg[interxw] = minx2[interxw];
				start2g[interxw] = timx2[interxw];
				start2tg2[stor2s] = 100000000;
			}
			cout << "Change result of translation DOF detector: " << endl;
			cout << start2g << endl;
			cout << start2tg << endl;

		}
	}
	if (check <= number+1 && check2 <= number+1){
		cout << "For this simulation, its seems that will be more safe to use " << check-1 << " number of partition" << endl;
		if (stdd != 0){
			//for (int o = 0; o < start.size(); ++o){
				startg = start;
				starttg = startt;
			//}
		}
		if (stddd != 0){
			//for (int oe = 0; oe < start2.size(); ++oe){
				start2g = start2;
				start2tg = start2t;
			//}
		}
	}
////////////////////////////output of results//////////////////////////////////////////////////////////////////

	cout << "The intervals of the  DoFs based analysis are:" << endl;
	if ( stdd != 0){
		cout << "rotation values: " << startg << endl;
		cout << "rotation time: " << starttg << endl;
	}
	if (stddd != 0 ){
		cout << "translation values: " << start2g << endl;
		cout << "translation time: " << start2tg << endl;
	}
	if (stddd != 0 && stdd != 0){
		cout << "Do you want the sampling became based the rotation or the translation DoFs of the joint? (r/t)" << endl;
		char kindd;
		if (kindd == 'r'){ return starttg; }
		if (kindd == 't'){ return start2tg; }
	}
	if (stddd == 0 && stdd != 0){
		 return starttg; }
	if (stddd != 0 && stdd == 0){
		return start2tg;
	}
}

void Num_Of_Partition::analysiswriter(double start, double end,char mode){
			

			//char line[100];
			string residul = "C:/Users/ece/Desktop/Co_SiM_withFEBiocoding/data/";
			ifstream step1(residul + "setup1.ini");
			ofstream st(residul + "setup.ini");
			
			string line1;
			string line2;
			int except = 6;
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
					if (line1.compare("POINT0=0") == 0)
					{
						st << "START_TIME =" << start << endl;
						st << "END_TIME =" << end << endl;
						except = 1;

					}
					
				}
				if (step1.is_open()){
					// static step
					if (line1.compare("POINT2=0") == 0)
					{
						if (mode == 'F' || mode == 'R'){
							st << "ANALYSIS2 = dynamic" << endl;
							

						}
						if (mode == 'T'){
							st << "ANALYSIS2 = static" << endl;
							

						}
						except = 2;//we write 1 new lines

					}

				}
			}
			step1.close();
			st.close();

			ifstream step2(residul + "setup.ini");
			ofstream st2(residul + "setup1.ini");

			for (int i = 0; i < 200; i++)
			{
				getline(step2, line2);
					st2 << line2 << endl;
				
			}
			step2.close();
			st2.close();
			//vision in set up file
			string answer1;
			cout << "Do you want to modified or check the setup file of interval time? (y/n) " << endl;
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
				cout << ".... " << endl;
			}
		}

HANDLE Num_Of_Partition::ShellExecute(string name)
{
	HANDLE hProcess = NULL;
	SHELLEXECUTEINFO shellInfo;
	::ZeroMemory(&shellInfo, sizeof(shellInfo));
	shellInfo.cbSize = sizeof(shellInfo);
	shellInfo.fMask = SEE_MASK_FLAG_NO_UI | SEE_MASK_NOCLOSEPROCESS;
	shellInfo.lpVerb = "edit";
	shellInfo.lpFile = name.c_str();
	shellInfo.nShow = 10;
	if (::ShellExecuteEx(&shellInfo))
	{ // success 
		hProcess = shellInfo.hProcess;
	} // success 
	return hProcess;
}