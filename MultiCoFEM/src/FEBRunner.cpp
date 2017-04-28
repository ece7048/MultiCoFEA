
#include "FEBRunner.h"
#include <string>
#include <stdio.h>

FEBRunner::FEBRunner(void)
{
}

FEBRunner::~FEBRunner(void)
{
}


void FEBRunner::Run(int num)
{
	INIReader ini = INIReader(INI_FILE);

	string modelPath = BASE_DIR + ini.Get("PATH", "MODELF", "");
	//string resultDir = BASE_DIR + ini.Get("PATH", "RESULT_DIR", "");
	string programname = ini.Get("FEBIOSTEP", "PROGRAMMNAME", "");
	string name= ini.Get("FEBIOSTEP", "NAME", "");
	int numThreads = ini.GetReal("FEBIOSTEP", "NUMTHREADS", 2);
	string arg1 = " -i ";
	//char numbereach = in + '0';
	//string arg2 = modelPath + numbereach;
	int count = 0;
//#pragma omp parallel num_threads(numThreads)
	//{
//#pragma omp for schedule(dynamic)
		for (int n = 0; n < num; ++n) // looping though all sample points in spVec
		{
			char numbereach = n + '0';
			string arg2 = modelPath + numbereach;
			//int ID = omp_get_thread_num();

			cout << "" << endl;
			{   cout << "START THE FEBIO OUTPUT FILES WRTTING " << endl;

			string workingFileName = arg2;
			string workingFileNameLog = workingFileName + ".log";
			string workingFileNameFeb = workingFileName + ".feb";
			string workingFileNameXplt = workingFileName + ".xplt";
			string arguments = arg1 + workingFileName + ".feb";

			HANDLE process = ShellExecuteHandler(programname, arguments, name);

			//HINSTANCE ShellExecute(handle, "open", programname, arguments, NULL, SW_SHOW);




			if (process != NULL)
			{ // success 
				//cout << "ok" << endl;
				//::WaitForSingleObject(process, INFINITE);
				//::CloseHandle(process);
				//cout << "ok" << endl;
			} // success 
			else
			{
				try
				{
					throw "Could not run the following program: ";
				}
				catch (const char* msg)
				{
					cerr << msg << programname << endl;
				}
			}
			}//void
		}
	}
//}

void FEBRunner::CopyFEBFile(int number){
	INIReader ini = INIReader(INI_FILE);
	string modelPath = BASE_DIR + ini.Get("PATH", "MODELFeb", "");
	string modd = BASE_DIR + ini.Get("PATH", "MODELFeb", "");
	modd.erase(modd.end() - 4, modd.end());
	string mod = modd;
	//string model = BASE_DIR + ini.Get("PATH", "MODELF", "");
	for (int countt = 0; countt < number; ++countt){
		char numbereach = countt + '0';


		//cout << mod << endl;
		string modf = mod + numbereach + ".feb";
		ofstream febnew(modf);
		ifstream febFile(modelPath);


		// add the files of lc and steps
		string line;
		int j = 0;

		for (int i = 0; i < 1000000; i++)
		{
			if (febFile.is_open()){
				//PASS Load data/////////

				getline(febFile, line);
				febnew << line << endl;

			}

		}
	}
		//char lined[100];
		string residul = "C:/Users/ece/Desktop/Co_SiM_withFEBiocoding/data/";
		ifstream step1(residul + "setup1.ini");
		ofstream st(residul + "setup.ini");
		string line1;
		int except = 600;
		for (int i = 0; i < 200; i++)
		{
			getline(step1, line1);
			//if (except >= number){
			st << line1 << endl;
			//}
			//if (except < number){
			//++except; //we write 2 new lines
			//}
			if (step1.is_open()){
				// static step
				if (line1.compare("[PATH]") == 0)
				{
					for (int numbereach = 0; numbereach < number; ++numbereach){
						
						st << "MODELF" << numbereach << '=' << mod << numbereach << endl;
						

						//++except;

					}

				}
			}
		}
		step1.close();
		st.close();
		ifstream ste(residul + "setup.ini");
		ofstream s(residul + "setup1.ini");
		string line12;
		
		for (int i = 0; i < 200; i++)
		{
			getline(ste, line12);
			//if (except >= number){
			s << line12 << endl;
		}

}

void FEBRunner::clearsetup(){
	INIReader ini = INIReader(INI_FILE);
	//string residul = "C:/Users/ece/Desktop/Co_SiM_withFEBiocoding/data/";
	string residul = BASE_DIR;
	ifstream step1(residul + "setup2.ini");
	ofstream st(residul + "setup.ini");
	ofstream st2(residul + "setup1.ini");
	string line1;
	
	for (int i = 0; i < 200; i++)
	{
		getline(step1, line1);
		//if (except >= number){
		st << line1 << endl;
		st2 << line1 << endl;
	}
	
		remove("C:/Users/ece/Desktop/Co_SiM_withFEBiocoding/data/setup2.ini");
		remove("C:/Users/ece/Desktop/Co_SiM_withFEBiocoding/data/setup1.ini");
		remove("C:/Users/ece/Desktop/Co_SiM_withFEBiocoding/data/setup3.ini");
	
}



void FEBRunner::WriteFEBFile(int itteration, char mode)
{
	cout << " " << endl;
	cout << " " << endl;
	cout << "START THE FEBIO XML WRTTING " << endl;
	cout << " " << endl;
	INIReader ini = INIReader(INI_FILE);
	string resultDir1 = BASE_DIR + ini.Get("PATH", "RESULT_DIR2", "");
	char itter = itteration + '0';
	string newfolder = resultDir1 + itter;
	string resultDir2 = newfolder;

	string bn1 = ini.Get("BODYFORCES", "BDNAME1", "");
	string bn2 = ini.Get("BODYFORCES", "BDNAME2", "");


	/////////////////////////////add all the degrees files in one///////////
	//cout << "xaxaxaxax" << endl;
	ofstream dof(resultDir2 + "/FEBdatapositionsdynamic" + itter + ".txt");

	ifstream dof1(resultDir2 + "/dof1.txt");
	string line1;
	while (!dof1.eof()){

		getline(dof1, line1);
		//if (except >= number){
		dof << line1 << endl;
	}
	ifstream dof2(resultDir2 + "/dof2.txt");
	string line2;
	while (!dof2.eof()){

		getline(dof2, line2);
		//if (except >= number){
		dof << line2 << endl;
	}
	ifstream dof3(resultDir2 + "/dof3.txt");
	string line3;
	while (!dof3.eof()){

		getline(dof3, line3);
		//if (except >= number){
		dof << line3 << endl;
	}
	ifstream dof4(resultDir2 + "/dof4.txt");
	string line4;
	while (!dof4.eof()){

		getline(dof4, line4);
		//if (except >= number){
		dof << line4 << endl;
	}
	ifstream dof5(resultDir2 + "/dof5.txt");
	string line5;
	while (!dof5.eof()){

		getline(dof5, line5);
		//if (except >= number){
		dof << line5 << endl;
	}
	ifstream dof6(resultDir2 + "/dof6.txt");
	string line6;
	while (!dof6.eof()){

		getline(dof6, line6);
		//if (except >= number){
		dof << line6 << endl;
	}
	ifstream dof7(resultDir2 + "/dof7.txt");
	string line7;
	while (!dof7.eof()){

		getline(dof7, line7);
		//if (except >= number){
		dof << line7 << endl;
	}
	ifstream dof8(resultDir2 + "/dof8.txt");
	string line8;
	while (!dof8.eof()){

		getline(dof8, line8);
		//if (except >= number){
		dof << line8 << endl;
	}
	ifstream dof9(resultDir2 + "/dof9.txt");
	string line9;
	while (!dof9.eof()){

		getline(dof9, line9);
		//if (except >= number){
		dof << line9 << endl;
	}
	ifstream dof10(resultDir2 + "/dof10.txt");
	string line10;
	while (!dof10.eof()){

		getline(dof10, line10);
		//if (except >= number){
		dof << line10 << endl;
	}
	ifstream dof11(resultDir2 + "/dof11.txt");
	string line11;
	while (!dof11.eof()){

		getline(dof11, line11);
		//if (except >= number){
		dof << line11 << endl;
	}
	ifstream dof12(resultDir2 + "/dof12.txt");
	string line12;
	while (!dof12.eof()){

		getline(dof12, line12);
		//if (except >= number){
		dof << line12 << endl;
	}

	/////////////////////////////////////////////////////////////////////////////////////////////
	ifstream stepdynamicpositions(resultDir2 + "/FEBstepdynamic_position" + itter + ".txt");

	ifstream stepstaticpositions(resultDir2 + "/FEBstepstatic_position" + itter + ".txt");
	ifstream lcdynamicpositions(resultDir2 + "/FEBdatapositionsdynamic" + itter + ".txt");
	ifstream lcstaticpositions(resultDir2 + "/FEBdatapositionsstatic" + itter + ".txt");

	ifstream stepdynamicforces(resultDir2 + "/FEBstepdynamic_forces" + itter + ".txt");
	ifstream stepstaticforces(resultDir2 + "/FEBstepstatic_forces" + itter + ".txt");
	ifstream lcdynamicforces(resultDir2 + "/FEBdataforcesdynamic" + itter + ".txt");
	ifstream lcstaticforces(resultDir2 + "/FEBdataforcestatic" + itter + ".txt");


	string modelPath = BASE_DIR + ini.Get("PATH", "MODELFeb", "");
	string modd = BASE_DIR + ini.Get("PATH", "MODELFeb", "");
	modd.erase(modd.end() - 4, modd.end());
	string mod = modd;
	string modelPathcopy = mod;
	//string resultDir = BASE_DIR + ini.Get("PATH", "RESULT_DIR", "");
	int numThreads = ini.GetReal("FEBIOSTEP", "NUMTHREADS", 1);
	///STEP ONE///
	string type1 = ini.Get("FEBIOSTEP", "TYPE1", "");
	string ts1 = ini.Get("FEBIOSTEP", "TIMESTEPS1", "");
	string ss1 = ini.Get("FEBIOSTEP", "STEPSIZE1", "");
	string dt1 = ini.Get("FEBIOSTEP", "DTOL1", "");
	string et1 = ini.Get("FEBIOSTEP", "ETOL1", "");
	string rt1 = ini.Get("FEBIOSTEP", "RTOL1", "");
	string lst1 = ini.Get("FEBIOSTEP", "LSTOL1", "");
	string ps1 = ini.Get("FEBIOSTEP", "PRES_STIF1", "");
	string dtm1 = ini.Get("FEBIOSTEP", "DTMIN1", "");
	string dtma1 = ini.Get("FEBIOSTEP", "DTMAX1", "");
	string maxre1 = ini.Get("FEBIOSTEP", "MAX_RET1", "");
	string oi1 = ini.Get("FEBIOSTEP", "OPT_ITER1", "");
	string a1 = ini.Get("FEBIOSTEP", "AGGRES1", "");
	string mr1 = ini.Get("FEBIOSTEP", "MAX_REF1", "");
	string mu1 = ini.Get("FEBIOSTEP", "MAX_UPS1", "");
	string ob1 = ini.Get("FEBIOSTEP", "OPT_BW1", "");
	string re1 = ini.Get("FEBIOSTEP", "RESTART1", "");
	string pl1 = ini.Get("FEBIOSTEP", "PLOT_LEVEL1", "");
	string cm1 = ini.Get("FEBIOSTEP", "CMAX1", "");
	string analysis1 = ini.Get("FEBIOSTEP", "ANALYSIS1", "");
	string prl1 = ini.Get("FEBIOSTEP", "PRINT_LEVEL1", "");
	string mres1 = ini.Get("FEBIOSTEP", "MINRESIDUAL1", "");
	string out1 = ini.Get("FEBIOSTEP", "OUT1", "");
	//STEP TWO
	string type2 = ini.Get("FEBIOSTEP", "TYPE2", "");
	string ts2 = ini.Get("FEBIOSTEP", "TIMESTEPS2", "");
	string ss2 = ini.Get("FEBIOSTEP", "STEPSIZE2", "");
	string dt2 = ini.Get("FEBIOSTEP", "DTOL2", "");
	string et2 = ini.Get("FEBIOSTEP", "ETOL2", "");
	string rt2 = ini.Get("FEBIOSTEP", "RTOL2", "");
	string lst2 = ini.Get("FEBIOSTEP", "LSTOL2", "");
	string ps2 = ini.Get("FEBIOSTEP", "PRES_STIF2", "");
	string dtm2 = ini.Get("FEBIOSTEP", "DTMIN2", "");
	string dtma2 = ini.Get("FEBIOSTEP", "DTMAX2", "");
	string maxre2 = ini.Get("FEBIOSTEP", "MAX_RET2", "");
	string oi2 = ini.Get("FEBIOSTEP", "OPT_ITER2", "");
	string a2 = ini.Get("FEBIOSTEP", "AGGRES2", "");
	string mr2 = ini.Get("FEBIOSTEP", "MAX_REF2", "");
	string mu2 = ini.Get("FEBIOSTEP", "MAX_UPS2", "");
	string ob2 = ini.Get("FEBIOSTEP", "OPT_BW2", "");
	string re2 = ini.Get("FEBIOSTEP", "RESTART2", "");
	string pl2 = ini.Get("FEBIOSTEP", "PLOT_LEVEL2", "");
	string cm2 = ini.Get("FEBIOSTEP", "CMAX2", "");
	string analysis2 = ini.Get("FEBIOSTEP", "ANALYSIS2", "");
	string prl2 = ini.Get("FEBIOSTEP", "PRINT_LEVEL2", "");
	string mres2 = ini.Get("FEBIOSTEP", "MINRESIDUAL2", "");
	string out2 = ini.Get("FEBIOSTEP", "OUT2", "");
	//newmark parameters////
	double alpha1 = ini.GetReal("FEBIOSTEP", "alpha1", 1);
	double beta1 = ini.GetReal("FEBIOSTEP", "beta1", 0.25);
	double gamma1 = ini.GetReal("FEBIOSTEP", "gamma1", 0.5);

	double alpha2 = ini.GetReal("FEBIOSTEP", "alpha2", 1);
	double beta2 = ini.GetReal("FEBIOSTEP", "beta2", 0.25);
	double gamma2 = ini.GetReal("FEBIOSTEP", "gamma2", 0.5);

	// build the step one //////////
	ofstream stepone(resultDir2 + "/stepone" + itter + ".txt", ofstream::out);
	ofstream steptwo(resultDir2 + "/steptwo" + itter + ".txt", ofstream::out);
	ofstream output(resultDir2 + "/output" + itter + ".txt", ofstream::out);

	/////////////start building//////////////////////////

	stepone << "<Step name = " << '"' << "Step01" << '"' << ">" << endl;
	stepone << " <Module type = " << '"' << type1 << '"' << "/>" << endl;
	stepone << " <Control>" << endl;
	stepone << " <title>tf_joint</title>" << endl;
	stepone << " <time_steps>" << ts1 << "</time_steps>" << endl;
	stepone << " <step_size>" << ss1 << "</step_size>" << endl;
	if (dt1 != "NAN"){
		stepone << " <dtol>" << dt1 << "</dtol>" << endl;
	}
	if (et1 != "NAN"){
		stepone << " <etol>" << et1 << "</etol>" << endl;
	}
	if (rt1 != "NAN"){
		stepone << " <rtol>" << rt1 << "</rtol>" << endl;
	}
	if (lst1 != "NAN"){
		stepone << " <lstol>" << lst1 << "</lstol>" << endl;
	}
	if (ps1 != "NAN"){
		stepone << " <pressure_stiffness>" << ps1 << "</pressure_stiffness>" << endl;
	}

	stepone << " <time_stepper>" << endl;

	if (dtm1 != "NAN"){
		stepone << "  <dtmin>" << dtm1 << "</dtmin>" << endl;
	}
	if (dtma1 != "NAN"){
		stepone << "  <dtmax>" << dtma1 << "</dtmax>" << endl;
	}
	if (maxre1 != "NAN"){
		stepone << "  <max_retries>" << maxre1 << "</max_retries>" << endl;
	}
	if (oi1 != "NAN"){
		stepone << "  <opt_iter>" << oi1 << "</opt_iter>" << endl;
	}
	if (a1 != "NAN"){
		stepone << "  <aggressiveness>" << a1 << "</aggressiveness>" << endl;
	}
	stepone << " </time_stepper>" << endl;
	if (mr1 != "NAN"){
		stepone << " <max_refs>" << mr1 << "</max_refs>" << endl;
	}
	if (mu1 != "NAN"){
		stepone << " <max_ups>" << mu1 << "</max_ups>" << endl;
	}
	if (ob1 != "NAN"){
		stepone << " <optimize_bw>" << ob1 << "</optimize_bw>" << endl;
	}
	if (re1 != "NAN"){
		stepone << " <restart>" << re1 << "</restart>" << endl;
	}
	if (pl1 != "NAN"){
		stepone << " <plot_level>" << pl1 << "</plot_level>" << endl;
	}
	if (cm1 != "NAN"){
		stepone << " <cmax>" << cm1 << "</cmax>" << endl;
	}

	stepone << " <analysis type =" << '"' << analysis1 << '"' << "/>" << endl;
	if (alpha1 != 0){
		stepone << " <alpha>" << alpha1 << "</alpha>" << endl;

	}
	if (beta1 != 0){
		stepone << " <beta>" << beta1 << "</beta>" << endl;

	}
	if (gamma1 != 0){
		stepone << " <gamma>" << gamma1 << "</gamma>" << endl;

	}

	if (prl1 != "NAN"){
		stepone << " <print_level>" << prl1 << "</print_level>" << endl;
	}
	if (mres1 != "NAN"){
		stepone << " <min_residual>" << mres1 << "</min_residual>" << endl;
	}
	if (out1 != "NAN"){
		stepone << "<output_level>" << out1 << "</output_level> " << endl;
	}

	stepone << " </Control>" << endl;


	steptwo << "<Step name = " << '"' << "Step02" << '"' << ">" << endl;
	steptwo << " <Module type = " << '"' << type2 << '"' << "/>" << endl;
	steptwo << " <Control>" << endl;
	steptwo << " <title>tf_joint</title>" << endl;
	steptwo << " <time_steps>" << ts2 << "</time_steps>" << endl;
	steptwo << " <step_size>" << ss2 << "</step_size>" << endl;
	if (dt2 != "NAN"){
		steptwo << " <dtol>" << dt2 << "</dtol>" << endl;
	}
	if (et2 != "NAN"){
		steptwo << " <etol>" << et2 << "</etol>" << endl;
	}
	if (rt2 != "NAN"){
		steptwo << " <rtol>" << rt2 << "</rtol>" << endl;
	}
	if (lst2 != "NAN"){
		steptwo << " <lstol>" << lst2 << "</lstol>" << endl;
	}
	if (ps2 != "NAN"){
		steptwo << " <pressure_stiffness>" << ps2 << "</pressure_stiffness>" << endl;
	}

	steptwo << " <time_stepper>" << endl;

	if (dtm2 != "NAN"){
		steptwo << "  <dtmin>" << dtm2 << "</dtmin>" << endl;
	}
	if (dtma2 != "NAN"){
		steptwo << "  <dtmax>" << dtma2 << "</dtmax>" << endl;
	}
	if (maxre2 != "NAN"){
		steptwo << "  <max_retries>" << maxre2 << "</max_retries>" << endl;
	}
	if (oi2 != "NAN"){
		steptwo << "  <opt_iter>" << oi2 << "</opt_iter>" << endl;
	}
	if (a2 != "NAN"){
		steptwo << "  <aggressiveness>" << a2 << "</aggressiveness>" << endl;
	}
	steptwo << " </time_stepper>" << endl;
	if (mr2 != "NAN"){
		steptwo << " <max_refs>" << mr2 << "</max_refs>" << endl;
	}
	if (mu2 != "NAN"){
		steptwo << " <max_ups>" << mu2 << "</max_ups>" << endl;
	}
	if (ob2 != "NAN"){
		steptwo << " <optimize_bw>" << ob2 << "</optimize_bw>" << endl;
	}
	if (re2 != "NAN"){
		steptwo << " <restart>" << re2 << "</restart>" << endl;
	}
	if (pl2 != "NAN"){
		steptwo << " <plot_level>" << pl2 << "</plot_level>" << endl;
	}
	if (cm2 != "NAN"){
		steptwo << " <cmax>" << cm2 << "</cmax>" << endl;
	}

	steptwo << " <analysis type =" << '"' << analysis2 << '"' << "/>" << endl;
	if (alpha2 != 0){
		steptwo << " <alpha>" << alpha2 << "</alpha>" << endl;

	}
	if (beta2 != 0){
		steptwo << " <beta>" << beta2 << "</beta>" << endl;

	}
	if (gamma2 != 0){
		steptwo << " <gamma>" << gamma2 << "</gamma>" << endl;

	}

	if (prl2 != "NAN"){
		steptwo << " <print_level>" << prl2 << "</print_level>" << endl;
	}
	if (mres2 != "NAN"){
		steptwo << " <min_residual>" << mres2 << "</min_residual>" << endl;
	}
	if (out2 != "NAN"){
		steptwo << "<output_level>" << out2 << "</output_level> " << endl;
	}
	steptwo << " </Control>" << endl;


	/////////////////////////Output section////////////////////////////////////////////////////////////

	output << "<Output>" << endl;
	output << "	<logfile>" << endl;
	output << "	 <rigid_body_data data =" << '"' << "Fx;Fy;Fz;Mx;My;Mz" << '"' << " name =" << '"' << bn1 << "load" << '"' << "file =" << '"' << bn1 << "_kinetics" << itteration << ".txt" << '"' << ">1</rigid_body_data>" << endl;
	output << "	 <rigid_body_data data =" << '"' << "Fx;Fy;Fz;Mx;My;Mz" << '"' << " name =" << '"' << bn2 << "load" << '"' << "file =" << '"' << bn2 << "_kinetics" << itteration << ".txt" << '"' << ">2</rigid_body_data>" << endl;
	output << "	 <rigid_body_data data =" << '"' << "x;y;z;thx;thy;thz" << '"' << " name =" << '"' << bn1 << "motion" << '"' << "file =" << '"' << bn1 << "_kinematics" << itteration << ".txt" << '"' << ">1</rigid_body_data>" << endl;
	output << "	 <rigid_body_data data =" << '"' << "x;y;z;thx;thy;thz" << '"' << " name =" << '"' << bn2 << "motion" << '"' << "file =" << '"' << bn2 << "_kinematics" << itteration << ".txt" << '"' << ">2</rigid_body_data>" << endl;
	output << "	</logfile>" << endl;
	//output << "	<plotfile type = "<<'"'<<"febio"<<'"'<<"/>" << endl;
	output << "	<plotfile type = " << '"' << "febio" << '"' << ">" << endl;
	output << "<var type = " << '"' << "strain energy density" << '"' << "/>" << endl;
	output << "<var type = " << '"' << "stress" << '"' << "/>" << endl;
	output << "<var type = " << '"' << "contact force" << '"' << "/>" << endl;
	output << "<var type = " << '"' << "contact pressure" << '"' << "/>" << endl;
	output << "<var type = " << '"' << "reaction forces" << '"' << "/>" << endl;

	//output << "<var type = " << '"' << "rigid torque" << '"' << "/>" << endl;
	//output << "<var type = " << '"' << "rigid position" << '"' << "/>" << endl;
	output << "<var type = " << '"' << "rigid velocity" << '"' << "/>" << endl;
	output << "<var type = " << '"' << "rigid acceleration" << '"' << "/>" << endl;
	//output << "<var type = " << '"' << "rigid angular position" << '"' << "/>" << endl;
	output << "<var type = " << '"' << "rigid angular velocity" << '"' << "/>" << endl;
	output << "<var type = " << '"' << "rigid angular acceleration" << '"' << "/>" << endl;
	output << "<var type = " << '"' << "displacement" << '"' << "/>" << endl;


	output << "</plotfile>" << endl;
	output << "</Output>" << endl;

	steptwo.close();
	stepone.close();
	output.close();
	//////////////////////////////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////////////////////////////////
	/////////////CASE of prescribed motion rotation///////////////

	string presc1f = ini.Get("BASICSETUP", "Prescribed_DOF_1", "");
	string presc2f = ini.Get("BASICSETUP", "Prescribed_DOF_2", "");
	string jointfree = ini.Get("BASICSETUP", "Joint_free", "");
	if (jointfree == "y"){
		if (presc1f[1] == '_' || presc1f[2] == '_' || presc1f[3] == '_' || presc1f[0] == 'r' || presc1f[0] == 'N'){

			jointwrite(itteration, 1);
		}
	}
	if(jointfree == "y"){
	if (presc2f[1] == '_' || presc2f[2] == '_' || presc2f[3] == '_' || presc2f[0] == 'r' || presc2f[0] == 'N'){

		jointwrite(itteration, 2);
	}

}


///////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
	
	ifstream steptwo1(resultDir2 + "/steptwo" + itter + ".txt");
	ifstream stepone1(resultDir2 + "/stepone" + itter + ".txt");
	ifstream outputfir(resultDir2 + "/output" + itter + ".txt");
	ifstream velocity(resultDir2 + "/initialvel" + itter + ".txt");
	ifstream jointf1(resultDir2 + "/joints1" + itter + ".txt");
	ifstream jointfo1(resultDir2 + "/jointd1" + itter + ".txt");
	ifstream jointf2(resultDir2 + "/joints2" + itter + ".txt");
	ifstream jointfo2(resultDir2 + "/jointd2" + itter + ".txt");

	//create a file for contact surfaces data from user
	/*
	ifstream febFile0(modelPath);
	ofstream contact(resultDir2 + "/contactsurfacesdata" + itter + ".txt");
	string lines;
	
	for (int i = 0; i < 1000000; i++)
	{
		if (febFile0.is_open()){
			//PASS Load data/////////

			getline(febFile0, lines);


			if (lines.compare("	<Contact>") == 0)
			{
				contact << lines << endl;
				cout << "START COPING THE CONTACT DATA FROM USER" << endl;

				while (lines.compare("	</Contact>") != 0){
					getline(febFile0, lines);
					contact << lines << endl;
				}
			}
		}
	}
	//
	contact.close();
	ifstream contact1(resultDir2 + "/contactsurfacesdata" + itter + ".txt");
	ifstream contact2(resultDir2 + "/contactsurfacesdata" + itter + ".txt");
	*/
	///////////////////////////////////////////////////////////////

	//read feb file line by line
	//char numbe = itteration + '0';
	string modelPathcopyv = modelPathcopy + itter + ".feb";
	ofstream feb(modelPathcopyv);
	ifstream febFile(modelPath);


	// add the files of lc and steps
	string line;
	int j = 0;

	for (int i = 0; i < 1000000;i++)
	{
		if (febFile.is_open()){
		//PASS Load data/////////
			
		getline(febFile, line);
		feb << line << endl;

		if (line.compare("	</Discrete>") == 0)
		{
			i = 10000000;
			cout << "START WRITING THE TWO STEPS" << endl;

			char line1[100];

			if (lcstaticforces.is_open()){
				while (!lcstaticforces.eof()){
					lcstaticforces.getline(line1, 100, '\n');
                
					feb << line1 << endl;
					//i++;
						//cout << i << endl;

				}
				cout <<" 0%" << endl;
				lcstaticforces.close();
			}
			else{
				cout << "a.txt couldn't be opened. Creat and write something in a.txt, and try again." << endl;
			}

			char line2[100];
			if (lcdynamicforces.is_open()){
				while (!lcdynamicforces.eof()){
					lcdynamicforces.getline(line2, 100, '\n');
					feb << line2 << endl;
					//i++;
					//cout << i << endl;
				}	cout << " 10%" << endl;
				lcdynamicforces.close();
			}
			else{
				cout << "a.txt couldn't be opened. Creat and write something in a.txt, and try again." << endl;


			}
			char line3[100];
			if (lcstaticpositions.is_open()){
				while (!lcstaticpositions.eof()){
					lcstaticpositions.getline(line3, 100, '\n');
					feb << line3 << endl;
					//i++;
					//cout << i << endl;
				}cout << " 20%" << endl;
				lcstaticpositions.close();
			}
			else{
				cout << "a.txt couldn't be opened. Creat and write something in a.txt, and try again." << endl;


			}
			char line4[100];
			if (lcdynamicpositions.is_open()){
				while (!lcdynamicpositions.eof()){
					lcdynamicpositions.getline(line4, 100, '\n');
					feb << line4 << endl;
					//i++;
					//cout << i << endl;
				}cout << " 30%" << endl;
				lcdynamicpositions.close();
			}
			else{
				cout << "a.txt couldn't be opened. Creat and write something in a.txt, and try again." << endl;

			
		}
		char line55[1000];
		if (outputfir.is_open()){
			while (!outputfir.eof()){
				outputfir.getline(line55, 1000, '\n');
				feb << line55 << endl;
				//i++;
				//cout << i << endl;
			}cout << " 40%" << endl;
			outputfir.close();
		}
		else{
			cout << "a.txt couldn't be opened. Creat and write something in a.txt, and try again." << endl;

		}

			
			////////Pass steps////////////



			char line5[100];
			if (stepone1.is_open()){
				while (!stepone1.eof()){
					stepone1.getline(line5, 100, '\n');
					feb << line5 << endl;
					//i++;
					//cout << i << endl;
				}cout << " 50%" << endl;
				stepone1.close();
			}
			else{
				cout << "a.txt couldn't be opened. Creat and write something in a.txt, and try again." << endl;

			}
			/*
			
			char line61[100];
			if (contact1.is_open()){
				while (!contact1.eof()){
					contact1.getline(line61, 100, '\n');
					feb << line61 << endl;
					//i++;
					//cout << i << endl;
				}cout << " 55%" << endl;
				contact1.close();
			}
			else{
				cout << "a.txt couldn't be opened. Creat and write something in a.txt, and try again." << endl;
}*/

			//// ADD the steps1 parameter based the INI file ////////////

			char line6[100];
			if (stepstaticforces.is_open()){
				while (!stepstaticforces.eof()){
					stepstaticforces.getline(line6, 100, '\n');
					feb << line6 << endl;
					//i++;
					//cout << i << endl;
				}cout << " 60%" << endl;
				stepstaticforces.close();
			}
			else{
				cout << "a.txt couldn't be opened. Creat and write something in a.txt, and try again." << endl;

			}
///////////////////////////joint/////////////////////////////////////
			if (jointfree == "y"){
				char line7f[100];
				if (jointf1.is_open()){
					while (!jointf1.eof()){
						jointf1.getline(line7f, 100, '\n');
						feb << line7f << endl;
						//i++;
						//cout << i << endl;
					}cout << " 62.5%" << endl;
					jointf1.close();
				}
			}

			if (jointfree == "y"){
				char line7f2[100];
				if (jointf2.is_open()){
					while (!jointf2.eof()){
						jointf2.getline(line7f2, 100, '\n');
						feb << line7f2 << endl;
						//i++;
						//cout << i << endl;
					}cout << " 65%" << endl;
					jointf2.close();
				}
			}
/////////////////////////////////////////////////////////////////////
			char line7[100];
			if (stepstaticpositions.is_open()){
				while (!stepstaticpositions.eof()){
					stepstaticpositions.getline(line7, 100, '\n');
					feb << line7 << endl;
					//i++;
					//cout << i << endl;
				}cout << " 70%" << endl;
				stepstaticpositions.close();
			}
			else{
				cout << "a.txt couldn't be opened. Creat and write something in a.txt, and try again." << endl;

			}
			//if

			char line8[100];
			if (steptwo1.is_open()){
				while (!steptwo1.eof()){
					steptwo1.getline(line8, 100, '\n');
					feb << line8 << endl;
					//i++;
					//cout << i << endl;
				}cout << " 80%" << endl;
				steptwo1.close();
			}
			else{
				cout << "a.txt couldn't be opened. Creat and write something in a.txt, and try again." << endl;

			}
			
			///////////////////////////////////conatct//////////////////////////////
			
			
				/////////////////////////////////////////////////////////
			char line9[100];
			if (stepdynamicforces.is_open()){
				while (!stepdynamicforces.eof()){
					stepdynamicforces.getline(line9, 100, '\n');

					feb << line9 << endl;
					//i++;
					//cout << i << endl;
				}cout << " 90%" << endl;
				stepdynamicforces.close();
			}
			else{
				cout << "a.txt couldn't be opened. Creat and write something in a.txt, and try again." << endl;

			}
///////////////////////////joint/////////////////////////////////////
			if (jointfree == "y"){
				char line7sf[100];
				if (jointfo1.is_open()){
					while (!jointfo1.eof()){
						jointfo1.getline(line7sf, 100, '\n');
						feb << line7sf << endl;
						//i++;
						//cout << i << endl;
					}
					cout << " 92.5%" << endl;
					jointfo1.close();
				}
			}
			if (jointfree == "y"){
				char line7sf22[100];
				if (jointfo2.is_open()){
					while (!jointfo2.eof()){
						jointfo2.getline(line7sf22, 100, '\n');
						feb << line7sf22 << endl;
						//i++;
						//cout << i << endl;
					}
					cout << " 95%" << endl;
					jointfo2.close();
				}
			}
/////////////////////////////////////////////////////////////////////////////////////
			if (mode == 'F' || mode == 'R'){
				char line612[100];
				if (velocity.is_open()){
					while (!velocity.eof()){
						velocity.getline(line612, 100, '\n');
						feb << line612 << endl;
						//i++;
						//cout << i << endl;
					}cout << " 97.5%" << endl;
					velocity.close();
				}
				else{
					cout << "a.txt couldn't be opened. Creat and write something in a.txt, and try again." << endl;
				}
			}
			char line10[100];
			if (stepdynamicpositions.is_open()){
				while (!stepdynamicpositions.eof()){
					stepdynamicpositions.getline(line10, 100, '\n');
					feb << line10 << endl;
					//i++;
					//cout << i << endl;
				}cout << " 100%" << endl;
				stepdynamicpositions.close();
			}
			else{
				cout << "a.txt couldn't be opened. Creat and write something in a.txt, and try again." << endl;

			}
			feb.close();
			febFile.close();
		}//if
		}//if
	}//while
					

}//void

void FEBRunner::jointwrite(int itteration,int bodynum){
	/////////////CASE of prescribed motion rotation///////////////
	INIReader ini = INIReader(INI_FILE);
	string resultDir1 = BASE_DIR + ini.Get("PATH", "RESULT_DIR2", "");
	char itter = itteration + '0';
	string newfolder = resultDir1 + itter;
	string resultDir2 = newfolder;

	string bn1 = ini.Get("BODYFORCES", "BDNAME1", "");
	string bn2 = ini.Get("BODYFORCES", "BDNAME2", "");

	ofstream dof(resultDir2 + "/FEBdatapositionsdynamic" + itter + ".txt");

	string presc1f = ini.Get("BASICSETUP", "Prescribed_DOF_1", "");
	string presc2f = ini.Get("BASICSETUP", "Prescribed_DOF_2", "");
	string fix1f = ini.Get("BASICSETUP", "Fixed_DOF_1", "");
	string fix2f = ini.Get("BASICSETUP", "Fixed_DOF_2", "");

////detect how many dof is prescribed one or two  and which and how many are fixed...
	////set the kind of free dof in axis and axis2...
	
	char axis='n';
	char axis2='n';
	char axis0 = 'n';
	int number=0;
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	if (bodynum == 1){
		if (presc1f[0] == 'N'){
			if (fix1f[1] == '_' || fix1f[2] == '_' || fix1f[3] == '_'){
				if (fix1f[3] != 'z' && fix1f[4] != 'z'&& fix1f[5] != 'z' && fix1f[6] != 'z'&& fix1f[7] != 'z'&& fix1f[8] != 'z'&& fix1f[9] != 'z'&& fix1f[10] != 'z'&& fix1f[11] != 'z'){ axis = 'z'; }
				if (fix1f[3] != 'y' && fix1f[4] != 'y'&& fix1f[5] != 'y' && fix1f[6] != 'y'&& fix1f[7] != 'y'&& fix1f[8] != 'y'&& fix1f[9] != 'y'&& fix1f[10] != 'y'&& fix1f[11] != 'y'){ axis2 = 'y'; }
				if (fix1f[3] != 'x' && fix1f[4] != 'x'&& fix1f[5] != 'x' && fix1f[6] != 'x'&& fix1f[7] != 'x'&& fix1f[8] != 'x'&& fix1f[9] != 'x'&& fix1f[10] != 'x'&& fix1f[11] != 'x'){ axis0 = 'x'; }

			}
			if (fix1f[0] == 'r'){
				if (fix1f[1] != 'z' && fix1f[3] != 'z'&& fix1f[5] != 'z'){ axis = 'z'; }
				if (fix1f[1] != 'y' && fix1f[3] != 'y'&& fix1f[5] != 'y'){ axis2 = 'y'; }
				if (fix1f[1] != 'x' && fix1f[3] != 'x'&& fix1f[5] != 'x'){ axis0 = 'x'; }

			}
			if (fix1f[0] != 'r' && fix1f[1] != '_' && fix1f[2] != '_' && fix1f[3] != '_'){
				axis = 'z';
				axis2 = 'y';
				axis0 = 'x';
			}
			

		}
		if (presc1f[1] == '_' || presc1f[2] == '_' || presc1f[3] == '_'){
			number = 1;
			if (presc1f[4] == 'r' || presc1f[5] == 'r' || presc1f[6] == 'r'){
				number = 2;
			}
		}
		if (presc1f[0] == 'r'){
			number = 1;
			if (presc1f[2] == 'r'){
				number = 2;
			}
		}
		//////////////////
		if (number == 2){
			

			if (presc1f[0] == 'r'){
				if (presc1f[1] == 'x' || presc1f[3] == 'x' || presc1f[5] == 'x'){
					if (presc1f[1] == 'y' || presc1f[3] == 'y' || presc1f[5] == 'y'){
						if (fix1f[1] == '_' || fix1f[2] == '_' || fix1f[3] == '_'){
							if (fix1f[3] != 'z' && fix1f[4] != 'z'&& fix1f[5] != 'z' && fix1f[6] != 'z'&& fix1f[7] != 'z'&& fix1f[8] != 'z'&& fix1f[9] != 'z'&& fix1f[10] != 'z'&& fix1f[11] != 'z'){ axis = 'z'; }
						}
						if (fix1f[0] == 'r'){
							if (fix1f[1] != 'z' && fix1f[3] != 'z'&& fix1f[5] != 'z'){ axis = 'z'; }
						}
						if (fix1f[0] != 'r' && fix1f[1] != '_' && fix1f[2] != '_' && fix1f[3] != '_'){
							axis = 'z';
						}
					}
				}

				if (presc1f[1] == 'x' || presc1f[3] == 'x' || presc1f[5] == 'x'){
					if (presc1f[1] == 'z' || presc1f[3] == 'z' || presc1f[5] == 'z'){
						if (fix1f[1] == '_' || fix1f[2] == '_' || fix1f[3] == '_'){
							if (fix1f[3] != 'y' && fix1f[4] != 'y'&& fix1f[5] != 'y' && fix1f[6] != 'y'&& fix1f[7] != 'y'&& fix1f[8] != 'y'&& fix1f[9] != 'y'&& fix1f[10] != 'y'&& fix1f[11] != 'y'){ axis = 'y'; }
						}
						if (fix1f[0] == 'r'){
							if (fix1f[1] != 'y' && fix1f[3] != 'y'&& fix1f[5] != 'y'){ axis = 'y'; }

						}
						if (fix1f[0] != 'r' && fix1f[1] != '_' && fix1f[2] != '_' && fix1f[3] != '_'){
							axis = 'y';
						}

					}
				}


				if (presc1f[1] == 'y' || presc1f[3] == 'y' || presc1f[5] == 'y'){
					if (presc1f[1] == 'z' || presc1f[3] == 'z' || presc1f[5] == 'z'){
						if (fix1f[1] == '_' || fix1f[2] == '_' || fix1f[3] == '_'){
							if (fix1f[3] != 'x' && fix1f[4] != 'x'&& fix1f[5] != 'x' && fix1f[6] != 'x'&& fix1f[7] != 'x'&& fix1f[8] != 'x'&& fix1f[9] != 'x'&& fix1f[10] != 'x'&& fix1f[11] != 'x'){ axis = 'x'; }
						}
						if (fix1f[0] == 'r'){
							if (fix1f[1] != 'x' && fix1f[3] != 'x'&& fix1f[5] != 'x'){ axis = 'x'; }
						}
						if (fix1f[0] != 'r' && fix1f[1] != '_' && fix1f[2] != '_' && fix1f[3] != '_'){
							axis = 'x';
						}
					}
				}
			}
			///////////////////
			if (presc1f[1] == '_' || presc1f[2] == '_' || presc1f[3] == '_'){
				if (presc1f[3] == 'x' || presc1f[4] == 'x' || presc1f[5] == 'x' || presc1f[6] == 'x' || presc1f[7] == 'x' || presc1f[8] == 'x' || presc1f[9] == 'x' || presc1f[10] == 'x' || presc1f[11] == 'x'){
					if (presc1f[3] == 'y' || presc1f[4] == 'y' || presc1f[5] == 'y' || presc1f[6] == 'y' || presc1f[7] == 'y' || presc1f[8] == 'y' || presc1f[9] == 'y' || presc1f[10] == 'y' || presc1f[11] == 'y'){
						if (presc1f[3] != 'z' || presc1f[4] != 'z' || presc1f[5] != 'z' || presc1f[6] != 'z' || presc1f[7] != 'z' || presc1f[8] != 'z' || presc1f[9] != 'z' || presc1f[10] != 'z' || presc1f[11] != 'z'){
							if (fix1f[1] == '_' || fix1f[2] == '_' || fix1f[3] == '_'){
								if (fix1f[3] != 'z' && fix1f[4] != 'z'&& fix1f[5] != 'z' && fix1f[6] != 'z'&& fix1f[7] != 'z'&& fix1f[8] != 'z'&& fix1f[9] != 'z'&& fix1f[10] != 'z'&& fix1f[11] != 'z'){ axis = 'z'; }
							}
							if (fix1f[0] == 'r'){
								if (fix1f[1] != 'z' && fix1f[3] != 'z'&& fix1f[5] != 'z'){ axis = 'z'; }
							}
							if (fix1f[0] != 'r' && fix1f[1] != '_' && fix1f[2] != '_' && fix1f[3] != '_'){
								axis = 'z';
							}
						}
					}
				}

				if (presc1f[3] == 'z' || presc1f[4] == 'z' || presc1f[5] == 'z' || presc1f[6] == 'z' || presc1f[7] == 'z' || presc1f[8] == 'z' || presc1f[9] == 'z' || presc1f[10] == 'z' || presc1f[11] == 'z'){
					if (presc1f[3] == 'y' || presc1f[4] == 'y' || presc1f[5] == 'y' || presc1f[6] == 'y' || presc1f[7] == 'y' || presc1f[8] == 'y' || presc1f[9] == 'y' || presc1f[10] == 'y' || presc1f[11] == 'y'){
						if (presc1f[3] != 'x' || presc1f[4] != 'x' || presc1f[5] != 'x' || presc1f[6] != 'x' || presc1f[7] != 'x' || presc1f[8] != 'x' || presc1f[9] != 'x' || presc1f[10] != 'x' || presc1f[11] != 'x'){
							if (fix1f[1] == '_' || fix1f[2] == '_' || fix1f[3] == '_'){
								if (fix1f[3] != 'x' && fix1f[4] != 'x'&& fix1f[5] != 'x' && fix1f[6] != 'x'&& fix1f[7] != 'x'&& fix1f[8] != 'x'&& fix1f[9] != 'x'&& fix1f[10] != 'x'&& fix1f[11] != 'x'){ axis = 'x'; }
							}
							if (fix1f[0] == 'r'){
								if (fix1f[1] != 'x' && fix1f[3] != 'x'&& fix1f[5] != 'x'){ axis = 'x'; }
							}
							if (fix1f[0] != 'r' && fix1f[1] != '_' && fix1f[2] != '_' && fix1f[3] != '_'){
								axis = 'x';
							}
						}
					}
				}
				if (presc1f[3] == 'x' || presc1f[4] == 'x' || presc1f[5] == 'x' || presc1f[6] == 'x' || presc1f[7] == 'x' || presc1f[8] == 'x' || presc1f[9] == 'x' || presc1f[10] == 'x' || presc1f[11] == 'x'){
					if (presc1f[3] == 'z' || presc1f[4] == 'z' || presc1f[5] == 'z' || presc1f[6] == 'z' || presc1f[7] == 'z' || presc1f[8] == 'z' || presc1f[9] == 'z' || presc1f[10] == 'z' || presc1f[11] == 'z'){
						if (presc1f[3] != 'y' || presc1f[4] != 'y' || presc1f[5] != 'y' || presc1f[6] != 'y' || presc1f[7] != 'y' || presc1f[8] != 'y' || presc1f[9] != 'y' || presc1f[10] != 'y' || presc1f[11] != 'y'){
							if (fix1f[1] == '_' || fix1f[2] == '_' || fix1f[3] == '_'){
								if (fix1f[3] != 'y' && fix1f[4] != 'y'&& fix1f[5] != 'y' && fix1f[6] != 'y'&& fix1f[7] != 'y'&& fix1f[8] != 'y'&& fix1f[9] != 'y'&& fix1f[10] != 'y'&& fix1f[11] != 'y'){ axis = 'y'; }
							}
							if (fix1f[0] == 'r'){
								if (fix1f[1] != 'y' && fix1f[3] != 'y'&& fix1f[5] != 'y'){ axis = 'y'; }

							}
							if (fix1f[0] != 'r' && fix1f[1] != '_' && fix1f[2] != '_' && fix1f[3] != '_'){
								axis = 'y';
							}
						}
					}

				}
			}
		}
		////////////////////////////////////////////		
		if (number == 1){


			if (presc1f[0] == 'r'){
				if (presc1f[1] == 'x' || presc1f[3] == 'x' || presc1f[5] == 'x'){
					if (fix1f[1] == '_' || fix1f[2] == '_' || fix1f[3] == '_'){
						if (fix1f[3] != 'z' && fix1f[4] != 'z'&& fix1f[5] != 'z' && fix1f[6] != 'z'&& fix1f[7] != 'z'&& fix1f[8] != 'z'&& fix1f[9] != 'z'&& fix1f[10] != 'z'&& fix1f[11] != 'z'){ axis = 'z'; }
						if (fix1f[3] != 'y' && fix1f[4] != 'y'&& fix1f[5] != 'y' && fix1f[6] != 'y'&& fix1f[7] != 'y'&& fix1f[8] != 'y'&& fix1f[9] != 'y'&& fix1f[10] != 'y'&& fix1f[11] != 'y'){ axis2 = 'y'; }
					}
					if (fix1f[0] == 'r'){ if (fix1f[1] != 'z' && fix1f[3] != 'z'&& fix1f[5] != 'z'){ axis = 'z'; } if (fix1f[1] != 'y' && fix1f[3] != 'y'&& fix1f[5] != 'y'){ axis2 = 'y'; } }
					if (fix1f[0] != 'r' && fix1f[1] != '_' && fix1f[2] != '_' && fix1f[3] != '_'){
						axis = 'z'; axis2 = 'y';
					}
				}

				if (presc1f[1] == 'y' || presc1f[3] == 'y' || presc1f[5] == 'y'){
					if (fix1f[1] == '_' || fix1f[2] == '_' || fix1f[3] == '_'){
						if (fix1f[3] != 'z' && fix1f[4] != 'z'&& fix1f[5] != 'z' && fix1f[6] != 'z'&& fix1f[7] != 'z'&& fix1f[8] != 'z'&& fix1f[9] != 'z'&& fix1f[10] != 'z'&& fix1f[11] != 'z'){ axis = 'z'; }
						if (fix1f[3] != 'x' && fix1f[4] != 'x'&& fix1f[5] != 'x' && fix1f[6] != 'x'&& fix1f[7] != 'x'&& fix1f[8] != 'x'&& fix1f[9] != 'x'&& fix1f[10] != 'x'&& fix1f[11] != 'x'){ axis2 = 'x'; }
					}
					if (fix1f[0] == 'r'){ if (fix1f[1] != 'z' && fix1f[3] != 'z'&& fix1f[5] != 'z'){ axis = 'z'; } if (fix1f[1] != 'x' && fix1f[3] != 'x'&& fix1f[5] != 'x'){ axis2 = 'x'; } }

					if (fix1f[0] != 'r' && fix1f[1] != '_'&& fix1f[2] != '_' && fix1f[3] != '_'){
						axis = 'z'; axis2 = 'x';
					}
				}
				if (presc1f[1] == 'z' || presc1f[3] == 'z' || presc1f[5] == 'z'){
					if (fix1f[1] == '_' || fix1f[2] == '_' || fix1f[3] == '_'){
						if (fix1f[3] != 'x' && fix1f[4] != 'x'&& fix1f[5] != 'x' && fix1f[6] != 'x'&& fix1f[7] != 'x'&& fix1f[8] != 'x'&& fix1f[9] != 'x'&& fix1f[10] != 'x'&& fix1f[11] != 'x'){ axis = 'x'; }
						if (fix1f[3] != 'y' && fix1f[4] != 'y'&& fix1f[5] != 'y' && fix1f[6] != 'y'&& fix1f[7] != 'y'&& fix1f[8] != 'y'&& fix1f[9] != 'y'&& fix1f[10] != 'y'&& fix1f[11] != 'y'){ axis2 = 'y'; }
					}
					if (fix1f[0] == 'r'){ if (fix1f[1] != 'y' && fix1f[3] != 'y'&& fix1f[5] != 'y'){ axis = 'y'; } if (fix1f[1] != 'x' && fix1f[3] != 'x'&& fix1f[5] != 'x'){ axis2 = 'x'; } }
					if (fix1f[0] != 'r' && fix1f[1] != '_' && fix1f[2] != '_' && fix1f[3] != '_'){
						axis = 'y'; axis2 = 'x';
					}
				}

			}







			////////////////////////////////////////////

			if (presc1f[3] == 'x' || presc1f[4] == 'x' || presc1f[5] == 'x' || presc1f[6] == 'x' || presc1f[7] == 'x' || presc1f[8] == 'x' || presc1f[9] == 'x' || presc1f[10] == 'x' || presc1f[11] == 'x'){
				if (fix1f[1] == '_' || fix1f[2] == '_' || fix1f[3] == '_'){
					if (fix1f[3] != 'z' && fix1f[4] != 'z'&& fix1f[5] != 'z' && fix1f[6] != 'z'&& fix1f[7] != 'z'&& fix1f[8] != 'z'&& fix1f[9] != 'z'&& fix1f[10] != 'z'&& fix1f[11] != 'z'){ axis = 'z'; }
					if (fix1f[3] != 'y' && fix1f[4] != 'y'&& fix1f[5] != 'y' && fix1f[6] != 'y'&& fix1f[7] != 'y'&& fix1f[8] != 'y'&& fix1f[9] != 'y'&& fix1f[10] != 'y'&& fix1f[11] != 'y'){ axis2 = 'y'; }
				}
				if (fix1f[0] == 'r'){ if (fix1f[1] != 'z' && fix1f[3] != 'z'&& fix1f[5] != 'z'){ axis = 'z'; } if (fix1f[1] != 'y' && fix1f[3] != 'y'&& fix1f[5] != 'y'){ axis2 = 'y'; } }
				if (fix1f[0] != 'r' && fix1f[1] != '_' && fix1f[2] != '_' && fix1f[3] != '_'){
					axis = 'z'; axis2 = 'y';
				}
			}

			if (presc1f[3] == 'y' || presc1f[4] == 'y' || presc1f[5] == 'y' || presc1f[6] == 'y' || presc1f[7] == 'y' || presc1f[8] == 'y' || presc1f[9] == 'y' || presc1f[10] == 'y' || presc1f[11] == 'y'){
				if (fix1f[1] == '_' || fix1f[2] == '_' || fix1f[3] == '_'){
					if (fix1f[3] != 'z' && fix1f[4] != 'z'&& fix1f[5] != 'z' && fix1f[6] != 'z'&& fix1f[7] != 'z'&& fix1f[8] != 'z'&& fix1f[9] != 'z'&& fix1f[10] != 'z'&& fix1f[11] != 'z'){ axis = 'z'; }
					if (fix1f[3] != 'x' && fix1f[4] != 'x'&& fix1f[5] != 'x' && fix1f[6] != 'x'&& fix1f[7] != 'x'&& fix1f[8] != 'x'&& fix1f[9] != 'x'&& fix1f[10] != 'x'&& fix1f[11] != 'x'){ axis2 = 'x'; }
				}
				if (fix1f[0] == 'r'){ if (fix1f[1] != 'z' && fix1f[3] != 'z'&& fix1f[5] != 'z'){ axis = 'z'; } if (fix1f[1] != 'x' && fix1f[3] != 'x'&& fix1f[5] != 'x'){ axis2 = 'x'; } }
				if (fix1f[0] != 'r' && fix1f[1] != '_' && fix1f[2] != '_' && fix1f[3] != '_'){
					axis = 'z'; axis2 = 'x';
				}
			}

			if (presc1f[3] == 'z' || presc1f[4] == 'z' || presc1f[5] == 'z' || presc1f[6] == 'z' || presc1f[7] == 'z' || presc1f[8] == 'z' || presc1f[9] == 'z' || presc1f[10] == 'z' || presc1f[11] == 'z'){
				if (fix1f[1] == '_' || fix1f[2] == '_' || fix1f[3] == '_'){
					if (fix1f[3] != 'x' && fix1f[4] != 'x'&& fix1f[5] != 'x' && fix1f[6] != 'x'&& fix1f[7] != 'x'&& fix1f[8] != 'x'&& fix1f[9] != 'x'&& fix1f[10] != 'x'&& fix1f[11] != 'x'){ axis = 'x'; }
					if (fix1f[3] != 'y' && fix1f[4] != 'y'&& fix1f[5] != 'y' && fix1f[6] != 'y'&& fix1f[7] != 'y'&& fix1f[8] != 'y'&& fix1f[9] != 'y'&& fix1f[10] != 'y'&& fix1f[11] != 'y'){ axis2 = 'y'; }
				}
				if (fix1f[0] == 'r'){ if (fix1f[1] != 'y' && fix1f[3] != 'y'&& fix1f[5] != 'y'){ axis = 'y'; } if (fix1f[1] != 'x' && fix1f[3] != 'x'&& fix1f[5] != 'x'){ axis2 = 'x'; } }
				if (fix1f[0] != 'r' && fix1f[1] != '_' && fix1f[2] != '_' && fix1f[3] != '_'){
					axis = 'y'; axis2 = 'x';
				}
			}

		}


	}
/////////////////////////////////////////////////////////////////////////////////////////////////
				////////////////////////////////////////////////////////////////
		if (bodynum == 2){


			if (presc2f[0] == 'N'){
				if (fix2f[1] == '_' || fix2f[2] == '_' || fix2f[3] == '_'){
					if (fix2f[3] != 'z' && fix2f[4] != 'z'&& fix2f[5] != 'z' && fix2f[6] != 'z'&& fix2f[7] != 'z'&& fix2f[8] != 'z'&& fix2f[9] != 'z'&& fix2f[10] != 'z'&& fix2f[11] != 'z'){ axis = 'z'; }
					if (fix2f[3] != 'y' && fix2f[4] != 'y'&& fix2f[5] != 'y' && fix2f[6] != 'y'&& fix2f[7] != 'y'&& fix2f[8] != 'y'&& fix2f[9] != 'y'&& fix2f[10] != 'y'&& fix2f[11] != 'y'){ axis2 = 'y'; }
					if (fix2f[3] != 'x' && fix2f[4] != 'x'&& fix2f[5] != 'x' && fix2f[6] != 'x'&& fix2f[7] != 'x'&& fix2f[8] != 'x'&& fix2f[9] != 'x'&& fix2f[10] != 'x'&& fix2f[11] != 'x'){ axis0 = 'x'; }

				}
				if (fix2f[0] == 'r'){
					if (fix2f[1] != 'z' && fix2f[3] != 'z'&& fix2f[5] != 'z'){ axis = 'z'; }
					if (fix2f[1] != 'y' && fix2f[3] != 'y'&& fix2f[5] != 'y'){ axis2 = 'y'; }
					if (fix2f[1] != 'x' && fix2f[3] != 'x'&& fix2f[5] != 'x'){ axis0 = 'x'; }

				}
				if (fix2f[0] != 'r' && fix2f[1] != '_' && fix2f[2] != '_' && fix2f[3] != '_'){
					axis = 'z';
					axis2 = 'y';
					axis0 = 'x';
				}
			}

					if (presc2f[1] == '_' || presc2f[2] == '_' || presc2f[3] == '_'){
				number = 1;
				if (presc2f[4] == 'r' || presc2f[5] == 'r' || presc2f[6] == 'r'){
					number = 2;
				}
			}
					if (presc2f[0] == 'r'){
						number = 1;
						if (presc2f[2] == 'r'){
							number = 2;
						}
					}
					//////////////////////////////////////
			if (number == 2){

				if (presc2f[0] == 'r'){
					if (presc2f[1] == 'x' || presc2f[3] == 'x' || presc2f[5] == 'x'){
						if (presc2f[1] == 'y' || presc2f[3] == 'y' || presc2f[5] == 'y'){
							if (fix2f[1] == '_' || fix2f[2] == '_' || fix2f[3] == '_'){
								if (fix2f[3] != 'z' && fix2f[4] != 'z'&& fix2f[5] != 'z' && fix2f[6] != 'z'&& fix2f[7] != 'z'&& fix2f[8] != 'z'&& fix2f[9] != 'z'&& fix2f[10] != 'z'&& fix2f[11] != 'z'){ axis = 'z'; }
							}
							if (fix2f[0] == 'r'){
								if (fix2f[1] != 'z' && fix2f[3] != 'z'&& fix2f[5] != 'z'){ axis = 'z'; }
							}
							if (fix2f[0] != 'r' && fix2f[1] != '_' && fix2f[2] != '_'&& fix2f[3] != '_'){
								axis = 'z';
							}
						}
					}

					if (presc2f[1] == 'x' || presc2f[3] == 'x' || presc2f[5] == 'x'){
						if (presc2f[1] == 'z' || presc2f[3] == 'z' || presc2f[5] == 'z'){
							if (fix2f[1] == '_' || fix2f[2] == '_' || fix2f[3] == '_'){
								if (fix2f[3] != 'y' && fix2f[4] != 'y'&& fix2f[5] != 'y' && fix2f[6] != 'y'&& fix2f[7] != 'y'&& fix2f[8] != 'y'&& fix2f[9] != 'y'&& fix2f[10] != 'y'&& fix2f[11] != 'y'){ axis = 'y'; }
							}
							if (fix2f[0] == 'r'){
								if (fix2f[1] != 'y' && fix2f[3] != 'y'&& fix2f[5] != 'y'){ axis = 'y'; }

							}
							if (fix2f[0] != 'r' && fix2f[1] != '_' && fix2f[2] != '_' && fix2f[3] != '_'){
								axis = 'y';
							}

						}
					}


					if (presc2f[1] == 'y' || presc2f[3] == 'y' || presc2f[5] == 'y'){
						if (presc2f[1] == 'z' || presc2f[3] == 'z' || presc2f[5] == 'z'){
							if (fix2f[1] == '_' || fix2f[2] == '_' || fix2f[3] == '_'){
								if (fix2f[3] != 'x' && fix2f[4] != 'x'&& fix2f[5] != 'x' && fix2f[6] != 'x'&& fix2f[7] != 'x'&& fix2f[8] != 'x'&& fix2f[9] != 'x'&& fix2f[10] != 'x'&& fix2f[11] != 'x'){ axis = 'x'; }
							}
							if (fix2f[0] == 'r'){
								if (fix2f[1] != 'x' && fix2f[3] != 'x'&& fix2f[5] != 'x'){ axis = 'x'; }
							}
							if (fix2f[0] != 'r' && fix2f[1] != '_' && fix2f[2] != '_' && fix2f[3] != '_'){
								axis = 'x';
							}
						}
					}
				}
				/////////////////////////////

				if (presc2f[1] == '_' || presc2f[2] == '_' || presc2f[3] == '_'){
					if (presc2f[3] == 'x' || presc2f[4] == 'x' || presc2f[5] == 'x' || presc2f[6] == 'x' || presc2f[7] == 'x' || presc2f[8] == 'x' || presc2f[9] == 'x' || presc2f[10] == 'x' || presc2f[11] == 'x'){
						if (presc2f[3] == 'y' || presc2f[4] == 'y' || presc2f[5] == 'y' || presc2f[6] == 'y' || presc2f[7] == 'y' || presc2f[8] == 'y' || presc2f[9] == 'y' || presc2f[10] == 'y' || presc2f[11] == 'y'){
							if (presc2f[3] != 'z' || presc2f[4] != 'z' || presc2f[5] != 'z' || presc2f[6] != 'z' || presc2f[7] != 'z' || presc2f[8] != 'z' || presc2f[9] != 'z' || presc2f[10] != 'z' || presc2f[11] != 'z'){
								if (fix2f[1] == '_' || fix2f[2] == '_' || fix2f[3] == '_'){
									if (fix2f[3] != 'z' && fix2f[4] != 'z'&& fix2f[5] != 'z' && fix2f[6] != 'z'&& fix2f[7] != 'z'&& fix2f[8] != 'z'&& fix2f[9] != 'z'&& fix2f[10] != 'z'&& fix2f[11] != 'z'){ axis = 'z'; }
								}
								if (fix2f[0] == 'r'){
									if (fix2f[1] != 'z' && fix2f[3] != 'z'&& fix2f[5] != 'z'){ axis = 'z'; }
								}
								if (fix2f[0] != 'r' && fix2f[1] != '_' && fix2f[2] != '_' && fix2f[3] != '_'){
									axis = 'z';
								}
							}
						}
					}
				
				if (presc2f[3] == 'z' || presc2f[4] == 'z' || presc2f[5] == 'z' || presc2f[6] == 'z' || presc2f[7] == 'z' || presc2f[8] == 'z' || presc2f[9] == 'z' || presc2f[10] == 'z' || presc2f[11] == 'z'){
					if (presc2f[3] == 'y' || presc2f[4] == 'y' || presc2f[5] == 'y' || presc2f[6] == 'y' || presc2f[7] == 'y' || presc2f[8] == 'y' || presc2f[9] == 'y' || presc2f[10] == 'y' || presc2f[11] == 'y'){
						if (presc2f[3] != 'x' || presc2f[4] != 'x' || presc2f[5] != 'x' || presc2f[6] != 'x' || presc2f[7] != 'x' || presc2f[8] != 'x' || presc2f[9] != 'x' || presc2f[10] != 'x' || presc2f[11] != 'x'){
							if (fix2f[1] == '_' || fix2f[2] == '_' || fix2f[3] == '_'){
								if (fix2f[3] != 'x' && fix2f[4] != 'x'&& fix2f[5] != 'x' && fix2f[6] != 'x'&& fix2f[7] != 'x'&& fix2f[8] != 'x'&& fix2f[9] != 'x'&& fix2f[10] != 'x'&& fix2f[11] != 'x'){ axis = 'x'; }
							}
							if (fix2f[0] == 'r'){
								if (fix2f[1] != 'x' && fix2f[3] != 'x'&& fix2f[5] != 'x'){ axis = 'x'; }
							}
							if (fix2f[0] != 'r' && fix2f[1] != '_' && fix2f[2] != '_' && fix2f[3] != '_'){
								axis = 'x';
							}
						}
					}
				}
				if (presc2f[3] == 'x' || presc2f[4] == 'x' || presc2f[5] == 'x' || presc2f[6] == 'x' || presc2f[7] == 'x' || presc2f[8] == 'x' || presc2f[9] == 'x' || presc2f[10] == 'x' || presc2f[11] == 'x'){
					if (presc2f[3] == 'z' || presc2f[4] == 'z' || presc2f[5] == 'z' || presc2f[6] == 'z' || presc2f[7] == 'z' || presc2f[8] == 'z' || presc2f[9] == 'z' || presc2f[10] == 'z' || presc2f[11] == 'z'){
						if (presc2f[3] != 'y' || presc2f[4] != 'y' || presc2f[5] != 'y' || presc2f[6] != 'y' || presc2f[7] != 'y' || presc2f[8] != 'y' || presc2f[9] != 'y' || presc2f[10] != 'y' || presc2f[11] != 'y'){
							if (fix2f[1] == '_' || fix2f[2] == '_' || fix2f[3] == '_'){
								if (fix2f[3] != 'y' && fix2f[4] != 'y'&& fix2f[5] != 'y' && fix2f[6] != 'y'&& fix2f[7] != 'y'&& fix2f[8] != 'y'&& fix2f[9] != 'y'&& fix2f[10] != 'y'&& fix2f[11] != 'y'){ axis = 'y'; }
							}
							if (fix2f[0] == 'r'){
								if (fix2f[1] != 'y' && fix2f[3] != 'y'&& fix2f[5] != 'y'){ axis = 'y'; }

							}
							if (fix2f[0] != 'r' && fix2f[1] != '_' && fix2f[2] != '_' && fix2f[3] != '_'){
								axis = 'y';
							}
						}
					}
				}
				}
			}
////////////////////////////////////////////////////////////////////
			if (number == 1){



			if (presc2f[0] == 'r'){ 	
					if (presc2f[1] == 'x' || presc2f[3] == 'x' || presc2f[5] == 'x'){
						if (fix2f[1] == '_' || fix2f[2] == '_' || fix2f[3] == '_'){
							if (fix2f[3] != 'z' && fix2f[4] != 'z'&& fix2f[5] != 'z' && fix2f[6] != 'z'&& fix2f[7] != 'z'&& fix2f[8] != 'z'&& fix2f[9] != 'z'&& fix2f[10] != 'z'&& fix2f[11] != 'z'){ axis = 'z'; }
							if (fix2f[3] != 'y' && fix2f[4] != 'y'&& fix2f[5] != 'y' && fix2f[6] != 'y'&& fix2f[7] != 'y'&& fix2f[8] != 'y'&& fix2f[9] != 'y'&& fix2f[10] != 'y'&& fix2f[11] != 'y'){ axis2 = 'y'; }
						}
						if (fix2f[0] == 'r'){ if (fix2f[1] != 'z' && fix2f[3] != 'z'&& fix2f[5] != 'z'){ axis = 'z'; } if (fix2f[1] != 'y' && fix2f[3] != 'y'&& fix2f[5] != 'y'){ axis2 = 'y'; } }
						if (fix2f[0] != 'r' && fix2f[1] != '_' && fix2f[2] != '_' && fix2f[3] != '_'){
							axis = 'z'; axis2 = 'y';
						}
					}

					if (presc2f[1] == 'y' || presc2f[3] == 'y' || presc2f[5] == 'y'){
						if (fix2f[1] == '_' || fix2f[2] == '_' || fix2f[3] == '_'){
							if (fix2f[3] != 'z' && fix2f[4] != 'z'&& fix2f[5] != 'z' && fix2f[6] != 'z'&& fix2f[7] != 'z'&& fix2f[8] != 'z'&& fix2f[9] != 'z'&& fix2f[10] != 'z'&& fix2f[11] != 'z'){ axis = 'z'; }
							if (fix2f[3] != 'x' && fix2f[4] != 'x'&& fix2f[5] != 'x' && fix2f[6] != 'x'&& fix2f[7] != 'x'&& fix2f[8] != 'x'&& fix2f[9] != 'x'&& fix2f[10] != 'x'&& fix2f[11] != 'x'){ axis2 = 'x'; }
						}
						if (fix2f[0] == 'r'){ if (fix2f[1] != 'z' && fix2f[3] != 'z'&& fix2f[5] != 'z'){ axis = 'z'; } if (fix2f[1] != 'x' && fix2f[3] != 'x'&& fix2f[5] != 'x'){ axis2 = 'x'; } }
						if (fix2f[0] != 'r' && fix2f[1] != '_' && fix2f[2] != '_' && fix2f[3] != '_'){
							axis = 'z'; axis2 = 'x';
						}
					}
				if (presc2f[1] == 'z' || presc2f[3] == 'z' || presc2f[5] == 'z'){
					if (fix2f[1] == '_' || fix2f[2] == '_' || fix2f[3] == '_'){
						if (fix2f[3] != 'x' && fix2f[4] != 'x'&& fix2f[5] != 'x' && fix2f[6] != 'x'&& fix2f[7] != 'x'&& fix2f[8] != 'x'&& fix2f[9] != 'x'&& fix2f[10] != 'x'&& fix2f[11] != 'x'){ axis = 'x'; }
						if (fix2f[3] != 'y' && fix2f[4] != 'y'&& fix2f[5] != 'y' && fix2f[6] != 'y'&& fix2f[7] != 'y'&& fix2f[8] != 'y'&& fix2f[9] != 'y'&& fix2f[10] != 'y'&& fix2f[11] != 'y'){ axis2 = 'y'; }
					}
					if (fix2f[0] == 'r'){ if (fix2f[1] != 'y' && fix2f[3] != 'y'&& fix2f[5] != 'y'){ axis = 'y'; } if (fix2f[1] != 'x' && fix2f[3] != 'x'&& fix2f[5] != 'x'){ axis2 = 'x'; } }
					if (fix2f[0] != 'r' && fix2f[1] != '_' && fix2f[2] != '_' && fix2f[3] != '_'){
						axis = 'y'; axis2 = 'x';
					}
				}


			}
///////////////////////////////////////////////////////////////////////
				if (presc2f[3] == 'x' || presc2f[4] == 'x' || presc2f[5] == 'x' || presc2f[6] == 'x' || presc2f[7] == 'x' || presc2f[8] == 'x' || presc2f[9] == 'x' || presc2f[10] == 'x' || presc2f[11] == 'x'){
					if (fix2f[1] == '_' || fix2f[2] == '_' || fix2f[3] == '_'){
						if (fix2f[3] != 'z' && fix2f[4] != 'z'&& fix2f[5] != 'z' && fix2f[6] != 'z'&& fix2f[7] != 'z'&& fix2f[8] != 'z'&& fix2f[9] != 'z'&& fix2f[10] != 'z'&& fix2f[11] != 'z'){ axis = 'z'; }
						if (fix2f[3] != 'y' && fix2f[4] != 'y'&& fix2f[5] != 'y' && fix2f[6] != 'y'&& fix2f[7] != 'y'&& fix2f[8] != 'y'&& fix2f[9] != 'y'&& fix2f[10] != 'y'&& fix2f[11] != 'y'){ axis2 = 'y'; }
					}
					if (fix2f[0] == 'r'){ if (fix2f[1] != 'z' && fix2f[3] != 'z'&& fix2f[5] != 'z'){ axis = 'z'; } if (fix2f[1] != 'y' && fix2f[3] != 'y'&& fix2f[5] != 'y'){ axis2 = 'y'; } }
					if (fix2f[0] != 'r' && fix2f[1] != '_' && fix2f[2] != '_' && fix2f[3] != '_'){
						axis = 'z'; axis2 = 'y';
					}
				}

				if (presc2f[3] == 'y' || presc2f[4] == 'y' || presc2f[5] == 'y' || presc2f[6] == 'y' || presc2f[7] == 'y' || presc2f[8] == 'y' || presc2f[9] == 'y' || presc2f[10] == 'y' || presc2f[11] == 'y'){
					if (fix2f[1] == '_' || fix2f[2] == '_' || fix2f[3] == '_'){
						if (fix2f[3] != 'z' && fix2f[4] != 'z'&& fix2f[5] != 'z' && fix2f[6] != 'z'&& fix2f[7] != 'z'&& fix2f[8] != 'z'&& fix2f[9] != 'z'&& fix2f[10] != 'z'&& fix2f[11] != 'z'){ axis = 'z'; }
						if (fix2f[3] != 'x' && fix2f[4] != 'x'&& fix2f[5] != 'x' && fix2f[6] != 'x'&& fix2f[7] != 'x'&& fix2f[8] != 'x'&& fix2f[9] != 'x'&& fix2f[10] != 'x'&& fix2f[11] != 'x'){ axis2 = 'x'; }
					}
					if (fix2f[0] == 'r'){ if (fix2f[1] != 'z' && fix2f[3] != 'z'&& fix2f[5] != 'z'){ axis = 'z'; } if (fix2f[1] != 'x' && fix2f[3] != 'x'&& fix2f[5] != 'x'){ axis2 = 'x'; } }
					if (fix2f[0] != 'r' && fix2f[1] != '_' && fix2f[2] != '_' && fix2f[3] != '_'){
						axis = 'z'; axis2 = 'x';
					}
				}

				if (presc2f[3] == 'z' || presc2f[4] == 'z' || presc2f[5] == 'z' || presc2f[6] == 'z' || presc2f[7] == 'z' || presc2f[8] == 'z' || presc2f[9] == 'z' || presc2f[10] == 'z' || presc2f[11] == 'z'){
					if (fix2f[1] == '_' || fix2f[2] == '_' || fix2f[3] == '_'){
						if (fix2f[3] != 'x' && fix2f[4] != 'x'&& fix2f[5] != 'x' && fix2f[6] != 'x'&& fix2f[7] != 'x'&& fix2f[8] != 'x'&& fix2f[9] != 'x'&& fix2f[10] != 'x'&& fix2f[11] != 'x'){ axis = 'x'; }
						if (fix2f[3] != 'y' && fix2f[4] != 'y'&& fix2f[5] != 'y' && fix2f[6] != 'y'&& fix2f[7] != 'y'&& fix2f[8] != 'y'&& fix2f[9] != 'y'&& fix2f[10] != 'y'&& fix2f[11] != 'y'){ axis2 = 'y'; }
					}
					if (fix2f[0] == 'r'){ if (fix2f[1] != 'y' && fix2f[3] != 'y'&& fix2f[5] != 'y'){ axis = 'y'; } if (fix2f[1] != 'x' && fix2f[3] != 'x'&& fix2f[5] != 'x'){ axis2 = 'x'; } }
					if (fix2f[0] != 'r' && fix2f[1] != '_' && fix2f[2] != '_' && fix2f[3] != '_'){
						axis = 'y'; axis2 = 'x';
					}
				}

			}
		}
////////////////////////////////////////////////////////////////
		//scan if there are forced in free component or not...
		string forc1 = ini.Get("BASICSETUP", "Forced_DOF_1", "");
		string forc2 = ini.Get("BASICSETUP", "Forced_DOF_2", "");
		char axis3 = 'n';
		char axis4 = 'n';
		char axis5 = 'n';
		cout << "The detected free joints are:" << endl;
		cout << axis << endl;
		cout << axis2 << endl;
		cout << axis0 << endl;
		
		if (forc1[1] == '_' || forc1[2] == '_' || forc1[3] == '_'){

			if (forc1[3] == 'x' || forc1[4] == 'x' || forc1[5] == 'x' || forc1[6] == 'x' || forc1[7] == 'x' || forc1[8] == 'x' || forc1[9] == 'x' || forc1[10] == 'x' || forc1[11] == 'x'){
				if (axis == 'x'){ axis3 = 'x'; }
				if (axis2 == 'x'){ axis4 = 'x'; }
				if (axis0 == 'x'){ axis5 = 'x'; }
			}
			if (forc1[3] == 'y' || forc1[4] == 'y' || forc1[5] == 'y' || forc1[6] == 'y' || forc1[7] == 'y' || forc1[8] == 'y' || forc1[9] == 'y' || forc1[10] == 'y' || forc1[11] == 'y'){
				if (axis == 'y'){ axis3 = 'y'; }
				if (axis2 == 'y'){ axis4 = 'y'; }
				if (axis0 == 'y'){ axis5 = 'y'; }
			}
			if (forc1[3] == 'z'|| forc1[4] == 'z' || forc1[5] == 'z' || forc1[6] == 'z' || forc1[7] == 'z'|| forc1[8] == 'z'|| forc1[9] == 'z'|| forc1[10] == 'z'|| forc1[11] == 'z'){
				if (axis == 'z'){ axis3 = 'z'; }
				if (axis2 == 'z'){ axis4 = 'z'; }
				if (axis0 == 'z'){ axis5 = 'z'; }
			}
		}
			
		if (forc1[0] == 'r'){
			if (forc1[1] == 'x' || forc1[3] == 'x' || forc1[5] == 'x'){
				if (axis == 'x'){ axis3 = 'x'; }
				if (axis2 == 'x'){ axis4 = 'x'; }
				if (axis0 == 'x'){ axis5 = 'x'; }
			}
			if (forc1[1] == 'y' || forc1[3] == 'y' || forc1[5] == 'y'){
				if (axis == 'y'){ axis3 = 'y'; }
				if (axis2 == 'y'){ axis4 = 'y'; }
				if (axis0 == 'z'){ axis5 = 'z'; }
			}
			if (forc1[1] == 'z' || forc1[3] == 'z' || forc1[5] == 'z'){
				if (axis == 'z'){ axis3 = 'z'; }
				if (axis2 == 'z'){ axis4 = 'z'; }
				if (axis0 == 'z'){ axis5 = 'z'; }
			}
			}

		ifstream stepdynamicforces(resultDir2 + "/FEBstepdynamic_forces" + itter + ".txt");
		ifstream stepstaticforces(resultDir2 + "/FEBstepstatic_forces" + itter + ".txt");

		ofstream stepdynamic2(resultDir2 + "/FEBstepdynamic_forces2" + itter + ".txt");
		ofstream stepstatic2(resultDir2 + "/FEBstepstatic_forces2" + itter + ".txt");

		string line1,line2;
		string d1 = "R" + axis3;
		string d2 = "R" + axis4;
		string d3 = "R" + axis5;
		while (!stepdynamicforces.eof()){

			getline(stepdynamicforces, line1);
			
			////process every line///////
			string modd = line1;
			size_t found1 = modd.find(d1);
			if (found1 != NAN){ modd.erase(modd.begin(), modd.end()); line1 = modd; cout << line1 << endl; }
			size_t found21 = modd.find(d2);
			if (found21 != NAN){ modd.erase(modd.begin(), modd.end()); line1 = modd; cout << line1 << endl; }
			size_t found211 = modd.find(d3);
			if (found211 != NAN){ modd.erase(modd.begin(), modd.end()); line1 = modd; cout << line1 << endl; }

			stepdynamic2 << line1 << endl;
		}

		while (!stepstaticforces.eof()){

			getline(stepstaticforces, line2);

			////process every line///////
			string modd2 = line2;
			size_t found = modd2.find(d1);
			if (found != NAN){ modd2.erase(modd2.begin(), modd2.end()); line2 = modd2; cout << line2 << endl; }
			size_t found2 = modd2.find(d2);
			if (found2 != NAN){ modd2.erase(modd2.begin(), modd2.end()); line2 = modd2; cout << line2 << endl; }
			size_t found212 = modd2.find(d3);
			if (found212 != NAN){ modd2.erase(modd2.begin(), modd2.end()); line2 = modd2; cout << line2 << endl; }

			stepstatic2 << line2 << endl;
		}


///////////////////////////////////////////////////////////////////

		//////write the  joints in the txt files...

		
		int numi=0;
		char bodyn = bodynum + '0';
		ofstream joint(resultDir2 + "/joints" + bodyn + itter + ".txt", ofstream::out);
		ofstream jointd(resultDir2 + "/jointd" + bodyn + itter + ".txt", ofstream::out);
				
		
		if (bodynum == 1){
			if (presc1f[0] == 'N')
			{
				if (axis != 'n'){
					joint << "<constraint type =" << '"' << "rigid revolute joint" << '"' << "name = " << '"' << "Joint 1" << '"' << ">" << endl;
					if (axis3 == axis){
						joint << "	<tolerance>0.1</tolerance>" << endl;
					}
					if (axis3 != axis){
						joint << "	<tolerance>0</tolerance>" << endl;
					}
					joint << "	<gaptol>1e-4</gaptol>" << endl;
					joint << "	<angtol>1e-4</angtol>" << endl;
					joint << "	<force_penalty>1e12</force_penalty>" << endl;
					joint << "	<moment_penalty>1e12</moment_penalty>" << endl;
					joint << "	<body_a>2</body_a>" << endl;
					joint << "	<body_b>1</body_b>" << endl;
					joint << "	<joint_origin>0, 0, 0</joint_origin>" << endl;
					if (axis == 'x'){
						joint << "	<rotation_axis>1, 0, 0</rotation_axis>" << endl;
					}
					if (axis == 'y'){
						joint << "	<rotation_axis>0, 1, 0</rotation_axis>" << endl;
					}
					if (axis == 'z'){
						joint << "	<rotation_axis>0, 0, 1</rotation_axis>" << endl;
					}
					joint << "<prescribed_rotation>0</prescribed_rotation>" << endl;
					if (axis3 == axis){
						if (axis == 'x'){ numi = 4; }
						if (axis == 'y'){ numi = 5; }
						if (axis == 'z'){ numi = 6; }
						joint << "<moment lc =" << '"' << numi << '"' << ">1</moment>" << endl;
					}
					joint << "	</constraint>" << endl;
					//////////set fixed the free of joint////////
					joint << "<rigid_body mat = " << '"' << "1" << '"' << ">" << endl;
					joint << "	<fixed bc= " << '"' << "R" << axis << '"' << "/>" << endl;
					joint << "	</rigid_body>" << endl;
				}
				if (axis2 != 'n'){
					joint << "<constraint type =" << '"' << "rigid revolute joint" << '"' << "name = " << '"' << "Joint 2" << '"' << ">" << endl;
					if (axis4 == axis2){
						joint << "	<tolerance>0.1</tolerance>" << endl;
					}
					if (axis4 != axis2){
						joint << "	<tolerance>0</tolerance>" << endl;
					}joint << "	<gaptol>1e-4</gaptol>" << endl;
					joint << "	<angtol>1e-4</angtol>" << endl;
					joint << "	<force_penalty>1e12</force_penalty>" << endl;
					joint << "	<moment_penalty>1e12</moment_penalty>" << endl;
					joint << "	<body_a>2</body_a>" << endl;
					joint << "	<body_b>1</body_b>" << endl;
					joint << "	<joint_origin>0, 0, 0</joint_origin>" << endl;
					if (axis2 == 'x'){
						joint << "	<rotation_axis>1, 0, 0</rotation_axis>" << endl;
					}
					if (axis2 == 'y'){
						joint << "	<rotation_axis>0, 1, 0</rotation_axis>" << endl;
					}
					if (axis2 == 'z'){
						joint << "	<rotation_axis>0, 0, 1</rotation_axis>" << endl;
					}
					joint << "<prescribed_rotation>0</prescribed_rotation>" << endl;
					if (axis4 == axis2){
						if (axis2 == 'x'){ numi = 4; }
						if (axis2 == 'y'){ numi = 5; }
						if (axis2 == 'z'){ numi = 6; }
						joint << "<moment lc =" << '"' << numi << '"' << ">1</moment>" << endl;
					}
					joint << "	</constraint>" << endl;
					//////////set fixed the free of joint////////
					joint << "<rigid_body mat = " << '"' << "1" << '"' << ">" << endl;
					joint << "	<fixed bc= " << '"' << "R" << axis2 << '"' << "/>" << endl;
					joint << "	</rigid_body>" << endl;
				}
			}
			if (axis0 != 'n'){
				joint << "<constraint type =" << '"' << "rigid revolute joint" << '"' << "name = " << '"' << "Joint 3" << '"' << ">" << endl;
				if (axis5 == axis0){
					joint << "	<tolerance>0.1</tolerance>" << endl;
				}
				if (axis5 != axis0){
					joint << "	<tolerance>0</tolerance>" << endl;
				}joint << "	<gaptol>1e-4</gaptol>" << endl;
				joint << "	<angtol>1e-4</angtol>" << endl;
				joint << "	<force_penalty>1e12</force_penalty>" << endl;
				joint << "	<moment_penalty>1e12</moment_penalty>" << endl;
				joint << "	<body_a>2</body_a>" << endl;
				joint << "	<body_b>1</body_b>" << endl;
				joint << "	<joint_origin>0, 0, 0</joint_origin>" << endl;
				if (axis0 == 'x'){
					joint << "	<rotation_axis>1, 0, 0</rotation_axis>" << endl;
				}
				if (axis0 == 'y'){
					joint << "	<rotation_axis>0, 1, 0</rotation_axis>" << endl;
				}
				if (axis0 == 'z'){
					joint << "	<rotation_axis>0, 0, 1</rotation_axis>" << endl;
				}
				joint << "<prescribed_rotation>0</prescribed_rotation>" << endl;
				if (axis5 == axis0){
					if (axis0 == 'x'){ numi = 4; }
					if (axis0 == 'y'){ numi = 5; }
					if (axis0 == 'z'){ numi = 6; }
					joint << "<moment lc =" << '"' << numi << '"' << ">1</moment>" << endl;
				}
				joint << "	</constraint>" << endl;
				//////////set fixed the free of joint////////
				joint << "<rigid_body mat = " << '"' << "1" << '"' << ">" << endl;
				joint << "	<fixed bc= " << '"' << "R" << axis0 << '"' << "/>" << endl;
				joint << "	</rigid_body>" << endl;
			}


			if (number == 2){
				if (axis != 'n'){
					joint << "<constraint type =" << '"' << "rigid revolute joint" << '"' << "name = " << '"' << "Joint 1" << '"' << ">" << endl;
					if (axis3 == axis){
						joint << "	<tolerance>0.1</tolerance>" << endl;
					}
					if (axis3 != axis){
						joint << "	<tolerance>0</tolerance>" << endl;
					}
					joint << "	<gaptol>1e-4</gaptol>" << endl;
					joint << "	<angtol>1e-4</angtol>" << endl;
					joint << "	<force_penalty>1e12</force_penalty>" << endl;
					joint << "	<moment_penalty>1e12</moment_penalty>" << endl;
					joint << "	<body_a>2</body_a>" << endl;
					joint << "	<body_b>1</body_b>" << endl;
					joint << "	<joint_origin>0, 0, 0</joint_origin>" << endl;
					if (axis == 'x'){
						joint << "	<rotation_axis>1, 0, 0</rotation_axis>" << endl;
					}
					if (axis == 'y'){
						joint << "	<rotation_axis>0, 1, 0</rotation_axis>" << endl;
					}
					if (axis == 'z'){
						joint << "	<rotation_axis>0, 0, 1</rotation_axis>" << endl;
					}
					joint << "<prescribed_rotation>0</prescribed_rotation>" << endl;
					if (axis3 == axis){
						if (axis == 'x'){ numi = 4; }
						if (axis == 'y'){ numi = 5; }
						if (axis == 'z'){ numi = 6; }
						joint << "<moment lc =" << '"' << numi << '"' << ">1</moment>" << endl;
					}
					joint << "	</constraint>" << endl;
					//////////set fixed the free of joint////////
					joint << "<rigid_body mat = " << '"' << "1" << '"' << ">" << endl;
					joint << "	<fixed bc= " << '"' << "R" << axis << '"' << "/>" << endl;
					joint << "	</rigid_body>" << endl;
				}
			}
			if (number == 1){
				if (axis != 'n'){
					joint << "<constraint type =" << '"' << "rigid revolute joint" << '"' << "name = " << '"' << "Joint 1" << '"' << ">" << endl;
					if (axis3 == axis){
						joint << "	<tolerance>0.1</tolerance>" << endl;
					}
					if (axis3 != axis){
						joint << "	<tolerance>0</tolerance>" << endl;
					}
					joint << "	<gaptol>1e-4</gaptol>" << endl;
					joint << "	<angtol>1e-4</angtol>" << endl;
					joint << "	<force_penalty>1e12</force_penalty>" << endl;
					joint << "	<moment_penalty>1e12</moment_penalty>" << endl;
					joint << "	<body_a>2</body_a>" << endl;
					joint << "	<body_b>1</body_b>" << endl;
					joint << "	<joint_origin>0, 0, 0</joint_origin>" << endl;
					if (axis == 'x'){
						joint << "	<rotation_axis>1, 0, 0</rotation_axis>" << endl;
					}
					if (axis == 'y'){
						joint << "	<rotation_axis>0, 1, 0</rotation_axis>" << endl;
					}
					if (axis == 'z'){
						joint << "	<rotation_axis>0, 0, 1</rotation_axis>" << endl;
					}
					joint << "<prescribed_rotation>0</prescribed_rotation>" << endl;
					if (axis3 == axis){
						if (axis == 'x'){ numi = 4; }
						if (axis == 'y'){ numi = 5; }
						if (axis == 'z'){ numi = 6; }
						joint << "<moment lc =" << '"' << numi << '"' << ">1</moment>" << endl;
					}
					joint << "	</constraint>" << endl;
					//////////set fixed the free of joint////////
					joint << "<rigid_body mat = " << '"' << "1" << '"' << ">" << endl;
					joint << "	<fixed bc= " << '"' << "R" << axis << '"' << "/>" << endl;
					joint << "	</rigid_body>" << endl;
				}
				if (axis2 != 'n'){
					joint << "<constraint type =" << '"' << "rigid revolute joint" << '"' << "name = " << '"' << "Joint 2" << '"' << ">" << endl;
					if (axis4 == axis2){
						joint << "	<tolerance>0.1</tolerance>" << endl;
					}
					if (axis4 != axis2){
						joint << "	<tolerance>0</tolerance>" << endl;
					}joint << "	<gaptol>1e-4</gaptol>" << endl;
					joint << "	<angtol>1e-4</angtol>" << endl;
					joint << "	<force_penalty>1e12</force_penalty>" << endl;
					joint << "	<moment_penalty>1e12</moment_penalty>" << endl;
					joint << "	<body_a>2</body_a>" << endl;
					joint << "	<body_b>1</body_b>" << endl;
					joint << "	<joint_origin>0, 0, 0</joint_origin>" << endl;
					if (axis2 == 'x'){
						joint << "	<rotation_axis>1, 0, 0</rotation_axis>" << endl;
					}
					if (axis2 == 'y'){
						joint << "	<rotation_axis>0, 1, 0</rotation_axis>" << endl;
					}
					if (axis2 == 'z'){
						joint << "	<rotation_axis>0, 0, 1</rotation_axis>" << endl;
					}
					joint << "<prescribed_rotation>0</prescribed_rotation>" << endl;
					if (axis4 == axis2){
						if (axis2 == 'x'){ numi = 4; }
						if (axis2 == 'y'){ numi = 5; }
						if (axis2 == 'z'){ numi = 6; }
						joint << "<moment lc =" << '"' << numi << '"' << ">1</moment>" << endl;
					}
					joint << "	</constraint>" << endl;
					//////////set fixed the free of joint////////
					joint << "<rigid_body mat = " << '"' << "1" << '"' << ">" << endl;
					joint << "	<fixed bc= " << '"' << "R" << axis2 << '"' << "/>" << endl;
					joint << "	</rigid_body>" << endl;
				}
			}
		}
				if (bodynum == 2){
					if (presc2f[0] == 'N')
					{
						if (axis != 'n'){
							joint << "<constraint type =" << '"' << "rigid revolute joint" << '"' << "name = " << '"' << "Joint 4" << '"' << ">" << endl;
							if (axis3 == axis){
								joint << "	<tolerance>0.1</tolerance>" << endl;
							}
							if (axis3 != axis){
								joint << "	<tolerance>0</tolerance>" << endl;
							}
							joint << "	<gaptol>1e-4</gaptol>" << endl;
							joint << "	<angtol>1e-4</angtol>" << endl;
							joint << "	<force_penalty>1e12</force_penalty>" << endl;
							joint << "	<moment_penalty>1e12</moment_penalty>" << endl;
							joint << "	<body_a>1</body_a>" << endl;
							joint << "	<body_b>2</body_b>" << endl;
							joint << "	<joint_origin>0, 0, 0</joint_origin>" << endl;
							if (axis == 'x'){
								joint << "	<rotation_axis>1, 0, 0</rotation_axis>" << endl;
							}
							if (axis == 'y'){
								joint << "	<rotation_axis>0, 1, 0</rotation_axis>" << endl;
							}
							if (axis == 'z'){
								joint << "	<rotation_axis>0, 0, 1</rotation_axis>" << endl;
							}
							joint << "<prescribed_rotation>0</prescribed_rotation>" << endl;
							if (axis3 == axis){
								if (axis == 'x'){ numi = 10; }
								if (axis == 'y'){ numi = 11; }
								if (axis == 'z'){ numi = 12; }
								joint << "<moment lc =" << '"' << numi << '"' << ">1</moment>" << endl;
							}
							joint << "	</constraint>" << endl;
							//////////set fixed the free of joint////////
							joint << "<rigid_body mat = " << '"' << "2" << '"' << ">" << endl;
							joint << "	<fixed bc= " << '"' << "R" << axis << '"' << "/>" << endl;
							joint << "	</rigid_body>" << endl;
						}
						if (axis2 != 'n'){
							joint << "<constraint type =" << '"' << "rigid revolute joint" << '"' << "name = " << '"' << "Joint 5" << '"' << ">" << endl;
							if (axis4 == axis2){
								joint << "	<tolerance>0.1</tolerance>" << endl;
							}
							if (axis4 != axis2){
								joint << "	<tolerance>0</tolerance>" << endl;
							}joint << "	<gaptol>1e-4</gaptol>" << endl;
							joint << "	<angtol>1e-4</angtol>" << endl;
							joint << "	<force_penalty>1e12</force_penalty>" << endl;
							joint << "	<moment_penalty>1e12</moment_penalty>" << endl;
							joint << "	<body_a>1</body_a>" << endl;
							joint << "	<body_b>2</body_b>" << endl;
							joint << "	<joint_origin>0, 0, 0</joint_origin>" << endl;
							if (axis2 == 'x'){
								joint << "	<rotation_axis>1, 0, 0</rotation_axis>" << endl;
							}
							if (axis2 == 'y'){
								joint << "	<rotation_axis>0, 1, 0</rotation_axis>" << endl;
							}
							if (axis2 == 'z'){
								joint << "	<rotation_axis>0, 0, 1</rotation_axis>" << endl;
							}
							joint << "<prescribed_rotation>0</prescribed_rotation>" << endl;
							if (axis4 == axis2){
								if (axis2 == 'x'){ numi = 10; }
								if (axis2 == 'y'){ numi = 11; }
								if (axis2 == 'z'){ numi = 12; }
								joint << "<moment lc =" << '"' << numi << '"' << ">1</moment>" << endl;
							}
							joint << "	</constraint>" << endl;
							//////////set fixed the free of joint////////
							joint << "<rigid_body mat = " << '"' << "2" << '"' << ">" << endl;
							joint << "	<fixed bc= " << '"' << "R" << axis2 << '"' << "/>" << endl;
							joint << "	</rigid_body>" << endl;
						}
					}
					if (axis0 != 'n'){
						joint << "<constraint type =" << '"' << "rigid revolute joint" << '"' << "name = " << '"' << "Joint 6" << '"' << ">" << endl;
						if (axis5 == axis0){
							joint << "	<tolerance>0.1</tolerance>" << endl;
						}
						if (axis5 != axis0){
							joint << "	<tolerance>0</tolerance>" << endl;
						}joint << "	<gaptol>1e-4</gaptol>" << endl;
						joint << "	<angtol>1e-4</angtol>" << endl;
						joint << "	<force_penalty>1e12</force_penalty>" << endl;
						joint << "	<moment_penalty>1e12</moment_penalty>" << endl;
						joint << "	<body_a>1</body_a>" << endl;
						joint << "	<body_b>2</body_b>" << endl;
						joint << "	<joint_origin>0, 0, 0</joint_origin>" << endl;
						if (axis0 == 'x'){
							joint << "	<rotation_axis>1, 0, 0</rotation_axis>" << endl;
						}
						if (axis0 == 'y'){
							joint << "	<rotation_axis>0, 1, 0</rotation_axis>" << endl;
						}
						if (axis0 == 'z'){
							joint << "	<rotation_axis>0, 0, 1</rotation_axis>" << endl;
						}
						joint << "<prescribed_rotation>0</prescribed_rotation>" << endl;
						if (axis5 == axis0){
							if (axis0 == 'x'){ numi = 10; }
							if (axis0 == 'y'){ numi = 11; }
							if (axis0 == 'z'){ numi = 12; }
							joint << "<moment lc =" << '"' << numi << '"' << ">1</moment>" << endl;
						}
						joint << "	</constraint>" << endl;
						//////////set fixed the free of joint////////
						joint << "<rigid_body mat = " << '"' << "2" << '"' << ">" << endl;
						joint << "	<fixed bc= " << '"' << "R" << axis0 << '"' << "/>" << endl;
						joint << "	</rigid_body>" << endl;
					}
					if (number == 2){
						if (axis != 'n'){
							joint << "<constraint type =" << '"' << "rigid revolute joint" << '"' << "name = " << '"' << "Joint 4" << '"' << ">" << endl;
							if (axis3 == axis){
								joint << "	<tolerance>0.1</tolerance>" << endl;
							}
							if (axis3 != axis){
								joint << "	<tolerance>0</tolerance>" << endl;
							}
							joint << "	<gaptol>1e-4</gaptol>" << endl;
							joint << "	<angtol>1e-4</angtol>" << endl;
							joint << "	<force_penalty>1e12</force_penalty>" << endl;
							joint << "	<moment_penalty>1e12</moment_penalty>" << endl;
							joint << "	<body_a>1</body_a>" << endl;
							joint << "	<body_b>2</body_b>" << endl;
							joint << "	<joint_origin>0, 0, 0</joint_origin>" << endl;
							if (axis == 'x'){
								joint << "	<rotation_axis>1, 0, 0</rotation_axis>" << endl;
							}
							if (axis == 'y'){
								joint << "	<rotation_axis>0, 1, 0</rotation_axis>" << endl;
							}
							if (axis == 'z'){
								joint << "	<rotation_axis>0, 0, 1</rotation_axis>" << endl;
							}
							joint << "<prescribed_rotation>0</prescribed_rotation>" << endl;
							if (axis3 == axis){
								if (axis == 'x'){ numi = 10; }
								if (axis == 'y'){ numi = 11; }
								if (axis == 'z'){ numi = 12; }
								joint << "<moment lc =" << '"' << numi << '"' << ">1</moment>" << endl;
							}
							joint << "	</constraint>" << endl;
							//////////set fixed the free of joint////////
							joint << "<rigid_body mat = " << '"' << "2" << '"' << ">" << endl;
							joint << "	<fixed bc= " << '"' << "R" << axis << '"' << "/>" << endl;
							joint << "	</rigid_body>" << endl;
						}
					}
					if (number == 1){
						if (axis != 'n'){
							joint << "<constraint type =" << '"' << "rigid revolute joint" << '"' << "name = " << '"' << "Joint 4" << '"' << ">" << endl;
							if (axis3 == axis){
								joint << "	<tolerance>0.1</tolerance>" << endl;
							}
							if (axis3 != axis){
								joint << "	<tolerance>0</tolerance>" << endl;
							}
							joint << "	<gaptol>1e-4</gaptol>" << endl;
							joint << "	<angtol>1e-4</angtol>" << endl;
							joint << "	<force_penalty>1e12</force_penalty>" << endl;
							joint << "	<moment_penalty>1e12</moment_penalty>" << endl;
							joint << "	<body_a>1</body_a>" << endl;
							joint << "	<body_b>2</body_b>" << endl;
							joint << "	<joint_origin>0, 0, 0</joint_origin>" << endl;
							if (axis == 'x'){
								joint << "	<rotation_axis>1, 0, 0</rotation_axis>" << endl;
							}
							if (axis == 'y'){
								joint << "	<rotation_axis>0, 1, 0</rotation_axis>" << endl;
							}
							if (axis == 'z'){
								joint << "	<rotation_axis>0, 0, 1</rotation_axis>" << endl;
							}
							joint << "<prescribed_rotation>0</prescribed_rotation>" << endl;
							if (axis3 == axis){
								if (axis == 'x'){ numi = 10; }
								if (axis == 'y'){ numi = 11; }
								if (axis == 'z'){ numi = 12; }
								joint << "<moment lc =" << '"' << numi << '"' << ">1</moment>" << endl;
							}
							joint << "	</constraint>" << endl;
							//////////set fixed the free of joint////////
							joint << "<rigid_body mat = " << '"' << "2" << '"' << ">" << endl;
							joint << "	<fixed bc= " << '"' << "R" << axis << '"' << "/>" << endl;
							joint << "	</rigid_body>" << endl;
						}
						if (axis2 != 'n'){
							joint << "<constraint type =" << '"' << "rigid revolute joint" << '"' << "name = " << '"' << "Joint 5" << '"' << ">" << endl;
							if (axis4 == axis2){
								joint << "	<tolerance>0.1</tolerance>" << endl;
							}
							if (axis4 != axis2){
								joint << "	<tolerance>0</tolerance>" << endl;
							}
							joint << "	<gaptol>1e-4</gaptol>" << endl;
							joint << "	<angtol>1e-4</angtol>" << endl;
							joint << "	<force_penalty>1e12</force_penalty>" << endl;
							joint << "	<moment_penalty>1e12</moment_penalty>" << endl;
							joint << "	<body_a>1</body_a>" << endl;
							joint << "	<body_b>2</body_b>" << endl;
							joint << "	<joint_origin>0, 0, 0</joint_origin>" << endl;
							if (axis2 == 'x'){
								joint << "	<rotation_axis>1, 0, 0</rotation_axis>" << endl;
							}
							if (axis2 == 'y'){
								joint << "	<rotation_axis>0, 1, 0</rotation_axis>" << endl;
							}
							if (axis2 == 'z'){
								joint << "	<rotation_axis>0, 0, 1</rotation_axis>" << endl;
							}
							joint << "<prescribed_rotation>0</prescribed_rotation>" << endl;
							if (axis4 == axis2){
								if (axis2 == 'x'){ numi = 10; }
								if (axis2 == 'y'){ numi = 11; }
								if (axis2 == 'z'){ numi = 12; } 
								joint << "<moment lc =" << '"' << numi << '"' << ">1</moment>" << endl;
							}
							joint << "	</constraint>" << endl;
							//////////set fixed the free of joint////////
							joint << "<rigid_body mat = " << '"' << "2" << '"' << ">" << endl;
							joint << "	<fixed bc= " << '"' << "R" << axis2 << '"' << "/>" << endl;
							joint << "	</rigid_body>" << endl;
						}
					}
				}


				///////////////////////DYNAMIC STEP///////////////////////////////////////
				int numii = 0;
				if (bodynum == 1){
					if (presc1f[0] == 'N')
					{
						if (axis != 'n'){
							jointd << "<constraint type =" << '"' << "rigid revolute joint" << '"' << "name = " << '"' << "Joint 1" << '"' << ">" << endl;
							if (axis3 == axis){
								jointd << "	<tolerance>0.1</tolerance>" << endl;
							}
							if (axis3 != axis){
								jointd << "	<tolerance>0</tolerance>" << endl;
							}
							jointd << "	<gaptol>1e-4</gaptol>" << endl;
							jointd << "	<angtol>1e-4</angtol>" << endl;
							jointd << "	<force_penalty>1e12</force_penalty>" << endl;
							jointd << "	<moment_penalty>1e12</moment_penalty>" << endl;
							jointd << "	<body_a>2</body_a>" << endl;
							jointd << "	<body_b>1</body_b>" << endl;
							jointd << "	<joint_origin>0, 0, 0</joint_origin>" << endl;
							if (axis == 'x'){
								jointd << "	<rotation_axis>1, 0, 0</rotation_axis>" << endl;
							}
							if (axis == 'y'){
								jointd << "	<rotation_axis>0, 1, 0</rotation_axis>" << endl;
							}
							if (axis == 'z'){
								jointd << "	<rotation_axis>0, 0, 1</rotation_axis>" << endl;
							}
							jointd << "<prescribed_rotation>0</prescribed_rotation>" << endl;
							if (axis3 == axis){
								if (axis == 'x'){ numii = 16; }
								if (axis == 'y'){ numii = 17; }
								if (axis == 'z'){ numii = 18; }
								jointd << "<moment lc =" << '"' << numii << '"' << ">1</moment>" << endl;
							}
							jointd << "	</constraint>" << endl;
							//////////set fixed the free of joint////////
							jointd << "<rigid_body mat = " << '"' << "1" << '"' << ">" << endl;
							jointd << "	<fixed bc= " << '"' << "R" << axis << '"' << "/>" << endl;
							jointd << "	</rigid_body>" << endl;
						}
						if (axis2 != 'n'){
							jointd << "<constraint type =" << '"' << "rigid revolute joint" << '"' << "name = " << '"' << "Joint 2" << '"' << ">" << endl;
							if (axis4 == axis2){
								jointd << "	<tolerance>0.1</tolerance>" << endl;
							}
							if (axis4 != axis2){
								jointd << "	<tolerance>0</tolerance>" << endl;
							}jointd << "	<gaptol>1e-4</gaptol>" << endl;
							jointd << "	<angtol>1e-4</angtol>" << endl;
							jointd << "	<force_penalty>1e12</force_penalty>" << endl;
							jointd << "	<moment_penalty>1e12</moment_penalty>" << endl;
							jointd << "	<body_a>2</body_a>" << endl;
							jointd << "	<body_b>1</body_b>" << endl;
							jointd << "	<joint_origin>0, 0, 0</joint_origin>" << endl;
							if (axis2 == 'x'){
								jointd << "	<rotation_axis>1, 0, 0</rotation_axis>" << endl;
							}
							if (axis2 == 'y'){
								jointd << "	<rotation_axis>0, 1, 0</rotation_axis>" << endl;
							}
							if (axis2 == 'z'){
								jointd << "	<rotation_axis>0, 0, 1</rotation_axis>" << endl;
							}
							jointd << "<prescribed_rotation>0</prescribed_rotation>" << endl;
							if (axis4 == axis2){
								if (axis2 == 'x'){ numii = 16; }
								if (axis2 == 'y'){ numii = 17; }
								if (axis2 == 'z'){ numii = 18; }
								jointd << "<moment lc =" << '"' << numii << '"' << ">1</moment>" << endl;
							}
							jointd << "	</constraint>" << endl;
							//////////set fixed the free of joint////////
							jointd << "<rigid_body mat = " << '"' << "1" << '"' << ">" << endl;
							jointd << "	<fixed bc= " << '"' << "R" << axis2 << '"' << "/>" << endl;
							jointd << "	</rigid_body>" << endl;
						}
					}
					if (axis0 != 'n'){
						jointd << "<constraint type =" << '"' << "rigid revolute joint" << '"' << "name = " << '"' << "Joint 3" << '"' << ">" << endl;
						if (axis5 == axis0){
							jointd << "	<tolerance>0.1</tolerance>" << endl;
						}
						if (axis5 != axis0){
							jointd << "	<tolerance>0</tolerance>" << endl;
						}jointd << "	<gaptol>1e-4</gaptol>" << endl;
						jointd << "	<angtol>1e-4</angtol>" << endl;
						jointd << "	<force_penalty>1e12</force_penalty>" << endl;
						jointd << "	<moment_penalty>1e12</moment_penalty>" << endl;
						jointd << "	<body_a>2</body_a>" << endl;
						jointd << "	<body_b>1</body_b>" << endl;
						jointd << "	<joint_origin>0, 0, 0</joint_origin>" << endl;
						if (axis0 == 'x'){
							jointd << "	<rotation_axis>1, 0, 0</rotation_axis>" << endl;
						}
						if (axis0 == 'y'){
							jointd << "	<rotation_axis>0, 1, 0</rotation_axis>" << endl;
						}
						if (axis0 == 'z'){
							jointd << "	<rotation_axis>0, 0, 1</rotation_axis>" << endl;
						}
						jointd << "<prescribed_rotation>0</prescribed_rotation>" << endl;
						if (axis5 == axis0){
							if (axis0 == 'x'){ numii = 16; }
							if (axis0 == 'y'){ numii = 17; }
							if (axis0 == 'z'){ numii = 18; }
							jointd << "<moment lc =" << '"' << numii << '"' << ">1</moment>" << endl;
						}
						jointd << "	</constraint>" << endl;
						//////////set fixed the free of joint////////
						jointd << "<rigid_body mat = " << '"' << "1" << '"' << ">" << endl;
						jointd << "	<fixed bc= " << '"' << "R" << axis0 << '"' << "/>" << endl;
						jointd << "	</rigid_body>" << endl;
					}
					if (number == 2){
						if (axis != 'n'){
							jointd << "<constraint type =" << '"' << "rigid revolute joint" << '"' << "name = " << '"' << "Joint 1" << '"' << ">" << endl;
							if (axis3 == axis){
								jointd << "	<tolerance>0.1</tolerance>" << endl;
							}
							if (axis3 != axis){
								jointd << "	<tolerance>0</tolerance>" << endl;
							}
							jointd << "	<gaptol>1e-4</gaptol>" << endl;
							jointd << "	<angtol>1e-4</angtol>" << endl;
							jointd << "	<force_penalty>1e12</force_penalty>" << endl;
							jointd << "	<moment_penalty>1e12</moment_penalty>" << endl;
							jointd << "	<body_a>2</body_a>" << endl;
							jointd << "	<body_b>1</body_b>" << endl;
							jointd << "	<joint_origin>0, 0, 0</joint_origin>" << endl;
							if (axis == 'x'){
								jointd << "	<rotation_axis>1, 0, 0</rotation_axis>" << endl;
							}
							if (axis == 'y'){
								jointd << "	<rotation_axis>0, 1, 0</rotation_axis>" << endl;
							}
							if (axis == 'z'){
								jointd << "	<rotation_axis>0, 0, 1</rotation_axis>" << endl;
							}
							jointd << "<prescribed_rotation>0</prescribed_rotation>" << endl;
							if (axis3 == axis){
								if (axis == 'x'){  numii = 16;  }
								if (axis == 'y'){  numii = 17;  }
								if (axis == 'z'){ numii = 18;}
								jointd << "<moment lc =" << '"' << numii << '"' << ">1</moment>" << endl;
							}
							jointd << "	</constraint>" << endl;
							//////////set fixed the free of joint////////
							jointd << "<rigid_body mat = " << '"' << "1" << '"' << ">" << endl;
							jointd << "	<fixed bc= " << '"' << "R" << axis << '"' << "/>" << endl;
							jointd << "	</rigid_body>" << endl;
						}
					}
					if (number == 1){
						if (axis != 'n'){
							jointd << "<constraint type =" << '"' << "rigid revolute joint" << '"' << "name = " << '"' << "Joint 1" << '"' << ">" << endl;
							if (axis3 == axis){
								jointd << "	<tolerance>0.1</tolerance>" << endl;
							}
							if (axis3 != axis){
								jointd << "	<tolerance>0</tolerance>" << endl;
							}
							jointd << "	<gaptol>1e-4</gaptol>" << endl;
							jointd << "	<angtol>1e-4</angtol>" << endl;
							jointd << "	<force_penalty>1e12</force_penalty>" << endl;
							jointd << "	<moment_penalty>1e12</moment_penalty>" << endl;
							jointd << "	<body_a>2</body_a>" << endl;
							jointd << "	<body_b>1</body_b>" << endl;
							jointd << "	<joint_origin>0, 0, 0</joint_origin>" << endl;
							if (axis == 'x'){
								jointd << "	<rotation_axis>1, 0, 0</rotation_axis>" << endl;
							}
							if (axis == 'y'){
								jointd << "	<rotation_axis>0, 1, 0</rotation_axis>" << endl;
							}
							if (axis == 'z'){
								jointd << "	<rotation_axis>0, 0, 1</rotation_axis>" << endl;
							}
							jointd << "<prescribed_rotation>0</prescribed_rotation>" << endl;
							if (axis3 == axis){
								if (axis == 'x'){ numii = 16; }
								if (axis == 'y'){ numii = 17; }
								if (axis == 'z'){ numii = 18; }
								jointd << "<moment lc =" << '"' << numii << '"' << ">1</moment>" << endl;
							}
							jointd << "	</constraint>" << endl;
							//////////set fixed the free of joint////////
							jointd << "<rigid_body mat = " << '"' << "1" << '"' << ">" << endl;
							jointd << "	<fixed bc= " << '"' << "R" << axis << '"' << "/>" << endl;
							jointd << "	</rigid_body>" << endl;
						}
						if (axis2 != 'n'){
							jointd << "<constraint type =" << '"' << "rigid revolute joint" << '"' << "name = " << '"' << "Joint 2" << '"' << ">" << endl;
							if (axis4 == axis2){
								jointd << "	<tolerance>0.1</tolerance>" << endl;
							}
							if (axis4 != axis2){
								jointd << "	<tolerance>0</tolerance>" << endl;
							}jointd << "	<gaptol>1e-4</gaptol>" << endl;
							jointd << "	<angtol>1e-4</angtol>" << endl;
							jointd << "	<force_penalty>1e12</force_penalty>" << endl;
							jointd << "	<moment_penalty>1e12</moment_penalty>" << endl;
							jointd << "	<body_a>2</body_a>" << endl;
							jointd << "	<body_b>1</body_b>" << endl;
							jointd << "	<joint_origin>0, 0, 0</joint_origin>" << endl;
							if (axis2 == 'x'){
								jointd << "	<rotation_axis>1, 0, 0</rotation_axis>" << endl;
							}
							if (axis2 == 'y'){
								jointd << "	<rotation_axis>0, 1, 0</rotation_axis>" << endl;
							}
							if (axis2 == 'z'){
								jointd << "	<rotation_axis>0, 0, 1</rotation_axis>" << endl;
							}
							jointd << "<prescribed_rotation>0</prescribed_rotation>" << endl;
							if (axis4 == axis2){
								if (axis2 == 'x'){ numii = 16; }
								if (axis2 == 'y'){ numii = 17; }
								if (axis2 == 'z'){ numii = 18; }
								jointd << "<moment lc =" << '"' << numii << '"' << ">1</moment>" << endl;
							}
							jointd << "	</constraint>" << endl;
							//////////set fixed the free of joint////////
							jointd << "<rigid_body mat = " << '"' << "1" << '"' << ">" << endl;
							jointd << "	<fixed bc= " << '"' << "R" << axis2 << '"' << "/>" << endl;
							jointd << "	</rigid_body>" << endl;
						}
					}
				}
				if (bodynum == 2){
					if (presc2f[0] == 'N')
					{
						if (axis != 'n'){
							jointd << "<constraint type =" << '"' << "rigid revolute joint" << '"' << "name = " << '"' << "Joint 4" << '"' << ">" << endl;
							if (axis3 == axis){
								jointd << "	<tolerance>0.1</tolerance>" << endl;
							}
							if (axis3 != axis){
								jointd << "	<tolerance>0</tolerance>" << endl;
							}
							jointd << "	<gaptol>1e-4</gaptol>" << endl;
							jointd << "	<angtol>1e-4</angtol>" << endl;
							jointd << "	<force_penalty>1e12</force_penalty>" << endl;
							jointd << "	<moment_penalty>1e12</moment_penalty>" << endl;
							jointd << "	<body_a>1</body_a>" << endl;
							jointd << "	<body_b>2</body_b>" << endl;
							jointd << "	<joint_origin>0, 0, 0</joint_origin>" << endl;
							if (axis == 'x'){
								jointd << "	<rotation_axis>1, 0, 0</rotation_axis>" << endl;
							}
							if (axis == 'y'){
								jointd << "	<rotation_axis>0, 1, 0</rotation_axis>" << endl;
							}
							if (axis == 'z'){
								jointd << "	<rotation_axis>0, 0, 1</rotation_axis>" << endl;
							}
							jointd << "<prescribed_rotation>0</prescribed_rotation>" << endl;
							if (axis3 == axis){
								if (axis == 'x'){ numii = 22; }
								if (axis == 'y'){ numii = 23; }
								if (axis == 'z'){ numii = 24; }
								jointd << "<moment lc =" << '"' << numii << '"' << ">1</moment>" << endl;
							}
							jointd << "	</constraint>" << endl;
							//////////set fixed the free of joint////////
							jointd << "<rigid_body mat = " << '"' << "2" << '"' << ">" << endl;
							jointd << "	<fixed bc= " << '"' << "R" << axis << '"' << "/>" << endl;
							jointd << "	</rigid_body>" << endl;
						}
						if (axis2 != 'n'){
							jointd << "<constraint type =" << '"' << "rigid revolute joint" << '"' << "name = " << '"' << "Joint 5" << '"' << ">" << endl;
							if (axis4 == axis2){
								jointd << "	<tolerance>0.1</tolerance>" << endl;
							}
							if (axis4 != axis2){
								jointd << "	<tolerance>0</tolerance>" << endl;
							}jointd << "	<gaptol>1e-4</gaptol>" << endl;
							jointd << "	<angtol>1e-4</angtol>" << endl;
							jointd << "	<force_penalty>1e12</force_penalty>" << endl;
							jointd << "	<moment_penalty>1e12</moment_penalty>" << endl;
							jointd << "	<body_a>1</body_a>" << endl;
							jointd << "	<body_b>2</body_b>" << endl;
							jointd << "	<joint_origin>0, 0, 0</joint_origin>" << endl;
							if (axis2 == 'x'){
								jointd << "	<rotation_axis>1, 0, 0</rotation_axis>" << endl;
							}
							if (axis2 == 'y'){
								jointd << "	<rotation_axis>0, 1, 0</rotation_axis>" << endl;
							}
							if (axis2 == 'z'){
								jointd << "	<rotation_axis>0, 0, 1</rotation_axis>" << endl;
							}
							jointd << "<prescribed_rotation>0</prescribed_rotation>" << endl;
							if (axis4 == axis2){
								if (axis2 == 'x'){ numii = 22; }
								if (axis2 == 'y'){ numii = 23; }
								if (axis2 == 'z'){ numii = 24; }
								jointd << "<moment lc =" << '"' << numii << '"' << ">1</moment>" << endl;
							}
							jointd << "	</constraint>" << endl;
							//////////set fixed the free of joint////////
							jointd << "<rigid_body mat = " << '"' << "2" << '"' << ">" << endl;
							jointd << "	<fixed bc= " << '"' << "R" << axis2 << '"' << "/>" << endl;
							jointd << "	</rigid_body>" << endl;
						}
					}
					if (axis0 != 'n'){
						jointd << "<constraint type =" << '"' << "rigid revolute joint" << '"' << "name = " << '"' << "Joint 6" << '"' << ">" << endl;
						if (axis5 == axis0){
							jointd << "	<tolerance>0.1</tolerance>" << endl;
						}
						if (axis5 != axis0){
							jointd << "	<tolerance>0</tolerance>" << endl;
						}jointd << "	<gaptol>1e-4</gaptol>" << endl;
						jointd << "	<angtol>1e-4</angtol>" << endl;
						jointd << "	<force_penalty>1e12</force_penalty>" << endl;
						jointd << "	<moment_penalty>1e12</moment_penalty>" << endl;
						jointd << "	<body_a>1</body_a>" << endl;
						jointd << "	<body_b>2</body_b>" << endl;
						jointd << "	<joint_origin>0, 0, 0</joint_origin>" << endl;
						if (axis0 == 'x'){
							jointd << "	<rotation_axis>1, 0, 0</rotation_axis>" << endl;
						}
						if (axis0 == 'y'){
							jointd << "	<rotation_axis>0, 1, 0</rotation_axis>" << endl;
						}
						if (axis0 == 'z'){
							jointd << "	<rotation_axis>0, 0, 1</rotation_axis>" << endl;
						}
						jointd << "<prescribed_rotation>0</prescribed_rotation>" << endl;
						if (axis5 == axis0){
							if (axis0 == 'x'){ numii = 22; }
							if (axis0 == 'y'){ numii = 23; }
							if (axis0 == 'z'){ numii = 24; }
							jointd << "<moment lc =" << '"' << numii << '"' << ">1</moment>" << endl;
						}
						jointd << "	</constraint>" << endl;
						//////////set fixed the free of joint////////
						jointd << "<rigid_body mat = " << '"' << "2" << '"' << ">" << endl;
						jointd << "	<fixed bc= " << '"' << "R" << axis0 << '"' << "/>" << endl;
						jointd << "	</rigid_body>" << endl;
					}
					if (number == 2){
						if (axis != 'n'){
							jointd << "<constraint type =" << '"' << "rigid revolute joint" << '"' << "name = " << '"' << "Joint 4" << '"' << ">" << endl;
							if (axis3 == axis){
								jointd << "	<tolerance>0.1</tolerance>" << endl;
							}
							if (axis3 != axis){
								jointd << "	<tolerance>0</tolerance>" << endl;
							}
							jointd << "	<gaptol>1e-4</gaptol>" << endl;
							jointd << "	<angtol>1e-4</angtol>" << endl;
							jointd << "	<force_penalty>1e12</force_penalty>" << endl;
							jointd << "	<moment_penalty>1e12</moment_penalty>" << endl;
							jointd << "	<body_a>1</body_a>" << endl;
							jointd << "	<body_b>2</body_b>" << endl;
							jointd << "	<joint_origin>0, 0, 0</joint_origin>" << endl;
							if (axis == 'x'){
								jointd << "	<rotation_axis>1, 0, 0</rotation_axis>" << endl;
							}
							if (axis == 'y'){
								jointd << "	<rotation_axis>0, 1, 0</rotation_axis>" << endl;
							}
							if (axis == 'z'){
								jointd << "	<rotation_axis>0, 0, 1</rotation_axis>" << endl;
							}
							jointd << "<prescribed_rotation>0</prescribed_rotation>" << endl;
							if (axis3 == axis){
								if (axis == 'x'){ numii = 22; }
								if (axis == 'y'){ numii = 23; }
								if (axis == 'z'){ numii = 24; }
								jointd << "<moment lc =" << '"' << numii << '"' << ">1</moment>" << endl;
							}
							jointd << "	</constraint>" << endl;
							//////////set fixed the free of joint////////
							jointd << "<rigid_body mat = " << '"' << "2" << '"' << ">" << endl;
							jointd << "	<fixed bc= " << '"' << "R" << axis << '"' << "/>" << endl;
							jointd << "	</rigid_body>" << endl;
						}
					}
					if (number == 1){
						if (axis != 'n'){
							jointd << "<constraint type =" << '"' << "rigid revolute joint" << '"' << "name = " << '"' << "Joint 4" << '"' << ">" << endl;
							if (axis3 == axis){
								jointd << "	<tolerance>0.1</tolerance>" << endl;
							}
							if (axis3 != axis){
								jointd << "	<tolerance>0</tolerance>" << endl;
							}
							jointd << "	<gaptol>1e-4</gaptol>" << endl;
							jointd << "	<angtol>1e-4</angtol>" << endl;
							jointd << "	<force_penalty>1e12</force_penalty>" << endl;
							jointd << "	<moment_penalty>1e12</moment_penalty>" << endl;
							jointd << "	<body_a>1</body_a>" << endl;
							jointd << "	<body_b>2</body_b>" << endl;
							jointd << "	<joint_origin>0, 0, 0</joint_origin>" << endl;
							if (axis == 'x'){
								jointd << "	<rotation_axis>1, 0, 0</rotation_axis>" << endl;
							}
							if (axis == 'y'){
								jointd << "	<rotation_axis>0, 1, 0</rotation_axis>" << endl;
							}
							if (axis == 'z'){
								jointd << "	<rotation_axis>0, 0, 1</rotation_axis>" << endl;
							}
							jointd << "<prescribed_rotation>0</prescribed_rotation>" << endl;
							if (axis3 == axis){
								if (axis == 'x'){ numii = 22; }
								if (axis == 'y'){ numii = 23; }
								if (axis == 'z'){ numii = 24; }
								jointd << "<moment lc =" << '"' << numii << '"' << ">1</moment>" << endl;
							}
							jointd << "	</constraint>" << endl;
							//////////set fixed the free of joint////////
							jointd << "<rigid_body mat = " << '"' << "2" << '"' << ">" << endl;
							jointd << "	<fixed bc= " << '"' << "R" << axis << '"' << "/>" << endl;
							jointd << "	</rigid_body>" << endl;
						}
						if (axis2 != 'n'){
							jointd << "<constraint type =" << '"' << "rigid revolute joint" << '"' << "name = " << '"' << "Joint 5" << '"' << ">" << endl;
							if (axis4 == axis2){
								jointd << "	<tolerance>0.1</tolerance>" << endl;
							}
							if (axis4 != axis2){
								jointd << "	<tolerance>0</tolerance>" << endl;
							}
							jointd << "	<gaptol>1e-4</gaptol>" << endl;
							jointd << "	<angtol>1e-4</angtol>" << endl;
							jointd << "	<force_penalty>1e12</force_penalty>" << endl;
							jointd << "	<moment_penalty>1e12</moment_penalty>" << endl;
							jointd << "	<body_a>1</body_a>" << endl;
							jointd << "	<body_b>2</body_b>" << endl;
							jointd << "	<joint_origin>0, 0, 0</joint_origin>" << endl;
							if (axis2 == 'x'){
								jointd << "	<rotation_axis>1, 0, 0</rotation_axis>" << endl;
							}
							if (axis2 == 'y'){
								jointd << "	<rotation_axis>0, 1, 0</rotation_axis>" << endl;
							}
							if (axis2 == 'z'){
								jointd << "	<rotation_axis>0, 0, 1</rotation_axis>" << endl;
							}
							jointd << "<prescribed_rotation>0</prescribed_rotation>" << endl;
							if (axis4 == axis2){
								if (axis2 == 'x'){ numii = 22; } 
								if (axis2 == 'y'){ numii = 23; } 
								if (axis2 == 'z'){ numii = 24; } 
								jointd << "<moment lc =" << '"' << numii << '"' << ">1</moment>" << endl;
							}
							jointd << "	</constraint>" << endl;
							//////////set fixed the free of joint////////
							jointd << "<rigid_body mat = " << '"' << "2" << '"' << ">" << endl;
							jointd << "	<fixed bc= " << '"' << "R" << axis2 << '"' << "/>" << endl;
							jointd << "	</rigid_body>" << endl;
						}
					}
				}
			
		
		//////////////////////////////////////////////////////////////////////////////
				joint.close();
				jointd.close();
		}
	


HANDLE FEBRunner::ShellExecuteHandler(string program, string args,string name)
{
	cout << "" << endl;
	cout << "START THE FEBIO RUN " << endl;
	//std::wstring w_program = std::wstring(program.begin(), program.end());
	//std::wstring w_args = std::wstring(args.begin(), args.end());
	HANDLE hProcess = NULL;
	SHELLEXECUTEINFO shellInfo;
	::ZeroMemory(&shellInfo, sizeof(shellInfo));
	shellInfo.cbSize = sizeof(shellInfo);
	//shellInfo.fMask = SEE_MASK_FLAG_NO_UI | SEE_MASK_NOCLOSEPROCESS;// for the paraller process i will need it...
	
	shellInfo.fMask = SEE_MASK_FLAG_NO_UI | SEE_MASK_NOCLOSEPROCESS ;
	//cout << args << endl;
	cout << "" << endl;
	cout << "programm: if you want to start the  "<<name << " now press any key..."<<endl;
	system("pause");
	//shellInfo.
	shellInfo.lpVerb = "open";
	shellInfo.lpFile = name.c_str();
	shellInfo.lpParameters =args.c_str();
	shellInfo.lpDirectory = program.c_str();
	shellInfo.nShow = 10;
	//::ShellExecuteEx(&shellInfo);
	if (::ShellExecuteEx(&shellInfo))
	{ // success 
		//cout << "xixixixixix" << endl;
		hProcess = shellInfo.hProcess;
	} // success 
	return hProcess;
}


/*
VOID startup(LPCTSTR lpApplicationName)
{
	// additional information
	STARTUPINFO si;
	PROCESS_INFORMATION pi;

	// set the size of the structures
	ZeroMemory(&si, sizeof(si));
	si.cb = sizeof(si);
	ZeroMemory(&pi, sizeof(pi));

	// start the program up
	CreateProcess(lpApplicationName,   // the path
		argv[1],        // Command line
		NULL,           // Process handle not inheritable
		NULL,           // Thread handle not inheritable
		FALSE,          // Set handle inheritance to FALSE
		0,              // No creation flags
		NULL,           // Use parent's environment block
		NULL,           // Use parent's starting directory 
		&si,            // Pointer to STARTUPINFO structure
		&pi)           // Pointer to PROCESS_INFORMATION structure
		);
		// Close process and thread handles. 
		CloseHandle(pi.hProcess);
		CloseHandle(pi.hThread);
}
*/