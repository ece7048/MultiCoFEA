#include <OpenSim/OpenSim.h>
#include <OpenSim/Analyses/BodyKinematics.h>
#include <OpenSim/Tools/AnalyzeTool.h>
#include <OpenSim/Simulation/SimbodyEngine/Body.h>
#include <stdlib.h>
#include <ctime>
#include <exception>
#include "Settings.h"
#include "INIReader.h"
#include "BF_structor.h"
#include "StaticAnalysis.h"
#include "BodyForceAnalysis.h"
//#include "BodyForceAnalysis1B.h"
#include "BodyForceAnalysis2.h"
//#include "BodyForceAnalysis1B2.h"
#include "BK_structor.h"
#include "TSC.h"
#include "BF_starter.h"
#include "ReSampler.h"
#include <iostream>
#include <fstream>
#include <algorithm>
#include <math.h>
#include "DOFResample.h"
#include "FEBRunner.h"
#include <direct.h>
#include "Initial_VEL.h"
#include "Visualizerfebgeo.h"
#include "CASE_ONE.h"
#include "CASE_TWO.h"


using namespace OpenSim;
using namespace SimTK;
using namespace std;

BF_starter::BF_starter(void)
{
}

BF_starter::~BF_starter(void)
{
}
void BF_starter::started(int number, int itteration, char mode){


	// if the analysis is one number=1 itteration=1....

	INIReader ini = INIReader(INI_FILE);
	ofstream valuenon;
	ofstream valueinn;
	ofstream valpo;
	ofstream valpi;
	//ofstream valueno;
	string modelPath = BASE_DIR + ini.Get("PATH", "MODELS", "");
	string resultDir = BASE_DIR + ini.Get("PATH", "RESULT_DIR", "");
	string resultDir1 = BASE_DIR + ini.Get("PATH", "RESULT_DIR2", "");
	double t0 = ini.GetReal("BODYFORCES", "START_TIME", 0);
	double tf = ini.GetReal("BODYFORCES", "END_TIME", 0);
	string stateFile = BASE_DIR + ini.Get("BODYFORCES", "STATE", "");
	string xmlpath = BASE_DIR + ini.Get("PATH", "XML", "");
	string bodyname1 = ini.Get("BODYFORCES", "BDNAME1", "");
	string bodyname2 = ini.Get("BODYFORCES", "BDNAME2", "");
	string bodyname3 = ini.Get("BODYFORCES", "BDNAME1", "");
	string bodyname4 = ini.Get("BODYFORCES", "BDNAME2", "");
	string joint = ini.Get("BODYFORCES", "JOINT", "");
	string joint1 = ini.Get("BODYFORCES", "JOINT1", "");
	string joint2 = ini.Get("BODYFORCES", "JOINT2", "");
	string joint3 = ini.Get("BODYFORCES", "JOINT3", "");
	string joint4 = ini.Get("BODYFORCES", "JOINT4", "");
	string joint5 = ini.Get("BODYFORCES", "JOINT5", "");
	string joint6 = ini.Get("BODYFORCES", "JOINT6", "");

	//ofstream valueno;
	Model model2(modelPath);
	char itter = itteration + '0';
	////create a new folder for the analysis/////////
	String newfolder = resultDir1 + itter;
	mkdir(newfolder);
	string resultDir2 = newfolder;

	//////////////////////////////////////END//////////////////////////////////////////
	////////////////////////////////////////////////////////////////////////

	//////////STATIC CALIBRATION OF THE POINT OF INTEREST/////////////
	if (itteration == 0){
		cout << " GIVE THE POINT OF INTEREST OF CHILD BODY : " << bodyname2 << "(x/y/z of OpenSim orientation)" << endl;
		cout << "For default value set: d ,  or to give your own value set: nd  " << endl;
		string setcase = ini.Get("BASICSETUP", "Point_of_acting_forces", "");
		if (setcase == ""){
			cout << " PLEASE, GIVE THE POINT OF INTEREST OF CHILD BODY : " << bodyname2 << "(x/y/z of OpenSim orientation)" << endl;
			cin >> setcase;
		}
		cout << setcase << endl;
		double xval;
		double yval;
		double zval;
		if (setcase == "d"){ xval = 0; yval = -0.025; zval = 0; cout << "default point is: 0, -0.025, 0" << endl; }
		if (setcase == "nd"){
			cout << "x-axis value" << endl;
			double  xval = ini.GetReal("BASICSETUP", "xvalue", 0);
			cout << xval << endl;
			cout << "y-axis value" << endl;
			double  yval = ini.GetReal("BASICSETUP", "yvalue", 0);
			cout << yval << endl;
			cout << "z-axis value" << endl;
			double  zval = ini.GetReal("BASICSETUP", "zvalue", 0);
			cout << zval << endl;
		}
		char num = itteration + '0';

		String pathss = resultDir + "/_Staticpoint";
		mkdir(pathss);
		string newpath = pathss;
		ofstream staticpoint;
		staticpoint = ofstream(newpath + "/staticpoint.txt", ofstream::out);
		staticpoint << xval << endl;
		staticpoint << yval << endl;
		staticpoint << zval << endl;

		///////////////////////WRITE THE SET UP FILE WITH THE DIRECTION OF THE STATES FOR THE ANALYSIS ///////////////////
		string residul = "C:/Users/ece/Desktop/Co_SiM_withFEBiocoding/data/";
		string resultDirst = BASE_DIR + ini.Get("INVERSEKINEMATICSTATIC", "RESULT_DIR", "");
		string finaldest = resultDirst + "/stmotion.mot";

		ifstream step1(residul + "setup1.ini");
		ofstream st(residul + "setup.ini");
		string line1;
		string line2;
		string line3;
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
				if (line1.compare("[STATIC]") == 0)
				{
					st << "STATE =" << finaldest << endl;
					except = 2;

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
		/*
		ifstream step21(residul + "setup.ini");
		ofstream st21(residul + "setup2.ini");

		for (int i = 0; i < 200; i++)
		{
		getline(step21, line3);
		st21 << line3 << endl;

		}
		step21.close();
		st21.close();
		*/
		string stateFilestatic = ini.Get("STATIC", "STATE", "");
		double t0s = ini.GetReal("STATIC", "START_TIME", 0);
		double tfs = ini.GetReal("STATIC", "END_TIME", 0.1);

		/*
		////////////////////////////////////////////////////////////////////////////////////////////////////////
		/////////////////rewrite in txt the femur values we need////////////////////
		string resultDir = BASE_DIR + ini.Get("PATH", "RESULT_DIR", "");
		string pathst = resultDir + "/_StaticAnalysis0";
		Storage  pointtt(pathst + "/_StaticAnalysis.sto");
		Array<double> tim;

		Array<double> pppz;
		Array<double> ppx;
		Array<double> pppx;
		Array<double> ppy;
		Array<double> pppy;
		Array<double> ppz;

		Array<double> pppgz;
		Array<double> ppgx;
		Array<double> pppgx;
		Array<double> ppgy;
		Array<double> pppgy;
		Array<double> ppgz;

		Array<double> ooz;
		Array<double> ooox;
		Array<double> oox;
		Array<double> oooy;
		Array<double> ooy;
		Array<double> oooz;

		pointtt.getTimeColumn(tim);
		pointtt.getDataColumn(bodyname1 + "_ox", oox);
		pointtt.getDataColumn(bodyname2 + "_ox", ooox);
		pointtt.getDataColumn(bodyname1 + "_oy", ooy);
		pointtt.getDataColumn(bodyname2 + "_oy", oooy);
		pointtt.getDataColumn(bodyname1 + "_oz", ooz);
		pointtt.getDataColumn(bodyname2 + "_oz", oooz);


		pointtt.getDataColumn(bodyname1 + "_px", ppx);
		pointtt.getDataColumn(bodyname2 + "_px", pppx);
		pointtt.getDataColumn(bodyname1 + "_py", ppy);
		pointtt.getDataColumn(bodyname2 + "_py", pppy);
		pointtt.getDataColumn(bodyname1 + "_pz", ppz);
		pointtt.getDataColumn(bodyname2 + "_pz", pppz);

		pointtt.getDataColumn(bodyname1 + "_pgx", ppgx);
		pointtt.getDataColumn(bodyname2 + "_pgx", pppgx);
		pointtt.getDataColumn(bodyname1 + "_pgy", ppgy);
		pointtt.getDataColumn(bodyname2 + "_pgy", pppgy);
		pointtt.getDataColumn(bodyname1 + "_pgz", ppgz);
		pointtt.getDataColumn(bodyname2 + "_pgz", pppgz);

		Vector point1(3,1);
		Vector point2(3, 1);
		Vector pointofapplication2(3, 1);
		Vector pointofapplication21(3, 1);
		Vector or1(3, 1);
		Vector or2(3, 1);
		for (int i = 0; i < 7; ++i){
		point1[i] = pppx[i];
		point1[i + 1] = pppy[i];
		point1[i + 2] = pppz[i];

		point2[i] = ppx[i];
		point2[i + 1] = ppy[i];
		point2[i + 2] = ppz[i];


		pointofapplication21[i] = pppgx[i];
		pointofapplication21[i + 1] = pppgy[i];
		pointofapplication21[i + 2] = pppgz[i];

		pointofapplication2[i] = ppgx[i];
		pointofapplication2[i + 1] = ppgy[i];
		pointofapplication2[i + 2] = ppgz[i];



		or1[i] = ooox[i];
		or1[i + 1] = oooy[i];
		or1[i + 2] = oooz[i];

		or2[i] = oox[i];
		or2[i + 1] = ooy[i];
		or2[i + 2] = ooz[i];

		}
		cout << tim << endl;
		cout << ppx << endl;
		cout << ppx << endl;
		cout << ooox << endl;
		cout << oox << endl;
		cout << ppgx << endl;
		cout << pppgx << endl;


		cout << point2 << endl;
		cout << point1 << endl;
		cout << or2 << endl;
		cout << or1 << endl;
		cout << pointofapplication2 << endl;
		cout << pointofapplication21 << endl;


		ofstream staticpointvalues;
		staticpointvalues = ofstream(pathst + "/_staticanalysis.txt", ofstream::out);
		staticpointvalues << point1[0] << endl;
		staticpointvalues << point1[1] << endl;
		staticpointvalues << point1[2] << endl;
		staticpointvalues << point2[0] << endl;
		staticpointvalues << point2[1] << endl;
		staticpointvalues << point2[2] << endl;
		staticpointvalues << pointofapplication2[0] << endl;
		staticpointvalues << pointofapplication2[1] << endl;
		staticpointvalues << pointofapplication2[2] << endl;
		staticpointvalues << pointofapplication21[0] << endl;
		staticpointvalues << pointofapplication21[1] << endl;
		staticpointvalues << pointofapplication21[2] << endl;
		staticpointvalues << or1[0] << endl;
		staticpointvalues << or1[1] << endl;
		staticpointvalues << or1[2] << endl;
		staticpointvalues << or2[0] << endl;
		staticpointvalues << or2[1] << endl;
		staticpointvalues << or2[2] << endl;
		*/

		///////////////////////STATIC ANALYSIS AND RESULTS///////////////////////////////////////////////////
		string pathst = resultDir + "/_StaticAnalysis" + num;
		cout << "" << endl;
		cout << " Start the Body Static Analysis..." << endl;
		cout << "" << endl;
		//////////////////////////////////////////////////////////////////////////////////////////////////
		///////////////////////body static analysis//////////////////////////////////////////////
		////////////////////////////////////////////////////////////////

		StaticAnalysis staticposition(&model2);
		model2.addAnalysis(&staticposition);
		model2.buildSystem();
		State& sis = model2.initializeState();

		AnalyzeTool tool2s(model2);
		tool2s.setStartTime(t0s);
		tool2s.setFinalTime(tfs);
		tool2s.setResultsDir(pathst);
		tool2s.setLowpassCutoffFrequency(6);
		tool2s.setStatesFromMotion(sis, stateFilestatic, 1);
		//cout << stateFile << endl;
		tool2s.run();


	}
	////////////////////////////////////////////////////////////////////////////////////////////////////
	//////////////////////////////////////////////////////////////////////////////////////////////////
	char num = itteration + '0';
	String path = resultDir + "/_BodyForceAnalysis" + num;


	cout << "" << endl;
	cout << " Start the Body Force-Kinematics Analysis..." << endl;
	cout << "" << endl;
	//////////////////////////////////////////////////////////////////////////////////////////////////
	///////////////////////body forces//////////////////////////////////////////////
	////////////////////////////////////////////////////////////////



	if (mode == 'F'){
		BodyForceAnalysis bodyForce(&model2);
		model2.addAnalysis(&bodyForce);
		model2.buildSystem();
		State& si = model2.initializeState();
		char num = itteration + '0';
		Storage set = stateFile;
		String path = resultDir + "/_BodyForceAnalysis" + num;

		AnalyzeTool tool2(model2);
		tool2.setStartTime(t0);
		tool2.setFinalTime(tf);
		tool2.setResultsDir(path);
		tool2.setLowpassCutoffFrequency(6);
		tool2.setStatesFileName(stateFile);
		//cout << stateFile << endl;
		tool2.setStatesStorage(set);

		tool2.run();

	}

	if (mode == 'T'){
		BodyForceAnalysis2 bodyForce(&model2);
		model2.addAnalysis(&bodyForce);
		model2.buildSystem();
		State& si = model2.initializeState();
		char num = itteration + '0';
		Storage set = stateFile;
		String path = resultDir + "/_BodyForceAnalysis" + num;

		AnalyzeTool tool2(model2);
		tool2.setStartTime(t0);
		tool2.setFinalTime(tf);
		tool2.setResultsDir(path);
		tool2.setLowpassCutoffFrequency(6);
		tool2.setStatesFileName(stateFile);
		//cout << stateFile << endl;
		tool2.setStatesStorage(set);

		tool2.run();
	}




	////////////
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	/// The Forces vectors DATA

	Array<double> tim;
	Array<double> femfx;
	Array<double> tibfx;
	Array<double> femfy;
	Array<double> tibfy;
	Array<double> femfz;
	Array<double> tibfz;
	Array<double> femmx;
	Array<double> tibmx;
	Array<double> femmy;
	Array<double> tibmy;
	Array<double> femmz;
	Array<double> tibmz;
	Array<double> fempx;
	Array<double> tibpx;
	Array<double> fempy;
	Array<double> tibpy;
	Array<double> fempz;
	Array<double> tibpz;
	Array<double> femox;
	Array<double> tibox;
	Array<double> femoy;
	Array<double> tiboy;
	Array<double> femoz;
	Array<double> tiboz;

	Array<double> g;
	Array<string> f;

	Storage  forces(path + "/_BodyForceAnalysis.sto");
	forces.getTimeColumn(tim);
	system("pause");




	forces.getDataColumn(bodyname1 + "_fx", femfx);
	forces.getDataColumn(bodyname2 + "_fx", tibfx);
	forces.getDataColumn(bodyname1 + "_fy", femfy);
	forces.getDataColumn(bodyname2 + "_fy", tibfy);
	forces.getDataColumn(bodyname1 + "_fz", femfz);
	forces.getDataColumn(bodyname2 + "_fz", tibfz);
	forces.getDataColumn(bodyname1 + "_mx", femmx);
	forces.getDataColumn(bodyname2 + "_mx", tibmx);
	forces.getDataColumn(bodyname1 + "_my", femmy);
	forces.getDataColumn(bodyname2 + "_my", tibmy);
	forces.getDataColumn(bodyname1 + "_mz", femmz);
	forces.getDataColumn(bodyname2 + "_mz", tibmz);
	forces.getDataColumn(bodyname3 + "_px", fempx);
	forces.getDataColumn(bodyname4 + "_px", tibpx);
	forces.getDataColumn(bodyname3 + "_py", fempy);
	forces.getDataColumn(bodyname4 + "_py", tibpy);
	forces.getDataColumn(bodyname3 + "_pz", fempz);
	forces.getDataColumn(bodyname4 + "_pz", tibpz);
	forces.getDataColumn(bodyname3 + "_ox", femox);
	forces.getDataColumn(bodyname4 + "_ox", tibox);
	forces.getDataColumn(bodyname3 + "_oy", femoy);
	forces.getDataColumn(bodyname4 + "_oy", tiboy);
	forces.getDataColumn(bodyname3 + "_oz", femoz);
	forces.getDataColumn(bodyname4 + "_oz", tiboz);


	int endend = tim.size();
	//Vector time(endend, tim[endend]);
	Vector timefinal(endend, tim[endend]);

	cout << "Which is the initial time you want to reach the model for the initial prescribed motions and forces of the model?" << endl;
	cout << "Set the end time of the Initial step one analysis (THE STEP FOR REACH the initial contitions)" << endl;
	cout << " in the .ini file (setup) set it in variable Initialtime...if is empty will ask to you here to set it..." << endl;

	double intim = ini.GetReal("BASICSETUP", "Initialtime", NAN);
	if (intim == NAN){
		cout << "Please answer: Set the end time of Initial step analysis" << endl;
		cin >> intim;
	}
	if (intim != 0){
		cout << intim << endl;
	}
	if (intim == 0){
		cout << "Give the initialtime end here..." << endl;
		cin >> intim;
	}

	for (int i = 0; i < endend; ++i){
		timefinal[i] = tim[i] - tim[0];//+0.01;//calibration step until 0.01 of two models
		timefinal[i] = timefinal[i] * 100000;
		int pat = (int)timefinal[i];
		timefinal[i] = pat * 0.00001;
		timefinal[i] = timefinal[i] + intim;
	}
	timefinal[0] = 0;

	///TRANSFORM MATRIX FOR COPERATION/////////////////////////////////////////////////
	Vector fFx(endend, 1);
	Vector tFx(endend, 1);
	Vector fFy(endend, 1);
	Vector tFy(endend, 1);
	Vector fFz(endend, 1);
	Vector tFz(endend, 1);
	Vector fMx(endend, 1);
	Vector tMx(endend, 1);
	Vector fMy(endend, 1);
	Vector tMy(endend, 1);
	Vector fMz(endend, 1);
	Vector tMz(endend, 1);
	Vector fPx(endend, 1);
	Vector tPx(endend, 1);
	Vector fPy(endend, 1);
	Vector tPy(endend, 1);
	Vector fPz(endend, 1);
	Vector tPz(endend, 1);
	Vector fOx(endend, 1);
	Vector tOx(endend, 1);
	Vector fOy(endend, 1);
	Vector tOy(endend, 1);
	Vector fOz(endend, 1);
	Vector tOz(endend, 1);

	for (int i = 0; i < endend; ++i){
		fFx[i] = femfz[i];
		tFx[i] = tibfz[i];
		fFy[i] = femfx[i];
		tFy[i] = tibfx[i];
		fFz[i] = femfy[i];
		tFz[i] = tibfy[i];
		fMx[i] = femmz[i];
		tMx[i] = tibmz[i];
		fMy[i] = femmx[i];
		tMy[i] = tibmx[i];
		fMz[i] = femmy[i];
		tMz[i] = tibmy[i];
		fPx[i] = fempz[i];
		tPx[i] = tibpz[i];
		fPy[i] = fempx[i];
		tPy[i] = tibpx[i];
		fPz[i] = fempy[i];
		tPz[i] = tibpy[i];
		fOx[i] = femoz[i];
		tOx[i] = tiboz[i];
		fOy[i] = femox[i];
		tOy[i] = tibox[i];
		fOz[i] = -femoy[i];
		tOz[i] = -tiboy[i];
	}

	//DC offset of position filter!
	double mean1 = 0;
	double mean2 = 0;
	double mean3 = 0;
	double mean4 = 0;
	double mean5 = 0;
	double mean6 = 0;
	for (int i = 0; i < endend; ++i){


		fPx[i] = fPx[i] * 10000;
		int parast1 = (int)fPx[i];
		fPx[i] = parast1 * 0.0001;
		fPy[i] = fPy[i] * 10000;
		int parasty1 = (int)fPy[i];
		fPy[i] = parasty1 * 0.0001;
		fPz[i] = fPz[i] * 10000;
		int parastz1 = (int)fPz[i];
		fPz[i] = parastz1 * 0.0001;

		tPx[i] = tPx[i] * 10000;
		int parast11 = (int)tPx[i];
		tPx[i] = parast11 * 0.0001;
		tPy[i] = tPy[i] * 10000;
		int parasty11 = (int)tPy[i];
		tPy[i] = parasty11 * 0.0001;
		tPz[i] = tPz[i] * 10000;
		int parastz11 = (int)tPz[i];
		tPz[i] = parastz11 * 0.0001;


		mean1 = fPz[i] + mean1;

		mean2 = fPy[i] + mean2;

		mean3 = fPx[i] + mean3;

		mean4 = tPz[i] + mean4;

		mean5 = tPx[i] + mean5;

		mean6 = tPy[i] + mean6;

	}
	int mean11 = (mean1 / endend) * 10000;
	int mean21 = (mean2 / endend) * 10000;
	int mean31 = (mean3 / endend) * 10000;
	int mean41 = (mean4 / endend) * 10000;
	int mean51 = (mean5 / endend) * 10000;
	int mean61 = (mean6 / endend) * 10000;

	double mean12 = mean11 * 0.0001;
	double mean22 = mean21 * 0.0001;
	double mean32 = mean31 * 0.0001;
	double mean42 = mean41 * 0.0001;
	double mean52 = mean51 * 0.0001;
	double mean62 = mean61 * 0.0001;
	cout << "The DC filter mean values respectively are:  " << endl;
	cout << "BD1x = " << mean32 << endl;
	cout << "BD1y = " << mean22 << endl;
	cout << "BD1z = " << mean12 << endl;
	cout << "BD2x = " << mean52 << endl;
	cout << "BD2y = " << mean62 << endl;
	cout << "BD2z = " << mean42 << endl;

	for (int i = 0; i < endend; ++i){

		fPz[i] = fPz[i] - (mean12);

		fPy[i] = fPy[i] - (mean22);

		fPx[i] = fPx[i] - (mean32);

		tPz[i] = tPz[i] - (mean42);

		tPx[i] = tPx[i] - (mean52);

		tPy[i] = tPy[i] - (mean62);

	}
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//////////////////////////////////////////////////////////////
	//	SET SAME POSITION FROM JOINT CAPTURE

	cout << "Do you want to set the prescribed translation DOF you allready set in stragety setup in femur from Joint capture?" << endl;
	cout << "end set the tibia fixed only in translation dof (y/n)" << endl;
	cout << "We saggest yes!!" << endl;
	string done = ini.Get("BASICSETUP", "Translation_from_JR", "");
	if (done != "y"){
		cout << done << endl;
	}
	if (done == "y"){
		cout << "Give the initialtime end here..." << endl;
		cin >> done;
	}

	if (done == "y"){
		JointReaction reactForce(&model2);
		// set the frame of parent to solve

		//void SimTK::SimbodyMatterSubsystem::calcMobilizerReactionForcesUsingFreebodyMethod(const State &  	state,
		//Vector_< SpatialVec > &  	forcesAtMInG
		//https://simtk.org/api_docs/simbody/latest/classSimTK_1_1SimbodyMatterSubsystem.html#a98b55fad58c0968702ae7c1d92238bcc
		int jointw = model2.getNumJoints();

		Array <string> inFrame("parent", jointw);
		reactForce.setInFrame(inFrame);
		//cout << inFrame<< endl;

		model2.addAnalysis(&reactForce);
		model2.buildSystem();
		State& si = model2.initializeState();
		char num = itteration + '0';
		Storage set = stateFile;
		string path = resultDir + "/_JointReactionForces" + num;

		AnalyzeTool tool2(model2);
		tool2.setStartTime(t0);
		tool2.setFinalTime(tf);
		tool2.setResultsDir(path);
		tool2.setLowpassCutoffFrequency(6);
		tool2.setStatesFileName(stateFile);
		cout << stateFile << endl;
		tool2.setStatesStorage(set);

		tool2.run();



		////////////
		////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		/// The Forces vectors DATA

		Array<double> tim;




		Array<double> g;
		Array<string> f;

		Storage  forces(path + "/_Un-named analysis._ReactionLoads.sto");
		forces.getTimeColumn(tim);
		//system("pause");

		Array<double> jfx;
		Array<double> jfy;
		Array<double> jfz;
		Array<double> jmx;
		Array<double> jmy;
		Array<double> jmz;

		Array<double> jpx;
		Array<double> jpy;
		Array<double> jpz;
		Array<double> jox;
		Array<double> joy;
		Array<double> joz;


		forces.getDataColumn(joint + "_on_" + bodyname2 + "_in_" + bodyname1 + "_fx", jfx);
		forces.getDataColumn(joint + "_on_" + bodyname2 + "_in_" + bodyname1 + "_fy", jfy);
		forces.getDataColumn(joint + "_on_" + bodyname2 + "_in_" + bodyname1 + "_fz", jfz);
		forces.getDataColumn(joint + "_on_" + bodyname2 + "_in_" + bodyname1 + "_mx", jmx);
		forces.getDataColumn(joint + "_on_" + bodyname2 + "_in_" + bodyname1 + "_my", jmy);
		forces.getDataColumn(joint + "_on_" + bodyname2 + "_in_" + bodyname1 + "_mz", jmz);

		//////////I have to take them from other file... the state file .sto has this but we have to set the joints dof../////// do them as vector and the velocities has...

		Storage  motion(stateFile);
		if (joint1 != "0"){
			motion.getDataColumn(joint1, jpx);
		}

		if (joint2 != "0"){
			motion.getDataColumn(joint2, jpy);
		}


		if (joint3 != "0"){
			motion.getDataColumn(joint3, jpz);
		}

		if (joint4 != "0"){
			motion.getDataColumn(joint4, jox);

		}

		if (joint5 != "0"){
			motion.getDataColumn(joint5, joy);


		}

		if (joint6 != "0"){
			motion.getDataColumn(joint6, joz);

		}
for (int i = 0; i < endend; ++i){

		if (joint1 == "0"){
			fPx[i] = fempx[i] - tibpx[i];

		}
		if (joint1 != "0"){
			fPx[i] = -jpx[i];
		}

		if (joint2 == "0"){
			fPy[i] = fempy[i] - tibpy[i];

		}
		if (joint2 != "0"){
			fPy[i] = -jpy[i];
		}

		if (joint3 == "0"){
			fPz[i] = fempz[i] - tibpz[i];

		}
		if (joint3 != "0"){
			fPz[i] = -jpz[i];
		}



	}


	tPx = 0;

	tPy = 0;

	tPz = 0;



}

		//////////////////////////////////////////////////////////////////////////////
		////////////////////////////////////ASK IF WANT MOTION DETECTION BASED THE POINT OR BASED THE DOF OF JOINTS//////////////////////////


		cout <<"Do you want to detect the motion of two bodies base the point of interest you give previewsly ? (/P)" << endl;
		cout << "or you want to set the degree of freedom of two bodies based the virtual joint of OpenSim ? (/J)" << endl;
		string dof = ini.Get("BASICSETUP", "DOF_case_F", "");
		if (dof == ""){
			cout << "Please answer: Which case you want J/P?" << endl;
			cin >> dof;
		}
		cout << dof << endl;
		if (dof == "P"){
			cout << "we will continue with point of interest detection motion for the " << bodyname1 << " and " << bodyname2 << " now..." << endl;
		}
		if (dof == "J"){	
			//for (int i = 0; i < endend; ++i){


				if (joint1 == "0"){
					fPx = 0;
					tPx = 0;
				}


				if (joint2 == "0"){
					fPy = 0;
					tPy = 0;
				}


				if (joint3 == "0"){
					fPz = 0;
					tPz = 0;
				}


				if (joint4 == "0"){
					fOx = 0;
					tOx = 0;
				}


				if (joint5 == "0"){
					fOy = 0;
					tOy = 0;
				}


				if (joint6 == "0"){
					fOz = 0;
					tOz = 0;
				}

			

		}
//////////////////////////////////////////////////////////////////////////////
			

/////////////////////////////////////////////ASK for detect the interpolation kind/////////////////////////////////////////////////////////////////////
			//////////////////////////////////////////////////////////////////////////////////////////

		
			string kind[24];
			cout << "" << endl;
			cout << "Do you want to detect the interpolation matching kind for all the DoFs of model? (y/n)" << endl;
			//TODO ask for give the sample he want!!!!!!!!
			string dicession = ini.Get("BASICSETUP", "Interpolation_Detect", "");
			if (dicession == ""){
				cout << "Please answer: Do you want to detect the interpolation matching kind for all the DoFs of model? (y/n)" << endl;
				cin >> dicession;
			}
			cout << dicession << endl;
			//char kin;
			//cout << dicession << endl;
			//send the sums for fixed the tibia body
			if (dicession == "n"){
				cout << "Do you want to the interpolation be smooth 's' or linear 'l' for the DoF? (s/l)" << endl;
				string kin = ini.Get("BASICSETUP", "Interpolation_kind", "");
				if (kin == ""){
					cout << "give the kind of interpolation function:" << endl;
					cin >> kin;
				}
				if (kin == "l"){
					int co = 0;
					while (co < 24){
						kind[co] = "linear";
						++co;
					}
				}
				if (kin == "s"){
					int co = 0;
					while (co < 24){
						kind[co] = "smooth";
						++co;
					}
				}
				cout << "" << endl;
				cout << "The interpolation kind for the DoF of the 6D force and the 6D position " << endl;
				cout << "FIRST BODY DoF of the 6D force: " << endl;
				cout << kind[0] << ", " << kind[1] << ", " << kind[2] << ", " << kind[3] << ", " << kind[4] << ", " << kind[5] << endl;
				cout << "SECOND BODY DoF of the 6D force: " << endl;
				cout << kind[6] << ", " << kind[7] << ", " << kind[8] << ", " << kind[9] << ", " << kind[10] << ", " << kind[11] << endl;

				cout << "FIRST BODY DoF of the 6D  position: " << endl;
				cout << kind[12] << ", " << kind[13] << ", " << kind[14] << ", " << kind[15] << ", " << kind[16] << ", " << kind[17] << endl;

				cout << "SECOND BODY DoF of the 6D  position: " << endl;
				cout << kind[18] << ", " << kind[19] << ", " << kind[20] << ", " << kind[21] << ", " << kind[22] << ", " << kind[23] << endl;

				cout << "" << endl;
			}

			if (dicession == "y"){



				cout << "" << endl;

				cout << "" << endl;
				cout << "DETECT THE KIND OF INTERPOLATION FOR THE RESAMPLE POINTS" << endl;
				cout << "" << endl;

				ReSampler resampl;

				kind[0] = resampl.interpolationkind(fFx, timefinal);
				kind[1] = resampl.interpolationkind(fFy, timefinal);
				kind[2] = resampl.interpolationkind(fFz, timefinal);
				kind[3] = resampl.interpolationkind(fMx, timefinal);
				kind[4] = resampl.interpolationkind(fMy, timefinal);
				kind[5] = resampl.interpolationkind(fMz, timefinal);
				kind[6] = resampl.interpolationkind(tFx, timefinal);
				kind[7] = resampl.interpolationkind(tFy, timefinal);
				kind[8] = resampl.interpolationkind(tFz, timefinal);
				kind[9] = resampl.interpolationkind(tMx, timefinal);
				kind[10] = resampl.interpolationkind(tMy, timefinal);
				kind[11] = resampl.interpolationkind(tMz, timefinal);
				kind[12] = resampl.interpolationkind(fPx, timefinal);
				kind[13] = resampl.interpolationkind(fPy, timefinal);
				kind[14] = resampl.interpolationkind(fPz, timefinal);
				kind[15] = resampl.interpolationkind(fOx, timefinal);
				kind[16] = resampl.interpolationkind(fOy, timefinal);
				kind[17] = resampl.interpolationkind(fOz, timefinal);
				kind[18] = resampl.interpolationkind(tPx, timefinal);
				kind[19] = resampl.interpolationkind(tPy, timefinal);
				kind[20] = resampl.interpolationkind(tPz, timefinal);
				kind[21] = resampl.interpolationkind(tOx, timefinal);
				kind[22] = resampl.interpolationkind(tOy, timefinal);
				kind[23] = resampl.interpolationkind(tOz, timefinal);

				resampl.~ReSampler();

				cout << "" << endl;
				cout << "The interpolation kind for the DoF of the 6D force and the 6D position " << endl;
				cout << "FIRST BODY DoF of the 6D force: " << endl;
				cout << kind[0] << ", " << kind[1] << ", " << kind[2] << ", " << kind[3] << ", " << kind[4] << ", " << kind[5] << endl;
				cout << "SECOND BODY DoF of the 6D force: " << endl;
				cout << kind[6] << ", " << kind[7] << ", " << kind[8] << ", " << kind[9] << ", " << kind[10] << ", " << kind[11] << endl;

				cout << "FIRST BODY DoF of the 6D  position: " << endl;
				cout << kind[12] << ", " << kind[13] << ", " << kind[14] << ", " << kind[15] << ", " << kind[16] << ", " << kind[17] << endl;

				cout << "SECOND BODY DoF of the 6D  position: " << endl;
				cout << kind[18] << ", " << kind[19] << ", " << kind[20] << ", " << kind[21] << ", " << kind[22] << ", " << kind[23] << endl;
			}
			cout << "" << endl;
			
//////////////////////////////////////////////////////////////////////////END interpolation///////////////////////////////////////////////////////////////////

			

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
			//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
			////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%			


			//////////////////////////////////////////////////////////////////////////START THE RESAMPLER SECTION ///////////////////////////////////////////////////////////////////
			////////////////TIME RESAMPLER///////////////
			///////////////FORCE-MOTION RESAMPLER///////////////
			////////INITIAL-VELOCITY RESAMPLER//////////////
			/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

			
			cout << "" << endl;
			cout << "The size of DoF's sample after the OpenSim analysis is: " << endend << " elements" << endl;
			cout << "Do you want to resample the DoF's sample? (y/n)" << endl;
			//TODO ask for give the sample he want!!!!!!!!
			string  dicession1 = ini.Get("BASICSETUP", "Time_Resample", "");
			if (dicession1 == ""){
				cout << "Please answer: Do you want to resample the DoF's sample? (y/n)" << endl;
				cin >> dicession1;
			}
			cout << dicession1 << endl;
			//cout << dicession << endl;
			//send the sums for fixed the tibia body


			///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
			////////////////////////////////////////////////////////////// A. NO  TIME RESAMPLE  ///////////////////////////////////////////////////////////////////////////
			/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
			
			if (dicession1 == "n"){
				CASE_ONE c1;
				c1.store(timefinal, 0);
				c1.store(fPx, 1);
				c1.store(fPy, 2);
				c1.store(fPz, 3);
				c1.store(fOx, 4);
				c1.store(fOy, 5);
				c1.store(fOz, 6);
				c1.store(tPx, 7);
				c1.store(tPy, 8);
				c1.store(tPz, 9);
				c1.store(tOx, 10);
				c1.store(tOy, 11);
				c1.store(tOz, 12);
				c1.store(fFx, 13);
				c1.store(fFy, 14);
				c1.store(fFz, 15);
				c1.store(fMx, 16);
				c1.store(fMy, 17);
				c1.store(fMz, 18);
				c1.store(tFx, 19);
				c1.store(tFy, 20);
				c1.store(tFz, 21);
				c1.store(tMx, 22);
				c1.store(tMy, 23);
				c1.store(tMz, 24);
				char dof2 = dof[0];
				c1.run(itteration, kind, endend, resultDir2, 1, dof2);
				//c1.~CASE_ONE();
				//cout << "debug0" << endl;

			}
			///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
			////////////////////////////////////////////////////////////// B. YES  TIME RESAMPLE  ///////////////////////////////////////////////////////////////////////////
			/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


			if (dicession1 == "y"){
				CASE_TWO c2;
				c2.store(timefinal, 0);
				c2.store(fPx, 1);
				c2.store(fPy, 2);
				c2.store(fPz, 3);
				c2.store(fOx, 4);
				c2.store(fOy, 5);
				c2.store(fOz, 6);
				c2.store(tPx, 7);
				c2.store(tPy, 8);
				c2.store(tPz, 9);
				c2.store(tOx, 10);
				c2.store(tOy, 11);
				c2.store(tOz, 12);
				c2.store(fFx, 13);
				c2.store(fFy, 14);
				c2.store(fFz, 15);
				c2.store(fMx, 16);
				c2.store(fMy, 17);
				c2.store(fMz, 18);
				c2.store(tFx, 19);
				c2.store(tFy, 20);
				c2.store(tFz, 21);
				c2.store(tMx, 22);
				c2.store(tMy, 23);
				c2.store(tMz, 24);
				char dof2 = dof[0];
				c2.run(itteration, kind, endend, resultDir2, dof2);
				//c2.~CASE_TWO();

			}


//cout << "FEBWRITTER" << endl;
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
FEBRunner feb;
feb.WriteFEBFile(itteration,mode);
feb.~FEBRunner();

}//end of starter



	/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	///////////////////////////////////END STARTER/////////////////////////////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


	/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	///////////////////////////////////P.S.  SECTION/////////////////////////////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	//////
	///1//
	//////

	/* Attempt one : CHANGING THE COM OF TWO BODIES
	/////SET the center of mass of bodies interest in (0,0,0) point for co-simulation with FEBio/////////////////////////////
	/*

	double xx;
	double yy;
	double zz;
	string kind2;
	cout << "Do you want callibration step? (y/n)" << endl;
	cin >> kind2 ;
	if (kind2 == "y"){
	cout << "Give the start value of calibration search of center of mass for the: " << bodyname2 << "[one value at a time with x-y-z order]" << endl;
	cout << "If you want the default values press the point 0, 0, 0 " << endl;
	cin >> xx;
	cin >> yy;
	cin >> zz;
	if (xx == 0){ xx = 0; }
	if (yy == 0){ yy = -0.025; }
	if (zz == 0){ zz = 0; }
	//for general toolbox use
	Vec3 com2(xx, yy, zz);
	const BodySet &bodies1 = model2.getBodySet();
	const JointSet &joint1 = model2.getJointSet();
	int nb1 = model2.getNumBodies();
	int nj1 = model2.getNumJoints();
	Vec3 com3;
	for (int i = 0; i < nb1; ++i){
	if (bodies1[i].getName() == bodyname2){
	bodies1[i].setMassCenter(com2);
	//bodies1[i].getMassCenter(com3);
	model2.updBodySet();
	}



	}


	}

	else if (kind2 != "y" | kind2 != "n"){ cout << "wrong answer" << endl; system("pause"); }

	*/
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


	//////
	//2///
	/////

	/*

	//correct the distances normalized in 0,0,0 point for FEBio position normalization
	//Need it when take generalized position with respect of ground here we take the local position so ok with out it!!
	/////////////////////////////////////////////////////////////////////

	Vector distx(endend, 1);
	Vector tibnx(endend, 1);
	Vector disty(endend, 1);
	Vector tibny(endend, 1);
	Vector distz(endend, 1);
	Vector tibnz(endend, 1);
	Vector nfemx(endend, 1);
	Vector ntibnx(endend, 1);
	Vector nfemy(endend, 1);
	Vector ntibny(endend, 1);
	Vector nfemz(endend, 1);
	Vector ntibnz(endend, 1);
	Vector ndistx(endend, 1);
	Vector ndisty(endend, 1);
	Vector ndistz(endend, 1);
	Vector scalex(endend, 1);
	Vector scaley(endend, 1);
	Vector scalez(endend, 1);
	Vector femurx(endend, 1);
	Vector tibianx(endend, 1);
	Vector femury(endend, 1);
	Vector tibiany(endend, 1);
	Vector femurz(endend, 1);
	Vector tibianz(endend, 1);

	// Remove the DC offsets from opensim .sto file

	for (int i = 0; i < endend; ++i){
	// compute the distance matrix of the two bodies
	distx[i] = abs(fempx[i] - tibpx[i]);
	disty[i] = abs(fempy[i] - tibpy[i]);
	distz[i] = abs(fempz[i] - tibpz[i]);
	// send the child body in the opposite orientation
	tibnx[i] = -tibfx[i];
	tibny[i] = -tibfy[i];
	tibnz[i] = -tibfz[i];
	// normalized the distances
	nfemx[i] = femfx[i] / distx[i];
	nfemy[i] = femfy[i] / disty[i];
	nfemz[i] = femfz[i] / distz[i];
	ntibnx[i] = tibnx[i] / distx[i];
	ntibny[i] = tibny[i] / disty[i];
	ntibnz[i] = tibnz[i] / distz[i];
	// take the new normalized distances
	ndistx[i] = abs(nfemx[i] - ntibnx[i]);
	ndisty[i] = abs(nfemy[i] - ntibny[i]);
	ndistz[i] = abs(nfemz[i] - ntibnz[i]);
	//divide the distances
	scalex[i] = distx[i] / ndistx[i];
	scaley[i] = disty[i] / ndisty[i];
	scalez[i] = distz[i] / ndistz[i];
	//scaled the new values of center of masses
	femurx[i] = nfemx[i] * scalex[i];
	femury[i] = nfemy[i] * scaley[i];
	femurz[i] = nfemz[i] * scalez[i];
	tibianx[i] = ntibnx[i] * scalex[i];
	tibiany[i] = ntibny[i] * scaley[i];
	tibianz[i] = ntibnz[i] * scalez[i];
	//dc offset of time if it has
	time[i] = tim[i] - tim[0];
	time[i] = time[i] * 1000;
	int pat = (int)time[i];
	timefinal[i] = pat * 0.001;
	}
	//stored in a 7:endend matrix
	//is this correct??
	//Matrix values( time, femurx, femury, femurz, tibianx, tibiany, tibianz );

	/// DEbug///
	//cout << tibianx << endl;
	//cout << femurx << endl;
	//cout << time << endl;
	//cout << timefinal << endl;
	/////
	*/

	/////////////////////////////////////////////////////////////////////////////////////////////


//////////////
/////3/////
/////////


//Do the xml style txt for prescribe motion of two bodies

/////////////////////////////////// FOR FIX 2ST BODY CONCEPT//////////////////////////////////////////
/*
// FORCES VECTORS FOR 1ST BODY

fFx= fFx + tFx;
fFy= fFy + tFy;
fFz= fFz + tFz;

fMx= fMx + tMx;
fMy= fMy + tMy;
fMz= fMz + tMz;


// POSITIONS VECTORS FOR 1ST BODY
*/
//we want only the distance between the bodies so we will move only the one of two not both
/*
tPx = abs(fPx - tPx);
tPy = abs(fPy - tPy);
tPz = abs(fPz - tPz);
Vector tPxn = -tPx;
Vector tPyn = -tPy;
Vector tPzn = -tPz;
fPx = 0;
fPy = 0;
fPz = 0;
*/
/*
// if we have rads!!!
fOx =  abs(fOx - tOx);
fOy =  abs(fOy - tOy);
fOz =  abs(fOz - tOz);
*/
////////////////////////////////////////////////////////////////////////////////////////////////////



/////////////////
//////////4/////
/////////////
/////////

// THE _1B scenarion not Correct!!!!


//// NOT IN USE////////////////////
/*
if (actforces == 'y'){
//compute relative positions and forces//
//(we have to set the points of the acting forces of two bodies in the same point to take the relative differences so we call the function bellow)
DOFResample dreslq;
dreslq.store(timefinal, 0);
dreslq.store(fPx, 1);
dreslq.store(fPy, 2);
dreslq.store(fPz, 3);
dreslq.store(fOx, 4);
dreslq.store(fOy, 5);
dreslq.store(fOz, 6);
dreslq.store(tPx, 7);
dreslq.store(tPy, 8);
dreslq.store(tPz, 9);
dreslq.store(tOx, 10);
dreslq.store(tOy, 11);
dreslq.store(tOz, 12);
dreslq.store(fFx, 13);
dreslq.store(fFy, 14);
dreslq.store(fFz, 15);
dreslq.store(fMx, 16);
dreslq.store(fMy, 17);
dreslq.store(fMz, 18);
dreslq.store(tFx, 19);
dreslq.store(tFy, 20);
dreslq.store(tFz, 21);
dreslq.store(tMx, 22);
dreslq.store(tMy, 23);
dreslq.store(tMz, 24);
dreslq._1B_resampler();

timefinal = dreslq.return_vect(0);

/////global vector////
tFx = dreslq.return_vect(19);
tFy = dreslq.return_vect(20);
tFz = dreslq.return_vect(21);
tMx = dreslq.return_vect(22);
tMy = dreslq.return_vect(23);
tMz = dreslq.return_vect(24);
dreslq.~DOFResample();


fFx = fFx -tFx;

fFy = fFy - tFy;

fFz = fFz - tFz;

tFx =0;

tFy=0;

tFz=0;

fMx = fMx - tMx;

fMy = fMy - tMy;

fMz = fMz - tMz;

tMx = 0;

tMy = 0;

tMz = 0;

//////////////POSITIONS///////////

fPx = fPx - tPx;

fPy = fPy - tPy;

fPz = fPz - tPz;

tPx = 0;

tPy = 0;

tPz = 0;

fOx = fOx - tOx;

fOy = fOy - tOy;

fOz = fOz - tOz;

tOx = 0;

tOy = 0;

tOz = 0;
/*
for (int i=0; i < timefinal.size(); ++i){

if (abs(fFx[i]) > abs(tFx[i])){
fFx[i] = (abs(fFx[i]) - abs(tFx[i]))*(fFx[i] / fFx[i]);
}
if (abs(fFx[i]) < abs(tFx[i])){
fFx[i] = (abs(tFx[i]) - abs(fFx[i]))*(tFx[i] / tFx[i]);
}
if (abs(fFx[i]) == abs(tFx[i])){
fFx[i] = 0;
}
tFx[i] = 0;
if (abs(fFy[i]) > abs(tFy[i])){
fFy[i] = (abs(fFy[i]) - abs(tFy[i]))*(fFy[i] / fFy[i]);
}
if (abs(fFy[i]) < abs(tFy[i])){
fFy[i] = (abs(tFy[i]) - abs(fFy[i]))*(tFy[i] / tFy[i]);
}
if (abs(fFy[i]) == abs(tFy[i])){
fFy[i] = 0;
}
tFy[i] = 0;
if (abs(fFz[i]) > abs(tFz[i])){
fFz[i] = (abs(fFz[i]) - abs(tFz[i]))*(fFz[i] / fFz[i]);
}
if (abs(fFz[i]) < abs(tFz[i])){
fFz[i] = (abs(tFz[i]) - abs(fFz[i]))*(tFz[i] / tFz[i]);
}
if (abs(fFz[i]) == abs(tFz[i])){
fFz[i] = 0;
}
tFz[i] = 0;
if (abs(fMx[i]) > abs(tMx[i])){
fMx[i] = (abs(fMx[i]) - abs(tMx[i]))*(fMx[i] / fMx[i]);
}
if (abs(fMx[i]) < abs(tMx[i])){
fMx[i] = (abs(tMx[i]) - abs(fMx[i]))*(tMx[i] / tMx[i]);
}
if (abs(fMx[i]) == abs(tMx[i])){
fMx[i] = 0;
}
tMx[i] = 0;
if (abs(fMy[i]) > abs(tMy[i])){
fMy[i] = (abs(fMy[i]) - abs(tMy[i]))*(fMy[i] / fMy[i]);
}
if (abs(fMy[i]) < abs(tMy[i])){
fMy[i] = (abs(tMy[i]) - abs(fMy[i]))*(tMy[i] / tMy[i]);
}
if (abs(fMy[i]) == abs(tMy[i])){
fMy[i] = 0;
}
tMy[i] = 0;
if (abs(fMz[i]) > abs(tMz[i])){
fMz[i] = (abs(fMz[i]) - abs(tMz[i]))*(fMz[i] / fMz[i]);
}
if (abs(fMz[i]) < abs(tMz[i])){
fMz[i] = (abs(tMz[i]) - abs(fMz[i]))*(tMz[i] / tMz[i]);
}
if (abs(fMz[i]) == abs(tMz[i])){
fMz[i] = 0;
}
tMz[i] = 0;

///////////POSITONS//////////////

if (abs(fPx[i]) > abs(tPx[i])){
fPx[i] = (abs(fPx[i]) - abs(tPx[i]))*(fPx[i] / fPx[i]);
}
if (abs(fPx[i]) < abs(tPx[i])){
fPx[i] = (abs(tPx[i]) - abs(fPx[i]))*(tPx[i] / tPx[i]);
}
if (abs(fPx[i]) == abs(tPx[i])){
fPx[i] = 0;
}
tPx[i] = 0;
if (abs(fPy[i]) > abs(tPy[i])){
fPy[i] = (abs(fPy[i]) - abs(tPy[i]))*(fPy[i] / fPy[i]);
}
if (abs(fPy[i]) < abs(tPy[i])){
fPy[i] = (abs(tPy[i]) - abs(fPy[i]))*(tPy[i] / tPy[i]);
}
if (abs(fPy[i]) == abs(tPy[i])){
fPy[i] = 0;
}
tPy[i] = 0;
if (abs(fPz[i]) > abs(tPz[i])){
fPz[i] = (abs(fPz[i]) - abs(tPz[i]))*(fPz[i] / fPz[i]);
}
if (abs(fPz[i]) < abs(tPz[i])){
fPz[i] = (abs(tPz[i]) - abs(fPz[i]))*(tPz[i] / tPz[i]);
}
if (abs(fPz[i]) == abs(tPz[i])){
fPz[i] = 0;
}
tPz[i] = 0;
if (abs(fOx[i]) > abs(tOx[i])){
fOx[i] = (abs(fOx[i]) - abs(tOx[i]))*(fOx[i] / fOx[i]);
}
if (abs(fOx[i]) < abs(tOx[i])){
fOx[i] = (abs(tOx[i]) - abs(fOx[i]))*(tOx[i] / tOx[i]);
}
if (abs(fOx[i]) == abs(tOx[i])){
fOx[i] = 0;
}
tOx[i] = 0;
if (abs(fOy[i]) > abs(tOy[i])){
fOy[i] = (abs(fOy[i]) - abs(tOy[i]))*(fOy[i] / fOy[i]);
}
if (abs(fOy[i]) < abs(tOy[i])){
fOy[i] = (abs(tOy[i]) - abs(fOy[i]))*(tOy[i] / tOy[i]);
}
if (abs(fOy[i]) == abs(tOy[i])){
fOy[i] = 0;
}
tOy[i] = 0;
if (abs(fOz[i]) > abs(tOz[i])){
fOz[i] = (abs(fOz[i]) - abs(tOz[i]))*(fOz[i] / fOz[i]);
}
if (abs(fOz[i]) < abs(tOz[i])){
fOz[i] = (abs(tOz[i]) - abs(fOz[i]))*(tOz[i] / tOz[i]);
}
if (abs(fOz[i]) == abs(tOz[i])){
fOz[i] = 0;
}
tOz[i] = 0;

}

}
*/


/*
Before the upcode...

cout << "Do you want the applied forces act all in the " << bodyname1 << " body?(y/n)" << endl;
cin >> actforces;
if (actforces == 'n'){

	if (actforces == 'y'){
		BodyForceAnalysis1B bodyForce(&model2);
		model2.addAnalysis(&bodyForce);
		model2.buildSystem();
		State& si = model2.initializeState();
		char num = itteration + '0';
		Storage set = stateFile;
		String path = resultDir + "/_BodyForceAnalysis" + num;

		AnalyzeTool tool2(model2);
		tool2.setStartTime(t0);
		tool2.setFinalTime(tf);
		tool2.setResultsDir(path);
		tool2.setLowpassCutoffFrequency(6);
		tool2.setStatesFileName(stateFile);
		//cout << stateFile << endl;
		tool2.setStatesStorage(set);

		tool2.run();
	}

	cout << "Do you want the applied forces act all in the " << bodyname1 << " body?(y/n)" << endl;
	cin >> actforces;
	if (actforces == 'n'){

	if (actforces == 'y'){
	BodyForceAnalysis1B2 bodyForce(&model2);
	model2.addAnalysis(&bodyForce);
	model2.buildSystem();
	State& si = model2.initializeState();
	char num = itteration + '0';
	Storage set = stateFile;
	String path = resultDir + "/_BodyForceAnalysis" + num;

	AnalyzeTool tool2(model2);
	tool2.setStartTime(t0);
	tool2.setFinalTime(tf);
	tool2.setResultsDir(path);
	tool2.setLowpassCutoffFrequency(6);
	tool2.setStatesFileName(stateFile);
	//cout << stateFile << endl;
	tool2.setStatesStorage(set);

	tool2.run();
	}
	*/