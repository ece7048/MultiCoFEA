#include <OpenSim/OpenSim.h>
#include <OpenSim/Analyses/JointReaction.h>
#include <OpenSim/Tools/AnalyzeTool.h>
#include <OpenSim/Simulation/SimbodyEngine/Body.h>
#include <stdlib.h>
#include <ctime>
#include <exception>
#include "Settings.h"
#include "INIReader.h"
#include "BF_structor.h"
#include "BodyForceAnalysis.h"
#include "BodyForceAnalysis2.h"
#include "BK_structor.h"
#include "TSC.h"
#include "JR_starter.h"
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


using namespace OpenSim;
using namespace SimTK;
using namespace std;

JR_starter::JR_starter(void)
{
}

JR_starter::~JR_starter(void)
{
}
void JR_starter::started(int number, int itteration)
{

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


	cout << "" << endl;
	cout << " Start the Joint Reaction Forces Analysis..." << endl;
	cout << "" << endl;
	//////////////////////////////////////////////////////////////////////////////////////////////////
	///////////////////////body forces//////////////////////////////////////////////
	////////////////////////////////////////////////////////////////


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
		motion.getDataColumn(joint1, jox);
	}

	if (joint2 != "0"){
		motion.getDataColumn(joint2, joy);
	}


	if (joint3 != "0"){
		motion.getDataColumn(joint3, joz);
	}

	if (joint4 != "0"){
		motion.getDataColumn(joint4, jpx);

	}

	if (joint5 != "0"){
		motion.getDataColumn(joint5, jpy);


	}

	if (joint6 != "0"){
		motion.getDataColumn(joint6, jpz);

	}


	//cout << jfx << endl;
	//cout << jpx << endl;
	//cout << joz << endl;

	int endend = tim.size();
	//Vector time(endend, tim[endend]);
	Vector timefinal(endend, tim[endend]);
	//////////////Kind of initial step you need//////////
	cout << "Which is the initial time you want to reach the model for the initial prescribed motions and forces for the model?" << endl;
		cout << "Set the end time of the Initila step one analysis (THE STEP FOR REACH the initial contitions)" << endl;
		cout << " in the .ini file (setup) set it in variable Initialtime...if is empty will ask to you here to set it..." << endl;
		double intim = ini.GetReal("BASICSETUP", "Initialtime", 0);
		if (intim != 0){
			cout << intim << endl;
		}
		if (intim == 0){
			cout<<"Give the initialtime end here..." << endl;
			cin >> intim;
		}

	for (int i = 0; i < endend; ++i){
		timefinal[i] = tim[i] - tim[0];//+0.01;//calibration step until 0.01 of two models
		timefinal[i] = timefinal[i] * 100000;
		int pat = (int)timefinal[i];
		timefinal[i] = pat * 0.00001;
		timefinal[i] = timefinal[i]+intim;
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


	////REsample the forces (DOF-resample) base the motion of joint and not the motion of femur....

	///set the positions////
	///read the positions of femur-tibia...///

	BodyForceAnalysis bodyForce(&model2);
	model2.addAnalysis(&bodyForce);
	model2.buildSystem();
	State& sio = model2.initializeState();
	char numo = itteration + '0';
	Storage seto = stateFile;
	String patho = resultDir + "/_BodyForceAnalysis" + numo;

	AnalyzeTool tool2o(model2);
	tool2o.setStartTime(t0);
	tool2o.setFinalTime(tf);
	tool2o.setResultsDir(patho);
	tool2o.setLowpassCutoffFrequency(6);
	tool2o.setStatesFileName(stateFile);
	//cout << stateFile << endl;
	tool2o.setStatesStorage(seto);

	tool2o.run();

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

	

	Storage  forces2(patho + "/_BodyForceAnalysis.sto");
	////forces.getTimeColumn(tim);
	//system("pause");




	
	forces2.getDataColumn(bodyname1 + "_px", fempx);
	forces2.getDataColumn(bodyname2 + "_px", tibpx);
	forces2.getDataColumn(bodyname1 + "_py", fempy);
	forces2.getDataColumn(bodyname2 + "_py", tibpy);
	forces2.getDataColumn(bodyname1 + "_pz", fempz);
	forces2.getDataColumn(bodyname2 + "_pz", tibpz);
	forces2.getDataColumn(bodyname1 + "_ox", femox);
	forces2.getDataColumn(bodyname2 + "_ox", tibox);
	forces2.getDataColumn(bodyname1 + "_oy", femoy);
	forces2.getDataColumn(bodyname2 + "_oy", tiboy);
	forces2.getDataColumn(bodyname1 + "_oz", femoz);
	forces2.getDataColumn(bodyname2 + "_oz", tiboz);

	for (int i = 0; i < endend; ++i){

		fFx[i] = jfx[i];
		
		fFy[i] = jfy[i];
		
		fFz[i] = jfz[i];
		
		fMx[i] = jmx[i];
				
		fMy[i] = jmy[i];

		fMz[i] = jmz[i];
		


/////////////////////////////////////
		/////////////////////
		////////////////

	if (joint4 == "0"){
		fPx[i] = fempx[i];


	}
	if (joint4 != "0"){
		fPx[i] = tibpx[i];
	}

	if (joint5 == "0"){
		fPy[i] = fempy[i];

	}
	if (joint5 != "0"){
		fPy[i] = tibpy[i];
	}

	if (joint6 == "0"){
		fPz[i] = fempz[i];

	}
	if (joint6 != "0"){
		fPz[i] = tibpz[i];
	}


	if (joint1 == "0"){
		fOx[i] = femox[i];

	}
	if (joint1 != "0"){
		fOx[i] = tibox[i];
	}

	if (joint2 == "0"){
		fOy[i] = femoy[i];

	}
	if (joint2 != "0"){
		fOy[i] = tiboy[i];
	}

	if (joint3 == "0"){
		fOz[i] = femoz[i];

	}
	if (joint3 != "0"){
		fOz[i] = tiboz[i];
	}


	}
	tFx = 0;

	tFy = 0;

	tFz = 0;

	tMx = 0;

	tMy = 0;

	tMz = 0;

	tPx = 0;

	tPy = 0;

	tPz = 0;

	tOx = 0;

	tOy = 0;

	tOz = 0;


	DOFResample dresl2q;
	dresl2q.store(timefinal, 0);
	dresl2q.store(fPx, 1);
	dresl2q.store(fPy, 2);
	dresl2q.store(fPz, 3);
	dresl2q.store(fOx, 4);
	dresl2q.store(fOy, 5);
	dresl2q.store(fOz, 6);

	dresl2q.store(fFx, 13);
	dresl2q.store(fFy, 14);
	dresl2q.store(fFz, 15);
	dresl2q.store(fMx, 16);
	dresl2q.store(fMy, 17);
	dresl2q.store(fMz, 18);

	dresl2q._JR_resampler();
	

	
	fFy = dresl2q.return_vect(13);
	fFz = dresl2q.return_vect(14);
	fFx = dresl2q.return_vect(15);
	fMy = dresl2q.return_vect(16);
	fMz = dresl2q.return_vect(17);
	fMx = dresl2q.return_vect(18);

	dresl2q.~DOFResample();

for (int i = 0; i < endend; ++i){

		// the translation dof not need to compute we want to resample the rotational dof ony so we set them zero///
		//because expressed in the femur frame
			fPx[i] = 0;
			fPy[i] = 0;
			fPz[i] = 0;

		if (joint3 == "0"){
			fOx[i] = 0;

		}
		if (joint3 != "0"){
		fOx[i] = joz[i];
		}

		if (joint1 == "0"){
			fOy[i] = 0;

		}
		if (joint1 != "0"){
			fOy[i] = jox[i];
		}

		if (joint2 == "0"){
			fOz[i] = 0;

		}
		if (joint2 != "0"){
			fOz[i] = joy[i];
		}

		
	}

//cout << fOx << endl;
//cout << fOy << endl;

//cout << fFz << endl;
//cout << fMx << endl;
	//DC offset of position filter!

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


	}

	//////////////////////////////////////////////////////////////////////////////


	/////////////////////////////////////////////ASK for detect the interpolation kind/////////////////////////////////////////////////////////////////////
	//////////////////////////////////////////////////////////////////////////////////////////

	
	string kind[24];
	cout << "" << endl;
	cout << "Do you want to detect the interpolation matching kind for all the DoFs of model? (y/n)" << endl;
	//TODO ask for give the sample he want!!!!!!!!
	string dicession = ini.Get("BASICSETUP", "Interpolation_Detect", "");
	cout<<dicession<<endl;
	char kin;
	//cout << dicession << endl;
	//send the sums for fixed the tibia body
	if (dicession == "n"){
		cout << "Do you want to the interpolation be smooth 's' or linear 'l' for the DoF? (s/l)" << endl;
		cin >> kin;
		if (kin == 'l'){
			int co = 0;
			while (co < 24){
				kind[co] = "linear";
				++co;
			}
		}
		if (kin == 's'){
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
	cout << dicession1 << endl;
	//cout << dicession << endl;
	//send the sums for fixed the tibia body


	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////// A. NO  TIME RESAMPLE  ///////////////////////////////////////////////////////////////////////////
	/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	char resamplernow = 'n';
	if (dicession1 == "n"){

		TSC t;
		t.stepswriter(timefinal);
		t.~TSC();
		char which;
		
		////////////////////////////////////
		////B. NO MOTION RESAMPLER CASE/////////
		//////////////////////////////////
			//////////////////////////////////

			////////////////////////////////////
			//////////////// a. The initial velocity RESAMPLER  start////////////
			/////////////////////////////////////
			Initial_VEL vel;
			vel.initial(itteration, 'n', 3, 'P');
			//vel.~Initial_VEL();
			////////////////////////////////////
			//////////////// a. The initial velocity RESAMPLER end////////////
			/////////////////////////////////////

			cout << "Do you want a visualizer of pre motion of the FEBio geometry? (y/n)" << endl;
			string f = ini.Get("BASICSETUP", "Visualizer_premotion", "");
			if (f == ""){ cout << "Please,answer the question!" << endl;
			cin >> f;
			}
			cout << f << endl;
			if (f == "y"){

				///WRITTING THE MOTION FILE for geofem
				ofstream opengeostate;
				INIReader ini = INIReader(INI_FILE);
				string resultDir1 = BASE_DIR + ini.Get("FEBIOSTEP", "GEO", "");
				string geof = ini.Get("FEBIOSTEP", "GEOF", "");
				string geos = ini.Get("FEBIOSTEP", "GEOS", "");
				string bd1 = ini.Get("BODYFORCES", "BDNAME1", "");
				string bd2 = ini.Get("BODYFORCES", "BDNAME2", "");
				//char itter = itteration + '0';
				////create a new folder for the analysis/////////
				int sizer = timefinal.size();

				opengeostate = ofstream(resultDir1 + "/motionstate.mot", ofstream::out);
				opengeostate << "Coordinates" << endl;
				opengeostate << "version=1" << endl;
				opengeostate << "nRows=" << sizer - 1 << endl;
				opengeostate << "nColumns=13" << endl;
				opengeostate << "inDegrees=no" << endl;
				opengeostate << "" << endl;
				opengeostate << "Units are S.I.units(second, meters, Newtons, ...)" << endl;
				opengeostate << "Angles are not in degrees." << endl;
				opengeostate << "" << endl;
				opengeostate << "endheader" << endl;

				opengeostate << "  time  " << bd1 << "_tilt  " << bd1 << "_list  " << bd1 << "_rotation  " << bd1 << "_tx  " << bd1 << "_ty  " << bd1 << "_tz  " << bd2 << "_tilt  " << bd2 << "_list  " << bd2 << "_rotation  " << bd2 << "_tx  " << bd2 << "_ty  " << bd2 << "_tz  " << endl;
				for (int i = 0; i < sizer; ++i){
					opengeostate << "  " << timefinal[i] << "  " << fOz[i] << "  " << fOx[i] << "  " << fOy[i] << "  " << fPz[i] << "  " << fPx[i] << "  " << fPy[i] << "  " << tOz[i] << "  " << tOx[i] << "  " << tOy[i] << "  " << tPz[i] << "  " << tPx[i] << "  " << tPy[i] << endl;

				}
				Run();

				////////////////////////////////////
				//////////////// b. The VISION SECTION start////////////
				/////////////////////////////////////

				/////////////////ASK to MAKE CHANGES IN THE POSITIONS OF MOTION///////////////////
				char caser4 = 'y';
				char caser5 = 'y';

				while (caser5 == 'y'){


					cout << "" << endl;
					cout << "Do you want to make changes in position vector based of the pre-FEM_motion_vision you just saw? (y/n)" << endl;
					char caser1;
					cin >> caser1;
					if (caser1 == 'n'){
						caser5 = 'n';
					}

					if (caser1 == 'y'){
						resamplernow = 'y';
						while (caser4 == 'y'){

							cout << "Which of the first body 6DOF you want to make changes first? (x/y/z/X/Y/Z)" << endl;
							cout << "P.S. the capital X,Y,Z are the rotation degrees of freedom" << endl;
							char L;
							cin >> L;
							cout << "Now give: first time value,second the new offset value of the dof." << endl;
							if (L == 'x'){
								char caser3 = 'y';
								while (caser3 == 'y'){
									double timee;
									double x;
									double caser2;
									cout << "You want : " << endl;
									cout << "1) a time interval with fix value? " << endl;
									cout << "2) a time interval with linear max and min value?" << endl;
									cout << "3) a point? " << endl;
									cout << "choose by but 1/2/3..." << endl;
									cin >> caser2;
									if (caser2 == 1){
										cout << "START TIME:" << endl;
										cin >> timee;
										cout << "END TIME:" << endl;
										double timel;
										cin >> timel;
										cout << "x_value:" << endl;
										cin >> x;
										int i = 0;
										while (timefinal[i] <= timee){
											++i;
										}
										if (timee == 0){ i = 0; }
										int o = 0;
										while (timefinal[o] <= timel){
											++o;
										}
										for (int run = i; run < o + 1; ++run){
											fPx[run] = fPx[run] + x;
										}
									}
									if (caser2 == 2){
										double xm;
										cout << "START TIME:" << endl;
										cin >> timee;
										cout << "END TIME:" << endl;
										double timel;
										cin >> timel;
										cout << "max_x_value:" << endl;
										cin >> x;
										cout << "min_x_value:" << endl;
										cin >> xm;

										int i = 0;
										while (timefinal[i] <= timee){
											++i;
										}
										if (timee == 0){ i = 0; }
										int o = 0;
										while (timefinal[o] <= timel){
											++o;
										}
										int dist = o - i;
										double dist2 = x - xm;
										double step = dist2 / dist;
										double xl = xm;
										for (int run = i; run < o + 1; ++run){
											xl = xl + step;
											fPx[run] = fPx[run] + xl;
										}
									}
									if (caser2 == 3){
										int i = -1;
										while (i == -1){
											cout << "TIME:" << endl;
											cin >> timee;
											cout << "x_value:" << endl;
											cin >> x;
											int k = 0;
											while (k < timefinal.size()){
												if (timefinal[k] == timee){
													i = k;
												}++k;
											}
											if (i == -1){ cout << "Error no exist value of time of the point" << endl; }
										}
										if (timee == 0){ i = 0; }
										fPx[i] = fPx[i] + x;
									}
									cout << "Do you want an other value " << L << " axis change? (y/n)" << endl;
									cin >> caser3;
								}
							}
							if (L == 'y'){
								char caser3 = 'y';
								while (caser3 == 'y'){
									double timee;
									double y;
									double caser2;
									cout << "You want : " << endl;
									cout << "1) a time interval with fix value? " << endl;
									cout << "2) a time interval with linear max and min value?" << endl;
									cout << "3) a point? " << endl;
									cout << "choose by but 1/2/3..." << endl;
									cin >> caser2;
									if (caser2 == 1){
										cout << "START TIME:" << endl;
										cin >> timee;
										cout << "END TIME:" << endl;
										double timel;
										cin >> timel;
										cout << "y_value:" << endl;
										cin >> y;
										int i = 0;
										while (timefinal[i] <= timee){
											++i;
										}
										if (timee == 0){ i = 0; }
										int o = 0;
										while (timefinal[o] <= timel){
											++o;
										}
										for (int run = i; run < o + 1; ++run){
											fPy[run] = fPy[run] + y;
										}
									}
									if (caser2 == 2){
										double ym;
										cout << "START TIME:" << endl;
										cin >> timee;
										cout << "END TIME:" << endl;
										double timel;
										cin >> timel;
										cout << "max_y_value:" << endl;
										cin >> y;
										cout << "min_y_value:" << endl;
										cin >> ym;

										int i = 0;
										while (timefinal[i] <= timee){
											++i;
										}
										if (timee == 0){ i = 0; }
										int o = 0;
										while (timefinal[o] <= timel){
											++o;
										}
										int dist = o - i;
										double dist2 = y - ym;
										double step = dist2 / dist;
										double yl = ym;
										for (int run = i; run < o + 1; ++run){
											yl = yl + step;
											fPy[run] = fPy[run] + yl;
										}
									}
									if (caser2 == 3){
										int i = -1;
										while (i == -1){
											cout << "TIME:" << endl;
											cin >> timee;
											cout << "y_value:" << endl;
											cin >> y;
											int k = 0;
											while (k < timefinal.size()){
												if (timefinal[k] == timee){
													i = k;
												}++k;
											}
											if (i == -1){ cout << "Error no exist value of time of the point" << endl; }
										}
										if (timee == 0){ i = 0; }
										fPy[i] = fPy[i] + y;
									}
									cout << "Do you want an other value " << L << " axis change? (y/n)" << endl;
									cin >> caser3;
								}
							}


							if (L == 'z'){
								char caser3 = 'y';
								while (caser3 == 'y'){
									double timee;
									double z;
									double caser2;
									cout << "You want : " << endl;
									cout << "1) a time interval with fix value? " << endl;
									cout << "2) a time interval with linear max and min value?" << endl;
									cout << "3) a point? " << endl;
									cout << "choose by but 1/2/3..." << endl;
									cin >> caser2;
									if (caser2 == 1){
										cout << "START TIME:" << endl;
										cin >> timee;
										cout << "END TIME:" << endl;
										double timel;
										cin >> timel;
										cout << "z_value:" << endl;
										cin >> z;
										int i = 0;
										while (timefinal[i] <= timee){
											++i;
										}
										if (timee == 0){ i = 0; }
										int o = 0;
										while (timefinal[o] <= timel){
											++o;
										}
										for (int run = i; run < o + 1; ++run){
											fPz[run] = fPz[run] + z;
										}
									}
									if (caser2 == 2){
										double zm;
										cout << "START TIME:" << endl;
										cin >> timee;
										cout << "END TIME:" << endl;
										double timel;
										cin >> timel;
										cout << "max_z_value:" << endl;
										cin >> z;
										cout << "min_z_value:" << endl;
										cin >> zm;

										int i = 0;
										while (timefinal[i] <= timee){
											++i;
										}
										if (timee == 0){ i = 0; }
										int o = 0;
										while (timefinal[o] <= timel){
											++o;
										}
										int dist = o - i;
										double dist2 = z - zm;
										double step = dist2 / dist;
										double zl = zm;
										for (int run = i; run < o + 1; ++run){
											zl = zl + step;
											fPz[run] = fPz[run] + zl;
										}
									}
									if (caser2 == 3){
										int i = -1;
										while (i == -1){
											cout << "TIME:" << endl;
											cin >> timee;
											cout << "z_value:" << endl;
											cin >> z;
											int k = 0;
											while (k < timefinal.size()){
												if (timefinal[k] == timee){
													i = k;
												}++k;
											}
											if (i == -1){ cout << "Error no exist value of time of the point" << endl; }
										}
										if (timee == 0){ i = 0; }
										fPz[i] = fPz[i] + z;
									}
									cout << "Do you want an other value " << L << " axis change? (y/n)" << endl;
									cin >> caser3;
								}
							}
							if (L == 'X'){
								char caser3 = 'y';
								while (caser3 == 'y'){
									double timee;
									double X;
									double caser2;
									cout << "You want : " << endl;
									cout << "1) a time interval with fix value? " << endl;
									cout << "2) a time interval with linear max and min value?" << endl;
									cout << "3) a point? " << endl;
									cout << "choose by but 1/2/3..." << endl;
									cin >> caser2;
									if (caser2 == 1){
										cout << "START TIME:" << endl;
										cin >> timee;
										cout << "END TIME:" << endl;
										double timel;
										cin >> timel;
										cout << "Rx_value:" << endl;
										cin >> X;
										int i = 0;
										while (timefinal[i] <= timee){
											++i;
										}if (timee == 0){ i = 0; }
										int o = 0;
										while (timefinal[o] <= timel){
											++o;
										}
										for (int run = i; run < o + 1; ++run){
											fOx[run] = fOx[run] + X*3.14 / 180;
										}
									}
									if (caser2 == 2){
										double Rxm;
										cout << "START TIME:" << endl;
										cin >> timee;
										cout << "END TIME:" << endl;
										double timel;
										cin >> timel;
										cout << "max_Rx_value:" << endl;
										cin >> X;
										cout << "min_Rx_value:" << endl;
										cin >> Rxm;

										int i = 0;
										while (timefinal[i] <= timee){
											++i;
										}
										if (timee == 0){ i = 0; }
										int o = 0;
										while (timefinal[o] <= timel){
											++o;
										}
										int dist = o - i;
										double dist2 = X - Rxm;
										double step = dist2 / dist;
										double Rxl = Rxm;
										for (int run = i; run < o + 1; ++run){
											Rxl = Rxl + step;
											fOx[run] = fOx[run] + Rxl*3.14 / 180;
										}
									}
									if (caser2 == 3){
										int i = -1;
										while (i == -1){
											cout << "TIME:" << endl;
											cin >> timee;
											cout << "Rx_value:" << endl;
											cin >> X;
											int k = 0;
											while (k < timefinal.size()){
												if (timefinal[k] == timee){
													i = k;
												}++k;
											}
											if (i == -1){ cout << "Error no exist value of time of the point" << endl; }
										}
										if (timee == 0){ i = 0; }
										fOx[i] = fOx[i] + X*3.14 / 180;
									}
									cout << "Do you want an other value " << L << " axis change? (y/n)" << endl;
									cin >> caser3;
								}

							}
							if (L == 'Y'){
								char caser3 = 'y';
								while (caser3 == 'y'){
									double timee;
									double Ry;
									double caser2;
									cout << "You want : " << endl;
									cout << "1) a time interval with fix value? " << endl;
									cout << "2) a time interval with linear max and min value?" << endl;
									cout << "3) a point? " << endl;
									cout << "choose by but 1/2/3..." << endl;
									cin >> caser2;
									if (caser2 == 1){
										cout << "START TIME:" << endl;
										cin >> timee;
										cout << "END TIME:" << endl;
										double timel;
										cin >> timel;
										cout << "Ry_value:" << endl;
										cin >> Ry;
										int i = 0;
										while (timefinal[i] <= timee){
											++i;
										}
										if (timee == 0){ i = 0; }
										int o = 0;
										while (timefinal[o] <= timel){
											++o;
										}
										for (int run = i; run < o + 1; ++run){
											fOy[run] = fOy[run] + Ry*3.14 / 180;
										}
									}
									if (caser2 == 2){
										double Rym;
										cout << "START TIME:" << endl;
										cin >> timee;
										cout << "END TIME:" << endl;
										double timel;
										cin >> timel;
										cout << "max_Ry_value:" << endl;
										cin >> Ry;
										cout << "min_Ry_value:" << endl;
										cin >> Rym;

										int i = 0;
										while (timefinal[i] <= timee){
											++i;
										}
										if (timee == 0){ i = 0; }
										int o = 0;
										while (timefinal[o] <= timel){
											++o;
										}
										int dist = o - i;
										double dist2 = Ry - Rym;
										double step = dist2 / dist;
										double Ryl = Rym;
										for (int run = i; run < o + 1; ++run){
											Ryl = Ryl + step;
											fOy[run] = fOy[run] + Ryl*3.14 / 180;
										}
									}
									if (caser2 == 3){
										int i = -1;
										while (i == -1){
											cout << "TIME:" << endl;
											cin >> timee;
											cout << "Ry_value:" << endl;
											cin >> Ry;
											int k = 0;
											while (k < timefinal.size()){
												if (timefinal[k] == timee){
													i = k;

												}++k;
											}
											if (i == -1){ cout << "Error no exist value of time of the point" << endl; }
										}
										if (timee == 0){ i = 0; }
										fOy[i] = fOy[i] + Ry*3.14 / 180;
									}
									cout << "Do you want an other value " << L << " axis change? (y/n)" << endl;
									cin >> caser3;
								}
							}
							if (L == 'Z'){
								char caser3 = 'y';
								while (caser3 == 'y'){
									double timee;
									double Rz;
									double caser2;
									cout << "You want : " << endl;
									cout << "1) a time interval with fix value? " << endl;
									cout << "2) a time interval with linear max and min value?" << endl;
									cout << "3) a point? " << endl;
									cout << "choose by but 1/2/3..." << endl;
									cin >> caser2;
									if (caser2 == 1){
										cout << "START TIME:" << endl;
										cin >> timee;
										cout << "END TIME:" << endl;
										double timel;
										cin >> timel;
										cout << "Rz_value:" << endl;
										cin >> Rz;
										int i = 0;
										while (timefinal[i] <= timee){
											++i;
										}if (timee == 0){ i = 0; }
										int o = 0;
										while (timefinal[o] <= timel){
											++o;
										}
										for (int run = i; run < o + 1; ++run){
											fOz[run] = fOz[run] + Rz*3.14 / 180;
										}
									}
									if (caser2 == 2){
										double Rzm;
										cout << "START TIME:" << endl;
										cin >> timee;
										cout << "END TIME:" << endl;
										double timel;
										cin >> timel;
										cout << "max_Rz_value:" << endl;
										cin >> Rz;
										cout << "min_Rz_value:" << endl;
										cin >> Rzm;

										int i = 0;
										while (timefinal[i] <= timee){
											++i;
										}
										if (timee == 0){ i = 0; }
										int o = 0;
										while (timefinal[o] <= timel){
											++o;
										}
										int dist = o - i;
										double dist2 = Rz - Rzm;
										double step = dist2 / dist;
										double Rzl = Rzm;
										for (int run = i; run < o + 1; ++run){
											Rzl = Rzl + step;
											fOz[run] = fOz[run] + Rzl*3.14 / 180;
										}
									}
									if (caser2 == 3){
										int i = -1;
										while (i == -1){
											cout << "TIME:" << endl;
											cin >> timee;
											cout << "Rz_value:" << endl;
											cin >> Rz;
											int k = 0;
											while (k < timefinal.size()){
												if (timefinal[k] == timee){
													i = k;
												}++k;
											}
											if (i == -1){ cout << "Error no exist value of time of the point" << endl; }
										}
										if (timee == 0){ i = 0; }
										fOz[i] = fOz[i] + Rz*3.14 / 180;
									}
									cout << "Do you want an other value " << L << " axis change? (y/n)" << endl;
									cin >> caser3;

								}
							}
							cout << "Do you want to change another DoF motion values? (y/n)" << endl;
							cin >> caser4;
						}
						cout << "Do you want to vision again the motion? (y/n)" << endl;
						cin >> caser5;
						if (caser5 == 'y'){

							///WRITTING THE MOTION FILE for geofem
							ofstream opengeostate;
							INIReader ini = INIReader(INI_FILE);
							string resultDir1 = BASE_DIR + ini.Get("FEBIOSTEP", "GEO", "");
							string geof = ini.Get("FEBIOSTEP", "GEOF", "");
							string geos = ini.Get("FEBIOSTEP", "GEOS", "");
							string bd1 = ini.Get("BODYFORCES", "BDNAME1", "");
							string bd2 = ini.Get("BODYFORCES", "BDNAME2", "");
							//char itter = itteration + '0';
							////create a new folder for the analysis/////////
							int sizer = timefinal.size();

							opengeostate = ofstream(resultDir1 + "/motionstate.mot", ofstream::out);
							opengeostate << "Coordinates" << endl;
							opengeostate << "version=1" << endl;
							opengeostate << "nRows=" << sizer - 1 << endl;
							opengeostate << "nColumns=13" << endl;
							opengeostate << "inDegrees=no" << endl;
							opengeostate << "" << endl;
							opengeostate << "Units are S.I.units(second, meters, Newtons, ...)" << endl;
							opengeostate << "Angles are not in degrees." << endl;
							opengeostate << "" << endl;
							opengeostate << "endheader" << endl;

							opengeostate << "  time  " << bd1 << "_tilt  " << bd1 << "_list  " << bd1 << "_rotation  " << bd1 << "_tx  " << bd1 << "_ty  " << bd1 << "_tz  " << bd2 << "_tilt  " << bd2 << "_list  " << bd2 << "_rotation  " << bd2 << "_tx  " << bd2 << "_ty  " << bd2 << "_tz  " << endl;
							for (int i = 0; i < sizer; ++i){
								opengeostate << "  " << timefinal[i] << "  " << fOz[i] << "  " << fOx[i] << "  " << fOy[i] << "  " << fPz[i] << "  " << fPx[i] << "  " << fPy[i] << "  " << tOz[i] << "  " << tOx[i] << "  " << tOy[i] << "  " << tPz[i] << "  " << tPx[i] << "  " << tPy[i] << endl;


							}
							caser4 = 'y';
						}
					}
				}
			}


			////////////////////////////////////
			//////////////// b. The VISION SECTION end////////////
			/////////////////////////////////////

			////////////////////////////////////
			//////////////// b1. Forces resample for the new positions by user start////////////
			/////////////////////////////////////

			if (resamplernow == 'y'){
				DOFResample dresl;
				dresl.store(timefinal, 0);
				dresl.store(fPx, 1);
				dresl.store(fPy, 2);
				dresl.store(fPz, 3);
				dresl.store(fOx, 4);
				dresl.store(fOy, 5);
				dresl.store(fOz, 6);
				dresl.store(tPx, 7);
				dresl.store(tPy, 8);
				dresl.store(tPz, 9);
				dresl.store(tOx, 10);
				dresl.store(tOy, 11);
				dresl.store(tOz, 12);
				dresl.store(fFx, 13);
				dresl.store(fFy, 14);
				dresl.store(fFz, 15);
				dresl.store(fMx, 16);
				dresl.store(fMy, 17);
				dresl.store(fMz, 18);
				dresl.store(tFx, 19);
				dresl.store(tFy, 20);
				dresl.store(tFz, 21);
				dresl.store(tMx, 22);
				dresl.store(tMy, 23);
				dresl.store(tMz, 24);
				dresl.detect(itteration, kind, endend, resultDir2, 1);

				timefinal = dresl.return_vect(0);

				/////global vector////
				fPx = dresl.return_vect(1);
				fPy = dresl.return_vect(2);
				fPz = dresl.return_vect(3);
				fOx = dresl.return_vect(4);
				fOy = dresl.return_vect(5);
				fOz = dresl.return_vect(6);
				tPx = dresl.return_vect(7);
				tPy = dresl.return_vect(8);
				tPz = dresl.return_vect(9);
				tOx = dresl.return_vect(10);
				tOy = dresl.return_vect(11);
				tOz = dresl.return_vect(12);
				/////globall vector////
				fFx = dresl.return_vect(13);
				fFy = dresl.return_vect(14);
				fFz = dresl.return_vect(15);
				fMx = dresl.return_vect(16);
				fMy = dresl.return_vect(17);
				fMz = dresl.return_vect(18);
				tFx = dresl.return_vect(19);
				tFy = dresl.return_vect(20);
				tFz = dresl.return_vect(21);
				tMx = dresl.return_vect(22);
				tMy = dresl.return_vect(23);
				tMz = dresl.return_vect(24);
				dresl.~DOFResample();

				cout << "Resample the initial velocities base the new prescribe trajectory" << endl;
				vel.store(timefinal, 0);
				vel.store(fPx, 1);
				vel.store(fPy, 2);
				vel.store(fPz, 3);
				vel.store(fOx, 4);
				vel.store(fOy, 5);
				vel.store(fOz, 6);
				vel.store(tPx, 7);
				vel.store(tPy, 8);
				vel.store(tPz, 9);
				vel.store(tOx, 10);
				vel.store(tOy, 11);
				vel.store(tOz, 12);
				vel.initial(itteration, 'n', 1, 'P');
				vel.~Initial_VEL();

			}
			////////////////////////////////////
			//////////////// b1. Forces resample for the new positions by user end////////////
			/////////////////////////////////////

			////////////////////////////////////
			//////////////// c. The STRUCTOR SECTION start////////////
			/////////////////////////////////////
			////////////////////
			BF_structor bff;
			bff.begining(itteration, kind, endend, resultDir2, timefinal, fFx, fFy, fFz, fMx, fMy, fMz, tFx, tFy, tFz, tMx, tMy, tMz);
			//bff.~BF_structor();
			BK_structor bkk;
			bkk.begining(itteration, kind, 0, endend, resultDir2, timefinal, fPx, fPy, fPz, fOx, fOy, fOz, tPx, tPy, tPz, tOx, tOy, tOz);
			//bkk.~BK_structor();
			////////////////////////////////////
			//////////////// c. The STRUCTOR SECTION end////////////
			/////////////////////////////////////

		

	}//end of no time resampler


	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////// B. YES  TIME RESAMPLE  ///////////////////////////////////////////////////////////////////////////
	/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


	if (dicession1 == "y"){
		char resamplernow = 'n';
		//////////////////////////////////////////////////////////////
		//////////Store the vectors for time resampler class/////////////////
		/////////////////////////////////////////////////////////////
		ReSampler res;
		res.store(timefinal, 0);
		res.store(fFx, 1);
		res.store(fFy, 2);
		res.store(fFz, 3);
		res.store(fMx, 4);
		res.store(fMy, 5);
		res.store(fMz, 6);
		Vector timenew = res.runvec(1, resultDir2, res.detector(0, timefinal.size(), timefinal, fFx));
		//cout << "The time vector after the forces DoF of first body: " << endl;
		//cout << timenew3 << endl;
		res.~ReSampler();

		ReSampler res1;
		res1.store(timefinal, 0);
		res1.store(tFx, 1);
		res1.store(tFy, 2);
		res1.store(tFz, 3);
		res1.store(tMx, 4);
		res1.store(tMy, 5);
		res1.store(tMz, 6);
		timenew = res1.runvec(2, resultDir2, timenew);
		//cout << "The time vector after the forces DoF of second body: " << endl;
		//cout << timenew1 << endl;
		res1.~ReSampler();

		ReSampler res2;
		res2.store(timefinal, 0);
		res2.store(fPx, 1);
		res2.store(fPy, 2);
		res2.store(fPz, 3);
		res2.store(fOx, 4);
		res2.store(fOy, 5);
		res2.store(fOz, 6);
		timenew = res2.runvec(3, resultDir2, timenew);
		//cout << "The time vector after the positions DoF of first body: " << endl;
		//cout << timenew2 << endl;
		res2.~ReSampler();

		ReSampler res3;
		res3.store(timefinal, 0);
		res3.store(tPx, 1);
		res3.store(tPy, 2);
		res3.store(tPz, 3);
		res3.store(tOx, 4);
		res3.store(tOy, 5);
		res3.store(tOz, 6);
		timenew = res3.runvec(4, resultDir2, timenew);
		//cout << "The time vector after the positions DoF of second body: " << endl;
		//cout << timenew << endl;
		res3.~ReSampler();


		Vector fFxf(timenew.size(), 1);
		Vector fFyf(timenew.size(), 1);
		Vector fFzf(timenew.size(), 1);
		Vector fMxf(timenew.size(), 1);
		Vector fMyf(timenew.size(), 1);
		Vector fMzf(timenew.size(), 1);
		Vector tFxf(timenew.size(), 1);
		Vector tFyf(timenew.size(), 1);
		Vector tFzf(timenew.size(), 1);
		Vector tMxf(timenew.size(), 1);
		Vector tMyf(timenew.size(), 1);
		Vector tMzf(timenew.size(), 1);
		Vector fPxf(timenew.size(), 1);
		Vector fPyf(timenew.size(), 1);
		Vector fPzf(timenew.size(), 1);
		Vector fOxf(timenew.size(), 1);
		Vector fOyf(timenew.size(), 1);
		Vector fOzf(timenew.size(), 1);
		Vector tPxf(timenew.size(), 1);
		Vector tPyf(timenew.size(), 1);
		Vector tPzf(timenew.size(), 1);
		Vector tOxf(timenew.size(), 1);
		Vector tOyf(timenew.size(), 1);
		Vector tOzf(timenew.size(), 1);


		ReSampler finalres;

		finalres.store(timefinal, 0);
		/////forces//////////////////////////
		fFxf = finalres.DoFbuilder(fFx, timenew);
		fFyf = (finalres.DoFbuilder(fFy, timenew));
		fFzf = (finalres.DoFbuilder(fFz, timenew));
		fMxf = (finalres.DoFbuilder(fMx, timenew));
		fMyf = (finalres.DoFbuilder(fMy, timenew));
		fMzf = (finalres.DoFbuilder(fMz, timenew));
		tFxf = (finalres.DoFbuilder(tFx, timenew));
		tFyf = (finalres.DoFbuilder(tFy, timenew));
		tFzf = (finalres.DoFbuilder(tFz, timenew));
		tMxf = (finalres.DoFbuilder(tMx, timenew));
		tMyf = (finalres.DoFbuilder(tMy, timenew));
		tMzf = (finalres.DoFbuilder(tMz, timenew));
		finalres.~ReSampler();


		ReSampler finalres2;

		finalres2.store(timefinal, 0);
		/////positions//////////////////////////
		fPxf = (finalres2.DoFbuilder(fPx, timenew));
		fPyf = (finalres2.DoFbuilder(fPy, timenew));
		fPzf = (finalres2.DoFbuilder(fPz, timenew));
		fOxf = (finalres2.DoFbuilder(fOx, timenew));
		fOyf = (finalres2.DoFbuilder(fOy, timenew));
		fOzf = (finalres2.DoFbuilder(fOz, timenew));
		tPxf = (finalres2.DoFbuilder(tPx, timenew));
		tPyf = (finalres2.DoFbuilder(tPy, timenew));
		tPzf = (finalres2.DoFbuilder(tPz, timenew));
		tOxf = (finalres2.DoFbuilder(tOx, timenew));
		tOyf = (finalres2.DoFbuilder(tOy, timenew));
		tOzf = (finalres2.DoFbuilder(tOz, timenew));
		finalres2.~ReSampler();


		//////////////////////////// and call the structor class for write the feb file//////////////////////////////////////////////////
		int endend = timenew.size() - 1;
		int firstsize = timefinal.size();
		cout << "" << endl;
		cout << "CONCLUSION OF RESAMPLER ANALYSIS" << endl;
		cout << "" << endl;
		cout << "The size of DoF's sample in the beginning of ReSample phase was: " << firstsize << " elements" << endl;
		cout << "The size of the ReSample time vector for the DoF is now: " << endend << " elements" << endl;

		TSC tsc;
		tsc.run(fPxf, fPyf, fPzf, fOxf, fOyf, fOzf, tPxf, tPyf, tPzf, tOxf, tOyf, tOzf, timenew);
		tsc.~TSC();

		char which;

		
			////////////////////////////////////
			//////////////// a. The initial velocity RESAMPLER  start////////////
			/////////////////////////////////////
			Initial_VEL vel;
			vel.initial(itteration, 'n', 3, 'P');

			////////////////////////////////////
			//////////////// a. The initial velocity RESAMPLER  end////////////
			/////////////////////////////////////



			cout << "Do you want a visualizer of pre motion of the FEBio geometry? (y/n)" << endl;
			string f = ini.Get("BASICSETUP", "Visualizer_premotion", "");
			cout << f << endl;
			if (f == "y"){

				//////WRITE THE XML FILE//////
				ofstream opengeostate;

				INIReader ini = INIReader(INI_FILE);
				string resultDir1 = BASE_DIR + ini.Get("FEBIOSTEP", "GEO", "");
				string geof = ini.Get("FEBIOSTEP", "GEOF", "");
				string geos = ini.Get("FEBIOSTEP", "GEOS", "");
				string bd1 = ini.Get("BODYFORCES", "BDNAME1", "");
				string bd2 = ini.Get("BODYFORCES", "BDNAME2", "");
				//char itter = itteration + '0';
				////create a new folder for the analysis/////////
				int sizer = timenew.size();

				opengeostate = ofstream(resultDir1 + "/motionstate.mot", ofstream::out);
				opengeostate << "Coordinates" << endl;
				opengeostate << "version=1" << endl;
				opengeostate << "nRows=" << sizer - 1 << endl;
				opengeostate << "nColumns=13" << endl;
				opengeostate << "inDegrees=no" << endl;
				opengeostate << "" << endl;
				opengeostate << "Units are S.I.units(second, meters, Newtons, ...)" << endl;
				opengeostate << "Angles are not in degrees." << endl;
				opengeostate << "" << endl;
				opengeostate << "endheader" << endl;

				opengeostate << "  time  " << bd1 << "_tilt  " << bd1 << "_list  " << bd1 << "_rotation  " << bd1 << "_tx  " << bd1 << "_ty  " << bd1 << "_tz  " << bd2 << "_tilt  " << bd2 << "_list  " << bd2 << "_rotation  " << bd2 << "_tx  " << bd2 << "_ty  " << bd2 << "_tz  " << endl;
				for (int i = 0; i < sizer; ++i){
					opengeostate << "  " << timenew[i] << "  " << fOzf[i] << "  " << fOxf[i] << "  " << fOyf[i] << "  " << fPzf[i] << "  " << fPxf[i] << "  " << fPyf[i] << "  " << tOzf[i] << "  " << tOxf[i] << "  " << tOyf[i] << "  " << tPzf[i] << "  " << tPxf[i] << "  " << tPyf[i] << endl;

				}
				Run();

				////////////////////////////////////
				//////////////// b. The VISION SECTION start////////////
				/////////////////////////////////////

				/////////////////ASK to MAKE CHANGES IN THE POSITIONS OF MOTION///////////////////
				char caser4 = 'y';
				char caser5 = 'y';

				while (caser5 == 'y'){


					cout << "" << endl;
					cout << "Do you want to make changes in position vector based of the pre-FEM_motion_vision you just saw? (y/n)" << endl;
					char caser1;
					cin >> caser1;
					if (caser1 == 'n'){
						caser5 = 'n';
					}

					if (caser1 == 'y'){
						resamplernow = 'y';
						while (caser4 == 'y'){

							cout << "Which of the first body 6DOF you want to make changes first? (x/y/z/X/Y/Z)" << endl;
							cout << "P.S. the capital X,Y,Z are the rotation degrees of freedom" << endl;
							char L;
							cin >> L;
							cout << "Now give: first time value,second the new offset value of the dof." << endl;
							if (L == 'x'){
								char caser3 = 'y';
								while (caser3 == 'y'){
									double timee;
									double x;
									double caser2;
									cout << "You want : " << endl;
									cout << "1) a time interval with fix value? " << endl;
									cout << "2) a time interval with linear max and min value?" << endl;
									cout << "3) a point? " << endl;
									cout << "choose by but 1/2/3..." << endl;
									cin >> caser2;
									if (caser2 == 1){
										cout << "START TIME:" << endl;
										cin >> timee;
										cout << "END TIME:" << endl;
										double timel;
										cin >> timel;
										cout << "x_value:" << endl;
										cin >> x;
										int i = 0;
										while (timenew[i] <= timee){
											++i;
										}
										if (timee == 0){ i = 0; }
										int o = 0;
										while (timenew[o] <= timel){
											++o;
										}
										for (int run = i; run < o + 1; ++run){
											fPxf[run] = fPxf[run] + x;
										}
									}
									if (caser2 == 2){
										double xm;
										cout << "START TIME:" << endl;
										cin >> timee;
										cout << "END TIME:" << endl;
										double timel;
										cin >> timel;
										cout << "max_x_value:" << endl;
										cin >> x;
										cout << "min_x_value:" << endl;
										cin >> xm;

										int i = 0;
										while (timenew[i] <= timee){
											++i;
										}
										if (timee == 0){ i = 0; }
										int o = 0;
										while (timenew[o] <= timel){
											++o;
										}
										int dist = o - i;
										double dist2 = x - xm;
										double step = dist2 / dist;
										double xl = xm;
										for (int run = i; run < o + 1; ++run){
											xl = xl + step;
											fPxf[run] = fPxf[run] + xl;
										}
									}
									if (caser2 == 3){
										int i = -1;
										while (i == -1){
											cout << "TIME:" << endl;
											cin >> timee;
											cout << "x_value:" << endl;
											cin >> x;
											int k = 0;
											while (k < timenew.size()){
												if (timenew[k] == x){
													i = k;
												}++k;
											}
											if (i == -1){ cout << "Error no exist value of time of the point" << endl; }
										}
										if (timee == 0){ i = 0; }
										fPxf[i] = fPxf[i] + x;
									}
									cout << "Do you want an other value " << L << " axis change? (y/n)" << endl;
									cin >> caser3;
								}
							}
							if (L == 'y'){
								char caser3 = 'y';
								while (caser3 == 'y'){
									double timee;
									double y;
									double caser2;
									cout << "You want : " << endl;
									cout << "1) a time interval with fix value? " << endl;
									cout << "2) a time interval with linear max and min value?" << endl;
									cout << "3) a point? " << endl;
									cout << "choose by but 1/2/3..." << endl;
									cin >> caser2;
									if (caser2 == 1){
										cout << "START TIME:" << endl;
										cin >> timee;
										cout << "END TIME:" << endl;
										double timel;
										cin >> timel;
										cout << "y_value:" << endl;
										cin >> y;
										int i = 0;
										while (timenew[i] <= timee){
											++i;
										}
										if (timee == 0){ i = 0; }
										int o = 0;
										while (timenew[o] <= timel){
											++o;
										}
										for (int run = i; run < o + 1; ++run){
											fPyf[run] = fPyf[run] + y;
										}
									}
									if (caser2 == 2){
										double ym;
										cout << "START TIME:" << endl;
										cin >> timee;
										cout << "END TIME:" << endl;
										double timel;
										cin >> timel;
										cout << "max_y_value:" << endl;
										cin >> y;
										cout << "min_y_value:" << endl;
										cin >> ym;

										int i = 0;
										while (timenew[i] <= timee){
											++i;
										}
										if (timee == 0){ i = 0; }
										int o = 0;
										while (timenew[o] <= timel){
											++o;
										}
										int dist = o - i;
										double dist2 = y - ym;
										double step = dist2 / dist;
										double yl = ym;
										for (int run = i; run < o + 1; ++run){
											yl = yl + step;
											fPyf[run] = fPyf[run] + yl;
										}
									}
									if (caser2 == 3){
										int i = -1;
										while (i == -1){
											cout << "TIME:" << endl;
											cin >> timee;
											cout << "y_value:" << endl;
											cin >> y;
											int k = 0;
											while (k < timenew.size()){
												if (timenew[k] == y){
													i = k;
												}++k;
											}
											if (i == -1){ cout << "Error no exist value of time of the point" << endl; }
										}
										if (timee == 0){ i = 0; }
										fPyf[i] = fPyf[i] + y;
									}
									cout << "Do you want an other value " << L << " axis change? (y/n)" << endl;
									cin >> caser3;
								}
							}


							if (L == 'z'){
								char caser3 = 'y';
								while (caser3 == 'y'){
									double timee;
									double z;
									double caser2;
									cout << "You want : " << endl;
									cout << "1) a time interval with fix value? " << endl;
									cout << "2) a time interval with linear max and min value?" << endl;
									cout << "3) a point? " << endl;
									cout << "choose by but 1/2/3..." << endl;
									cin >> caser2;
									if (caser2 == 1){
										cout << "START TIME:" << endl;
										cin >> timee;
										cout << "END TIME:" << endl;
										double timel;
										cin >> timel;
										cout << "z_value:" << endl;
										cin >> z;
										int i = 0;
										while (timenew[i] <= timee){
											++i;
										}
										if (timee == 0){ i = 0; }
										int o = 0;
										while (timenew[o] <= timel){
											++o;
										}
										for (int run = i; run < o + 1; ++run){
											fPzf[run] = fPzf[run] + z;
										}
									}
									if (caser2 == 2){
										double zm;
										cout << "START TIME:" << endl;
										cin >> timee;
										cout << "END TIME:" << endl;
										double timel;
										cin >> timel;
										cout << "max_z_value:" << endl;
										cin >> z;
										cout << "min_z_value:" << endl;
										cin >> zm;

										int i = 0;
										while (timenew[i] <= timee){
											++i;
										}
										if (timee == 0){ i = 0; }
										int o = 0;
										while (timenew[o] <= timel){
											++o;
										}
										int dist = o - i;
										double dist2 = z - zm;
										double step = dist2 / dist;
										double zl = zm;
										for (int run = i; run < o + 1; ++run){
											zl = zl + step;
											fPzf[run] = fPzf[run] + zl;
										}
									}
									if (caser2 == 3){
										int i = -1;
										while (i == -1){
											cout << "TIME:" << endl;
											cin >> timee;
											cout << "z_value:" << endl;
											cin >> z;
											int k = 0;
											while (k < timenew.size()){
												if (timenew[k] == z){
													i = k;
												}++k;
											}
											if (i == -1){ cout << "Error no exist value of time of the point" << endl; }
										}
										if (timee == 0){ i = 0; }
										fPzf[i] = fPzf[i] + z;
									}
									cout << "Do you want an other value " << L << " axis change? (y/n)" << endl;
									cin >> caser3;
								}
							}
							if (L == 'X'){
								char caser3 = 'y';
								while (caser3 == 'y'){
									double timee;
									double X;
									double caser2;
									cout << "You want : " << endl;
									cout << "1) a time interval with fix value? " << endl;
									cout << "2) a time interval with linear max and min value?" << endl;
									cout << "3) a point? " << endl;
									cout << "choose by but 1/2/3..." << endl;
									cin >> caser2;
									if (caser2 == 1){
										cout << "START TIME:" << endl;
										cin >> timee;
										cout << "END TIME:" << endl;
										double timel;
										cin >> timel;
										cout << "Rx_value:" << endl;
										cin >> X;
										int i = 0;
										while (timenew[i] <= timee){
											++i;
										}if (timee == 0){ i = 0; }
										int o = 0;
										while (timenew[o] <= timel){
											++o;
										}
										for (int run = i; run < o + 1; ++run){
											fOxf[run] = fOxf[run] + X*3.14 / 180;
										}
									}
									if (caser2 == 2){
										double Rxm;
										cout << "START TIME:" << endl;
										cin >> timee;
										cout << "END TIME:" << endl;
										double timel;
										cin >> timel;
										cout << "max_Rx_value:" << endl;
										cin >> X;
										cout << "min_Rx_value:" << endl;
										cin >> Rxm;

										int i = 0;
										while (timenew[i] <= timee){
											++i;
										}
										if (timee == 0){ i = 0; }
										int o = 0;
										while (timenew[o] <= timel){
											++o;
										}
										int dist = o - i;
										double dist2 = X - Rxm;
										double step = dist2 / dist;
										double Rxl = Rxm;
										for (int run = i; run < o + 1; ++run){
											Rxl = Rxl + step;
											fOxf[run] = fOxf[run] + Rxl*3.14 / 180;
										}
									}
									if (caser2 == 3){
										int i = -1;
										while (i == -1){
											cout << "TIME:" << endl;
											cin >> timee;
											cout << "Rx_value:" << endl;
											cin >> X;
											int k = 0;
											while (k < timenew.size()){
												if (timenew[k] == X){
													i = k;
												}++k;
											}
											if (i == -1){ cout << "Error no exist value of time of the point" << endl; }
										}
										if (timee == 0){ i = 0; }
										fOxf[i] = fOxf[i] + X*3.14 / 180;
									}
									cout << "Do you want an other value " << L << " axis change? (y/n)" << endl;
									cin >> caser3;
								}

							}
							if (L == 'Y'){
								char caser3 = 'y';
								while (caser3 == 'y'){
									double timee;
									double Ry;
									double caser2;
									cout << "You want : " << endl;
									cout << "1) a time interval with fix value? " << endl;
									cout << "2) a time interval with linear max and min value?" << endl;
									cout << "3) a point? " << endl;
									cout << "choose by but 1/2/3..." << endl;
									cin >> caser2;
									if (caser2 == 1){
										cout << "START TIME:" << endl;
										cin >> timee;
										cout << "END TIME:" << endl;
										double timel;
										cin >> timel;
										cout << "Ry_value:" << endl;
										cin >> Ry;
										int i = 0;
										while (timenew[i] <= timee){
											++i;
										}
										if (timee == 0){ i = 0; }
										int o = 0;
										while (timenew[o] <= timel){
											++o;
										}
										for (int run = i; run < o + 1; ++run){
											fOyf[run] = fOyf[run] + Ry*3.14 / 180;
										}
									}
									if (caser2 == 2){
										double Rym;
										cout << "START TIME:" << endl;
										cin >> timee;
										cout << "END TIME:" << endl;
										double timel;
										cin >> timel;
										cout << "max_Ry_value:" << endl;
										cin >> Ry;
										cout << "min_Ry_value:" << endl;
										cin >> Rym;

										int i = 0;
										while (timenew[i] <= timee){
											++i;
										}
										if (timee == 0){ i = 0; }
										int o = 0;
										while (timenew[o] <= timel){
											++o;
										}
										int dist = o - i;
										double dist2 = Ry - Rym;
										double step = dist2 / dist;
										double Ryl = Rym;
										for (int run = i; run < o + 1; ++run){
											Ryl = Ryl + step;
											fOyf[run] = fOyf[run] + Ryl*3.14 / 180;
										}
									}
									if (caser2 == 3){
										int i = -1;
										while (i == -1){
											cout << "TIME:" << endl;
											cin >> timee;
											cout << "Ry_value:" << endl;
											cin >> Ry;
											int k = 0;
											while (k < timenew.size()){
												if (timenew[k] == Ry){
													i = k;
												}++k;
											}
											if (i == -1){ cout << "Error no exist value of time of the point" << endl; }
										}
										if (timee == 0){ i = 0; }
										fOyf[i] = fOyf[i] + Ry*3.14 / 180;
									}
									cout << "Do you want an other value " << L << " axis change? (y/n)" << endl;
									cin >> caser3;
								}
							}
							if (L == 'Z'){
								char caser3 = 'y';
								while (caser3 == 'y'){
									double timee;
									double Rz;
									double caser2;
									cout << "You want : " << endl;
									cout << "1) a time interval with fix value? " << endl;
									cout << "2) a time interval with linear max and min value?" << endl;
									cout << "3) a point? " << endl;
									cout << "choose by but 1/2/3..." << endl;
									cin >> caser2;
									if (caser2 == 1){
										cout << "START TIME:" << endl;
										cin >> timee;
										cout << "END TIME:" << endl;
										double timel;
										cin >> timel;
										cout << "Rz_value:" << endl;
										cin >> Rz;
										int i = 0;
										while (timenew[i] <= timee){
											++i;
										}if (timee == 0){ i = 0; }
										int o = 0;
										while (timenew[o] <= timel){
											++o;
										}
										for (int run = i; run < o + 1; ++run){
											fOzf[run] = fOzf[run] + Rz*3.14 / 180;
										}
									}
									if (caser2 == 2){
										double Rzm;
										cout << "START TIME:" << endl;
										cin >> timee;
										cout << "END TIME:" << endl;
										double timel;
										cin >> timel;
										cout << "max_Rz_value:" << endl;
										cin >> Rz;
										cout << "min_Rz_value:" << endl;
										cin >> Rzm;

										int i = 0;
										while (timenew[i] <= timee){
											++i;
										}
										if (timee == 0){ i = 0; }
										int o = 0;
										while (timenew[o] <= timel){
											++o;
										}
										int dist = o - i;
										double dist2 = Rz - Rzm;
										double step = dist2 / dist;
										double Rzl = Rzm;
										for (int run = i; run < o + 1; ++run){
											Rzl = Rzl + step;
											fOzf[run] = fOzf[run] + Rzl*3.14 / 180;
										}
									}
									if (caser2 == 3){
										int i = -1;
										while (i == -1){
											cout << "TIME:" << endl;
											cin >> timee;
											cout << "Rz_value:" << endl;
											cin >> Rz;
											int k = 0;
											while (k < timenew.size()){
												if (timenew[k] == timee){
													i = k;
												}++k;
											}
											if (i == -1){ cout << "Error no exist value of time of the point" << endl; }
										}
										if (timee == 0){ i = 0; }
										fOzf[i] = fOzf[i] + Rz*3.14 / 180;
									}
									cout << "Do you want an other value " << L << " axis change? (y/n)" << endl;
									cin >> caser3;

								}
							}
							cout << "Do you want to change another DoF motion values? (y/n)" << endl;
							cin >> caser4;
						}
						cout << "Do you want to vision again the motion? (y/n)" << endl;
						cin >> caser5;
						if (caser5 == 'y'){

							///RE-WRITTING THE MOTION FILE for geofem
							ofstream opengeostate;
							INIReader ini = INIReader(INI_FILE);
							string resultDir1 = BASE_DIR + ini.Get("FEBIOSTEP", "GEO", "");
							string geof = ini.Get("FEBIOSTEP", "GEOF", "");
							string geos = ini.Get("FEBIOSTEP", "GEOS", "");
							string bd1 = ini.Get("BODYFORCES", "BDNAME1", "");
							string bd2 = ini.Get("BODYFORCES", "BDNAME2", "");
							//char itter = itteration + '0';
							////create a new folder for the analysis/////////
							int sizer = timenew.size();

							opengeostate = ofstream(resultDir1 + "/motionstate.mot", ofstream::out);
							opengeostate << "Coordinates" << endl;
							opengeostate << "version=1" << endl;
							opengeostate << "nRows=" << sizer - 1 << endl;
							opengeostate << "nColumns=13" << endl;
							opengeostate << "inDegrees=no" << endl;
							opengeostate << "" << endl;
							opengeostate << "Units are S.I.units(second, meters, Newtons, ...)" << endl;
							opengeostate << "Angles are not in degrees." << endl;
							opengeostate << "" << endl;
							opengeostate << "endheader" << endl;

							opengeostate << "  time  " << bd1 << "_tilt  " << bd1 << "_list  " << bd1 << "_rotation  " << bd1 << "_tx  " << bd1 << "_ty  " << bd1 << "_tz  " << bd2 << "_tilt  " << bd2 << "_list  " << bd2 << "_rotation  " << bd2 << "_tx  " << bd2 << "_ty  " << bd2 << "_tz  " << endl;
							for (int i = 0; i < sizer; ++i){
								opengeostate << "  " << timenew[i] << "  " << fOzf[i] << "  " << fOxf[i] << "  " << fOyf[i] << "  " << fPzf[i] << "  " << fPxf[i] << "  " << fPyf[i] << "  " << tOzf[i] << "  " << tOxf[i] << "  " << tOyf[i] << "  " << tPzf[i] << "  " << tPxf[i] << "  " << tPyf[i] << endl;


							}
							caser4 = 'y';
						}
					}


				}
			}
			////////////////////////////////////
			//////////////// b. The VISION SECTION end////////////
			/////////////////////////////////////

			if (resamplernow == 'y'){
				DOFResample dresl;
				dresl.store(timenew, 0);
				dresl.store(fPxf, 1);
				dresl.store(fPyf, 2);
				dresl.store(fPzf, 3);
				dresl.store(fOxf, 4);
				dresl.store(fOyf, 5);
				dresl.store(fOzf, 6);
				dresl.store(tPxf, 7);
				dresl.store(tPyf, 8);
				dresl.store(tPzf, 9);
				dresl.store(tOxf, 10);
				dresl.store(tOyf, 11);
				dresl.store(tOzf, 12);
				dresl.store(fFxf, 13);
				dresl.store(fFyf, 14);
				dresl.store(fFzf, 15);
				dresl.store(fMxf, 16);
				dresl.store(fMyf, 17);
				dresl.store(fMzf, 18);
				dresl.store(tFxf, 19);
				dresl.store(tFyf, 20);
				dresl.store(tFzf, 21);
				dresl.store(tMxf, 22);
				dresl.store(tMyf, 23);
				dresl.store(tMzf, 24);
				dresl.detect(itteration, kind, endend, resultDir2, 1);

				timenew = dresl.return_vect(0);

				/////global vector////
				fPxf = dresl.return_vect(1);
				fPyf = dresl.return_vect(2);
				fPzf = dresl.return_vect(3);
				fOxf = dresl.return_vect(4);
				fOyf = dresl.return_vect(5);
				fOzf = dresl.return_vect(6);
				tPxf = dresl.return_vect(7);
				tPyf = dresl.return_vect(8);
				tPzf = dresl.return_vect(9);
				tOxf = dresl.return_vect(10);
				tOyf = dresl.return_vect(11);
				tOzf = dresl.return_vect(12);
				/////globall vector////
				fFxf = dresl.return_vect(13);
				fFyf = dresl.return_vect(14);
				fFzf = dresl.return_vect(15);
				fMxf = dresl.return_vect(16);
				fMyf = dresl.return_vect(17);
				fMzf = dresl.return_vect(18);
				tFxf = dresl.return_vect(19);
				tFyf = dresl.return_vect(20);
				tFzf = dresl.return_vect(21);
				tMxf = dresl.return_vect(22);
				tMyf = dresl.return_vect(23);
				tMzf = dresl.return_vect(24);
				dresl.~DOFResample();

				cout << "Resample the initial velocities base the new prescribe trajectory" << endl;
				vel.store(timenew, 0);
				vel.store(fPxf, 1);
				vel.store(fPyf, 2);
				vel.store(fPzf, 3);
				vel.store(fOxf, 4);
				vel.store(fOyf, 5);
				vel.store(fOzf, 6);
				vel.store(tPxf, 7);
				vel.store(tPyf, 8);
				vel.store(tPzf, 9);
				vel.store(tOxf, 10);
				vel.store(tOyf, 11);
				vel.store(tOzf, 12);
				vel.initial(itteration, 'n', 1, 'P');
				vel.~Initial_VEL();

			}

			////////////////////////////////////
			//////////////// c. The STRUCTOR SECTION start////////////
			/////////////////////////////////////
			cout << "" << endl;
			cout << "CALL THE FEBio's XML STRUCTORS" << endl;
			cout << "" << endl;
			BF_structor bff;
			bff.begining(itteration, kind, endend, resultDir2, timenew, fFxf, fFyf, fFzf, fMxf, fMyf, fMzf, tFxf, tFyf, tFzf, tMxf, tMyf, tMzf);
			//bff.~BF_structor();
			BK_structor bkk;
			bkk.begining(itteration, kind, 0, endend + 1, resultDir2, timenew, fPxf, fPyf, fPzf, fOxf, fOyf, fOzf, tPxf, tPyf, tPzf, tOxf, tOyf, tOzf);
			//bkk.~BK_structor();
			////////////////////////////////////
			//////////////// c. The STRUCTOR SECTION end////////////
			/////////////////////////////////////

	
		//cout << "FEBWRITTER1" << endl;

		//dynamic memory allocate...
		//free;

	}//yes time resampler



	//cout << "FEBWRITTER" << endl;
	/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	FEBRunner feb;
	feb.WriteFEBFile(itteration,'R');
	feb.~FEBRunner();

}//end of starter



/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////END STARTER/////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
void JR_starter::Run()
{
	INIReader ini = INIReader(INI_FILE);
	string resultDir1 = BASE_DIR + ini.Get("FEBIOSTEP", "GEO", "");
	string modelPath = resultDir1 + "/geopensim.osim";
	string programname = ini.Get("PATH", "PROGRAMMNAME", "");
	string name = ini.Get("PATH", "NAME", "");
	string arg1 = " --console suppress ";

	int count = 0;
	string arg2 = modelPath;
	cout << "" << endl;
	cout << "START THE OPENSIM GUI " << endl;

	string workingFileName = arg2;

	string arguments = arg1;

	HANDLE process = ShellExecuteHandler(programname, arguments, name);
	cout << "" << endl;
	cout << "PASS THE .osim FILE AND THE .mot FILE IN OPENSIM GUI " << endl;
	string name1 = "";
	string arguments1 = "";
	HANDLE process1 = Shellfile(resultDir1);


}//void


//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

HANDLE JR_starter::ShellExecuteHandler(string program, string args, string name)
{
	HANDLE hProcess = NULL;
	SHELLEXECUTEINFO shellInfo;
	::ZeroMemory(&shellInfo, sizeof(shellInfo));
	shellInfo.cbSize = sizeof(shellInfo);
	shellInfo.fMask = SEE_MASK_FLAG_NO_UI | SEE_MASK_NOCLOSEPROCESS;
	shellInfo.lpVerb = "open";
	shellInfo.lpFile = name.c_str();
	shellInfo.lpParameters = args.c_str();
	shellInfo.lpDirectory = program.c_str();
	shellInfo.nShow = 10;
	if (::ShellExecuteEx(&shellInfo))
	{ // success 
		hProcess = shellInfo.hProcess;
	} // success 
	return hProcess;
}
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

HANDLE JR_starter::Shellfile(string program)
{
	HANDLE hProcess = NULL;
	SHELLEXECUTEINFO shellInfo;
	::ZeroMemory(&shellInfo, sizeof(shellInfo));
	shellInfo.cbSize = sizeof(shellInfo);
	shellInfo.fMask = SEE_MASK_FLAG_NO_UI | SEE_MASK_NOCLOSEPROCESS;
	shellInfo.lpVerb = "open";
	shellInfo.lpDirectory = program.c_str();
	shellInfo.nShow = 10;
	if (::ShellExecuteEx(&shellInfo))
	{ // success 
		hProcess = shellInfo.hProcess;
	} // success 
	return hProcess;
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


