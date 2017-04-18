#include <OpenSim/Tools/InverseKinematicsTool.h>
#include <OpenSim/Tools/RRATool.h>
#include <OpenSim/Tools/InverseKinematicsTool.h>
#include "OpenSimRunner.h"


void OpenSimRunner::runIK(){
	cout << "" << endl;
	cout << " Start the Inverse-Kinematics Analysis..." << endl;
	cout << "" << endl;
	INIReader ini = INIReader(INI_FILE);
	ofstream valuenon;
	ofstream valueinn;
	ofstream valpo;
	ofstream valpi;
	//ofstream valueno;
	string modelPath= BASE_DIR + ini.Get("INVERSEKINEMATICS", "MODEL", "");
	string resultDir = BASE_DIR + ini.Get("INVERSEKINEMATICS", "RESULT_DIR", "");
	string coord = BASE_DIR + ini.Get("INVERSEKINEMATICS", "COORDINATION", "");
	//string task = BASE_DIR + ini.Get("INVERSEKINEMATICS", "TASK", "");
	string setup= BASE_DIR + ini.Get("INVERSEKINEMATICS", "SETUP", "");
	double t1 = ini.GetReal("INVERSEKINEMATICS", "START_TIME", 0);
	double t2 = ini.GetReal("INVERSEKINEMATICS", "END_TIME", 0);
	string marker = BASE_DIR + ini.Get("INVERSEKINEMATICS", "MARKER", "");
	//ofstream valueno;
	Model model(modelPath);
	//////////////////////////////////////////////////////////////////////////////////////////////////

	if (setup != BASE_DIR){
		InverseKinematicsTool tool(setup);
		tool.setModel(model);
		tool.setStartTime(t1);
		tool.setEndTime(t2);
		tool.setResultsDir(resultDir);
		tool.setOutputMotionFileName(resultDir + "/ikmotion.mot");
		tool.run();
	}
	else if (setup == BASE_DIR){}
	else { cout << "ERROR...Please add in setp.ini file in RRA section the setup file or the include files Desiare motion, RRA Tasks, RRA actuators, External forces (grf), Initial-Final time, Model" << endl; }

}

void OpenSimRunner::runST(){
	cout << "" << endl;
	cout << " Start the Inverse-Kinematics STATIC Analysis..." << endl;
	cout << "" << endl;
	INIReader ini = INIReader(INI_FILE);
	ofstream valuenon;
	ofstream valueinn;
	ofstream valpo;
	ofstream valpi;
	//ofstream valueno;
	string modelPath = BASE_DIR + ini.Get("INVERSEKINEMATICSTATIC", "MODEL", "");
	string resultDir = BASE_DIR + ini.Get("INVERSEKINEMATICSTATIC", "RESULT_DIR", "");
	string coord = BASE_DIR + ini.Get("INVERSEKINEMATICSTATIC", "COORDINATION", "");
	//string task = BASE_DIR + ini.Get("INVERSEKINEMATICS", "TASK", "");
	string setup = BASE_DIR + ini.Get("INVERSEKINEMATICSTATIC", "SETUP", "");
	double t1 = ini.GetReal("INVERSEKINEMATICSTATIC", "START_TIME", 0);
	double t2 = ini.GetReal("INVERSEKINEMATICSTATIC", "END_TIME", 0);
	string marker = BASE_DIR + ini.Get("INVERSEKINEMATICSTATIC", "MARKER", "");
	//ofstream valueno;
	Model model(modelPath);
	//////////////////////////////////////////////////////////////////////////////////////////////////

	if (setup != BASE_DIR){
		InverseKinematicsTool tool(setup);
		tool.setModel(model);
		tool.setStartTime(t1);
		tool.setEndTime(t2);
		tool.setResultsDir(resultDir);
		tool.setOutputMotionFileName(resultDir + "/stmotion.mot");
		tool.run();
	}
	else if (setup == BASE_DIR){}
	else { cout << "ERROR...Please add in setp.ini file in RRA section the setup file or the include files Desiare motion, RRA Tasks, RRA actuators, External forces (grf), Initial-Final time, Model" << endl; }

}
void OpenSimRunner::runRRA(){
	cout << "" << endl;
	cout << " Start the RRA of firt body..." << endl;
	cout << "" << endl;
	INIReader ini = INIReader(INI_FILE);
	ofstream valuenon;
	ofstream valueinn;
	ofstream valpo;
	ofstream valpi;
	//ofstream valueno;
	string modelPath = BASE_DIR + ini.Get("INVERSEKINEMATICS", "MODEL", "");
	string resultDir1 = BASE_DIR + ini.Get("RRA", "RESULT_DIR", "");
	string coord = BASE_DIR + ini.Get("RRA", "COORDINATION", "");
	string task = BASE_DIR + ini.Get("RRA", "TASK", "");
	string setup = BASE_DIR + ini.Get("RRA", "SETUP", "");
	double t0 = ini.GetReal("RRA", "START_TIME", 0);
	double tf = ini.GetReal("RRA", "END_TIME", 0);
	string bd1 = ini.Get("BODYFORCES", "BDNAME1", "");
	string bd2 = ini.Get("BODYFORCES", "BDNAME2", "");
	string control = BASE_DIR + ini.Get("RRA", "CONTROL", "");
	string actuator = BASE_DIR + ini.Get("RRA", "ACTUATOR", "");
	string force = BASE_DIR + ini.Get("RRA", "EXFORCES", "");

	Model model1(modelPath);
	//////////////////////////////////////////////////////////////////////////////////////////////////
	string motionfile = BASE_DIR + ini.Get("INVERSEKINEMATICS", "RESULT_DIR", "") + "/ikmotion.mot";
	//double t0 = detecter_time(0, motionfile);
	//double tf = detecter_time(1, motionfile);
	if (setup != BASE_DIR){
		RRATool tool1(setup);
		//if (modelPath != BASE_DIR){
			//tool1.setModel(model1);
		//}
		//
		tool1.setAdjustCOMToReduceResiduals(true);
		//
		if (actuator != BASE_DIR){
			tool1.setReplaceForceSet(true);
		tool1.setForceSetFiles(actuator);
	}
		//
		if (t0 != NAN){
			tool1.setInitialTime(t0);
		//
		}
		if (tf != NAN){
			tool1.setFinalTime(tf);
		}
		//
		tool1.setDesiredKinematicsFileName(motionfile);
		
		if (control != BASE_DIR){

			tool1.setConstraintsFileName(control);
		}
		
		if (task != BASE_DIR){

			tool1.setTaskSetFileName(task);
		}
		
		if (resultDir1 != BASE_DIR){
			tool1.setResultsDir(resultDir1);
			tool1.setOutputModelFileName(resultDir1 + "/rramodel2.osim");
		}
	     //
		tool1.setAdjustedCOMBody("torso");
         //
		if (force != BASE_DIR){

			tool1.setExternalLoadsFileName(force);
		}
		
		tool1.run();
	}
	else if (setup == BASE_DIR){
	//TODO solve without setupfile need modification to run...
		
		RRATool tool1;
		tool1.setModelFilename(modelPath);

		tool1.setInitialTime(t0);
		tool1.setFinalTime(tf);
		
		tool1.setReplaceForceSet(true);
		tool1.setForceSetFiles(actuator);

		tool1.setResultsDir(resultDir1);

		tool1.setSolveForEquilibrium(true);

		tool1.setExternalLoadsFileName(force);
		tool1.setDesiredKinematicsFileName(motionfile);
		tool1.setTaskSetFileName(task);
		tool1.setConstraintsFileName(control);

		tool1.setLowpassCutoffFrequency(6);
		
		tool1.setAdjustCOMToReduceResiduals(true);
		tool1.setAdjustedCOMBody("torso");
		
		tool1.setOutputModelFileName(resultDir1 + "/rramodel2.osim");
		
		tool1.run();
		
	}
	else { cout << "ERROR...Please add in setp.ini file in RRA section the setup file or the include files Desiare motion, RRA Tasks, RRA actuators, External forces (grf), Initial-Final time, Model" << endl; }
	
	
	//SECOND RRA for the second body
	/*
	
	cout << " Start the RRA of second body..." << endl;

	string mod = resultDir1 + "/rramodel1.osim";
	if (setup != BASE_DIR){
		RRATool tool2(setup);
		tool2.setModelFilename(mod);
		//
		tool2.setAdjustCOMToReduceResiduals(true);
		//
		if (t0 != NAN){
			tool2.setInitialTime(t0);
		}
		//
		if (actuator != BASE_DIR){
			tool2.setReplaceForceSet(true);
			tool2.setForceSetFiles(actuator);
		}
		//
		if (tf != NAN){
			tool2.setFinalTime(tf);
		}
		//
		tool2.setDesiredKinematicsFileName(motionfile);
		//
		if (control != BASE_DIR){

			tool2.setConstraintsFileName(control);
		}
		//
		if (task != BASE_DIR){

			tool2.setTaskSetFileName(task);
		}
		//
			tool2.setResultsDir(resultDir1);
			tool2.setOutputModelFileName(resultDir1 + "/rramodel2.osim");
		//
		tool2.setAdjustedCOMBody(bd2);
		//
		if (force != BASE_DIR){
			tool2.setExternalLoadsFileName(force);
		}
		//
	tool2.run();

	}
	else if (setup == BASE_DIR){
		//TODO solve without setupfile need modification to run...
		RRATool tool2(setup);
		tool2.setModelFilename(mod);
		tool2.setInitialTime(t0);
		tool2.setFinalTime(tf);
		tool2.setReplaceForceSet(true);
		tool2.setForceSetFiles(actuator);

		tool2.setResultsDir(resultDir1);

		tool2.setSolveForEquilibrium(true);

		tool2.setExternalLoadsFileName(force);
		tool2.setDesiredKinematicsFileName(motionfile);
		tool2.setTaskSetFileName(task);
		tool2.setConstraintsFileName(control);

		tool2.setLowpassCutoffFrequency(6);

		tool2.setAdjustCOMToReduceResiduals(true);
		tool2.setAdjustedCOMBody(bd1);
		
		tool2.setOutputModelFileName(resultDir1 + "/rramodel2.osim");
		

		tool2.run();
	}
	*/
}



void OpenSimRunner::runCMC(){
	cout << "" << endl;
	cout << " Start the CMC Analysis..." << endl;
	cout << "" << endl;
	INIReader ini = INIReader(INI_FILE);
	ofstream valuenon;
	ofstream valueinn;
	ofstream valpo;
	ofstream valpi;
	//ofstream valueno;
	string resultDir2 = BASE_DIR + ini.Get("CMC", "RESULT_DIR", "");
	string motion = BASE_DIR + ini.Get("CMC", "MOTION", "");
	string setup = BASE_DIR + ini.Get("CMC", "SETUP", "");
	double t0 = ini.GetReal("CMC", "START_TIME", 0);
	double tf = ini.GetReal("CMC", "END_TIME", 0);
	string bd1 = ini.Get("BODYFORCES", "BDNAME1", "");
	string bd2 = ini.Get("BODYFORCES", "BDNAME2", "");
	string control = BASE_DIR + ini.Get("CMC", "CONTROL", "");
	string task = BASE_DIR + ini.Get("CMC", "TASK", "");
	string actuator = BASE_DIR + ini.Get("CMC", "ACTUATOR", "");
	string force = BASE_DIR + ini.Get("CMC", "EXFORCES", "");
	string modelPath = BASE_DIR + ini.Get("RRA", "RESULT_DIR", "") + "/rramodel2.osim";
	Model model(modelPath);
	if (setup != BASE_DIR){
		CMCTool tool3(setup);
		tool3.setModelFilename(modelPath);
		if (motion != BASE_DIR){
			tool3.setDesiredKinematicsFileName(motion);
		}
		if (control != BASE_DIR){
			tool3.setConstraintsFileName(control);
		}
		if (task!= BASE_DIR){
			tool3.setTaskSetFileName(task);
		}
		//
		if (resultDir2 != BASE_DIR){
			tool3.setResultsDir(resultDir2);
		}
		//
		if (actuator != BASE_DIR){
			tool3.setReplaceForceSet(true);
			tool3.setForceSetFiles(actuator);
		}
//
		if (force != BASE_DIR){
			tool3.setExternalLoadsFileName(force);
		}
		//
		if (t0 != NAN){
			tool3.setInitialTime(t0);
		}
		if (tf != NAN){
			tool3.setFinalTime(tf);
		}
		tool3.run();
	}
	if (setup == BASE_DIR){
	
	
	}
	else { cout << "ERROR...Please add in setp.ini file in CMC section the setup file or the include files Desiare motion, CMC Tasks, CMC actuators, External forces (grf), Initial-Final time, Model" << endl; }

}


double OpenSimRunner::detecter_time(int kind,string motionfile){
//Todo do a detector for the best time to start the simulation of opnssim
	return 0;
}
