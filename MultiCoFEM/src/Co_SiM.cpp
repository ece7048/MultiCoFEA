#include <OpenSim/OpenSim.h>
#include <OpenSim/Analyses/BodyKinematics.h>
#include <OpenSim/Tools/AnalyzeTool.h>
#include <ctime>
#include <stdlib.h>
#include <exception>
#include "Settings.h"
#include "ReSampler.h"
#include "FEBRunner.h"
#include "OpenSimRunner.h"
#include "BF_starter.h"
#include "JR_starter.h"
#include "INIReader.h"
#include <iostream>
#include <fstream>
#include <algorithm>
//#include "FECore/FEModel.h"
#include "Num_Of_Partition.h"
#include "Visualizerfebgeo.h"
#include "INIReader.h"



using namespace OpenSim;
using namespace SimTK;
using namespace std;


std::clock_t optimizerStartTime;
INIReader ini;




int main()


{//TODO the initial ange of febio 5degrees so we have to run simulation for orientation of body
try{
	Visualizerfebgeo vs;
	INIReader ini = INIReader(INI_FILE);


	vs.XMLwrite();
	vs.~Visualizerfebgeo();
	
	cout << "///////////////////////////////////////////////////////////////////////////////////////////////////////////////////" << endl;
	cout << "///////////////////////////ooooo///////////////////////////////////////////////////ooooo///////////////////////////" << endl;
	cout << "///////////////////////////ooooo///////////////////////////////////////////////////ooooo///////////////////////////" << endl;
	cout << "///////////////////////////oo/ OpenSource C++ Program writting from Michail Mamalakis/oo///////////////////////////" << endl;
	cout << "///////////////////////////ooooo/////////////oo//////////////////oo////////////////ooooo///////////////////////////" << endl;
	cout << "///////////////////////////ooooo/////////////oo/////////////////oo/////////////////ooooo///////////////////////////" << endl;
	cout << "/////////////////////////////////////////////oo//////oooo//////oo//////////////////////////////////////////////////" << endl;
	cout << "//////////////////////////////////////////////oo/////oooo/////oo///////////////////////////////////////////////////" << endl;
	cout << "///////////////////////////////////////////////oo////oooo////oo////////////////////////////////////////////////////" << endl;
	cout << "////////////////////////////////////////////////oo////oo///oo//////////////////////////////////////////////////////" << endl;
	cout << "//////////////////////////////////////////////////ooooooooo////////////////////////////////////////////////////////" << endl;
	cout << "////////////////////////////////////////////////////ooo////////////////////////////////////////////////////////////" << endl;
	cout << "////////////////////////////////////////////////////ooo////////////////////////////////////////////////////////////" << endl;
	cout << "////////////////////////////////////////////////////ooo////////////////////////////////////////////////////////////" << endl;
	cout << "//////////////////////////////////////////////////oooooo///////////////////////////////////////////////////////////" << endl;
	cout << "////////////////////////////////////////////////////oo/////////////////////////////////////////////////////////////" << endl;
	cout << "////////////////////////////////////////////////oo////oo///////////////////////////////////////////////////////////" << endl;
	cout << "///////////////////////////////////////////////oo//////oo//////////////////////////////////////////////////////////" << endl;
	cout << "/////////////////////////////////////////////oo/////////oo/////////////////////////////////////////////////////////" << endl;
	cout << "////////////////////////////////////////////oo//////////oo/////////////////////////////////////////////////////////" << endl;
	cout << "//////////////////////////////////////////oo///////////oo//////////////////////////////////////////////////////////" << endl;
	cout << "////////////////////////////////////////oo////////////oo///////////////////////////////////////////////////////////" << endl;
	cout << "///////////////////////////////////////oo////////////oo////////////////////////////////////////////////////////////" << endl;
	cout << "//////////////////////////////////////o/////////////ooooo//////////////////////////////////////////////////////////" << endl;
	cout << "/////////////////////////////////////o///////////////////oooo///oooo/////////////ooooo/////o/////o/////o////////////" << endl;
	cout << "////////////////////////////////////////////////////////o//////o//o/////////////oo////////o/////o/o/o/o////////////" << endl;
	cout << "///////////////////////////////////////////////////////o//////o//o/////ooo////////oo/////o/////o//o//o/////////////" << endl;
	cout << "Copyright (c) all rights reserved/////////////////////oooo///oooo/////////////ooooo/////o/////o/////o/////////////" << endl;
	cout << "///////////////////////////////////////////////////////////////////////////////////////////////////////////////////" << endl;
	//cout << "" << endl;

	cout << "INITIALIZE MODE...please wait..." << endl;

	string residul = "C:/Users/ece/Desktop/Co_SiM_withFEBiocoding/data/";
	ifstream std(residul + "setup1.ini");
	ifstream std2(residul + "setup2.ini");
	std2.close();
	std.close();
	ifstream step1(residul + "setup.ini");
	ofstream st(residul + "setup1.ini");
	ofstream st2(residul + "setup2.ini");
	string line1;

	for (int i = 0; i < 200; i++)
	{
		getline(step1, line1);
		//if (except >= number){
		st << line1 << endl;
		st2 << line1 << endl;
	}
	step1.close();
	st.close();
	st2.close();

	cout << "For this co-simulation it will need the states after the two bodies of interest's CMC analysis. Do you have them? (y/n)" << endl;
	string kind = ini.Get("BASICSETUP", "CMCanalysis", "");
	cout<< kind << endl;
	if (kind == "n"){
		cout << "Which of the analysis below you do not have done?  " << endl;
		cout << "Inverse Kinematics (IK) Inverse Dynamic (ID) Reduce Residual Actuator (RRA) Compute Muscle Control (CMC) " << endl;
		OpenSimRunner open;
		open.runST();
		open.~OpenSimRunner();

OpenSimRunner open3;
open3.runIK();
open3.~OpenSimRunner();

		OpenSimRunner open1;
		open1.runRRA();
		open1.~OpenSimRunner();

		OpenSimRunner open2;
		open2.runCMC();
		open2.~OpenSimRunner();
	}
	if (kind == "y"){

		cout << "Please add the .sto and .osim file in the setup.ini file in the BODYFORCE section in STATE variable and in PATH sextion in MODELS variable"<< endl;
			system("pause");
	}
	cout << "#####################################################################################################################################################################" << endl;
	cout << "SAME BASIC INITIAL OPTIONS FOR THE ANALYSIS:" << endl;
	cout << "1. Before run the exe file be sure that the Center Of Mass (COM) of the two bodies in FEBio are in (0,0,0) point" << endl;
	cout << "2. In the [BODYFORCES] section of setup.ini and setup1.ini you have to set the Joints from 1-6 with first rotations dof and then translations with axis order x-y-z" << endl;
	cout << "3. If a DoF of joint is fixed set it as 0!!!" << endl;
	cout << "4. In the set up file in the [FEBIOSTEP] in GEO, GEOF, GEOS  set the two .stl or .vpl or .obj Rigid Geometries of the FEBio file. " << endl;
	cout << "   The Rigid Geometries have to be in millimeter. " << endl;
	cout << "5. Be sure that in [INVERSEKINEMATICSTATIC] section of setup.ini file in variabe RESULT_DIR, you have named the static motion " << endl;
	cout << "   file with | stmotion.mot | name or the analysis will not start!" << endl;
	cout << "6. If you want to set the FEM stragety DOF set in setup.ini file the field Forced_DOF, Prescribed_DOF,Fixed_DOF " << endl;
	cout << "   with the traslation dof and the rotation in case. You have to have in mind that if a rotation DOF set prescrided " << endl;
	cout << "   then the other rotation dof have to be prescribed or fixed not free, same with the forced case (forced or fixed)." << endl;
	cout << "   Set first the traslation dof with xyz (no space) and then the rotation with rxryrz (no space) seperate by _" << endl;
	cout << "   If you want nan dof set forced fixed or prescribed set it with NAN." << endl; 
	cout << "   The DOFs you want to be free, without acting force or prescribed motion or be fixed, just do not entry them in the variables." << endl;
	cout << "   Finaly, if you are not sure for the stragety wait and set the DOF at the end of the program and leave the variables " << endl;
	cout << "   which already mention with no character, empty (Forced_DOF, Prescribed_DOF,Fixed_DOF). " << endl;


	cout << "#####################################################################################################################################################################" << endl;
	cout << "Lets start now" << endl;
	//////////set the three cases//////////
	cout << "The forces you will apply in FEBio can be:" << endl;
	cout << "1) Total forces from OpenSim (muscle forces, weight forces, inertial forces, external forces e.t.c.) ? (T)" << endl;
	cout << "The analysis in FEBio would be quasi-static" << endl;
	cout << "2) Forces from OpenSim (muscle forces, weight forces, external forces) ? (F)" << endl;
	cout << "The analysis in FEBio would be dynamic" << endl;
	cout << "3) Reaction Joint forces from OpenSim ? (R)" << endl;
	cout << "The analysis in FEBio would be dynamic" << endl;
	
	string mode = ini.Get("BASICSETUP", "MODEanalysis", "");
	cout << mode << endl;

	if (mode == "T"|| mode=="F"){

		cout << "How many inderval time partitions at most, do you want to build for this analysis?" << endl;
		cout << "P.S. If you want to run the simulation with out partition time intervals analysis just set 1 interval..." << endl;
		
		double number = ini.GetReal("BASICSETUP", "PartitionIntervals", 0);
		cout << number<<endl;
		FEBRunner feb0;
		feb0.CopyFEBFile(number);

		cout << "Do you want to give the intervals of the analysis by your self? (y/n)" << endl;
		string process = ini.Get("BASICSETUP", "MANUALinterval", "");
		cout << process<<endl;
		////////////////////////////////////////////////
		Num_Of_Partition nop;
		Vector st = nop.start(number);
		cout << "This is the intervals we recomand!!" << endl;
		cout << "" << endl;
		///////////////////////////////////////////////////
		char mode2 = mode[0];
		for (int i = 0; i < number; ++i){
			if (process == "y"){
				cout << "Give the " << i + 1 << " time interval. " << endl;
				cout << "Give the start time. " << endl;
				double st;
				cin >> st;
				cout << "Give the end time. " << endl;
				double ed;
				cin >> ed;
				Num_Of_Partition nps;
				nps.analysiswriter(st, ed, mode2);
				//nps.~Num_Of_Partition();
			}
			if (process == "n"){
				cout << "The program will detect the  " << i + 1 << "  initial interval now..." << endl;
				cout << "start time: " << st[i] << endl;
				cout << "end time: " << st[i + 1] << endl;
				double starttime = st[i];
				double endtime = st[i + 1];
				Num_Of_Partition npss;
				npss.analysiswriter(starttime, endtime, mode2);
				//npss.~Num_Of_Partition();
			}
			cout << "The " << i + 1 << "/" << number << " Co-Simulation builder will start now... " << endl;
			cout << "" << endl;
			///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
			BF_starter bff;
			
			bff.started(number, i, mode2);
			//bff.~BF_starter();
			/*
			cout << "FEBWRITTER" << endl;
			/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
			FEBRunner feb;
			feb.WriteFEBFile(i);
			*/
			/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		}
		cout << "Rewrite the initial setup.ini file" << endl;
		FEBRunner feb2;
		feb2.clearsetup();
		//for (int i = 0; i < number; ++i){
		FEBRunner feb1;
		feb1.Run(number);
		feb1.~FEBRunner();
		//}


	}




	if (mode == "R"){
		cout << "How many inderval time partitions at most, do you want to build for this analysis?" << endl;
		cout << "P.S. If you want to run the simulation with out partition time intervals analysis just set 1 interval..." << endl;
		double number = ini.GetReal("BASICSETUP", "PartitionIntervals", 0);
		cout << number << endl;
		FEBRunner feb0;
		feb0.CopyFEBFile(number);

		cout << "Do you want to give the intervals of the analysis by your self? (y/n)" << endl;
		string process = ini.Get("BASICSETUP", "MANUALinterval", "");
		cout << process << endl;
		////////////////////////////////////////////////
		Num_Of_Partition nop;
		Vector st = nop.start(number);
		cout << "This is the intervals we recomand!!" << endl;
		cout << "" << endl;
		char mode2 = mode[0];
		///////////////////////////////////////////////////
		for (int i = 0; i < number; ++i){
			if (process == "y"){
				cout << "Give the " << i + 1 << " time interval. " << endl;
				cout << "Give the start time. " << endl;
				double st;
				cin >> st;
				cout << "Give the end time. " << endl;
				double ed;
				cin >> ed;
				Num_Of_Partition nps;
				nps.analysiswriter(st, ed, mode2);
				//nps.~Num_Of_Partition();
			}
			if (process == "n"){
				cout << "The program will detect the  " << i + 1 << "  initial interval now..." << endl;
				cout << "start time: " << st[i] << endl;
				cout << "end time: " << st[i + 1] << endl;
				double starttime = st[i];
				double endtime = st[i + 1];
				Num_Of_Partition npss;
				npss.analysiswriter(starttime, endtime, mode2);
				//npss.~Num_Of_Partition();
			}
			cout << "The " << i + 1 << "/" << number << " Co-Simulation builder will start now... " << endl;
			cout << "" << endl;
			///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

			JR_starter jr;
			jr.started(number, i);
			/////////////////////////////////////////////////////////////////////////////
		}
		cout << "Rewrite the initial setup.ini file" << endl;
		FEBRunner feb2;
		feb2.clearsetup();
		//for (int i = 0; i < number; ++i){
		FEBRunner feb1;
		feb1.Run(number);
		//feb1.~FEBRunner();
		//}
	}

	}
	
	catch (...)

	{
		cout << "we have an exception" << endl;
	}


	

	system("pause");
	return 0;

}

