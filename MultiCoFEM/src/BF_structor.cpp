
#include <OpenSim/OpenSim.h>
#include <OpenSim/Analyses/BodyKinematics.h>
#include <OpenSim/Tools/AnalyzeTool.h>
#include <iostream>
#include <fstream>
#include <ctime>
#include <exception>
#include <iostream>
#include <fstream>
#include <algorithm>
#include "INIReader.h"
#include "BF_structor.h"
#include "Settings.h"

BF_structor::BF_structor(void)
{
}

BF_structor::~BF_structor(void)
{
}
void BF_structor::begining(int iteration, string kind[24], int sizer, string resultDir, Vector time, Vector fx, Vector fy, Vector fz, Vector fox, Vector foy, Vector foz, Vector tx, Vector ty, Vector tz, Vector tox, Vector toy, Vector toz)
{
		INIReader ini = INIReader(INI_FILE);
		//////////////////////////////////////////////strategy fem ////////////////////////////////////////


		cout << "" << endl;
		cout << "Which will be the stragety that we will follow for the FEM analysis for the Forced DOF of the RBs?" << endl;
		cout << "Set the DOF which will be Prescribed for the two bodies in Forced_DOF_1 and Forced_DOF_2 in .ini file." << endl;
		cout << "PS if you set a rotation DOF Forced then the other rotDOF have to be fixed or Forced too" << endl;
		cout << "PS1 for the rotation dof set rx or ry or rz and for traslation x y or z." << endl;
		cout << "PS2 if you set in setup file set the Forced dof like xyz_rxryrz first the traslation DOF and second the rotation seperate with _ " << endl;
		cout << "PS3 if you do not want ant DOF Forced just set in setup.ini file the word, NAN" << endl;
		cout << "PS4 if you already set the Forced stragety in setup.ini file then your choice will be below :" << endl;


		string presc1 = ini.Get("BASICSETUP", "Forced_DOF_1", "");
		if (presc1 != ""){
			cout << "For the first Rigid body the Forced dof are:" << endl;
			cout << presc1 << endl;
		}
		string presc2 = ini.Get("BASICSETUP", "Forced_DOF_2", "");
		if (presc2 != ""){
			cout << "For the second Rigid body the Forced dof are:" << endl;
			cout << presc2 << endl;
		}
		if (presc1 == ""){
			cout << "give now the DOF Forced first traslation, second rotation seperated by _ for the first Rigid Body" << endl;
			cout << "If you do not want any DOF Forced set NAN, you can set only rotation if you want." << endl;
			string forc;
			cin >> forc;
			presc1 = forc;
		}
		if (presc2 == ""){
			cout << "give now the DOF Forced first traslation, second rotation seperated by _ for the second Rigid Body" << endl;
			cout << "If you do not want any DOF Forced set NAN, you can set only rotation if you want." << endl;
			string forc;
			cin >> forc;
			presc2 = forc;
		}


		if (presc1 == "NAN"){ fx = fy = fz = fox = foy = foz = 0; }
		if (presc1 != "NAN"){
			if (presc1[0] != 'x' && presc1[1] != 'x' && presc1[2] != 'x'){ fx = 0; }
			if (presc1[0] != 'y' && presc1[1] != 'y' && presc1[2] != 'y'){ fy = 0; }
			if (presc1[0] != 'z' && presc1[1] != 'z' && presc1[2] != 'z'){ fz = 0; }
			if (presc1[1] == '_' || presc1[2] == '_' || presc1[3] == '_'){
				cout << "new" << endl;
				if (presc1[3] != 'x'&& presc1[4] != 'x' && presc1[5] != 'x' && presc1[6] != 'x' && presc1[7] != 'x'&& presc1[8] != 'x'&& presc1[9] != 'x'&& presc1[10] != 'x'&& presc1[11] != 'x'){ fox = 0; }
				if (presc1[3] != 'y'&& presc1[4] != 'y' && presc1[5] != 'y' && presc1[6] != 'y' && presc1[7] != 'y'&& presc1[8] != 'y'&& presc1[9] != 'y'&& presc1[10] != 'y'&& presc1[11] != 'y'){ foy = 0; }
				if (presc1[3] != 'z'&& presc1[4] != 'z' && presc1[5] != 'z' && presc1[6] != 'z' && presc1[7] != 'z'&& presc1[8] != 'z'&& presc1[9] != 'z'&& presc1[10] != 'z'&& presc1[11] != 'z'){ foz = 0; }
			}
			if (presc1[0] == 'r'){
				if (presc1[1] != 'x' ){ fox = 0; }
				if (presc1[1] != 'y' ){ foy = 0; }
				if (presc1[1] != 'z' ){ foz = 0; }
			}
			if ( presc1[3] != 'x' && presc1[5] != 'x'){ fox = 0; }
			if ( presc1[3] != 'y' && presc1[5] != 'y'){ foy = 0; }
			if ( presc1[3] != 'z' && presc1[5] != 'z'){ foz = 0; }
		}

		if (presc2 == "NAN"){ tx = ty = tz = tox = toy = toz = 0; }
		if (presc2 != "NAN"){
			if (presc2[0] != 'x' && presc2[1] != 'x' && presc2[2] != 'x'){ tx = 0; }
			if (presc2[0] != 'y' && presc2[1] != 'y' && presc2[2] != 'y'){ ty = 0; }
			if (presc2[0] != 'z' && presc2[1] != 'z' && presc2[2] != 'z'){ tz = 0; }
			if (presc2[1] == '_' || presc2[2] == '_' || presc2[3] == '_'){
				cout << "new" << endl;
				if (presc2[3] != 'x'&& presc2[4] != 'x' && presc2[5] != 'x' && presc2[6] != 'x' && presc2[7] != 'x'&& presc2[8] != 'x'&& presc2[9] != 'x'&& presc2[10] != 'x'&& presc2[11] != 'x'){ tox = 0; }
				if (presc2[3] != 'y'&& presc2[4] != 'y' && presc2[5] != 'y' && presc2[6] != 'y' && presc2[7] != 'y'&& presc2[8] != 'y'&& presc2[9] != 'y'&& presc2[10] != 'y'&& presc2[11] != 'y'){ toy = 0; }
				if (presc2[3] != 'z'&& presc2[4] != 'z' && presc2[5] != 'z' && presc2[6] != 'z' && presc2[7] != 'z'&& presc2[8] != 'z'&& presc2[9] != 'z'&& presc2[10] != 'z'&& presc2[11] != 'z'){ toz = 0; }
			}
			if (presc2[0] == 'r'){
				if (presc2[1] != 'x' ){ tox = 0; }
				if (presc2[1] != 'y' ){ toy = 0; }
				if (presc2[1] != 'z' ){ toz = 0; }
			}
			if (presc2[3] != 'x' && presc2[5] != 'x'){ tox = 0; }
			if (presc2[3] != 'y' && presc2[5] != 'y'){ toy = 0; }
			if (presc2[3] != 'z' && presc2[5] != 'z'){ toz = 0; }
		}


		ofstream stepstatic;
		ofstream stepdynamic;
		ofstream valuenon;
		ofstream valueinn;
		char itter = iteration + '0';
		stepstatic = ofstream(resultDir + "/FEBstepstatic_forces" + itter + ".txt", ofstream::out);
		stepdynamic = ofstream(resultDir + "/FEBstepdynamic_forces" + itter + ".txt", ofstream::out);
		valuenon = ofstream(resultDir + "/FEBdataforcesdynamic" + itter + ".txt", ofstream::out);
		valueinn = ofstream(resultDir + "/FEBdataforcestatic" + itter + ".txt", ofstream::out);
		////////////STATIC PART/////////////////////////////////////////////////////////////////////////////////////
		//first
		fx[0] = fx[0] * 1000;
		int parast = (int)fx[0];
		fx[0] = parast * 0.001;
		fy[0] = fy[0] * 1000;
		int parasty = (int)fy[0];
		fy[0] = parasty * 0.001;
		fz[0] = fz[0] * 1000;
		int parastz = (int)fz[0];
		fz[0] = parastz * 0.001;
		//
		fox[0] = fox[0] * 10000;
		int paratfox = (int)fox[0];
		fox[0] = paratfox * 0.1;
		foy[0] = foy[0] * 10000;
		int paratfoy = (int)foy[0];
		foy[0] = paratfoy * 0.1;
		foz[0] = foz[0] * 10000;
		int paratfoz = (int)foz[0];
		foz[0] = paratfoz * 0.1;

		// second
		tx[0] = tx[0] * 1000;
		int parastt = (int)tx[0];
		tx[0] = parastt * 0.001;
		ty[0] = ty[0] * 1000;
		int parastty = (int)ty[0];
		ty[0] = parastty * 0.001;
		tz[0] = tz[0] * 1000;
		int parasttz = (int)tz[0];
		tz[0] = parasttz * 0.001;
		//
		tox[0] = tox[0] * 10000;
		int parattox = (int)tox[0];
		tox[0] = parattox * 0.1;
		toy[0] = toy[0] * 10000;
		int parattoy = (int)toy[0];
		toy[0] = parattoy * 0.1;
		toz[0] = toz[0] * 10000;
		int parattoz = (int)toz[0];
		toz[0] = parattoz * 0.1;

		
		valueinn << "<LoadData>" << endl;

		valueinn << "<loadcurve id=" << '"' << "1" << '"' << "type=" << '"' <<  "linear"  << '"' << "extend =" << '"' << "constant" << '"' << ">" << endl;
		valueinn << "<point>" << time[0]  << "," << "0" << "</point>" << endl;
		valueinn << "<point>" << time[1] << "," << fx[0] << "</point>" << endl;
		valueinn << "</loadcurve>" << endl;

		valueinn << "<loadcurve id=" << '"' << "2" << '"' << "type=" << '"'  << "linear" <<  '"' << "extend =" << '"' << "constant" << '"' << ">" << endl;
		valueinn << "<point>" << time[0]  << "," << "0" << "</point>" << endl;
		valueinn << "<point>" << time[1]  << "," << fy[0] << "</point>" << endl;
		valueinn << "</loadcurve>" << endl;

		valueinn << "<loadcurve id=" << '"' << "3" << '"' << "type=" << '"'  << "linear" <<  '"' << "extend =" << '"' << "constant" << '"' << ">" << endl;
		valueinn << "<point>" << time[0]  << "," << "0" << "</point>" << endl;
		valueinn << "<point>" << time[1]  << "," << fz[0] << "</point>" << endl;
		valueinn << "</loadcurve>" << endl;

		valueinn << "<loadcurve id=" << '"' << "4" << '"' << "type=" << '"' << "linear" << '"' << "extend =" << '"' << "constant" << '"' << ">" << endl;
		valueinn << "<point>" << time[0]  << "," << "0" << "</point>" << endl;
		valueinn << "<point>" << time[1]  << "," << fox[0] << "</point>" << endl; 
		valueinn << "</loadcurve>" << endl;

		valueinn << "<loadcurve id=" << '"' << "5" << '"' << "type=" << '"' << "linear" << '"' << "extend =" << '"' << "constant" << '"' << ">" << endl;
		valueinn << "<point>" << time[0] << "," << "0" << "</point>" << endl;
		valueinn << "<point>" << time[1]  << "," << foy[0] << "</point>" << endl;
		valueinn << "</loadcurve>" << endl;

		valueinn << "<loadcurve id=" << '"' << "6" << '"' << "type=" << '"' << "linear" <<'"' << "extend =" << '"' << "constant" << '"' << ">" << endl;
		valueinn << "<point>" << time[0]  << "," << "0" << "</point>" << endl;
		valueinn << "<point>" << time[1]  << "," << foz[0] << "</point>" << endl;
		valueinn << "</loadcurve>" << endl;


		valueinn << "<loadcurve id=" << '"' << "7" << '"' << "type=" << '"' << "linear" << '"' << "extend =" << '"' << "constant" << '"' << ">" << endl;
		valueinn << "<point>" << time[0] << "," << "0" << "</point>" << endl;
		valueinn << "<point>" << time[1]  << "," << tx[0] << "</point>" << endl;
		valueinn << "</loadcurve>" << endl;


		valueinn << "<loadcurve id=" << '"' << "8" << '"' << "type=" << '"' << "linear" << '"' << "extend =" << '"' << "constant" << '"' << ">" << endl;
		valueinn << "<point>" << time[0] << "," << "0" << "</point>" << endl;
		valueinn << "<point>" << time[1]  << "," << ty[0] << "</point>" << endl;
		valueinn << "</loadcurve>" << endl;


		valueinn << "<loadcurve id=" << '"' << "9" << '"' << "type=" << '"' << "linear"<< '"' << "extend =" << '"' << "constant" << '"' << ">" << endl;
		valueinn << "<point>" << time[0] << "," << "0" << "</point>" << endl;
		valueinn << "<point>" << time[1]  << "," << tz[0] << "</point>" << endl;
		valueinn << "</loadcurve>" << endl;


		valueinn << "<loadcurve id=" << '"' << "10" << '"' << "type=" << '"' << "linear" << '"' << "extend =" << '"' << "constant" << '"' << ">" << endl;
		valueinn << "<point>" << time[0] << "," << "0" << "</point>" << endl;
		valueinn << "<point>" << time[1]  << "," << tox[0] << "</point>" << endl;
		valueinn << "</loadcurve>" << endl;


		valueinn << "<loadcurve id=" << '"' << "11" << '"' << "type=" << '"' << "linear" << '"' << "extend =" << '"' << "constant" << '"' << ">" << endl;
		valueinn << "<point>" << time[0] << "," << "0" << "</point>" << endl;
		valueinn << "<point>" << time[1]  << "," << toy[0] << "</point>" << endl;
		valueinn << "</loadcurve>" << endl;

		valueinn << "<loadcurve id=" << '"' << "12" << '"' << "type=" << '"' << "linear" << '"' << "extend =" << '"' << "constant" << '"' << ">" << endl;
		valueinn << "<point>" << time[0] << "," << "0" << "</point>" << endl;
		valueinn << "<point>" << time[1]  << "," << toz[0] << "</point>" << endl;
		valueinn << "</loadcurve>" << endl;


		///////////////DYNAMIC PART//////////////////////////////////////////////////////////////////////////////////////////

		////FIRST BODY
		
		for (int i = 0; i < sizer-1; i++){
			if (i > 0){
				fx[i] = fx[i] * 1000;
				int para = (int)fx[i];
				fx[i] = para * 0.001;
			}
		}
		valuenon << "<loadcurve id=" << '"' << "13" << '"' << "type=" << '"' << kind[0] << '"' << "extend =" << '"' << "constant" << '"' << ">" << endl;

		for (int i = 0; i < sizer-1; ++i){
			valuenon << "<point>" << time[i+1] << "," << fx[i] << "</point>" << endl;

		}
		valuenon << "</loadcurve>" << endl;

		for (int i = 0; i < sizer-1; i++){
			if (i > 0){
				fy[i] = fy[i] * 1000;
				int paray = (int)fy[i];
				fy[i] = paray * 0.001;
			}
		}
		valuenon << "<loadcurve id=" << '"' << "14" << '"' << "type=" << '"' << kind[1] << '"' << "extend =" << '"' << "constant" << '"' << ">" << endl;

		for (int i = 0; i < sizer-1; ++i){
			valuenon << "<point>" << time[i+1] << "," << fy[i] << "</point>" << endl;

		}
		valuenon << "</loadcurve>" << endl;

		for (int i = 0; i < sizer-1; i++){
			if (i > 0){
				fz[i] = fz[i] * 1000;
				int paraz = (int)fz[i];
				fz[i] = paraz * 0.001;
			}
		}
		valuenon << "<loadcurve id=" << '"' << "15" << '"' << "type=" << '"' << kind[2] << '"' << "extend =" << '"' << "constant" << '"' << ">" << endl;

		for (int i = 0; i < sizer-1; ++i){
			valuenon << "<point>" << time[i+1] << "," << fz[i] << "</point>" << endl;

		}
		valuenon << "</loadcurve>" << endl;

		for (int i = 0; i < sizer-1; i++){
			if (i > 0){
				fox[i] = fox[i] * 10000;
				int parax = (int)fox[i];
				fox[i] = parax * 0.1;
			}
		}
		valuenon << "<loadcurve id=" << '"' << "16" << '"' << "type=" << '"' << kind[3] << '"' << "extend =" << '"' << "constant" << '"' << ">" << endl;

		for (int i =0 ; i < sizer-1; ++i){
			valuenon << "<point>" << time[i+1] << "," << fox[i] << "</point>" << endl;

		}
		valuenon << "</loadcurve>" << endl;

		for (int i = 0; i < sizer-1; i++){
			if (i > 0){
				foy[i] = foy[i] * 10000;
				int paraoy = (int)foy[i];
				foy[i] = paraoy * 0.1;
			}
		}
		valuenon << "<loadcurve id=" << '"' << "17" << '"' << "type=" << '"' << kind[4] << '"' << "extend =" << '"' << "constant" << '"' << ">" << endl;

		for (int i = 0; i < sizer-1; ++i){
			valuenon << "<point>" << time[i+1] << "," << foy[i] << "</point>" << endl;

		}
		valuenon << "</loadcurve>" << endl;

		for (int i = 0; i < sizer-1; i++){
			if (i > 0){
				foz[i] = foz[i] * 10000;
				int paraoz = (int)foz[i];
				foz[i] = paraoz * 0.1;
			}
		}
		valuenon << "<loadcurve id=" << '"' << "18" << '"' << "type=" << '"' << kind[5] << '"' << "extend =" << '"' << "constant" << '"' << ">" << endl;

		for (int i = 0; i < sizer-1; ++i){
			valuenon << "<point>" << time[i+1] << "," << foz[i] << "</point>" << endl;

		}
		valuenon << "</loadcurve>" << endl;



		//// SECOND BODY
		
		for (int i = 0; i < sizer-1; i++){
			if (i > 0){
				tx[i] = tx[i] * 1000;
				int parat = (int)tx[i];
				tx[i] = parat * 0.001;
			}
		}
		valuenon << "<loadcurve id=" << '"' << "19" << '"' << "type=" << '"' << kind[6] << '"' << "extend =" << '"' << "constant" << '"' << ">" << endl;

		for (int i = 0; i < sizer-1; ++i){
			valuenon << "<point>" << time[i+1] << "," << tx[i] << "</point>" << endl;

		}
		valuenon << "</loadcurve>" << endl;

		for (int i = 0; i < sizer-1; i++){
			if (i > 0){
				ty[i] = ty[i] * 1000;
				int paraty = (int)ty[i];
				ty[i] = paraty * 0.001;
			}
		}
		valuenon << "<loadcurve id=" << '"' << "20" << '"' << "type=" << '"' << kind[7] << '"' << "extend =" << '"' << "constant" << '"' << ">" << endl;

		for (int i = 0; i < sizer-1; ++i){
			valuenon << "<point>" << time[i+1] << "," << ty[i] << "</point>" << endl;

		}
		valuenon << "</loadcurve>" << endl;

		for (int i = 0; i < sizer-1; i++){
			if (i > 0){
				tz[i] = tz[i] * 1000;
				int paratz = (int)tz[i];
				tz[i] = paratz * 0.001;
			}
		}
		valuenon << "<loadcurve id=" << '"' << "21" << '"' << "type=" << '"' << kind[8] << '"' << "extend =" << '"' << "constant" << '"' << ">" << endl;

		for (int i = 0; i < sizer-1; ++i){
			valuenon << "<point>" << time[i+1] << "," << tz[i] << "</point>" << endl;

		}
		valuenon << "</loadcurve>" << endl;

		for (int i = 0; i < sizer-1; i++){
			if (i > 0){
				tox[i] = tox[i] * 10000;
				int parattx = (int)tox[i];
				tox[i] = parattx * 0.1;
			}
		}
		valuenon << "<loadcurve id=" << '"' << "22" << '"' << "type=" << '"' << kind[9] << '"' << "extend =" << '"' << "constant" << '"' << ">" << endl;

		for (int i = 0; i < sizer-1; ++i){
			valuenon << "<point>" << time[i+1] << "," << tox[i] << "</point>" << endl;

		}
		valuenon << "</loadcurve>" << endl;

		for (int i = 0; i < sizer-1; i++){
			if (i > 0){
				toy[i] = toy[i] * 10000;
				int paratoy = (int)toy[i];
				toy[i] = paratoy * 0.1;
			}
		}
		valuenon << "<loadcurve id=" << '"' << "23" << '"' << "type=" << '"' << kind[10] << '"' << "extend =" << '"' << "constant" << '"' << ">" << endl;

		for (int i = 0; i < sizer-1; ++i){
			valuenon << "<point>" << time[i+1] << "," << toy[i] << "</point>" << endl;

		}
		valuenon << "</loadcurve>" << endl;

		for (int i = 0; i < sizer-1; i++){
			if (i > 0){
				toz[i] = toz[i] * 10000;
				int paratoz = (int)toz[i];
				toz[i] = paratoz * 0.1;
			}
		}
		valuenon << "<loadcurve id=" << '"' << "24" << '"' << "type=" << '"' << kind[11] << '"' << "extend =" << '"' << "constant" << '"' << ">" << endl;

		for (int i = 0; i < sizer-1; ++i){
			valuenon << "<point>" << time[i+1] << "," << toz[i] << "</point>" << endl;

		}
		valuenon << "</loadcurve>" << endl;
		//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

		///////////////////////////////////////COSTRUCT the costrain of the forces for the two bodies/////////////////////////////////////

		//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

		stepstatic << "<Constraints>" << endl;
		stepstatic << "<rigid_body mat = " << '"' << "1" << '"' << ">" << endl;
		if (fx[0] != 0){
			stepstatic << "<force bc = " << '"' << "x" << '"' << " type="<<'"'<<"follow"<<'"'<< "lc=" << '"' << "1" << '"' << ">1<" << "/force>" << endl;
		}
		if (fy[0] != 0){
			stepstatic << "<force bc = " << '"' << "y" << '"' << " type=" << '"' << "follow" << '"' << "lc=" << '"' << "2" << '"' << ">1<" << "/force>" << endl;
		}
		if (fz[0] != 0){
			stepstatic << "<force bc = " << '"' << "z" << '"' << " type=" << '"' << "follow" << '"' << "lc=" << '"' << "3" << '"' << ">1<" << "/force>" << endl;
		}
		if (fox[0] != 0){
			stepstatic << "<force bc = " << '"' << "Rx" << '"' << " type=" << '"' << "follow" << '"' << "lc=" << '"' << "4" << '"' << ">1<" << "/force>" << endl;
		}
		if (foy[0] != 0){
			stepstatic << "<force bc = " << '"' << "Ry" << '"' << " type=" << '"' << "follow" << '"' << "lc=" << '"' << "5" << '"' << ">1<" << "/force>" << endl;
		}
		if (foz[0] != 0){
			stepstatic << "<force bc = " << '"' << "Rz" << '"' << " type=" << '"' << "follow" << '"' << "lc=" << '"' << "6" << '"' << ">1<" << "/force>" << endl;
		}
		stepstatic << "</rigid_body>" << endl;
///////////////////////////////////////////////////////////////////////////////////////////////
		
			//stepstatic << "<rigid_body mat = " << '"' << "1" << '"' << ">" << endl;
		/*
		if (fx[0] == 0){
			stepstatic << "<fixed bc = " << '"' << "x" << '"' << "/>" << endl;
		}
		if (fy[0] == 0){
			stepstatic << "<fixed bc = " << '"' << "y" << '"' << "/>" << endl;;
		}
		if (fz[0] == 0){
			stepstatic << "<fixed bc = " << '"' << "z" << '"' << "/>" << endl;;
		}
		if (fox[0] == 0){
			stepstatic << "<fixed bc = " << '"' << "Rx" << '"' << "/>" << endl;;
		}
		if (foy[0] == 0){
			stepstatic << "<fixed bc = " << '"' << "Ry" << '"' << "/>" << endl;;
		}
		if (foz[0] == 0){
			stepstatic << "<fixed bc = " << '"' << "Rz" << '"' << "/>" << endl;;
		}
		stepstatic << "</rigid_body>" << endl;
		*/
		//////////////////////////////////////////////////////////////////////////////////////////////
		//second body///////////////////////
		//////////////////////////////////
		
		stepstatic << "<rigid_body mat = " << '"' << "2" << '"' << ">" << endl;
		int out31 = compaire(fx, tx);
		if (tx[0] != 0){
			if (out31 == 2){
				stepstatic << "<force bc = " << '"' << "x" << '"' << " type=" << '"' << "follow" << '"' << "lc=" << '"' << "1" << '"' << ">1<" << "/force>" << endl;
			}
			else if (out31 != 2){
				stepstatic << "<force bc = " << '"' << "x" << '"' << " type=" << '"' << "follow" << '"' << "lc=" << '"' << "7" << '"' << ">1<" << "/force>" << endl;
			}
		}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		int out41 = compaire(fy, ty);
		if (ty[0] != 0){
			if (out41 == 2){
				stepstatic << "<force bc = " << '"' << "y" << '"' << " type=" << '"' << "follow" << '"' << "lc=" << '"' << "2" << '"' << ">1<" << "/force>" << endl;
			}
			else if (out41 != 2){
				stepstatic << "<force bc = " << '"' << "y" << '"' << " type=" << '"' << "follow" << '"' << "lc=" << '"' << "8" << '"' << ">1<" << "/force>" << endl;
			}
		}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////		
		int out51 = compaire(fz, tz);
		if (tz[0] != 0){
			if (out51 == 2){
				stepstatic << "<force bc = " << '"' << "z" << '"' << " type=" << '"' << "follow" << '"' << "lc=" << '"' << "3" << '"' << ">1<" << "/force>" << endl;
			}
			else if (out51 != 2){
				stepstatic << "<force bc = " << '"' << "z" << '"' << " type=" << '"' << "follow" << '"' << "lc=" << '"' << "9" << '"' << ">1<" << "/force>" << endl;
			}
		}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		int out61 = compaire(fox, tox);
		if (tox[0] != 0){
			if (out61 == 2){
				stepstatic << "<force bc = " << '"' << "Rx" << '"' << " type=" << '"' << "follow" << '"' << "lc=" << '"' << "4" << '"' << ">1<" << "/force>" << endl;
			}
			else if (out61 != 2){
				stepstatic << "<force bc = " << '"' << "Rx" << '"' << " type=" << '"' << "follow" << '"' << "lc=" << '"' << "10" << '"' << ">1<" << "/force>" << endl;
			}
		}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		int out71 = compaire(foy, toy);
		if (toy[0] != 0){
			if (out71 == 2){
				stepstatic << "<force bc = " << '"' << "Ry" << '"' << " type=" << '"' << "follow" << '"' << "lc=" << '"' << "5" << '"' << ">1<" << "/force>" << endl;
			}
			else if (out71 != 2){
				stepstatic << "<force bc = " << '"' << "Ry" << '"' << " type=" << '"' << "follow" << '"' << "lc=" << '"' << "11" << '"' << ">1<" << "/force>" << endl;
			}
		}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		int out81 = compaire(foz, toz);
		if (toz[0] != 0){
			if (out81 == 2){
				stepstatic << "<force bc = " << '"' << "Rz" << '"' << " type=" << '"' << "follow" << '"' << "lc=" << '"' << "6" << '"' << ">1<" << "/force>" << endl;
			}
			else if (out81 != 2){
				stepstatic << "<force bc = " << '"' << "Rz" << '"' << " type=" << '"' << "follow" << '"' << "lc=" << '"' << "12" << '"' << ">1<" << "/force>" << endl;
			}
		}
		
		stepstatic << "</rigid_body>" << endl;
		///////////////////////////////////////////////////////////////////////////////////////////////

		//stepstatic << "<rigid_body mat = " << '"' << "2" << '"' << ">" << endl;
		/*
		if (tx[0] == 0){
			stepstatic << "<fixed bc = " << '"' << "x" << '"' << "/>" << endl;
		}
		if (ty[0] == 0){
			stepstatic << "<fixed bc = " << '"' << "y" << '"' << "/>" << endl;;
		}
		if (tz[0] == 0){
			stepstatic << "<fixed bc = " << '"' << "z" << '"' << "/>" << endl;;
		}
		if (tox[0] == 0){
			stepstatic << "<fixed bc = " << '"' << "Rx" << '"' << "/>" << endl;;
		}
		if (toy[0] == 0){
			stepstatic << "<fixed bc = " << '"' << "Ry" << '"' << "/>" << endl;;
		}
		if (toz[0] == 0){
			stepstatic << "<fixed bc = " << '"' << "Rz" << '"' << "/>" << endl;;
		}
		stepstatic << "</rigid_body>" << endl;
		*/
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		////////////////////////////////////////////////////////

		stepdynamic << "<Constraints>" << endl;
		stepdynamic << "<rigid_body mat = " << '"' << "1" << '"' << ">" << endl;
		int out = detect(fx);
		//if (out == 1){
		//	stepdynamic << "<fixed bc = " << '"' << "x" << '"' << "/>" << endl;
		//}
		if (out == 0){
			stepdynamic << "<force bc = " << '"' << "x" << '"' << " type=" << '"' << "follow" << '"' << "lc=" << '"' << "13" << '"' << ">1<" << "/force>" << endl;
				}
			
			
		
		//////////////
		int out2 = detect(fy);
		//if (out2 == 1){
		//	stepdynamic << "<fixed bc = " << '"' << "y" << '"' << "/>" << endl;
		//}
		if (out2 == 0){
			stepdynamic << "<force bc = " << '"' << "y" << '"' << " type=" << '"' << "follow" << '"' << "lc=" << '"' << "14" << '"' << ">1<" << "/force>" << endl;
		}
		//////////////////
		int out3 = detect(fz);
		//if (out3 == 1){
		//	stepdynamic << "<fixed bc = " << '"' << "z" << '"' << "/>" << endl;
		//}
		if (out3 == 0){
			stepdynamic << "<force bc = " << '"' << "z" << '"' << " type=" << '"' << "follow" << '"' << "lc=" << '"' << "15" << '"' << ">1<" << "/force>" << endl;
		}
		/////////////////////
		int out4 = detect(fox);
		//if (out4 == 1){
		//	stepdynamic << "<fixed bc = " << '"' << "Rx" << '"' << "/>" << endl;
		//}
		if (out4 == 0){
			stepdynamic << "<force bc = " << '"' << "Rx" << '"' << " type=" << '"' << "follow" << '"' << "lc=" << '"' << "16" << '"' << ">1<" << "/force>" << endl;
		}
		///////////////////////
		int out5 = detect(foy);
		//if (out5 == 1){
		//	stepdynamic << "<fixed bc = " << '"' << "Ry" << '"' << "/>" << endl;
		//}
		if (out5 == 0){
			stepdynamic << "<force bc = " << '"' << "Ry" << '"' << " type=" << '"' << "follow" << '"' << "lc=" << '"' << "17" << '"' << ">1<" << "/force>" << endl;
		}
		/////////////////////////
		int out6 = detect(foz);
		//if (out6 == 1){
		//	stepdynamic << "<fixed bc = " << '"' << "Rz" << '"' << "/>" << endl;
		//}
		if (out6 == 0){
			stepdynamic << "<force bc = " << '"' << "Rz" << '"' << " type=" << '"' << "follow" << '"' << "lc=" << '"' << "18" << '"' << ">1<" << "/force>" << endl;
		}
		stepdynamic << "</rigid_body>" << endl;
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

		stepdynamic << "<rigid_body mat = " << '"' << "2" << '"' << ">" << endl;


int out7 = detect(tx);
		//if (out7 == 1){
		//	stepdynamic << "<fixed bc = " << '"' << "x" << '"' << "/>" << endl;
		//}
		if (out7 == 0){
			if (out31 == 2){
				stepdynamic << "<force bc = " << '"' << "x" << '"' << " type=" << '"' << "follow" << '"' << "lc=" << '"' << "13" << '"' << ">1<" << "/force>" << endl;
			}
			else if (out31 != 2){
				stepdynamic << "<force bc = " << '"' << "x" << '"' << " type=" << '"' << "follow" << '"' << "lc=" << '"' << "19" << '"' << ">1<" << "/force>" << endl;
			}
		}
		////////////////////
		int out8 = detect(ty);
		//if (out8 == 1){
		//	stepdynamic << "<fixed bc = " << '"' << "y" << '"' << "/>" << endl;
		//}
		if (out8 == 0){
			if (out41 == 2){
				stepdynamic << "<force bc = " << '"' << "y" << '"' << " type=" << '"' << "follow" << '"' << "lc=" << '"' << "14" << '"' << ">1<" << "/force>" << endl;
			}
			else if (out41 != 2){
				stepdynamic << "<force bc = " << '"' << "y" << '"' << " type=" << '"' << "follow" << '"' << "lc=" << '"' << "20" << '"' << ">1<" << "/force>" << endl;
			}
		}
		////////////////////
		int out9 = detect(tz);
		//if (out9 == 1){
		//	stepdynamic << "<fixed bc = " << '"' << "z" << '"' << "/>" << endl;
	//	}
		if (out9 == 0){
			if (out51 == 2){
				stepdynamic << "<force bc = " << '"' << "z" << '"' << " type=" << '"' << "follow" << '"' << "lc=" << '"' << "15" << '"' << ">1<" << "/force>" << endl;
			}
			else if (out51 != 2){
				stepdynamic << "<force bc = " << '"' << "z" << '"' << " type=" << '"' << "follow" << '"' << "lc=" << '"' << "21" << '"' << ">1<" << "/force>" << endl;
			}
		}
		//////////////////////////
		int out10 = detect(tox);
		//if (out10 == 1){
		//	stepdynamic << "<fixed bc = " << '"' << "Rx" << '"' << "/>" << endl;
		//}
		if (out10 == 0){
			if (out61 == 2){
				stepdynamic << "<force bc = " << '"' << "Rx" << '"' << " type=" << '"' << "follow" << '"' << "lc=" << '"' << "16" << '"' << ">1<" << "/force>" << endl;
			}
			else if (out61 != 2){
				stepdynamic << "<force bc = " << '"' << "Rx" << '"' << " type=" << '"' << "follow" << '"' << "lc=" << '"' << "22" << '"' << ">1<" << "/force>" << endl;
			}
		}
		////////////////////
		int out11 = detect(toy);
		//if (out11 == 1){
		//	stepdynamic << "<fixed bc = " << '"' << "Ry" << '"' << "/>" << endl;
		//}
		if (out11 == 0){
			if (out71 == 2){
				stepdynamic << "<force bc = " << '"' << "Ry" << '"' << " type=" << '"' << "follow" << '"' << "lc=" << '"' << "17" << '"' << ">1<" << "/force>" << endl;
			}
			else if (out71 != 2){
				stepdynamic << "<force bc = " << '"' << "Ry" << '"' << " type=" << '"' << "follow" << '"' << "lc=" << '"' << "23" << '"' << ">1<" << "/force>" << endl;
			}
		}
		/////////////////////////
		int out12 = detect(toz);
		//if (out12 == 1){
		//	stepdynamic << "<fixed bc = " << '"' << "Rz" << '"' << "/>" << endl;
		//}
		if (out12 == 0){
			if (out81 == 2){
				stepdynamic << "<force bc = " << '"' << "Rz" << '"' << " type=" << '"' << "follow" << '"' << "lc=" << '"' << "18" << '"' << ">1<" << "/force>" << endl;
			}
			else if (out81 != 2){
				stepdynamic << "<force bc = " << '"' << "Rz" << '"' << " type=" << '"' << "follow" << '"' << "lc=" << '"' << "24" << '"' << ">1<" << "/force>" << endl;
			}
		}
		//////////////////////////
		stepdynamic << "</rigid_body>" << endl;
		


		
	}


	int BF_structor::detect(Vector fx){
		int count = 0;
		int out = 0;
		int sizer = fx.size();
		for (int o = 0; o < sizer; ++o){if (fx(o) == 0){ ++count; }}
		if (count == sizer){ out = 1; }
		return out;
	}

	int BF_structor::compaire(Vector fx, Vector tx){
		int count = 0;
		int out = 0;
		int sizer = fx.size();
		for (int o = 0; o < sizer; ++o){ if (fx(o) == tx(o)){ ++count; } }
		if (count == sizer){ out = 2; }
		return out;
	}