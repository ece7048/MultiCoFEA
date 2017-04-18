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
#include "Settings.h"
#include "BK_structor.h"

BK_structor::BK_structor(void)
{
}

BK_structor::~BK_structor(void)
{
}
void BK_structor::begining(int iteration,string kind[24], int place, int sizer1, string resultDir1, Vector time1, Vector fx1, Vector fy1, Vector fz1, Vector fox1, Vector foy1, Vector foz1, Vector tx1, Vector ty1, Vector tz1, Vector tox1, Vector toy1, Vector toz1)
{
	INIReader ini = INIReader(INI_FILE);
		// Asking for constrain section if the user wants:
		char caser1;
		double costr1;
		double costr2;
		double costr3;
		cout << "" << endl;
		cout << "Do you want to set constrains betweeen the two bodies in each axis translation motion(y/n)" << endl;

		cin >> caser1;
		if (caser1 == 'y'){
			cout << "Give the max absolute distance between the two bodies in meter, first in x axis, second in y axis and in z axis" << endl;
			cin >> costr1;
			cin >> costr2;
			cin >> costr3;
			cout << "....................................................." << endl;
			fx1= costrain_translation(fx1,  tx1,  costr1,  'x');
			cout << "....................................................."<<endl;
			fy1= costrain_translation(fy1,  ty1,  costr2,  'y');
			cout << "....................................................." << endl;
			fz1= costrain_translation(fz1,  tz1,  costr3, 'z');
			cout << "....................................................." << endl;
		}
		if (caser1 != 'y'&& caser1 != 'n'){
			cout << "Error::invalied answer!!" << endl;
		}

	
		//////////////////////////////////////////////strategy fem ////////////////////////////////////////

	   
		cout << "" << endl;
		cout << "Which will be the stragety that we will follow for the FEM analysis for the Prescribed DOF of the RBs?" << endl;
		cout << "Set the DOF which will be Prescribed for the two bodies in Prescribed_DOF_1 and Prescribed_DOF_2 in .ini file." << endl;
		cout << "PS if you set a rotation DOF Prescribed then the other rotDOF have to be fixed or Prescribed too" << endl;
		cout << "PS1 for the rotation dof set rx or ry or rz and for traslation x y or z." << endl;
		cout << "PS2 if you set in setup file set the Prescribed dof like xyz_rxryrz first the traslation DOF and second the rotation seperate with _ " << endl;
		cout << "PS3 if you do not want ant DOF Prescribed just set in setup.ini file the word, NAN" << endl;
		cout << "PS4 if you already set the Prescribed stragety in setup.ini file then your choice will be below :" << endl;


		string presc1 = ini.Get("BASICSETUP", "Prescribed_DOF_1", "");
		if (presc1 != ""){
			cout << "For the first Rigid body the prescribed dof are:" << endl;
			cout << presc1 << endl;
		}
		string presc2 = ini.Get("BASICSETUP", "Prescribed_DOF_2", "");
		if (presc2 != ""){
			cout << "For the second Rigid body the prescribed dof are:" << endl;
			cout << presc2 << endl;
		}
		if (presc1 == ""){
			cout << "give now the DOF Prescribed first traslation, second rotation seperated by _ for the first Rigid Body" << endl;
			cout << "If you do not want any DOF Prescribed set NAN, you can set only rotation if you want." << endl;
			string forc;
			cin >> forc;
			presc1 = forc;
		}
		if (presc2 == ""){
			cout << "give now the DOF Prescribed first traslation, second rotation seperated by _ for the second Rigid Body" << endl;
			cout << "If you do not want any DOF Prescribed set NAN, you can set only rotation if you want." << endl;
			string forc;
			cin >> forc;
			presc2 = forc;
		}


		if (presc1 == "NAN"){ fx1 = fy1 = fz1 = fox1 = foy1 = foz1 =0; }
		if (presc1 != "NAN"){
			if (presc1[0] != 'x' && presc1[1] != 'x' && presc1[2] != 'x'){ fx1  = 0; }
			if (presc1[0] != 'y' && presc1[1] != 'y' && presc1[2] != 'y'){ fy1  = 0; }
			if (presc1[0] != 'z' && presc1[1] != 'z' && presc1[2] != 'z'){ fz1  = 0; }
			if (presc1[1] != '_' && presc1[2] != '_' && presc1[3] != '_'){
				if (presc1[3] != 'x' && presc1[5] != 'x' && presc1[7] != 'x'&& presc1[9] != 'x'&& presc1[11] != 'x'){ fox1  = 0; }
				if (presc1[3] != 'y' && presc1[5] != 'y' && presc1[7] != 'y'&& presc1[9] != 'y'&& presc1[11] != 'y'){ foy1  = 0; }
				if (presc1[3] != 'z' && presc1[5] != 'z' && presc1[7] != 'z'&& presc1[9] != 'z'&& presc1[11] != 'z'){ foz1  = 0; }
			}
			if (presc1[0] == 'r'){
				if (presc1[2] != 'x' && presc1[4] != 'x' && presc1[6] != 'x'){ fox1  = 0; }
				if (presc1[2] != 'y' && presc1[4] != 'y' && presc1[6] != 'y'){ foy1  = 0; }
				if (presc1[2] != 'z' && presc1[4] != 'z' && presc1[6] != 'z'){ foz1 = 0; }
			}
		}

		if (presc2 == "NAN"){ tx1 = ty1 = tz1 = tox1 = toy1 = toz1 = 0; }
		if (presc2 != "NAN"){
			if (presc2[0] != 'x' && presc2[1] != 'x' && presc2[2] != 'x'){ tx1 = 0; }
			if (presc2[0] != 'y' && presc2[1] != 'y' && presc2[2] != 'y'){ ty1 = 0; }
			if (presc2[0] != 'z' && presc2[1] != 'z' && presc2[2] != 'z'){ tz1 = 0; }
			if (presc2[1] != '_' && presc2[2] != '_' && presc2[3] != '_'){
				if (presc2[3] != 'x' && presc2[5] != 'x' && presc2[7] != 'x'&& presc2[9] != 'x'&& presc2[11] != 'x'){ tox1 = 0; }
				if (presc2[3] != 'y' && presc2[5] != 'y' && presc2[7] != 'y'&& presc2[9] != 'y'&& presc2[11] != 'y'){ toy1 = 0; }
				if (presc2[3] != 'z' && presc2[5] != 'z' && presc2[7] != 'z'&& presc2[9] != 'z'&& presc2[11] != 'z'){ toz1 = 0; }
			}
			if (presc2[0] == 'r'){
				if (presc2[2] != 'x' && presc2[4] != 'x' && presc2[6] != 'x'){ tox1 = 0; }
				if (presc2[2] != 'y' && presc2[4] != 'y' && presc2[6] != 'y'){ toy1 = 0; }
				if (presc2[2] != 'z' && presc2[4] != 'z' && presc2[6] != 'z'){ toz1 = 0; }
			}
		}






		cout << "" << endl;
		cout << "Which will be the stragety that we will follow for the FEM analysis for the Fixed DOF of the RBs?" << endl;
		cout << "Set the DOF which will be Fixed for the two bodies in Fixed_DOF_1 and Fixed_DOF_2 in .ini file." << endl;
		cout << "PS1 for the rotation dof set rx or ry or rz and for traslation x y or z." << endl;
		cout << "PS2 if you set in setup file set the Fixed dof like xyz_rxryrz first the traslation DOF and second the rotation seperate with _ " << endl;
		cout << "PS3 if you do not want any DOF Fixed just set in setup.ini file the word, NAN" << endl;
		cout << "PS4 if you already set the Fixed stragety in setup.ini file then your choice will be below :" << endl;


		string fix1 = ini.Get("BASICSETUP", "Fixed_DOF_1", "");
		if (fix1 != ""){
			cout << "For the first Rigid body the Fixed dof are:" << endl;
			cout << fix1 << endl;
		}
		string fix2 = ini.Get("BASICSETUP", "Fixed_DOF_2", "");
		if (fix2 != ""){
			cout << "For the second Rigid body the Fixed dof are:" << endl;
			cout << fix2 << endl;
		}
		if (fix1 == ""){
			cout << "give now the DOF Fixed first traslation, second rotation seperated by _ for the first Rigid Body" << endl;
			cout << "If you do not want any DOF Fixed set NAN, you can set only rotation if you want." << endl;
			string forc;
			cin >> forc;
			fix1 = forc;
		}
		if (fix2 == ""){
			cout << "give now the DOF Fixed first traslation, second rotation seperated by _ for the second Rigid Body" << endl;
			cout << "If you do not want any DOF Fixed set NAN, you can set only rotation if you want." << endl;
			string forc;
			cin >> forc;
			fix2 = forc;
		}


		int fx, fy, fz, fox, foy, foz, tx, ty, tz, tox, toy, toz;
		fx = fy = fz = fox = foy = foz = tox = toy = toz = tx = ty = tz = 1;
		
		if (fix1 == "NAN"){
			fx = fy = fz = fox = foy = foz = 0;
		}
		if (fix1 != "NAN"){
			if (fix1[0] != 'x' && fix1[1] != 'x' && fix1[2] != 'x'){ fx = 0; }
			if (fix1[0] != 'y' && fix1[1] != 'y' && fix1[2] != 'y'){ fy = 0; }
			if (fix1[0] != 'z' && fix1[1] != 'z' && fix1[2] != 'z'){ fz = 0; }
			if (fix1[1] != '_' && fix1[2] != '_' && fix1[3] != '_'){
				if (fix1[3] != 'x' && fix1[5] != 'x' && fix1[7] != 'x'&& fix1[9] != 'x'&& fix1[11] != 'x'){ fox = 0; }
				if (fix1[3] != 'y' && fix1[5] != 'y' && fix1[7] != 'y'&& fix1[9] != 'y'&& fix1[11] != 'y'){ foy = 0; }
				if (fix1[3] != 'z' && fix1[5] != 'z' && fix1[7] != 'z'&& fix1[9] != 'z'&& fix1[11] != 'z'){ foz= 0; }
			}
			if (fix1[0] == 'r'){
				if (fix1[2] != 'x' && fix1[4] != 'x' && fix1[6] != 'x'){ fox = 0; }
				if (fix1[2] != 'y' && fix1[4] != 'y' && fix1[6] != 'y'){ foy = 0; }
				if (fix1[2] != 'z' && fix1[4] != 'z' && fix1[6] != 'z'){ foz = 0; }
			}
		}

		if (fix2 == "NAN"){ tox = toy = toz = tx = ty = tz = 0; }
		if (fix2 != "NAN"){
			if (fix2[0] != 'x' && fix2[1] != 'x' && fix2[2] != 'x'){ tx = 0; }
			if (fix2[0] != 'y' && fix2[1] != 'y' && fix2[2] != 'y'){ ty = 0; }
			if (fix2[0] != 'z' && fix2[1] != 'z' && fix2[2] != 'z'){ tz = 0; }
			if (fix2[1] != '_' && fix2[2] != '_' && fix2[3] != '_'){
				if (fix2[3] != 'x' && fix2[5] != 'x' && fix2[7] != 'x'&& fix2[9] != 'x'&& fix2[11] != 'x'){ tox = 0; }
				if (fix2[3] != 'y' && fix2[5] != 'y' && fix2[7] != 'y'&& fix2[9] != 'y'&& fix2[11] != 'y'){ toy = 0; }
				if (fix2[3] != 'z' && fix2[5] != 'z' && fix2[7] != 'z'&& fix2[9] != 'z'&& fix2[11] != 'z'){ toz = 0; }
			}
			if (fix2[0] == 'r'){
				if (fix2[2] != 'x' && fix2[4] != 'x' && fix2[6] != 'x'){ tox = 0; }
				if (fix2[2] != 'y' && fix2[4] != 'y' && fix2[6] != 'y'){ toy = 0; }
				if (fix2[2] != 'z' && fix2[4] != 'z' && fix2[6] != 'z'){ toz = 0; }
			}
		}






		// place 1 is for angles input and the 0 for rads input in orientation angles (EUler)
		
		ofstream stepstatic;
		ofstream stepdynamic;
		ofstream valueno;
		ofstream valueno1;
		ofstream valueno2;
		ofstream valueno3;
		ofstream valueno4;
		ofstream valueno5;
		ofstream valueno6;
		ofstream valueno7;
		ofstream valueno8;
		ofstream valueno9;
		ofstream valueno10;
		ofstream valueno11;

		ofstream valuein;
		char itter = iteration + '0';
		stepstatic = ofstream(resultDir1 + "/FEBstepstatic_position"+itter+".txt", ofstream::out);
		stepdynamic = ofstream(resultDir1 + "/FEBstepdynamic_position" + itter + ".txt", ofstream::out);

		if (place == 1){
			valueno = ofstream(resultDir1 + "/dof1.txt", ofstream::out);
			valueno1 = ofstream(resultDir1 + "/dof2.txt", ofstream::out);
			valueno2 = ofstream(resultDir1 + "/dof3.txt", ofstream::out);
			valueno3 = ofstream(resultDir1 + "/dof4.txt", ofstream::out);
			valueno4 = ofstream(resultDir1 + "/dof5.txt", ofstream::out);
			valueno5 = ofstream(resultDir1 + "/dof6.txt", ofstream::out);
			valueno6 = ofstream(resultDir1 + "/dof7.txt", ofstream::out);
			valueno7 = ofstream(resultDir1 + "/dof8.txt", ofstream::out);
			valueno8 = ofstream(resultDir1 + "/dof9.txt", ofstream::out);
			valueno9 = ofstream(resultDir1 + "/dof10.txt", ofstream::out);
			valueno10 = ofstream(resultDir1 + "/dof11.txt", ofstream::out);
			valueno11 = ofstream(resultDir1 + "/dof12.txt", ofstream::out);
			
			valuein = ofstream(resultDir1 + "/FEBdatapositionsstatic" + itter + ".txt", ofstream::out);
		}
		if (place == 0){
			valueno = ofstream(resultDir1 + "/dof1.txt", ofstream::out);
			valueno1 = ofstream(resultDir1 + "/dof2.txt", ofstream::out);
			valueno2 = ofstream(resultDir1 + "/dof3.txt", ofstream::out);
			valueno3 = ofstream(resultDir1 + "/dof4.txt", ofstream::out);
			valueno4 = ofstream(resultDir1 + "/dof5.txt", ofstream::out);
			valueno5 = ofstream(resultDir1 + "/dof6.txt", ofstream::out);
			valueno6 = ofstream(resultDir1 + "/dof7.txt", ofstream::out);
			valueno7 = ofstream(resultDir1 + "/dof8.txt", ofstream::out);
			valueno8 = ofstream(resultDir1 + "/dof9.txt", ofstream::out);
			valueno9 = ofstream(resultDir1 + "/dof10.txt", ofstream::out);
			valueno10 = ofstream(resultDir1 + "/dof11.txt", ofstream::out);
			valueno11 = ofstream(resultDir1 + "/dof12.txt", ofstream::out);
			valuein = ofstream(resultDir1 + "/FEBdatapositionsstatic" + itter + ".txt", ofstream::out);
		}


		//asking for offset
		double offset1 = 0;
		double offset2 = 0;
		double offset3 = 0;
		double offset1s = 0;
		double offset2s = 0;
		double offset3s = 0;
		char caser;
		/*
		cout << "" << endl;
		cout << "Do you want an Initial offset betweeen the two bodies (y/n)" << endl;
		cin >> caser;
		if (caser == 'y'){
			cout << "Give the Initial offset values in mm, first in x axis, second in y axis and in z axis" << endl;
			cin >> offset1;
			cin >> offset2;
			cin >> offset3;
			// ADDING only initial offset for the model if the DOF is not translational...for one or more of them...
			offset1s = offset1;
			offset2s = offset2;
			offset3s = offset3;
			int on = detect(fx1);
			if (on== 1){ offset1 = 0; }
			int ony = detect(fy1);
			if (ony == 1){ offset2 = 0; }
			int onz = detect(fz1);
			if (onz == 1){ offset3 = 0; }
			//////////////////////////////////
		}
		if (caser != 'y'&& caser != 'n'){
			cout << "Error::invalied answer!!" << endl;
		}
		*/
		////////////STATIC PART/////////////////////////////////////////////////////////////////////////////////////


		//first
		fx1[0] = fx1[0] * 10000+offset1s*10;
		int parast1 = (int)fx1[0];
		fx1[0] = parast1 * 0.1;
		fy1[0] = fy1[0] * 10000 + offset2s * 10;
		int parasty1 = (int)fy1[0];
		fy1[0] = parasty1 * 0.1;
		fz1[0] = fz1[0] * 10000 + offset3s * 10;
		int parastz1 = (int)fz1[0];
		fz1[0] = parastz1 * 0.1;
		//
		if (place == 1){
			fox1[0] = fox1[0] * 10000 * 3.14 / 180;
			int paratfox1 = (int)fox1[0];
			fox1[0] = paratfox1 * 0.0001;
			foy1[0] = foy1[0] * 10000 * 3.14 / 180;
			int paratfoy1 = (int)foy1[0];
			foy1[0] = paratfoy1 * 0.0001;
			foz1[0] = foz1[0] * 10000 * 3.14 / 180;
			int paratfoz1 = (int)foz1[0];
			foz1[0] = paratfoz1 * 0.0001;
		}
		if (place == 0){
			fox1[0] = fox1[0] * 10000 ;
			int paratfox1 = (int)fox1[0];
			fox1[0] = paratfox1 * 0.0001;
			foy1[0] = foy1[0] * 10000;
			int paratfoy1 = (int)foy1[0];
			foy1[0] = paratfoy1 * 0.0001;
			foz1[0] = foz1[0] * 10000;
			int paratfoz1 = (int)foz1[0];
			foz1[0] = paratfoz1 * 0.0001;
		}





		// second
		tx1[0] = tx1[0] * 10000;
		int parastt1 = (int)tx1[0];
		tx1[0] = parastt1 * 0.1;
		ty1[0] = ty1[0] * 10000;
		int parastty1 = (int)ty1[0];
		ty1[0] = parastty1 * 0.1;
		tz1[0] = tz1[0] * 10000;
		int parasttz1 = (int)tz1[0];
		tz1[0] = parasttz1 * 0.1;
		//
		if (place == 1){
			tox1[0] = tox1[0] * 10000 * 3.14 / 180;
			int parattox1 = (int)tox1[0];
			tox1[0] = parattox1 * 0.0001;
			toy1[0] = toy1[0] * 10000 * 3.14 / 180;
			int parattoy1 = (int)toy1[0];
			toy1[0] = parattoy1 * 0.0001;
			toz1[0] = toz1[0] * 10000 * 3.14 / 180;
			int parattoz1 = (int)toz1[0];
			toz1[0] = parattoz1 * 0.0001;
		}

		if (place == 0){
			tox1[0] = tox1[0] * 10000;
			int parattox1 = (int)tox1[0];
			tox1[0] = parattox1 * 0.00010;
			toy1[0] = toy1[0] * 10000;
			int parattoy1 = (int)toy1[0];
			toy1[0] = parattoy1 * 0.0001;
			toz1[0] = toz1[0] * 10000;
			int parattoz1 = (int)toz1[0];
			toz1[0] = parattoz1 * 0.0001;
		}


		valuein << "<loadcurve id=" << '"' << "25" << '"' << "type=" << '"' << "linear" << '"' << "extend =" << '"' << "constant" << '"' << ">" << endl;
		valuein << "<point>" << time1[0] << "," << 0 << "</point>" << endl;
		
		valuein << "<point>" << time1[1]  << "," << fx1[0] << "</point>" << endl;
		valuein << "</loadcurve>" << endl;

		valuein << "<loadcurve id=" << '"' << "26" << '"' << "type=" << '"' << "linear" << '"' << "extend =" << '"' << "constant" << '"' << ">" << endl;
		valuein << "<point>" << time1[0] << "," << 0 << "</point>" << endl;
		
		valuein << "<point>" << time1[1]  << "," << fy1[0] << "</point>" << endl;
		valuein << "</loadcurve>" << endl;

		valuein << "<loadcurve id=" << '"' << "27" << '"' << "type=" << '"' << "linear" << '"' << "extend =" << '"' << "constant" << '"' << ">" << endl;
		valuein << "<point>" << time1[0] << "," << 0 << "</point>" << endl;
		
		valuein << "<point>" << time1[1]  << "," << fz1[0] << "</point>" << endl;
		valuein << "</loadcurve>" << endl;

		valuein << "<loadcurve id=" << '"' << "28" << '"' << "type=" << '"' << "linear" << '"' << "extend =" << '"' << "constant" << '"' << ">" << endl;
		valuein << "<point>" << time1[0] << "," << 0 << "</point>" << endl;
		
		valuein << "<point>" << time1[1]  << "," << fox1[0] << "</point>" << endl;
		valuein << "</loadcurve>" << endl;

		valuein << "<loadcurve id=" << '"' << "29" << '"' << "type=" << '"' << "linear" << '"' << "extend =" << '"' << "constant" << '"' << ">" << endl;
		valuein << "<point>" << time1[0] << "," << 0 << "</point>" << endl;
	
		valuein << "<point>" << time1[1]  << "," << foy1[0] << "</point>" << endl;
		valuein << "</loadcurve>" << endl;

		valuein << "<loadcurve id=" << '"' << "30" << '"' << "type=" << '"' << "linear" << '"' << "extend =" << '"' << "constant" << '"' << ">" << endl;
		valuein << "<point>" << time1[0] << "," << 0 << "</point>" << endl;
	
		valuein << "<point>" << time1[1]  << "," << foz1[0] << "</point>" << endl;
		valuein << "</loadcurve>" << endl;


		valuein << "<loadcurve id=" << '"' << "31" << '"' << "type=" << '"' << "linear" << '"' << "extend =" << '"' << "constant" << '"' << ">" << endl;
		valuein << "<point>" << time1[0] << "," << 0 << "</point>" << endl;
	
		valuein << "<point>" << time1[1]  << "," << tx1[0] << "</point>" << endl;
		valuein << "</loadcurve>" << endl;


		valuein << "<loadcurve id=" << '"' << "32" << '"' << "type=" << '"' << "linear" << '"' << "extend =" << '"' << "constant" << '"' << ">" << endl;
		valuein << "<point>" << time1[0] << "," << 0 << "</point>" << endl;
		valuein << "<point>" << time1[1]  << "," << ty1[0] << "</point>" << endl;
		valuein << "</loadcurve>" << endl;


		valuein << "<loadcurve id=" << '"' << "33" << '"' << "type=" << '"' << "linear" << '"' << "extend =" << '"' << "constant" << '"' << ">" << endl;
		valuein << "<point>" << time1[0] << "," << 0 << "</point>" << endl;
		
		valuein << "<point>" << time1[1]  << "," << tz1[0] << "</point>" << endl;
		valuein << "</loadcurve>" << endl;


		valuein << "<loadcurve id=" << '"' << "34" << '"' << "type=" << '"' << "linear" << '"' << "extend =" << '"' << "constant" << '"' << ">" << endl;
		valuein << "<point>" << time1[0] << "," << 0 << "</point>" << endl;
		
		valuein << "<point>" << time1[1]  << "," << tox1[0] << "</point>" << endl;
		valuein << "</loadcurve>" << endl;


		valuein << "<loadcurve id=" << '"' << "35" << '"' << "type=" << '"' << "linear" << '"' << "extend =" << '"' << "constant" << '"' << ">" << endl;
		valuein << "<point>" << time1[0] << "," << 0 << "</point>" << endl;
		
		valuein << "<point>" << time1[1]  << "," << toy1[0] << "</point>" << endl;
		valuein << "</loadcurve>" << endl;

		valuein << "<loadcurve id=" << '"' << "36" << '"' << "type=" << '"' << "linear" << '"' << "extend =" << '"' << "constant" << '"' << ">" << endl;
		valuein << "<point>" << time1[0] << "," << 0 << "</point>" << endl;
		
		valuein << "<point>" << time1[1]  << "," << toz1[0] << "</point>" << endl;
		valuein << "</loadcurve>" << endl;


		///////////////DYNAMIC PART//////////////////////////////////////////////////////////////////////////////////////////


for (int i = 1; i < sizer1 - 1; i++){


		////FIRST BODY
		
	


			if (i > 0){
				fx1[i] = fx1[i] * 10000 + offset1 * 10;
				int para1 = (int)fx1[i];
				fx1[i] = para1 * 0.1;
			}
			if (i == 1){
				valueno << "<loadcurve id=" << '"' << "37" << '"' << "type=" << '"' << kind[12] << '"' << "extend =" << '"' << "constant" << '"' << ">" << endl;
			}
			valueno << "<point>" << time1[i+1] << "," << fx1[i] << "</point>" << endl;

			if (i == sizer1-2){
				valueno << "</loadcurve>" << endl;
			}
///////////////////////////////////////////////////////
		
			if (i > 0){
				fy1[i] = fy1[i] * 10000 + offset2 * 10;
				int paray1 = (int)fy1[i];
				fy1[i] = paray1 * 0.1;
			}
			if (i == 1){
				valueno1 << "<loadcurve id=" << '"' << "38" << '"' << "type=" << '"' << kind[13] << '"' << "extend =" << '"' << "constant" << '"' << ">" << endl;
			}
			valueno1 << "<point>" << time1[i+1] << "," << fy1[i] << "</point>" << endl;

		
			if (i == sizer1 - 2){
				valueno1 << "</loadcurve>" << endl;
			}
///////////////////////////////////////
		
			if (i > 0){
				fz1[i] = fz1[i] * 10000 + offset3 * 10;
				int paraz1 = (int)fz1[i];
				fz1[i] = paraz1 * 0.1;
			}
			if (i == 1){
				valueno2 << "<loadcurve id=" << '"' << "39" << '"' << "type=" << '"' << kind[14] << '"' << "extend =" << '"' << "constant" << '"' << ">" << endl;
			}
			valueno2 << "<point>" << time1[i + 1] << "," << fz1[i] << "</point>" << endl;

		
		if (i == sizer1 - 2){
			valueno2 << "</loadcurve>" << endl;
		}
		////////////////////////////////////////////////////////////////////////

		if (place == 1){
		
				if (i > 0){
					fox1[i] = fox1[i] * 10000 * 3.14 / 180;
					int parax1 = (int)fox1[i];
					fox1[i] = parax1 * 0.0001;
				}
			}
		
		if (place == 0){
					if (i > 0){
					fox1[i] = fox1[i] * 10000;
					int parax1 = (int)fox1[i];
					fox1[i] = parax1 * 0.0001;
				}
			}
		if (i == 1){
			valueno3 << "<loadcurve id=" << '"' << "40" << '"' << "type=" << '"' << kind[15] << '"' << "extend =" << '"' << "constant" << '"' << ">" << endl;
		}
			valueno3 << "<point>" << time1[i+1] << "," << fox1[i] << "</point>" << endl;

		
			if (i == sizer1 - 2){
				valueno3 << "</loadcurve>" << endl;
			}
/////////////////////////			//////////////////////////////////////////////////////////////////
		if (place == 1){
				if (i > 0){
					foy1[i] = foy1[i] * 10000 * 3.14 / 180;
					int paraoy1 = (int)foy1[i];
					foy1[i] = paraoy1 * 0.0001;
				}
			}
		
		if (place == 0){
			
				if (i > 0){
					foy1[i] = foy1[i] * 10000;
					int paraoy1 = (int)foy1[i];
					foy1[i] = paraoy1 * 0.0001;
				}
			}
		if (i == 1){
			valueno4 << "<loadcurve id=" << '"' << "41" << '"' << "type=" << '"' << kind[16] << '"' << "extend =" << '"' << "constant" << '"' << ">" << endl;
		}
			valueno4 << "<point>" << time1[i+1] << "," << foy1[i] << "</point>" << endl;

			if (i == sizer1 - 2){
				valueno4 << "</loadcurve>" << endl;
			}
/////////////////////////////////////////////////////////////////////////////////////////

		if (place == 1){
				if (i > 0){
					foz1[i] = foz1[i] * 10000 * 3.14 / 180;
					int paraoz1 = (int)foz1[i];
					foz1[i] = paraoz1 * 0.0001;
				}
			}
		
		if (place == 0){
		
				if (i > 0){
					foz1[i] = foz1[i] * 10000;
					int paraoz1 = (int)foz1[i];
					foz1[i] = paraoz1 * 0.0001;
				}
			}
		if (i == 1){
			valueno5 << "<loadcurve id=" << '"' << "42" << '"' << "type=" << '"' << kind[17] << '"' << "extend =" << '"' << "constant" << '"' << ">" << endl;
		}
		valueno5 << "<point>" << time1[i+1] << "," << foz1[i] << "</point>" << endl;
		if (i == sizer1 - 2){
			valueno5 << "</loadcurve>" << endl;
		}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

		//// SECOND BODY
		
		
			if (i > 0){
				tx1[i] = tx1[i] * 10000;
				int parat1 = (int)tx1[i];
				tx1[i] = parat1 * 0.1;
			}
			if (i == 1){
				valueno6 << "<loadcurve id=" << '"' << "43" << '"' << "type=" << '"' << kind[18] << '"' << "extend =" << '"' << "constant" << '"' << ">" << endl;
			}
		
			valueno6 << "<point>" << time1[i+1] << "," << tx1[i] << "</point>" << endl;

		
			if (i == sizer1 - 2){
				valueno6 << "</loadcurve>" << endl;
			}
///////////////////////////////////////////////////////////////
		
			if (i > 0){
				ty1[i] = ty1[i] * 10000;
				int paraty1 = (int)ty1[i];
				ty1[i] = paraty1 * 0.1;
			}
			if (i == 1){
				valueno7 << "<loadcurve id=" << '"' << "44" << '"' << "type=" << '"' << kind[19] << '"' << "extend =" << '"' << "constant" << '"' << ">" << endl;
			}
			valueno7 << "<point>" << time1[i+1] << "," << ty1[i] << "</point>" << endl;
		if (i == sizer1 - 2){
			valueno7 << "</loadcurve>" << endl;
		}
////////////////////////////////////////////////////////////

			if (i > 0){
				tz1[i] = tz1[i] * 10000;
				int paratz1 = (int)tz1[i];
				tz1[i] = paratz1 * 0.1;
			}
			if (i == 1){
				valueno8 << "<loadcurve id=" << '"' << "45" << '"' << "type=" << '"' << kind[20] << '"' << "extend =" << '"' << "constant" << '"' << ">" << endl;
			}
		valueno8 << "<point>" << time1[i+1] << "," << tz1[i] << "</point>" << endl;

		
		if (i == sizer1 - 2){
			valueno8 << "</loadcurve>" << endl;
		}
//////////////////////////////////////////////////////////////////////////////////
		if (place == 1){
				if (i > 0){
					tox1[i] = tox1[i] * 10000 * 3.14 / 180;
					int parattx1 = (int)tox1[i];
					tox1[i] = parattx1 * 0.0001;
				}
			}
		
		if (place == 0){
			if (i > 0){
					tox1[i] = tox1[i] * 10000;
					int parattx1 = (int)tox1[i];
					tox1[i] = parattx1 * 0.0001;
				}
			}
		if (i == 1){
			valueno9 << "<loadcurve id=" << '"' << "46" << '"' << "type=" << '"' << kind[21] << '"' << "extend =" << '"' << "constant" << '"' << ">" << endl;
		}
		valueno9 << "<point>" << time1[i+1] << "," << tox1[i] << "</point>" << endl;

		
		if (i == sizer1 - 2){
			valueno9 << "</loadcurve>" << endl;
		}
/////////////////////////////////////////////////////////////////////////////////////////////////

		if (place == 1){
			if (i > 0){
					toy1[i] = toy1[i] * 10000 * 3.14 / 180;
					int paratoy1 = (int)toy1[i];
					toy1[i] = paratoy1 * 0.0001;
				}
			}
		
		if (place == 0){
		   if (i > 0){
					toy1[i] = toy1[i] * 10000;
					int paratoy1 = (int)toy1[i];
					toy1[i] = paratoy1 * 0.0001;
				}
			}
		if (i == 1){
			valueno10 << "<loadcurve id=" << '"' << "47" << '"' << "type=" << '"' << kind[22] << '"' << "extend =" << '"' << "constant" << '"' << ">" << endl;
		}
		valueno10 << "<point>" << time1[i+1] << "," << toy1[i] << "</point>" << endl;

		if (i == sizer1 - 2){
			valueno10 << "</loadcurve>" << endl;
		}
/////////////////////////////////////////////////////////////////////////////		
		if (place == 1){
			if (i > 0){
					toz1[i] = toz1[i] * 10000 * 3.14 / 180;
					int paratoz1 = (int)toz1[i];
					toz1[i] = paratoz1 * 0.0001;
				}
			}
	
		if (place == 0){
			if (i > 0){
					toz1[i] = toz1[i] * 10000;
					int paratoz1 = (int)toz1[i];
					toz1[i] = paratoz1 * 0.0001;
				}
			}
		if (i == 1){
			valueno11 << "<loadcurve id=" << '"' << "48" << '"' << "type=" << '"' << kind[23] << '"' << "extend =" << '"' << "constant" << '"' << ">" << endl;
		}
		valueno11 << "<point>" << time1[i+1] << "," << toz1[i] << "</point>" << endl;

		
		if (i == sizer1 - 2){
			valueno11 << "</loadcurve>" << endl;
			valueno11 << "</LoadData>" << endl;
		}

	} //end of for loop
	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

		///////////////////////////////////////COSTRUCT the costrain of the positions for the two bodies/////////////////////////////////////

	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////STATIC STEP/////////////////////////////////////
		///////////////////////////////////////////

	stepstatic << "<rigid_body mat = " << '"' << "1" << '"' << ">" << endl;
	if (fx1[0] != 0){
		stepstatic << "<prescribed bc = " << '"' << "x" << '"' << "lc=" << '"' << "25" << '"' << ">1<" << "/prescribed>" << endl;
	}
	if (fy1[0] != 0){
		stepstatic << "<prescribed bc = " << '"' << "y" << '"' << "lc=" << '"' << "26" << '"' << ">1<" << "/prescribed>" << endl;
	}
	if (fz1[0] != 0){
		stepstatic << "<prescribed bc = " << '"' << "z" << '"' << "lc=" << '"' << "27" << '"' << ">1<" << "/prescribed>" << endl;
	}
	if (fox1[0] != 0){
		stepstatic << "<prescribed bc = " << '"' << "Rx" << '"' << "lc=" << '"' << "28" << '"' << ">1<" << "/prescribed>" << endl;
	}
	if (foy1[0] != 0){
		stepstatic << "<prescribed bc = " << '"' << "Ry" << '"' << "lc=" << '"' << "29" << '"' << ">1<" << "/prescribed>" << endl;
	}
	if (foz1[0] != 0){
		stepstatic << "<prescribed bc = " << '"' << "Rz" << '"' << "lc=" << '"' << "30" << '"' << ">1<" << "/prescribed>" << endl;
	}
	//stepstatic << "</rigid_body>" << endl;
	///////////////////////////////////////////////////////////////////////////////////////////////

	//stepstatic << "<rigid_body mat = " << '"' << "1" << '"' << ">" << endl;

	if (fx1[0] == 0){
		stepstatic << "<fixed bc = " << '"' << "x" << '"' << "/>" << endl;
	}
	if (fy1[0] == 0){
		stepstatic << "<fixed bc = " << '"' << "y" << '"' << "/>" << endl;;
	}
	if (fz1[0] == 0){
		stepstatic << "<fixed bc = " << '"' << "z" << '"' << "/>" << endl;;
	}
	if (fox1[0] == 0){
		stepstatic << "<fixed bc = " << '"' << "Rx" << '"' << "/>" << endl;;
	}
	if (foy1[0] == 0){
		stepstatic << "<fixed bc = " << '"' << "Ry" << '"' << "/>" << endl;;
	}
	if (foz1[0] == 0){
		stepstatic << "<fixed bc = " << '"' << "Rz" << '"' << "/>" << endl;;
	}
	stepstatic << "</rigid_body>" << endl;
	//////////////////////////////////////////////////////////////////////////////////////////////
	//second body///////////////////////
	//////////////////////////////////
	stepstatic << "<rigid_body mat = " << '"' << "2" << '"' << ">" << endl;
	int out31 = compaire(fx1, tx1);
	if (tx1[0] != 0){
		if (out31 == 2){
			stepstatic << "<prescribed bc = " << '"' << "x" << '"' << "lc=" << '"' << "25" << '"' << ">1<" << "/prescribed>" << endl;
		}
		else if (out31 != 2){
			stepstatic << "<prescribed bc = " << '"' << "x" << '"' << "lc=" << '"' << "31" << '"' << ">1<" << "/prescribed>" << endl;
		}
	}
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	int out41 = compaire(fy1, ty1);
	if (ty1[0] != 0){
		if (out41 == 2){
			stepstatic << "<prescribed bc = " << '"' << "y" << '"' << "lc=" << '"' << "26" << '"' << ">1<" << "/prescribed>" << endl;
		}
		else if (out41 != 2){
			stepstatic << "<prescribed bc = " << '"' << "y" << '"' << "lc=" << '"' << "32" << '"' << ">1<" << "/prescribed>" << endl;
		}
	}
	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////		
	int out51 = compaire(fz1, tz1);
	if (tz1[0] != 0){
		if (out51 == 2){
			stepstatic << "<prescribed bc = " << '"' << "z" << '"' << "lc=" << '"' << "27" << '"' << ">1<" << "/prescribed>" << endl;
		}
		else if (out51 != 2){
			stepstatic << "<prescribed bc = " << '"' << "z" << '"' << "lc=" << '"' << "33" << '"' << ">1<" << "/prescribed>" << endl;
		}
	}
	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	int out61 = compaire(fox1, tox1);
	if (tox1[0] != 0){
		if (out61 == 2){
			stepstatic << "<prescribed bc = " << '"' << "Rx" << '"' << "lc=" << '"' << "28" << '"' << ">1<" << "/prescribed>" << endl;
		}
		else if (out61 != 2){
			stepstatic << "<prescribed bc = " << '"' << "Rx" << '"' << "lc=" << '"' << "34" << '"' << ">1<" << "/prescribed>" << endl;
		}
	}
	/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	int out71 = compaire(foy1, toy1);
	if (toy1[0] != 0){
		if (out71 == 2){
			stepstatic << "<prescribed bc = " << '"' << "Ry" << '"' << "lc=" << '"' << "29" << '"' << ">1<" << "/prescribed>" << endl;
		}
		else if (out71 != 2){
			stepstatic << "<prescribed bc = " << '"' << "Ry" << '"' << "lc=" << '"' << "35" << '"' << ">1<" << "/prescribed>" << endl;
		}
	}
	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	int out81 = compaire(foz1, toz1);
	if (toz1[0] != 0){
		if (out81 == 2){
			stepstatic << "<prescribed bc = " << '"' << "Rz" << '"' << "lc=" << '"' << "30" << '"' << ">1<" << "/prescribed>" << endl;
		}
		else if (out81 != 2){
			stepstatic << "<prescribed bc = " << '"' << "Rz" << '"' << "lc=" << '"' << "36" << '"' << ">1<" << "/prescribed>" << endl;
		}
	}

	//stepstatic << "</rigid_body>" << endl;
	///////////////////////////////////////////////////////////////////////////////////////////////

	//stepstatic << "<rigid_body mat = " << '"' << "2" << '"' << ">" << endl;

	if (tx== 1){
		stepstatic << "<fixed bc = " << '"' << "x" << '"' << "/>" << endl;
	}
	if (ty == 1){
		stepstatic << "<fixed bc = " << '"' << "y" << '"' << "/>" << endl;;
	}
	if (tz == 1){
		stepstatic << "<fixed bc = " << '"' << "z" << '"' << "/>" << endl;;
	}
	if (tox == 1){
		stepstatic << "<fixed bc = " << '"' << "Rx" << '"' << "/>" << endl;;
	}
	if (toy == 1){
		stepstatic << "<fixed bc = " << '"' << "Ry" << '"' << "/>" << endl;;
	}
	if (toz == 1){
		stepstatic << "<fixed bc = " << '"' << "Rz" << '"' << "/>" << endl;;
	}
	stepstatic << "</rigid_body>" << endl;
	stepstatic << " </Constraints>" << endl;
	stepstatic << "</Step>" << endl;
	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////
 //////////////////DYNAMIC STEP///////////////////
	///////////////////////////////////////////////
	stepdynamic << "<rigid_body mat = " << '"' << "1" << '"' << ">" << endl;
	int out = detect(fx1);
	if (fx==1){
		stepdynamic << "<fixed bc = " << '"' << "x" << '"' << "/>" << endl;
	}
	if (out == 0){
		stepdynamic << "<prescribed bc = " << '"' << "x" << '"' << "lc=" << '"' << "37" << '"' << ">1<" << "/prescribed>" << endl;
	}
	//////////////
	int out2 = detect(fy1);
	if (fy==1){
		stepdynamic << "<fixed bc = " << '"' << "y" << '"' << "/>" << endl;
	}
	if (out2 == 0){
		stepdynamic << "<prescribed bc = " << '"' << "y" << '"' << "lc=" << '"' << "38" << '"' << ">1<" << "/prescribed>" << endl;
	}
	//////////////////
	int out3 = detect(fz1);
	if (fz==1){
		stepdynamic << "<fixed bc = " << '"' << "z" << '"' << "/>" << endl;
	}
	if (out3 == 0){
		stepdynamic << "<prescribed bc = " << '"' << "z" << '"' << "lc=" << '"' << "39" << '"' << ">1<" << "/prescribed>" << endl;
	}
	/////////////////////
	int out4 = detect(fox1);
	if (fox == 1){
		stepdynamic << "<fixed bc = " << '"' << "Rx" << '"' << "/>" << endl;
	}
	if (out4 == 0){
		stepdynamic << "<prescribed bc = " << '"' << "Rx" << '"' << "lc=" << '"' << "40" << '"' << ">1<" << "/prescribed>" << endl;
	}
	///////////////////////
	int out5 = detect(foy1);
	if (foy == 1){
		stepdynamic << "<fixed bc = " << '"' << "Ry" << '"' << "/>" << endl;
	}
	if (out5 == 0){
		stepdynamic << "<prescribed bc = " << '"' << "Ry" << '"' << "lc=" << '"' << "41" << '"' << ">1<" << "/prescribed>" << endl;
	}
	/////////////////////////
	int out6 = detect(foz1);
	if (foz == 1){
		stepdynamic << "<fixed bc = " << '"' << "Rz" << '"' << "/>" << endl;
	}
	if (out6 == 0){
		stepdynamic << "<prescribed bc = " << '"' << "Rz" << '"' << "lc=" << '"' << "42" << '"' << ">1<" << "/prescribed>" << endl;
	}
	stepdynamic << "</rigid_body>" << endl;
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////second body/////////////////////////////////////
	stepdynamic << "<rigid_body mat = " << '"' << "2" << '"' << ">" << endl;


	int out7 = detect(tx1);
	if (tx == 1){
		stepdynamic << "<fixed bc = " << '"' << "x" << '"' << "/>" << endl;
	}
	if (out7 == 0){
		if (out31 == 2){
			stepdynamic << "<prescribed bc = " << '"' << "x" << '"' << "lc=" << '"' << "37" << '"' << ">1<" << "/prescribed>" << endl;
		}
		else if (out31 != 2){
			stepdynamic << "<prescribed bc = " << '"' << "x" << '"' << "lc=" << '"' << "43" << '"' << ">1<" << "/prescribed>" << endl;
		}
	}
	////////////////////
	int out8 = detect(ty1);
	if (ty == 1){
		stepdynamic << "<fixed bc = " << '"' << "y" << '"' << "/>" << endl;
	}
	if (out8 == 0){
		if (out41 == 2){
			stepdynamic << "<prescribed bc = " << '"' << "y" << '"' << "lc=" << '"' << "38" << '"' << ">1<" << "/prescribed>" << endl;
		}
		else if (out41 != 2){
			stepdynamic << "<prescribed bc = " << '"' << "y" << '"' << "lc=" << '"' << "44" << '"' << ">1<" << "/prescribed>" << endl;
		}
	}
	////////////////////
	int out9 = detect(tz1);
	if (tz == 1){
		stepdynamic << "<fixed bc = " << '"' << "z" << '"' << "/>" << endl;
	}
	if (out9 == 0){
		if (out51 == 2){
			stepdynamic << "<prescribed bc = " << '"' << "z" << '"' << "lc=" << '"' << "39" << '"' << ">1<" << "/prescribed>" << endl;
		}
		else if (out51 != 2){
			stepdynamic << "<prescribed bc = " << '"' << "z" << '"' << "lc=" << '"' << "45" << '"' << ">1<" << "/prescribed>" << endl;
		}
	}
	//////////////////////////
	int out10 = detect(tox1);
	if (tox == 1){
		stepdynamic << "<fixed bc = " << '"' << "Rx" << '"' << "/>" << endl;
	}
	if (out10 == 0){
		if (out61 == 2){
			stepdynamic << "<prescribed bc = " << '"' << "Rx" << '"' << "lc=" << '"' << "40" << '"' << ">1<" << "/prescribed>" << endl;
		}
		else if (out61 != 2){
			stepdynamic << "<prescribed bc = " << '"' << "Rx" << '"' << "lc=" << '"' << "46" << '"' << ">1<" << "/prescribed>" << endl;
		}
	}
	////////////////////
	int out11 = detect(toy1);
	if (toy == 1){
		stepdynamic << "<fixed bc = " << '"' << "Ry" << '"' << "/>" << endl;
	}
	if (out11 == 0){
		if (out71 == 2){
			stepdynamic << "<prescribed bc = " << '"' << "Ry" << '"' << "lc=" << '"' << "41" << '"' << ">1<" << "/prescribed>" << endl;
		}
		else if (out71 != 2){
			stepdynamic << "<prescribed bc = " << '"' << "Ry" << '"' << "lc=" << '"' << "47" << '"' << ">1<" << "/prescribed>" << endl;
		}
	}
	/////////////////////////
	int out12 = detect(toz1);
	if (toz == 1){
		stepdynamic << "<fixed bc = " << '"' << "Rz" << '"' << "/>" << endl;
	}
	if (out12 == 0){
		if (out81 == 2){
			stepdynamic << "<prescribed bc = " << '"' << "Rz" << '"' << "lc=" << '"' << "42" << '"' << ">1<" << "/prescribed>" << endl;
		}
		else if (out81 != 2){
			stepdynamic << "<prescribed bc = " << '"' << "Rz" << '"' << "lc=" << '"' << "48" << '"' << ">1<" << "/prescribed>" << endl;
		}
	}
	//////////////////////////
	stepdynamic << "</rigid_body>" << endl;
	stepdynamic << " </Constraints>" << endl;
	stepdynamic << "</Step>" << endl;
	stepdynamic << "</febio_spec>" << endl;



	}


	int BK_structor::detect(Vector fx){
		int count = 0;
		int out = 0;
		int sizer = fx.size();
		for (int o = 1; o < sizer; ++o){ if (fx(o) == 0){ ++count; } }
		if (count == sizer-1){ out = 1; }
		return out;
	}

	int BK_structor::compaire(Vector fx, Vector tx){
		int count = 0;
		int out = 0;
		int sizer = fx.size();
		for (int o = 0; o < sizer; ++o){ if (fx(o) == tx(o)){ ++count; } }
		if (count == sizer){ out = 2; }
		return out;
	}

	
Vector BK_structor::costrain_translation(Vector fx, Vector tx,double accur,char axis){
		int sizer = fx.size();
		// the smaller position is the initial for the z axis 
		if (axis == 'z'){
			Vector difference = abs(fx) - abs(tx);
			for (int o = 0; o < sizer; ++o){
				if (difference[o] < 0){
					fx[o] = tx[o];
					cout << "In the "<< o <<"time step value we detect negative distance in Z axis so set the difference in 0 between the two bodies!" << endl;

				}

				}

			}
	// see if the absolute distance is ok between the two bodies...
			Vector differenc = (fx - tx);
			for (int o = 0; o < sizer; ++o){
				//cout << differenc[o] << endl;
				if (abs(differenc[o]) > accur){
					cout << "constrain max overreach with value: " << differenc[o] << endl;
					cout << "resample the position of first body from: " << fx[o] << endl;
					if (differenc[o]> 0){ fx[o] = fx[o] - differenc[o] + accur; }
					if (differenc[o]< 0){ fx[o] = fx[o] - differenc[o] - accur; }
					cout << "resample the position of first body to: " << fx[o] << endl;

				}

			}

		
		return fx;
		}


		