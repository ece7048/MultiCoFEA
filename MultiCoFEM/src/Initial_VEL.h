#ifndef  INITIAL_VEL_H
#define  INITIAL_VEL_H

#include <OpenSim/OpenSim.h>
#include <vector>
#include <time.h>
#include <ctime>
#include <exception>
#include <iostream>
#include <fstream>
#include <algorithm>
#include "INIReader.h"
#include "Settings.h"
#include <windows.h>
#include <ShellAPI.h>

using namespace OpenSim;
using namespace SimTK;
using namespace std;


//#pragma comment(lib, "shell32.lib")

class  Initial_VEL
{
	

public:
	// ----- CONSTRUCTORS/DESTRUCTORS------------
	Initial_VEL(void);
	~Initial_VEL(void);


	Vector time1;
	Vector fx1;
	Vector fy1;
	Vector fz1;
	Vector fox1;
	Vector foy1;
	Vector foz1;
	Vector tx1;
	Vector ty1;
	Vector tz1;
	Vector tox1;
	Vector toy1;
	Vector toz1;


	void store(Vector fxo, int which);
	Vector return_vect(int which);


	//public functions
	///TXT file writer////
	void initial(int itteration, char kase,int behavior,char dof);

	void write(int iteration, string resultDir1, Vector fxf, Vector fyf, Vector fzf, Vector foxf, Vector foyf, Vector fozf, Vector txf, Vector tyf, Vector tzf, Vector toxf, Vector toyf, Vector tozf);
private:
	
};

#endif