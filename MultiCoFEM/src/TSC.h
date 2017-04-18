#ifndef TSC_H
#define TSC_H

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
#pragma comment(lib, "shell32.lib")

//#pragma once
class TSC
{

public:
	// ----- CONSTRUCTORS/DESTRUCTORS------------
	TSC(void);
	~TSC(void);

	void run(Vector fpx, Vector fpy, Vector fpz, Vector fox, Vector foy, Vector foz, Vector spx, Vector spy, Vector spz, Vector sox, Vector soy, Vector soz, Vector time);
	void stepswriter();
	HANDLE ShellExecute( string name);
	
private:

	void stepdetermine(int sizer, Vector t, Vector f, int maxtimestep, int mintimestep);

};

#endif