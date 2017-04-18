




#ifndef FEBRUNNER_H
#define FEBRUNNER_H

#include <OpenSim/OpenSim.h>
#include <vector>
#include <time.h>
#include <ctime>
#include <exception>
#include <iostream>
#include <fstream>
#include <algorithm>
#include <windows.h>
#include <ShellAPI.h>
#include <omp.h>
#include "INIReader.h"
#include "Settings.h"

using namespace OpenSim;
using namespace SimTK;
using namespace std;
#pragma comment(lib, "shell32.lib")

//#pragma once
class FEBRunner
{

public:
	// ----- CONSTRUCTORS/DESTRUCTORS------------
	FEBRunner(void);
	~FEBRunner(void);

		void Run(int in);
		void WriteFEBFile(int itteration, char mode);
		void CopyFEBFile(int number);
		void clearsetup();
		HANDLE ShellExecuteHandler(string program, string args,string name);
private:
	
};

#endif