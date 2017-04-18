#ifndef CASE_ONE_H
#define CASE_ONE_H

#include <OpenSim/OpenSim.h>
#include <OpenSim/Analyses/BodyKinematics.h>
#include <OpenSim/Tools/AnalyzeTool.h>

#include <ctime>
#include <exception>
#include "Settings.h"
#include "INIReader.h"
#include <OpenSim/Simulation/SimbodyEngine/Body.h>
#include <exception>
#include "Settings.h"
#include "INIReader.h"
#include "BF_structor.h"
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

using namespace OpenSim;
using namespace SimTK;
using namespace std;


class CASE_ONE{

public:
	CASE_ONE(void);
	~CASE_ONE(void);
	void run(int itteration, string kind[24], int endend, string resultDir2, int pass,char dof);
	Vector timefinal;
	Vector fPx;
	Vector fPy;
	Vector fPz;
	Vector fOx;
	Vector fOy;
	Vector fOz;
	Vector tPx;
	Vector tPy;
	Vector tPz;
	Vector tOx;
	Vector tOy;
	Vector tOz;
	Vector fFx;
	Vector fFy;
	Vector fFz;
	Vector fMx;
	Vector fMy;
	Vector fMz;
	Vector tFx;
	Vector tFy;
	Vector tFz;
	Vector tMx;
	Vector tMy;
	Vector tMz;
	void store(Vector fxo, int which);
	Vector return_vect(int which);
	void Run();

private:
	int datafiles;
	HANDLE ShellExecuteHandler(string program, string args, string name);
	HANDLE Shellfile(string program);
	std::clock_t optimizerStartTime;
	INIReader ini;
	
};
#endif