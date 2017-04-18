#ifndef VISUALIZERFEBGEO_H
#define VISUALIZERFEBGEO_H

#include <OpenSim/OpenSim.h>
#include <OpenSim/Analyses/BodyKinematics.h>
#include <OpenSim/Tools/AnalyzeTool.h>

#include <ctime>
#include <exception>
#include "Settings.h"
#include "INIReader.h"
#include "BF_structor.h"
#include "BodyForceAnalysis.h"
#include "ReSampler.h"
#include <iostream>
#include <fstream>
#include <algorithm>
#include <ShellAPI.h>
#include <OpenSim/Simulation/SimbodyEngine/Body.h>
using namespace OpenSim;
using namespace SimTK;
using namespace std;



class Visualizerfebgeo
{

public:
	Visualizerfebgeo(void);
	~Visualizerfebgeo(void);
	void XMLwrite();
	//void Run();
	//void XMLwritestate(Vector time, Vector fx, Vector fy, Vector fz, Vector fox, Vector foy, Vector foz, Vector tx, Vector ty, Vector tz, Vector tox, Vector toy, Vector toz);



private:

	INIReader ini;

	//HANDLE ShellExecuteHandler(string program, string args, string name);
	//HANDLE Shellfile(string program);


};
#endif