#ifndef JR_STARTER_H
#define JR_STARTER_H

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
#include <OpenSim/Simulation/SimbodyEngine/Body.h>
using namespace OpenSim;
using namespace SimTK;
using namespace std;



class JR_starter
{

public:
	JR_starter(void);
	~JR_starter(void);
	void started(int number, int itteration);
	void Run();



private:
	int datafiles;
	HANDLE ShellExecuteHandler(string program, string args, string name);
	HANDLE Shellfile(string program);
	std::clock_t optimizerStartTime;
	INIReader ini;




};
#endif