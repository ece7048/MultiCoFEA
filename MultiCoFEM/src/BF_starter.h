#ifndef BF_STARTER_H
#define BF_STARTER_H

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



class BF_starter 
{

public:
	BF_starter(void);
	~BF_starter(void);
	void started(int number, int itteration,char mode);
	
	


private:
	int datafiles;
	
	std::clock_t optimizerStartTime;
	INIReader ini;

	


};
#endif