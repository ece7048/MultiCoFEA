

#ifndef OPENSIMRUNNER_H
#define OPENSIMRUNNER_H

#include <OpenSim/OpenSim.h>
#include <OpenSim/Tools/InverseKinematicsTool.h>
#include <OpenSim/Tools/AnalyzeTool.h>
#include <ctime>
#include <exception>
#include "Settings.h"
#include "INIReader.h"
#include <iostream>
#include <fstream>
#include <algorithm>

using namespace OpenSim;
using namespace SimTK;
using namespace std;


class OpenSimRunner{

public:

	OpenSimRunner(){};
	~OpenSimRunner(){};
	void runIK();
	void runRRA();
	void runCMC();
	void runST();

private:
	double detecter_time(int kind, string motionfile);
	double t0;
	double tf;

};
#endif