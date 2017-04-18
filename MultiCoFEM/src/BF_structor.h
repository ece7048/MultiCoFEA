#ifndef BF_STRUCTOR_H
#define BF_STRUCTOR_H

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



using namespace OpenSim;
using namespace SimTK;
using namespace std;

class BF_structor {
public:
	BF_structor(void);
	~BF_structor(void);
	
	void begining(int iteration, string kind[24], int sizer, string resultDir, Vector time, Vector fx, Vector fy, Vector fz, Vector fox, Vector foy, Vector foz, Vector tx, Vector ty, Vector tz, Vector tox, Vector toy, Vector toz);
private:
	int detect(Vector fx);
	int compaire(Vector fx, Vector tx);
	int datafiles;
	ofstream valuenon;
	ofstream valueinn;
	std::clock_t optimizerStartTime;

};
#endif