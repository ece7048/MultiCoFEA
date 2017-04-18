#ifndef BK_STRUCTOR_H
#define BK_STRUCTOR_H

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

class BK_structor {
public:
	BK_structor(void);
	~BK_structor(void);
	void begining(int iteration,string kind[24], int place, int sizer, string resultDir, Vector time, Vector fx, Vector fy, Vector fz, Vector fox, Vector foy, Vector foz, Vector tx, Vector ty, Vector tz, Vector tox, Vector toy, Vector toz);
private:
	int compaire(Vector fx, Vector tx);
	int detect(Vector fx);
	Vector costrain_translation(Vector fx, Vector tx, double accur, char axis);
	int datafiles;
	
	std::clock_t optimizerStartTime;

};
#endif
