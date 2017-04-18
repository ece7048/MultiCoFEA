#ifndef DOFRESAMPLE_H
#define DOFRESAMPLE_H

#include <OpenSim/OpenSim.h>
#include <OpenSim/Analyses/BodyKinematics.h>
#include <OpenSim/Tools/AnalyzeTool.h>

#include <ctime>
#include <exception>
#include "Settings.h"
#include "INIReader.h"
#include "BF_structor.h"
//#include "BodyForceAnalysis.h"
#include "BK_structor.h"
#include <iostream>
#include <fstream>
#include <algorithm>
#include <OpenSim/Simulation/SimbodyEngine/Body.h>

using namespace OpenSim;
using namespace SimTK;
using namespace std;


class DOFResample{

public:
	DOFResample(void);
	~DOFResample(void);
	void Run();
	void detect(int itter, string kind[24], int sizer1, string resultDir1, int behavior);
	void _JR_resampler();
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

	Vector fx;
	Vector fy;
	Vector fz;
	Vector fox;
	Vector foy;
	Vector foz;
	Vector tx;
	Vector ty;
	Vector tz;
	Vector tox;
	Vector toy;
	Vector toz;

	

	char re;
	void resamplernow(char x);
	char return_resamplernow();
	void store(Vector fxo, int which);
	Vector return_vect(int which);

	
private:
	Mat33 Tait_Bryan(double x, double y, double z);
	Mat66 plucker(Mat33 E, Mat31 r);
	Mat33 cros(Mat31 a);
	HANDLE ShellExecuteHandler(string program, string args, string name);
	HANDLE Shellfile(string program);
};
#endif