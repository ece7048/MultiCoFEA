#ifndef RESAMPLER_H
#define RESAMPLER_H

#include <OpenSim/OpenSim.h>
#include <vector>
#include <time.h>
#include <ctime>
#include <exception>
#include <iostream>
#include <fstream>
#include <algorithm>
#include <windows.h>
#include "INIReader.h"
#include "Settings.h"
//#include "BK_structor.h"
//#include "BF_starter.h"


using namespace OpenSim;
using namespace SimTK;
using namespace std;


#pragma once
class ReSampler 
{
	
public:
	// ----- CONSTRUCTORS/DESTRUCTORS------------
	ReSampler(void);
	~ReSampler(void);

	///STORE////////
	void store(Vector fx, int which);
	Vector return_vect(int which);

	///HANDLER////////////
	Vector runvec(int itter, string resultDir, Vector time);
	String interpolationkind(Vector fx, Vector time);
	Vector detector(int caser, int sizer, Vector time, Vector fx);
	Vector DoFbuilder(Vector fx,  Vector timenew);

	/////global vector////
	Vector time;
    Vector Gfx;
	Vector Gfy;
	Vector Gfz;
	Vector Gmx;
	Vector Gmy;
	Vector Gmz;
	Vector Gf2x;
	Vector Gf2y;
	Vector Gf2z;
	Vector Gm2x;
	Vector Gm2y;
	Vector Gm2z;
	Vector Gpx;
	Vector Gpy;
	Vector Gpz;
	Vector Gox;
	Vector Goy;
	Vector Goz;
	Vector Gp2x;
	Vector Gp2y;
	Vector Gp2z;
	Vector Go2x;
	Vector Go2y;
	Vector Go2z;

private:
	Vector timebuilder(Vector timeupd, Vector time);
	
};

#endif