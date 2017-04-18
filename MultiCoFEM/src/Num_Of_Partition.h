#ifndef  NUM_OF_PARTITION_H
#define  NUM_OF_PARTITION_H

#include <OpenSim/OpenSim.h>
#include <vector>
#include <time.h>
#include <ctime>
#include <exception>
#include <iostream>
#include <fstream>
#include <algorithm>
#include "INIReader.h"
#include "Settings.h"
#include <windows.h>
#include <ShellAPI.h>

using namespace OpenSim;
using namespace SimTK;
using namespace std;


#pragma comment(lib, "shell32.lib")

class  Num_Of_Partition
{

public:
	// ----- CONSTRUCTORS/DESTRUCTORS------------
	Num_Of_Partition(void);
	~Num_Of_Partition(void);

	Vector start(int number);
	void analysiswriter(double start, double end, char mode);
	

private:
	HANDLE ShellExecute(string name);
};

#endif