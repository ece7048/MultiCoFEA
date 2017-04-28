#include <iostream>
#include <fstream>
#include <OpenSim/OpenSim.h>
#include <OpenSim/Simulation/Model/Actuator.h>
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/Model/BodySet.h>
#include "INIReader.h"
#include "Settings.h"
#include "StaticAnalysis.h"
#include "OpenSim/Simulation/Model/PointForceDirection.h"
#include "OpenSim/Simulation/Model/Force.h"
#include "simbody/internal/Force.h"
#include "Simbody.h"
using namespace OpenSim;
using namespace SimTK;
using namespace std;
/**
*
* @author Michail Mamalakis
*/

/*
BodyForceAnalysis::BodyForceAnalysis(Model* model)
: Analysis(model)
{
constructDescription();
constructColumnLabels();
setupStorage();
}

*/
void StaticAnalysis::setModel(Model& aModel)
{
	Super::setModel(aModel);

	constructDescription();
	constructColumnLabels();
	setupStorage();
}

int StaticAnalysis::begin(SimTK::State& s)
{
	if (!proceed()) return(0);

	storage.reset(s.getTime());

	int status = 0;
	if (storage.getSize() <= 0)
	{
		status = record(s);
	}

	return(status);
}

int StaticAnalysis::step(const SimTK::State& s, int stepNumber)
{
	if (!proceed(stepNumber)) return(0);

	record(s);

	return(0);
}

int StaticAnalysis::end(SimTK::State& s)
{
	if (!proceed()) return(0);

	record(s);

	return(0);
}
int StaticAnalysis::print(const std::string &path)
{
	storage.print(path);

	return(0);
}

int StaticAnalysis::printResults(
	const std::string &aBaseName, const std::string &aDir = "",
	double aDT = -1.0, const std::string &aExtension = ".sto")
{
	Storage::printResult(&storage, aBaseName + "_" + getName(), aDir, aDT, aExtension);

	return(0);
}

/*
Storage & getStorage()
{
Storage storage;

return storage;


}
*/

int StaticAnalysis::record(const SimTK::State& s)
{
	///////////////////////////////COMPUTE FORCES AND STORE THEM///////////////

	INIReader ini = INIReader(INI_FILE);
	string bd1 = ini.Get("BODYFORCES", "BDNAME1", "");
	string bd2 = ini.Get("BODYFORCES", "BDNAME2", "");
	double t0 = ini.GetReal("BODYFORCES", "START_TIME", 0);
	string resultDir = BASE_DIR + ini.Get("PATH", "RESULT_DIR", "");

	

	// get the number of bodies in the OpenSim model
	Body &ground = _model->getSimbodyEngine().getGroundBody();
	int nb = _model->getNumBodies();
	Vector data(9 * nb, 1);

	const BodySet &bodies = _model->getBodySet(); //Frame in body
	// find the forces in each body
	Vector_<Vec3>  pointsVec(2), pointsVec2(2), angleVec(2);
	int c = 0;
	double time = s.getTime();
	cout << "Static Position_Analysis time: " << time << endl;
	cout << "press enter..." << endl;

	
	fstream point2bs (resultDir + "/_Staticpoint" + "/staticpoint.txt", std::ios_base::in);
	double a1, b1, c1;
	if (point2bs.is_open()){


		point2bs >> a1 >> b1 >> c1;

		//printf("%f\t%f\t%f\t%f\t%f\t%f\%f\t%f\t%f\t%f\t%f\t%f\n", a, b, c, d, e, f, g, h, k, l, m, n);

		getchar();
	}

	Vec3 point1(0,0,0);

	point1[0] = a1;// d[0];
	point1[1] = b1;// d[1];
	point1[2] = c1;// d[2];
	//cout << "press enter.." << endl;
	//cout << point1 << endl;


	for (int i = 0; i < nb; ++i){

		///////////
		// reset the 4 vectors
		Vec3 force(0, 0, 0);
		Vec3 moment(0, 0, 0);
		Vec3 pointofapplication;
		Vec3 pointofapplication2;
		Vec3 pointofapplicationvel;
		Vec3 angveloc;
		Vec3 angVec(0, 0, 0);
		Vec3 com(0, 0, 0);
		Vec3 point(0, 0, 0);
		Vec3 base(0, 0, 0);
		//Vector_<SpatialVec> bodyForces;
		double  dirCos[3][3];



		Body& body = bodies[i];
		//Joint& joint = jointset[i];




		// attempt one // 
		body.getMassCenter(com);

		//TODO THE COM GAME...
		//point1= com; 

		/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		//////////////////////////////////////////////////////
		//////////////////////////////THE TWO BODIES OF INTERESTING/////////////////
		//////////////////////////////////////////////////////////////////	
		//Attempt two : POINT IN TWO BODIES
		if (bodies[i].getName() == bd1){


			Vec3 com1;
			if (bodies[i + 1].getName() == bd2){
				//TODO may not need the com set and get because the value remane the same...
				bodies[i + 1].getMassCenter(com1);
				bodies[i + 1].setMassCenter(point1);
				_model->updBodySet();
				bodies[i + 1].getMassCenter(point1);
				_model->getSimbodyEngine().transformPosition(s, bodies[i + 1], point1, ground, base);//take the global point and set it in bodies

				_model->getSimbodyEngine().transformPosition(s, ground, base, bodies[i], point);
				_model->getSimbodyEngine().getPosition(s, bodies[i], point, pointofapplication2);
				pointofapplication = point;
				_model->getSimbodyEngine().getDirectionCosines(s, bodies[i], dirCos);
				_model->getSimbodyEngine().convertDirectionCosinesToAngles(dirCos,
					&angVec[0], &angVec[1], &angVec[2]);

				bodies[i + 1].setMassCenter(com1);
				_model->updBodySet();

				pointsVec[0] = pointofapplication;/// so we will have as center of mass the 0,0,0 in both sys FEBio and OpenSim
				pointsVec2[0] = pointofapplication2;/// so we will have as center of mass the 0,0,0 in both sys FEBio and OpenSim

				angleVec[0] = angVec;
			}
		}
				if (bodies[i].getName() == bd2){
					
					body.setMassCenter(point1);
					_model->updBodySet();
					body.getMassCenter(point1);
					_model->getSimbodyEngine().getPosition(s, bodies[i], point1, pointofapplication2);
					pointofapplication = point1;

					_model->getSimbodyEngine().getDirectionCosines(s, bodies[i], dirCos);
					_model->getSimbodyEngine().convertDirectionCosinesToAngles(dirCos,
						&angVec[0], &angVec[1], &angVec[2]);
					body.setMassCenter(com);
					_model->updBodySet();
					pointsVec[1] = pointofapplication;/// so we will have as center of mass the 0,0,0 in both sys FEBio and OpenSim
					pointsVec2[1] = pointofapplication2;/// so we will have as center of mass the 0,0,0 in both sys FEBio and OpenSim

					angleVec[1] = angVec;
					i = nb + 1;
					
				}

				/*
				else if (bodies[i].getName() != bd1 && bodies[i].getName() != bd2){
					//If the bodie is not one of the two choosen...
					_model->getSimbodyEngine().getPosition(s, bodies[i], com, pointofapplication); // define the position of applying the forces


					_model->getSimbodyEngine().getDirectionCosines(s, bodies[i], dirCos);
					_model->getSimbodyEngine().convertDirectionCosinesToAngles(dirCos,
						&angVec[0], &angVec[1], &angVec[2]);

				}
				*/
				/////////////////////////////////////END/////////////////////////////////////////////////////
				//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


				


			}
		
	
	/* fill the construction array*/
	for (int i = 0; i<2; i++) {
		int I = 9 * i;
		for (int j = 0; j<3; j++) {

			data[I + j ] = pointsVec[i][j];
			data[I + j + 3] = pointsVec2[i][j];
			data[I + j + 6] = angleVec[i][j];


		}
	}
	//cout << data << endl;
	//cout << s.getTime() << endl;
	//cout << data.size()<< endl;
	//cout << &data[0] << endl;
	/* Write the data storage*/
	storage.append(s.getTime(), data.size(), &data[0]);





	return(0);
}


void StaticAnalysis::constructDescription()
{
	string descrip;

	descrip = "\nThis file contains the record of bodies position of the model in static position\n";
	descrip += "\nUnits are S.I. units (seconds, meters, Newtons, ...)";
	if (getInDegrees())
	{
		descrip += "\the forces are in Newton.";
	}
	else
	{
		descrip += "\n the Torques are in NewtonMeter.";
	}
	descrip += "\n\n";

	setDescription(descrip);
}

void StaticAnalysis::constructColumnLabels()
{
	INIReader ini = INIReader(INI_FILE);
	string bd1 = ini.Get("BODYFORCES", "BDNAME1", "");
	string bd2 = ini.Get("BODYFORCES", "BDNAME2", "");

	if (_model == NULL) return;

	Array<string> labels;
	labels.append("time");

	const BodySet& bodySet = _model->getBodySet();
	int nmb = bodySet.getSize();
	//int nmb = _model->getMatterSubsystem().getNumBodies();


	for (int i = 0; i<nmb; i++) {
		Body& body = bodySet[i];
		//bodySet[i].getName bodyName;
		//std::string  bodyName =]

		if (bodySet[i].getName() == bd1 || bodySet[i].getName() == bd2){
			//std::string labelRoot = bodyName;
			labels.append(body.getName() + "_px");
			labels.append(body.getName() + "_py");
			labels.append(body.getName() + "_pz");
			labels.append(body.getName() + "_pgx");
			labels.append(body.getName() + "_pgy");
			labels.append(body.getName() + "_pgz");
			labels.append(body.getName() + "_ox");
			labels.append(body.getName() + "_oy");
			labels.append(body.getName() + "_oz");
		}
	}

	setColumnLabels(labels);


}


void StaticAnalysis::setupStorage()
{
	storage.reset(0);
	storage.setDescription(getDescription());
	storage.setColumnLabels(getColumnLabels());
	setName("StaticAnalysis");
}


