#include <iostream>

#include <OpenSim/OpenSim.h>
#include <OpenSim/Simulation/Model/Actuator.h>
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/Model/BodySet.h>
#include "INIReader.h"
#include "Settings.h"
#include "BodyForceAnalysis2.h"
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
void BodyForceAnalysis2::setModel(Model& aModel)
{
	Super::setModel(aModel);

	constructDescription();
	constructColumnLabels();
	setupStorage();
}

int BodyForceAnalysis2::begin(SimTK::State& s)
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

int BodyForceAnalysis2::step(const SimTK::State& s, int stepNumber)
{
	if (!proceed(stepNumber)) return(0);

	record(s);

	return(0);
}

int BodyForceAnalysis2::end(SimTK::State& s)
{
	if (!proceed()) return(0);

	record(s);

	return(0);
}
int BodyForceAnalysis2::print(const std::string &path)
{
	storage.print(path);

	return(0);
}

int BodyForceAnalysis2::printResults(
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

int BodyForceAnalysis2::record(const SimTK::State& s)
{
	///////////////////////////////COMPUTE FORCES AND STORE THEM///////////////

	INIReader ini = INIReader(INI_FILE);
	string bd1 = ini.Get("BODYFORCES", "BDNAME1", "");
	string bd2 = ini.Get("BODYFORCES", "BDNAME2", "");
	double t0 = ini.GetReal("BODYFORCES", "START_TIME", 0);
	int nmb = _model->getMatterSubsystem().getNumBodies();
	Vector data(18 * nmb, 1);
	Vec3 temp;

	
	_model->getMultibodySystem().realize(s, SimTK::Stage::Position);

	//CALCULATE ALL THE BODY FORCES////
	//BASED    M udot + tau + f_inertial = f_applied  //////where tau=0 becaused is the prescribed forces here is zero//////////
	//////////////////////////////////////////////////////////////////////////////////////////////////////////////


/////////////////////////////////////////////////////
///////////////////////////////f_inertial///////////////////////
	//////////////////////Coriolis////////////////
	//     f_residual = M udot + f_inertial - f_applied(:is fmobilizer+fbody)
	///////////////////////////////////////////////
	_model->getMultibodySystem().realize(s, SimTK::Stage::Velocity);
	 Vector forcescoriolis;
	 _model->getMatterSubsystem().calcResidualForceIgnoringConstraints(s, SimTK::Vector(0), SimTK::Vector_<SimTK::SpatialVec>(0), SimTK::Vector(0), forcescoriolis);
		cout << " Compute Coriolis FORCES" << endl;
		Vector_< SpatialVec >  	FC;

		_model->getMatterSubsystem().multiplyBySystemJacobian(s,forcescoriolis, FC);

		//cout << FC << endl;


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	
////////////TAKE TOTAL APPLIED FORCES IN BODIES///////////////////////////////
///////////////////////////f_applied_body/////////////////////////
		/////////////////////////////////////////////////////
	_model->getMultibodySystem().realize(s, SimTK::Stage::Dynamics);
	Vector_<SpatialVec> forces;
	forces=_model->getMultibodySystem().getRigidBodyForces(s, Stage::Dynamics);
	cout << "APPLIED" << endl;
	//cout << forces << endl;
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	//ADD the INERTIA forces only in force:
	forces = forces + FC;
	//cout << "F_FC" << endl;
	//cout << forces << endl;
////////continiou/////////////


	///////////////////////////M_udot///////////////////////////////////////////
	// get the inertial forces  of  bodies //////////////////
	///////////////////////////////////////////////////////////


_model->getMultibodySystem().realize(s, SimTK::Stage::Acceleration);
	//set the bodies frames references
	const JointSet & jointset = _model->getJointSet();
	Body &ground = _model->getSimbodyEngine().getGroundBody();
	//BodySet& bs = _model->updBodySet();
	const BodySet &bodies = _model->getBodySet(); //Frame in bod
	
	Vector  MassAccel(6*nmb,1);
	Vector Forces(6 * nmb, 1);
	
	for (int i = 0; i < nmb; ++i){
	Vec3 pointofapplicationaccel(0, 0, 0);
	Vec3 angaccel(0, 0, 0);
	Vec3 comm(0, 0, 0);
	//Vec3 base(0, 0, 0);
	double mass1 = 0;
	Mat33 inertia1(0, 0, 0, 0, 0, 0, 0, 0, 0);
		bodies[i].getMassCenter(comm);

	//_model->getSimbodyEngine().transformPosition(s, bodies[i], comm, ground, base);//take the global point and set it in bodies
	
	_model->getSimbodyEngine().getAcceleration(s, bodies[i], comm, pointofapplicationaccel);
	_model->getSimbodyEngine().getAngularAcceleration(s, bodies[i], angaccel);
	bodies[i].getInertia(inertia1);
	mass1 = bodies[i].getMass();


		// Debug///	
		//	cout << mass1 << endl;
		//	cout << inertia1 << endl;
		//	cout << pointofapplicationaccel << endl;

	Vec3 iner = inertia1*angaccel;
	Vec3 forc(mass1* pointofapplicationaccel[0], mass1* pointofapplicationaccel[1], mass1* pointofapplicationaccel[2]);
		
	
	MassAccel[6*i] = iner[0];
	MassAccel[6 * i + 1] = iner[1];
	MassAccel[6 * i + 2] = iner[2];
	MassAccel[6 * i + 3] = forc[0];
	MassAccel[6 * i + 4] = forc[1];
	MassAccel[6 * i + 5] = forc[2];

	Forces[6 * i] = forces[i][0][0];
	Forces[6 * i + 1] = forces[i][0][1];
	Forces[6 * i + 2] = forces[i][0][2];
	Forces[6 * i + 3] = forces[i][1][0];
	Forces[6 * i + 4] = forces[i][1][1];
	Forces[6 * i + 5] = forces[i][1][2];
		
	}
		
	cout << "compute M_udot" << endl;
	//cout << MassAccel << endl;

	//cout << Mass1 << endl;
	//cout << MassAccel - forces << endl;
	
	/////////////////////////////////////////////////////////////////////////////////////////////////////////


	//CASE FOR QUASISTATIC FEBIO/////////////////////////////////////////

	/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	//continiou....///////
	//ADD the INERTIA forces only in force:
	Forces = Forces + MassAccel;
	cout << "add the Inertia Forces in Fbody" << endl;
	//cout << Forces << endl;


	///////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////


	// get the number of bodies in the OpenSim model

	int nb = _model->getNumBodies();
	int nf = forces.size();

nb = nmb;
	Vector_<Vec3> forcesVec(nb), momentsVec(nb), pointsVec(nb), angleVec(nb), angleVelocity(nb), Velocity(nb);
	

	//////////////////////Initialize the point of interest//////////////////////////////////////////////


	string resultDir = BASE_DIR + ini.Get("PATH", "RESULT_DIR", "");
	string pathst = resultDir + "/_StaticAnalysis0";
	Storage  pointtt(pathst + "/_StaticAnalysis.sto");

	Array<double> tim;

	Array<double> pppz;
	Array<double> ppx;
	Array<double> pppx;
	Array<double> ppy;
	Array<double> pppy;
	Array<double> ppz;

	Array<double> pppgz;
	Array<double> ppgx;
	Array<double> pppgx;
	Array<double> ppgy;
	Array<double> pppgy;
	Array<double> ppgz;

	Array<double> ooz;
	Array<double> ooox;
	Array<double> oox;
	Array<double> oooy;
	Array<double> ooy;
	Array<double> oooz;

	pointtt.getTimeColumn(tim);
	pointtt.getDataColumn(bd1 + "_ox", oox);
	pointtt.getDataColumn(bd2 + "_ox", ooox);
	pointtt.getDataColumn(bd1 + "_oy", ooy);
	pointtt.getDataColumn(bd2 + "_oy", oooy);
	pointtt.getDataColumn(bd1 + "_oz", ooz);
	pointtt.getDataColumn(bd2 + "_oz", oooz);


	pointtt.getDataColumn(bd1 + "_px", ppx);
	pointtt.getDataColumn(bd2 + "_px", pppx);
	pointtt.getDataColumn(bd1 + "_py", ppy);
	pointtt.getDataColumn(bd2 + "_py", pppy);
	pointtt.getDataColumn(bd1 + "_pz", ppz);
	pointtt.getDataColumn(bd2 + "_pz", pppz);

	pointtt.getDataColumn(bd1 + "_pgx", ppgx);
	pointtt.getDataColumn(bd2 + "_pgx", pppgx);
	pointtt.getDataColumn(bd1 + "_pgy", ppgy);
	pointtt.getDataColumn(bd2 + "_pgy", pppgy);
	pointtt.getDataColumn(bd1 + "_pgz", ppgz);
	pointtt.getDataColumn(bd2 + "_pgz", pppgz);

	Vec3 point1;
	Vec3 point2;
	Vec3 pointofapplication2;
	Vec3 pointofapplication21;

	point1[0] = pppx[0];
	point1[1] = pppy[0];
	point1[2] = pppz[0];

	point2[0] = ppx[0];
	point2[1] = ppy[0];
	point2[2] = ppz[0];


	pointofapplication21[0] = pppgx[0];
	pointofapplication21[1] = pppgy[0];
	pointofapplication21[2] = pppgz[0];

	pointofapplication2[0] = ppgx[0];
	pointofapplication2[1] = ppgy[0];
	pointofapplication2[2] = ppgz[0];

	Vec3 or1;
	Vec3 or2;
	or1[0] = ooox[1];
	or1[1] = oooy[1];
	or1[2] = oooz[1];

	or2[0] = oox[1];
	or2[1] = ooy[1];
	or2[2] = ooz[1];

	/*
	cout << tim << endl;
	cout << ppx[0] << endl;
	cout << ppx << endl;
	cout << ooox << endl;
	cout << oox << endl;
	cout << ppgx << endl;
	cout << pppgx << endl;


	cout << point2 << endl;
	cout << point1 << endl;
	cout << or2 << endl;
	cout << or1 << endl;
	cout << pointofapplication2 << endl;
	cout << pointofapplication21 << endl;
	*/
	////////////////////////////////////////////////////////////////////

	string kind2;
	string kind21 = "y";
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//////////////POINT FORCE COMPUTE///////////////////////////////////////////
	for (int i = 0; i < nb; ++i){

		///////////
		// reset the 4 vectors
		Vec3 force(0, 0, 0);
		Vec3 moment(0, 0, 0);
		Vec3 pointofapplication;
		Vec3 pointofapplicationvel;
		Vec3 angveloc;
		Vec3 angVec(0, 0, 0);
		Vec3 com(0, 0, 0);
		Vec3 point(0, 0, 0);
		Vec3 base(0, 0, 0);
		//Vector_<SpatialVec> bodyForces;
		double  dirCos[3][3];

		//compute the forces moment
		for (int j = 0; j < 3; ++j){
			moment = Forces[6*i+j];
			force = Forces[6*i+j+3];
		}
		//take body frame
		_model->getSimbodyEngine().transform(s, ground, force, bodies[i], force);
		_model->getSimbodyEngine().transform(s, ground, moment, bodies[i], moment);

		Body& body = bodies[i];
		//Joint& joint = jointset[i];




		// attempt one // 
		body.getMassCenter(com);


		/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		//////////////////////////////////////////////////////
		//////////////////////////////THE TWO BODIES OF INTERESTING/////////////////
		//////////////////////////////////////////////////////////////////	
		//Attempt two : POINT IN TWO BODIES
		if (bodies[i].getName() == bd1){


			Vec3 com1;

			//TODO may not need the com set and get because the value remane the same...
			bodies[i].getMassCenter(com1);
			bodies[i].setMassCenter(point2);
			_model->updBodySet();
			bodies[i].getMassCenter(point2);
			_model->getSimbodyEngine().transformPosition(s, bodies[i], point2, ground, base);//take the global point and set it in bodies

			_model->getSimbodyEngine().transformPosition(s, ground, base, bodies[i], point);

			_model->getSimbodyEngine().getPosition(s, bodies[i], point, pointofapplication);

			_model->getSimbodyEngine().getVelocity(s, bodies[i], point, pointofapplicationvel);
			_model->getSimbodyEngine().getAngularVelocity(s, bodies[i], angveloc);

			_model->getSimbodyEngine().getDirectionCosines(s, bodies[i], dirCos);
			_model->getSimbodyEngine().convertDirectionCosinesToAngles(dirCos,
				&angVec[0], &angVec[1], &angVec[2]);
			bodies[i + 1].setMassCenter(com1);
			_model->updBodySet();

			//_model->getSimbodyEngine().getPosition(s, bodies[i], point2, pointofapplication2);



			//second implementation based the MOMENTS: use the momentnew=momentold+ force x vectorpositiondistance so:
			Vec3 pointactingforce(0, 0, 0);
			Vec3 comp1;
			//cout << "" << endl;
			//cout << "Initial Moment of " << bodies[i] << " vector is: " << moment << endl;
			_model->getSimbodyEngine().getPosition(s, bodies[i], com, comp1);
			pointactingforce[0] = pointofapplication[0] - comp1[0];
			pointactingforce[1] = pointofapplication[1] - comp1[1];
			pointactingforce[2] = pointofapplication[2] - comp1[2];

			_model->getSimbodyEngine().transform(s, ground, pointactingforce, bodies[i], pointactingforce);


			//cout << "pointx2: " << pointx1 << endl;
			//cout << "pointy2: " << pointy1 << endl;
			//cout << "pointz2: " << pointz1 << endl;

			moment[0] = moment[0] + (-force[1] * pointactingforce[2] + force[2] * pointactingforce[1]);
			moment[1] = moment[1] + (-force[2] * pointactingforce[0] + force[0] * pointactingforce[2]);
			moment[2] = moment[2] + (-force[0] * pointactingforce[1] + force[1] * pointactingforce[0]);
			
			//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
			//cout << "After the Paraller Theoriem the Moment of " << bodies[i] << " vector is: " << moment << endl;
			//cout << "" << endl;

			//_model->getSimbodyEngine().transform(s, bodies[i], force, bodies[i + 1], force);
			//_model->getSimbodyEngine().transform(s, bodies[i], moment, bodies[i + 1], moment);
			pointsVec[i] = pointofapplication - pointofapplication2;//-point2;/// so we will have as center of mass the 0,0,0 in both sys FEBio and OpenSim
			angleVec[i] = angVec - or2;


		}

		if (bodies[i].getName() == bd2){




			body.setMassCenter(point1);
			_model->updBodySet();
			body.getMassCenter(point1);
			_model->getSimbodyEngine().transformPosition(s, bodies[i], point1, ground, base);//take the global point and set it in bodies

			_model->getSimbodyEngine().transformPosition(s, ground, base, bodies[i], point);
			_model->getSimbodyEngine().getPosition(s, bodies[i], point, pointofapplication);

			//_model->getSimbodyEngine().getPosition(s, bodies[i], point1, pointofapplication2);

			_model->getSimbodyEngine().getVelocity(s, bodies[i], point, pointofapplicationvel);
			_model->getSimbodyEngine().getAngularVelocity(s, bodies[i], angveloc);

			_model->getSimbodyEngine().getDirectionCosines(s, bodies[i], dirCos);
			_model->getSimbodyEngine().convertDirectionCosinesToAngles(dirCos,
				&angVec[0], &angVec[1], &angVec[2]);
			body.setMassCenter(com);
			_model->updBodySet();

			//second implementation based the MOMENTS: use the momentnew=momentold+ force x vectorpositiondistance so:
			Vec3 pointactingforce(0, 0, 0);
			Vec3 comp1;
			//cout << "" << endl;
			//cout << "Initial Moment of " << bodies[i] << " vector is: " << moment << endl;
			_model->getSimbodyEngine().getPosition(s, bodies[i], com, comp1);
			pointactingforce[0] = pointofapplication[0] - comp1[0];
			pointactingforce[1] = pointofapplication[1] - comp1[1];
			pointactingforce[2] = pointofapplication[2] - comp1[2];

			_model->getSimbodyEngine().transform(s, ground, pointactingforce, bodies[i], pointactingforce);


			//cout << "pointx2: " << pointx1 << endl;
			//cout << "pointy2: " << pointy1 << endl;
			//cout << "pointz2: " << pointz1 << endl;

			moment[0] = moment[0] + (-force[1] * pointactingforce[2] + force[2] * pointactingforce[1]);
			moment[1] = moment[1] + (-force[2] * pointactingforce[0] + force[0] * pointactingforce[2]);
			moment[2] = moment[2] + (-force[0] * pointactingforce[1] + force[1] * pointactingforce[0]);
			
			//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
			//cout << "After the Paraller Theoriem the Moment of " << bodies[i] << " vector is: " << moment << endl;
			//cout << "" << endl;
			

			pointsVec[i] = pointofapplication - pointofapplication21;//-point2;/// so we will have as center of mass the 0,0,0 in both sys FEBio and OpenSim
			angleVec[i] = angVec - or1;
		}

		else if (bodies[i].getName() != bd1 && bodies[i].getName() != bd2){
			//If the bodie is not one of the two choosen...
			_model->getSimbodyEngine().getPosition(s, bodies[i], com, pointofapplication); // define the position of applying the forces

			_model->getSimbodyEngine().getVelocity(s, bodies[i], com, pointofapplicationvel);
			_model->getSimbodyEngine().getAngularVelocity(s, bodies[i], angveloc);

			_model->getSimbodyEngine().getDirectionCosines(s, bodies[i], dirCos);
			_model->getSimbodyEngine().convertDirectionCosinesToAngles(dirCos,
				&angVec[0], &angVec[1], &angVec[2]);

			pointsVec[i] = pointofapplication;/// so we will have as center of mass the 0,0,0 in both sys FEBio and OpenSim
			angleVec[i] = angVec;
		}
		/////////////////////////////////////END/////////////////////////////////////////////////////
		//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

		/////////////////////////////////////END/////////////////////////////////////////////////////
		//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		
		forcesVec[i] = -force;
		momentsVec[i] = -moment;

		_model->getSimbodyEngine().transform(s, ground, pointsVec[i], bodies[i], pointsVec[i]);
		_model->getSimbodyEngine().transform(s, ground, angleVec[i], bodies[i], angleVec[i]);


		angleVelocity[i] = angveloc;
		Velocity[i] = pointofapplicationvel;

		_model->getSimbodyEngine().transform(s, ground, angleVelocity[i], bodies[i], angleVelocity[i]);
		_model->getSimbodyEngine().transform(s, ground, Velocity[i], bodies[i], Velocity[i]);



	}
	/* fill the construction array*/
	for (int i = 0; i<nb; i++) {
		int I = 18 * i;
		for (int j = 0; j<3; j++) {
			data[I + j] = forcesVec[i][j];
			data[I + j + 3] = momentsVec[i][j];
			data[I + j + 6] = pointsVec[i][j];
			data[I + j + 9] = angleVec[i][j];
			data[I + j + 12] = Velocity[i][j];
			data[I + j + 15] = angleVelocity[i][j];

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

void BodyForceAnalysis2::constructDescription()
{
	string descrip;

	descrip = "\nThis file contains the record of bodies forces of the model\n";
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

void BodyForceAnalysis2::constructColumnLabels()
{


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


		//std::string labelRoot = bodyName;
		labels.append(body.getName() + "_fx");
		labels.append(body.getName() + "_fy");
		labels.append(body.getName() + "_fz");
		labels.append(body.getName() + "_mx");
		labels.append(body.getName() + "_my");
		labels.append(body.getName() + "_mz");
		labels.append(body.getName() + "_px");
		labels.append(body.getName() + "_py");
		labels.append(body.getName() + "_pz");
		labels.append(body.getName() + "_ox");
		labels.append(body.getName() + "_oy");
		labels.append(body.getName() + "_oz");
		labels.append(body.getName() + "_ux");
		labels.append(body.getName() + "_uy");
		labels.append(body.getName() + "_uz");
		labels.append(body.getName() + "_ùx");
		labels.append(body.getName() + "_ùy");
		labels.append(body.getName() + "_ùz");
	}

	setColumnLabels(labels);


}


void BodyForceAnalysis2::setupStorage()
{
	storage.reset(0);
	storage.setDescription(getDescription());
	storage.setColumnLabels(getColumnLabels());
	setName("BodyForceAnalysis");
}


