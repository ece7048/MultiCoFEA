#ifndef BODYFORCEANALYSIS2_H
#define BODYFORCEANALYSIS2_H

#include <iostream>
#include <OpenSim/OpenSim.h>
#include <OpenSim/Simulation/Model/Actuator.h>
#include "INIReader.h"
#include "Settings.h"



namespace OpenSim{

	class PhysicalFrame;


	class BodyForceAnalysis2 : public Analysis
	{
		OpenSim_DECLARE_CONCRETE_OBJECT(BodyForceAnalysis2, Analysis);

		//friend class Force;

	public:
		//	friend  void applyTorque(State &, Body &,  Vec3&, SpatialVec> &);

		//friend  void applyForceToPoint(State& , Body & , Vec3& , Vec3 &, SpatialVec & );

		BodyForceAnalysis2(){};

		BodyForceAnalysis2(Model* model)
			: Analysis(model){};

		virtual void setModel(Model& aModel);

		virtual int begin(SimTK::State& s);

		virtual int step(const SimTK::State& s, int stepNumber);

		virtual int end(SimTK::State& s);


		virtual int print(const std::string &path);

		virtual int printResults(
			const std::string &aBaseName, const std::string &aDir,
			double aDT, const std::string &aExtension);

		//	Storage getStorage();

	protected:

		int record(const SimTK::State& s);


	private:

		std::string bodyName;
		SimTK::Vec3 markerOffset;
		SimTK::Vec6 markerOffset2;
		Storage storage;

		void constructDescription();

		void constructColumnLabels();

		void setupStorage();
	};
};


#endif