#include "CASE_TWO.h"

CASE_TWO::CASE_TWO(void)
{
}

CASE_TWO::~CASE_TWO(void)
{
}

void CASE_TWO::store(Vector fxo, int which){
	if (which == 0){ timefinal = fxo; }
	if (which == 1){ fPx = fxo; }
	if (which == 2){ fPy = fxo; }
	if (which == 3){ fPz = fxo; }
	if (which == 4){ fOx = fxo; }
	if (which == 5){ fOy = fxo; }
	if (which == 6){ fOz = fxo; }
	if (which == 7){ tPx = fxo; }
	if (which == 8){ tPy = fxo; }
	if (which == 9){ tPz = fxo; }
	if (which == 10){ tOx = fxo; }
	if (which == 11){ tOy = fxo; }
	if (which == 12){ tOz = fxo; }
	if (which == 13){ fFx = fxo; }
	if (which == 14){ fFy = fxo; }
	if (which == 15){ fFz = fxo; }
	if (which == 16){ fMx = fxo; }
	if (which == 17){ fMy = fxo; }
	if (which == 18){ fMz = fxo; }
	if (which == 19){ tFx = fxo; }
	if (which == 20){ tFy = fxo; }
	if (which == 21){ tFz = fxo; }
	if (which == 22){ tMx = fxo; }
	if (which == 23){ tMy = fxo; }
	if (which == 24){ tMz = fxo; }
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


Vector CASE_TWO::return_vect(int which){
	if (which == 0){ return timefinal; }
	if (which == 1){ return fPx; }
	if (which == 2){ return fPy; }
	if (which == 3){ return fPz; }
	if (which == 4){ return fOx; }
	if (which == 5){ return fOy; }
	if (which == 6){ return fOz; }
	if (which == 7){ return tPx; }
	if (which == 8){ return tPy; }
	if (which == 9){ return tPz; }
	if (which == 10){ return tOx; }
	if (which == 11){ return tOy; }
	if (which == 12){ return tOz; }
	if (which == 13){ return fFx; }
	if (which == 14){ return fFy; }
	if (which == 15){ return fFz; }
	if (which == 16){ return fMx; }
	if (which == 17){ return fMy; }
	if (which == 18){ return fMz; }
	if (which == 19){ return tFx; }
	if (which == 20){ return tFy; }
	if (which == 21){ return tFz; }
	if (which == 22){ return tMx; }
	if (which == 23){ return tMy; }
	if (which == 24){ return tMz; }
}


void CASE_TWO::run(int itteration, string kind[24], int endend, string resultDir2, char dof){

	timefinal = return_vect(0);
	fPx = return_vect(1);
	/////global vector////
	fPy = return_vect(2);
	fPz = return_vect(3);
	fOx = return_vect(4);
	fOy = return_vect(5);
	fOz = return_vect(6);
	tPx = return_vect(7);
	tPy = return_vect(8);
	tPz = return_vect(9);
	tOx = return_vect(10);
	tOy = return_vect(11);
	tOz = return_vect(12);
	fFx = return_vect(13);
	/////global vector////
	fFy = return_vect(14);
	fFz = return_vect(15);
	fMx = return_vect(16);
	fMy = return_vect(17);
	fMz = return_vect(18);
	tFx = return_vect(19);
	tFy = return_vect(20);
	tFz = return_vect(21);
	tMx = return_vect(22);
	tMy = return_vect(23);
	tMz = return_vect(24);

		char resamplernow = 'n';
		//////////////////////////////////////////////////////////////
		//////////Store the vectors for time resampler class/////////////////
		/////////////////////////////////////////////////////////////
		ReSampler res;
		res.store(timefinal, 0);
		res.store(fFx, 1);
		res.store(fFy, 2);
		res.store(fFz, 3);
		res.store(fMx, 4);
		res.store(fMy, 5);
		res.store(fMz, 6);
		Vector timenew = res.runvec(1, resultDir2, res.detector(0, timefinal.size(), timefinal, fFx));
		//cout << "The time vector after the forces DoF of first body: " << endl;
		//cout << timenew3 << endl;
		res.~ReSampler();

		ReSampler res1;
		res1.store(timefinal, 0);
		res1.store(tFx, 1);
		res1.store(tFy, 2);
		res1.store(tFz, 3);
		res1.store(tMx, 4);
		res1.store(tMy, 5);
		res1.store(tMz, 6);
		timenew = res1.runvec(2, resultDir2, timenew);
		//cout << "The time vector after the forces DoF of second body: " << endl;
		//cout << timenew1 << endl;
		res1.~ReSampler();

		ReSampler res2;
		res2.store(timefinal, 0);
		res2.store(fPx, 1);
		res2.store(fPy, 2);
		res2.store(fPz, 3);
		res2.store(fOx, 4);
		res2.store(fOy, 5);
		res2.store(fOz, 6);
		timenew = res2.runvec(3, resultDir2, timenew);
		//cout << "The time vector after the positions DoF of first body: " << endl;
		//cout << timenew2 << endl;
		res2.~ReSampler();

		ReSampler res3;
		res3.store(timefinal, 0);
		res3.store(tPx, 1);
		res3.store(tPy, 2);
		res3.store(tPz, 3);
		res3.store(tOx, 4);
		res3.store(tOy, 5);
		res3.store(tOz, 6);
		timenew = res3.runvec(4, resultDir2, timenew);
		//cout << "The time vector after the positions DoF of second body: " << endl;
		//cout << timenew << endl;
		res3.~ReSampler();


		Vector fFxf(timenew.size(), 1);
		Vector fFyf(timenew.size(), 1);
		Vector fFzf(timenew.size(), 1);
		Vector fMxf(timenew.size(), 1);
		Vector fMyf(timenew.size(), 1);
		Vector fMzf(timenew.size(), 1);
		Vector tFxf(timenew.size(), 1);
		Vector tFyf(timenew.size(), 1);
		Vector tFzf(timenew.size(), 1);
		Vector tMxf(timenew.size(), 1);
		Vector tMyf(timenew.size(), 1);
		Vector tMzf(timenew.size(), 1);
		Vector fPxf(timenew.size(), 1);
		Vector fPyf(timenew.size(), 1);
		Vector fPzf(timenew.size(), 1);
		Vector fOxf(timenew.size(), 1);
		Vector fOyf(timenew.size(), 1);
		Vector fOzf(timenew.size(), 1);
		Vector tPxf(timenew.size(), 1);
		Vector tPyf(timenew.size(), 1);
		Vector tPzf(timenew.size(), 1);
		Vector tOxf(timenew.size(), 1);
		Vector tOyf(timenew.size(), 1);
		Vector tOzf(timenew.size(), 1);


		ReSampler finalres;

		finalres.store(timefinal, 0);
		/////forces//////////////////////////
		fFxf = finalres.DoFbuilder(fFx, timenew);
		fFyf = (finalres.DoFbuilder(fFy, timenew));
		fFzf = (finalres.DoFbuilder(fFz, timenew));
		fMxf = (finalres.DoFbuilder(fMx, timenew));
		fMyf = (finalres.DoFbuilder(fMy, timenew));
		fMzf = (finalres.DoFbuilder(fMz, timenew));
		tFxf = (finalres.DoFbuilder(tFx, timenew));
		tFyf = (finalres.DoFbuilder(tFy, timenew));
		tFzf = (finalres.DoFbuilder(tFz, timenew));
		tMxf = (finalres.DoFbuilder(tMx, timenew));
		tMyf = (finalres.DoFbuilder(tMy, timenew));
		tMzf = (finalres.DoFbuilder(tMz, timenew));
		finalres.~ReSampler();


		ReSampler finalres2;

		finalres2.store(timefinal, 0);
		/////positions//////////////////////////
		fPxf = (finalres2.DoFbuilder(fPx, timenew));
		fPyf = (finalres2.DoFbuilder(fPy, timenew));
		fPzf = (finalres2.DoFbuilder(fPz, timenew));
		fOxf = (finalres2.DoFbuilder(fOx, timenew));
		fOyf = (finalres2.DoFbuilder(fOy, timenew));
		fOzf = (finalres2.DoFbuilder(fOz, timenew));
		tPxf = (finalres2.DoFbuilder(tPx, timenew));
		tPyf = (finalres2.DoFbuilder(tPy, timenew));
		tPzf = (finalres2.DoFbuilder(tPz, timenew));
		tOxf = (finalres2.DoFbuilder(tOx, timenew));
		tOyf = (finalres2.DoFbuilder(tOy, timenew));
		tOzf = (finalres2.DoFbuilder(tOz, timenew));
		finalres2.~ReSampler();


		//////////////////////////// and call the structor class for write the feb file//////////////////////////////////////////////////
		int endend2 = timenew.size() - 1;
		int firstsize = timefinal.size();
		cout << "" << endl;
		cout << "CONCLUSION OF RESAMPLER ANALYSIS" << endl;
		cout << "" << endl;
		cout << "The size of DoF's sample in the beginning of ReSample phase was: " << firstsize << " elements" << endl;
		cout << "The size of the ReSample time vector for the DoF is now: " << endend2 << " elements" << endl;
	
		

		TSC tsc;
		tsc.run(fPxf, fPyf, fPzf, fOxf, fOyf, fOzf, tPxf, tPyf, tPzf, tOxf, tOyf, tOzf, timenew);
		tsc.~TSC();

		CASE_ONE c1;
		c1.store(timenew, 0);
		c1.store(fPxf, 1);
		c1.store(fPyf, 2);
		c1.store(fPzf, 3);
		c1.store(fOxf, 4);
		c1.store(fOyf, 5);
		c1.store(fOzf, 6);
		c1.store(tPxf, 7);
		c1.store(tPyf, 8);
		c1.store(tPzf, 9);
		c1.store(tOxf, 10);
		c1.store(tOyf, 11);
		c1.store(tOzf, 12);
		c1.store(fFxf, 13);
		c1.store(fFyf, 14);
		c1.store(fFzf, 15);
		c1.store(fMxf, 16);
		c1.store(fMyf, 17);
		c1.store(fMzf, 18);
		c1.store(tFxf, 19);
		c1.store(tFyf, 20);
		c1.store(tFzf, 21);
		c1.store(tMxf, 22);
		c1.store(tMyf, 23);
		c1.store(tMzf, 24);
		c1.run(itteration, kind, endend2 + 1, resultDir2, 2, dof);
		//c1.~CASE_ONE();
}


