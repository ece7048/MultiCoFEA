//#include "BK_structor.h"
//#include "BF_structor.h"
#include "ReSampler.h"
//#include "FEBRunner.h"
//#include "TSC.h"
//#include "DOFResample.h"
#include <stdlib.h>

ReSampler::ReSampler(void)
{
}

ReSampler::~ReSampler(void)
{
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%	START  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

Vector ReSampler::runvec(int itter, string resultDir, Vector timeinput){

	//////////////store section/////////////
	
	//Vector newnew;

	time = return_vect(0);
	int sizer = time.size();
	Gfx = return_vect(1);
	/////global vector////
	Gfy = return_vect(2);
	Gfz = return_vect(3);
	Gmx = return_vect(4);
	Gmy = return_vect(5);
	Gmz = return_vect(6);
	//Gf2x = return_vect(7);
	//Gf2y = return_vect(8);
	//Gf2z = return_vect(9);
	//Gm2x = return_vect(10);
	//Gm2y = return_vect(11);
	//Gm2z = return_vect(12);
	
	//////////////////////START the detection /////////////////////////////////

	cout << "" << endl;
	cout << "THE DETECT POINTS PHASE FOR RESAMPLER ANALYSIS" << endl;
	cout << "" << endl;

///////////////////////////////write a file for the resampler files without general time vector (equal length for all degreeze for fix timestep in analysis)//////////////////////
////////////////////////////////////P.S. 1 copy paste here from the end...///////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////
	cout << "" << endl;
	cout << "NEW TIME VECTOR BUILD PHASE FOR RESAMPLER ANALYSIS" << endl;
	cout << "" << endl;
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	/////////////////////call function for common timestep///////////////////////////////////////////////////////////////////////////////////////////////
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////	
	int counter = 0;

	Vector timenew00 = timebuilder(timeinput, detector(0, sizer, time, Gfx));
	Vector timenew0 = timebuilder(timenew00, detector(0, sizer, time, Gfy));
	Vector timenew1 = timebuilder(timenew0, detector(0, sizer, time, Gfz));
	Vector timenew2 = timebuilder(timenew1, detector(0, sizer, time, Gmx));
	Vector timenew3 = timebuilder(timenew2, detector(0, sizer, time, Gmy));
	Vector timenew = timebuilder(timenew3, detector(0, sizer, time, Gmz));

	///////////////////////////////the timebuilder for the case of 24 vector input the call off the timebuilder )//////////////////////
	////////////////////////////////////P.S. 2 copy paste here from the end...///////////////////////////////////////////////////////////////
	
	ofstream resampletim;
	char numberr = itter + '0';
	resampletim = ofstream(resultDir + "/timeresample"+numberr+".txt", ofstream::out);
	resampletim << "Start" << endl;

	
	resampletim << " " << endl;
	resampletim << "First time values after compair with the second values: " << endl;
	resampletim << timenew0 << endl;
	resampletim << "Second time values after compair with the third values: " << endl;
	resampletim << " " << endl;
	resampletim << timenew1 << endl;
	resampletim << "Third time values after compair with the forth values: " << endl;
	resampletim << " " << endl;
	resampletim << timenew2 << endl;
	resampletim << "Forth time values after compair with the fifth values: " << endl;
	resampletim << " " << endl;
	resampletim << timenew3 << endl;
	resampletim << "Fifth  time values after compair with the sixth values: " << endl;
	resampletim << " " << endl;

	///////////////////////////////the timenew for the case of 24 vector input timenew writer )//////////////////////
	////////////////////////////////////P.S. 3 copy paste here from the end...///////////////////////////////////////////////////////////////

	
	resampletim << " " << endl;
	resampletim << timenew << endl;
	resampletim << " " << endl;
	
	resampletim << "END " << endl;
	///////////////////////////////the dof builder section for the case of 24 vector. this now is on the bf_starter in the end in section of if with y!!! answer )//////////////////////
	////////////////////////////////////P.S. 4 copy paste here from the end...///////////////////////////////////////////////////////////////

	return timenew;
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


Vector ReSampler::timebuilder(Vector timeupd, Vector time1){

//global variables
	time1;
	timeupd;
	int tim1 = time1.size();
	int tim2 = timeupd.size();
	int biggervalue;
	int terminated=1;

//case one....detect if there are values bigger than timupd in time...
			double value2 = time1[tim1-1];
				double value1 = timeupd[tim2-1]; 
				if (value1 < value2){
					for (int i1 = 0; i1 < tim1; ++i1){
						if (value1 < time1[i1]){
							biggervalue = i1; i1 = tim1 + 1; cout << "the biggervalue is on: " << time1[biggervalue] << endl;
						}
					}
					}
//output information for each Dof time vector
			cout << "last value time base: "<<value1 << endl;
			cout << "last value time search: " << value2 << endl;
			cout << "number size of base time vector: " << tim2 << endl;
			cout << "number size of search time vector: " << tim1 << endl;


			
			Vector timenew(tim2 + 1000, 1);
			for (int i = 0; i < tim2; ++i){ double value = timeupd[i]; timenew[i] = value; }
			int p = 0;
			int counter = 0;
			int counter2 = 0;
			int upd = 0;

			// time loop search
			
				while (counter < tim1){
					counter2 = 0;
					
					//if not detect bigger value in search time vector
					if (counter != biggervalue){

						double interestvalue = time1[counter];
						//timeupd loop search
						while (counter2 < tim2){

							double compairvalue = timeupd[counter2];
							
							
								if (compairvalue > interestvalue){

									double beforecompairvalue = timeupd[counter2 - 1];

									if (beforecompairvalue != interestvalue){
										timenew[counter2+upd] = interestvalue;
										for (int k = counter2 ; k < tim2; ++k){ double valuenext = timeupd[k]; timenew[k + 1+upd] = valuenext; }
										++p;
										++upd ;
									}
									counter2 = tim2 + 1; //terminate the second while
								}
								++counter2;
							}	
						}
					//if detect bigger value in search time vector 
					if (counter == biggervalue){ //pass all the value of time in upd version.
						cout << "ok" << endl;
						for (int k = counter; k < tim1; ++k){ double valuenext = time1[k]; timenew[k] = valuenext; ++p; }
						counter = tim1 + 1;
						terminated = 0;

					}
						++counter;
						if (terminated == 0){ counter = tim1 + 1; cout << "out" << endl; terminated = 1; }

					}
						
		// Cut the unused vector length
		Vector ftimenew(tim2 + p, 1);
		for (int sp = 0; sp < tim2 + p; ++sp){
			double duo = timenew[sp];
			ftimenew[sp] = duo;
		}
			
			cout << "the number of new input times value in base time vestor are: " << p << endl;
			cout << "" << endl;

			return ftimenew;

		
	}
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

Vector ReSampler::DoFbuilder(Vector fx,  Vector timenew){

	time = return_vect(0);
	int timesize = timenew.size();
	Vector fnew(timesize,1);
	int sizer = time.size();
	int counterr = 0;
	int counter1 = 0;
	int coun = 0;
	int last = 0;
	while (counterr < timesize){
		 counter1 = last;
		double one = timenew[counterr];
		while (counter1 < sizer){
			double two = time[counter1];
			if (one == two){
				double value = fx[counter1];
				fnew[coun] = value;
				last = counter1;
				++coun;
				counter1 = sizer + 1;

			}
			++counter1;
		}
		++counterr;
		}
	
	return fnew;
}
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

Vector ReSampler::detector(int caser,int sizer, Vector timer, Vector fx){
	
	double on, tw;
	on = fx[0];
	tw = fx[1];
	double startslope = (tw - on);
	double detect, detect2;
	int count = 2;
	int p = 1;
	Vector fout(sizer,1);
	Vector tout(sizer, 1);
	double one, two;
		one = fx[p - 1];
		two = timer[p - 1];
	fout[p-1] = one;
	tout[p-1] = two;
///////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////check if slope is zero////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////
	if (startslope == 0){
		for (int ii = 2; ii < sizer; ii++){
			one = fx[ii - 1];
			two = fx[ii];
			startslope = (two - one);
			if (startslope != 0){
				count = ii;
				ii = sizer + 1;
			}
		}
	}
	if (caser == 1){
		cout << "########################################################" << endl;
		cout << "The start slope of the DoF'sample is: " << startslope << endl;
	}
///////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////check if slope is positive ////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////
	if (startslope > 0){
		for (int i = count; i < sizer; i++){
			double one1 = fx[i - 1];
			double two1 = fx[i];
			detect = (two1 - one1);

			if (detect < 0){
				fout[p] = one1;
				double valuetim = timer[i - 1];
				tout[p] = valuetim;
				p = p + 1;
				if (caser == 1){
					cout << "negative slope for the DoF's sample" << endl;
				}
				for (int o = i; o < sizer; o++){
					double one12 = fx[o - 1];
					double two12 = fx[o];
					detect2 = (two12 - one12);
					if (detect2 > 0){
						double one1o = fx[o - 1];
						double two1o = timer[o-1];
						fout[p] = one1o;
						tout[p] = two1o;
						p = p + 1;
						i = o;
						o = sizer + 1;
						if (caser == 1){
							cout << "positive slope for the DoF's sample" << endl;
						}
					}
					if (o == sizer - 1){
						double one1o = fx[o];
						double two1o = timer[o];
						fout[p] = one1o;
						tout[p] = two1o;
						++p;
						i = sizer + 1;
					}
				}
			}
			if (i == sizer - 1){
				double one1o = fx[i];
				double two1o = timer[i];
				fout[p] = one1o;
				tout[p] = two1o;
				++p;
			}
		}
	}
///////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////check if slope is negative////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////
	if (startslope < 0){
		for (int i = count; i < sizer; i++){
			double one1 = fx[i - 1];
			double two1 = fx[i];
			detect = (two1 - one1);

			if (detect > 0){
				if (caser == 1){
					cout << "positive slope for the DoF's sample." << endl;
				}
				fout[p] = one1;
				double valuetim = timer[i - 1];
				tout[p] = valuetim;
				p = p + 1;
				for (int o = i; o < sizer; o++){
					double one12 = fx[o - 1];
					double two12 = fx[o];
					detect2 = (two12 - one12);
					if (detect2 < 0){
						if (caser == 1){
							cout << "positive slope for the DoF's sample." << endl;
						}
						double one1o = fx[o - 1];
						double two1o = timer[o - 1];
						fout[p] = one1o;
						tout[p] = two1o;
						p = p + 1;
						i = o;
						o = sizer + 1;
					}
					if (o == sizer - 1){
						double one1o = fx[o];
						double two1o = timer[o];
						fout[p] = one1o;
						tout[p] = two1o;
						++p;
						i = sizer + 1;
					}
				}
			}
			if (i == sizer - 1){
				double one1o = fx[i];
				double two1o = timer[i];
				fout[p] = one1o;
				tout[p] = two1o;
				++p;
			}
		}
	}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////// For fix DoF///////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	if (startslope = 0){
		fout[0] = 0; tout[0]=0;
	}
	Vector tout1(p, 1);
	Vector fout1(p, 1);
	for (int sp = 0; sp < p; ++sp){ double onee = tout[sp]; tout1[sp] = onee; double twoo = fout[sp]; fout1[sp] = twoo; }

	if (caser == 0){
		return tout1; }
	if (caser == 1){
		return fout1; }
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

String ReSampler::interpolationkind(Vector fx, Vector time){
 //variables local
	int sizer = fx.size();
	//counters
	int count1=0;
	int count2=0;
	int zero=0;
	// deviations
	double dev[10000];
	double diff[10000];
	double accur = 10000;//for linear fx accurace threshold 1% slope
	int critirio = (int)(10*sizer / 10)-1;
	string kind;
	//initialization of deviations///
	double prevf1 = fx[0];
	double f1 = fx[1];
	double t1 = time[1];
	double prevt1 = time[0];
	dev[0] = (f1 - prevf1);/// (t1 - prevt1);
	dev[1] = (fx[2] - fx[1]);
	dev[sizer-1] = (fx[sizer-1] - fx[sizer-2]);
	dev[sizer - 2] = (fx[sizer - 2] - fx[sizer - 3]);
	//find the deviation////
	//for a five point differation function as in PAN AND TOMPKINS: REA L-TIME QRS DETECTION ALGORITHM, PAPER....

	for (int i = 2; i < sizer-3; ++i){
		double prevf = fx[i - 1];
		double f = fx[i+1];
		//double t = time[i];
		double prevt = time[i - 1];
		dev[i] = abs(1/8*(fx[i+2]+2*f - 2*prevf-fx[i-2]));/// (t - prevt);
		 //assuption for step values in the indervals and 1 for time difference...//
		diff[i - 2] = abs(1 / 8 * (dev[i + 2] + 2 * dev[i + 1] - 2 * dev[i - 1] - dev[i - 2]));
		 
		 }
/////////////////////////////////////////////////////////////////////////////////
//////////detec the kind of smooth or linear  of step function of fx////////////
///////////////////////////////////////////////////////////////////////////////
	for (int o = 1; o < sizer; ++o){
		///linear fx mens const dfx so///
		if (diff[o]==0){ ++count1; }
		///smooth fx mens linear dfx so///
		if (diff[o] > 0){ ++count2; }
		///step fx mens zero dfx so///
		if (dev[o] == 0){ ++zero; }
	}
	int first = 9* sizer / 100;
	int second = 1 * sizer / 100;
	if (count1 > first){ kind = "linear"; }
	if (second <= count2){ kind = "smooth"; }
	if (zero >= critirio){ kind = "step"; }
	return kind;
}


//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

void ReSampler::store(Vector fx, int which){
	if (which == 0){ time = fx; }
	if (which == 1){ Gfx = fx; }
	if (which == 2){ Gfy = fx; }
	if (which == 3){ Gfz = fx; }
	if (which == 4){ Gmx = fx; }
	if (which == 5){ Gmy = fx; }
	if (which == 6){ Gmz = fx; }
	if (which == 7){ Gf2x = fx; }
	if (which == 8){ Gf2y = fx; }
	if (which == 9){ Gf2z = fx; }
	if (which == 10){ Gm2x = fx; }
	if (which == 11){ Gm2y = fx; }
	if (which == 12){ Gm2z = fx; }
	if (which == 13){ Gpx = fx; }
	if (which == 14){ Gpy = fx; }
	if (which == 15){ Gpz = fx; }
	if (which == 16){ Gox = fx; }
	if (which == 17){ Goy = fx; }
	if (which == 18){ Goz = fx; }
	if (which == 19){ Gp2x = fx; }
	if (which == 20){ Gp2y = fx; }
	if (which == 21){ Gp2z = fx; }
	if (which == 22){ Go2x = fx; }
	if (which == 23){ Go2y = fx; }
	if (which == 24){ Go2z = fx; }
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


Vector ReSampler::return_vect(int which){
	if (which == 0){ return time; }
	if (which == 1){ return Gfx; }
	if (which == 2){ return Gfy; }
	if (which == 3){ return Gfz; }
	if (which == 4){ return Gmx; }
	if (which == 5){ return Gmy; }
	if (which == 6){ return Gmz; }
	if (which == 7){ return Gf2x; }
	if (which == 8){ return Gf2y; }
	if (which == 9){ return Gf2z; }
	if (which == 10){ return Gm2x; }
	if (which == 11){ return Gm2y; }
	if (which == 12){ return Gm2z; }
	if (which == 13){ return Gpx; }
	if (which == 14){ return Gpy; }
	if (which == 15){ return Gpz; }
	if (which == 16){ return Gox; }
	if (which == 17){ return Goy; }
	if (which == 18){ return Goz; }
	if (which == 19){ return Gp2x; }
	if (which == 20){ return Gp2y; }
	if (which == 21){ return Gp2z; }
	if (which == 22){ return Go2x; }
	if (which == 23){ return Go2y; }
	if (which == 24){ return Go2z; }
}
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  THE  END    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%



//P.S....SECTION.....


/////////////////
//////0//////////
////////////////

//////////////////////////////////////////////////ALL THE 24 VECTOR WHICH HANDLE THE SIMULATION FROM THE BF-starter////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  ,Vector fFx, Vector fFy, Vector fFz, Vector fMx, Vector fMy, Vector fMz, Vector tFx, Vector tFy, Vector tFz, Vector tMx, Vector tMy, Vector tMz
//, Vector fPx, Vector fPy, Vector fPz, Vector fOx, Vector fOy, Vector fOz, Vector tPx, Vector tPy, Vector tPz, Vector tOx, Vector tOy, Vector tOz){
// int sizer, string kind[24],
/// old input in run void function

/////////////////
////////1///////
///////////////

/*
//Vector ffx(detector(1,sizer, time, fFx));
Vector time1 = detector(0, sizer, time, fFx);
//cout << time1 << endl;

Vector time2 = detector(0,sizer, time, fFy);
//Vector ffy = detector(1, sizer, time, fFy);

Vector time3 = detector(0,sizer, time, fFz);
//Vector ffz = detector(1, sizer, time, fFz);

Vector time4 = detector(0,sizer, time, fMx);
//Vector fmx = detector(1, sizer, time, fMx);

Vector time5 = detector(0,sizer, time, fMy);
//Vector fmy = detector(1, sizer, time, fMy);


Vector	time6 = detector(0,sizer, time, fMz);
//Vector	fmz = detector(1, sizer, time, fMz);

Vector time7 = detector(0, sizer, time, tFx);
//Vector	tfx = detector(1, sizer, time, tFx);

Vector	time8 = detector(0, sizer, time, tFy);
//Vector	tfy = detector(1, sizer, time, tFy);

Vector	time9 = detector(0, sizer, time, tFz);
//Vector	tfz = detector(1, sizer, time, tFz);

Vector	time10 = detector(0, sizer, time, tMx);
//Vector	tmx = detector(1, sizer, time, tMx);

Vector	time11 = detector(0, sizer, time, tMy);
//Vector	tmy = detector(1, sizer, time, tMy);

Vector	time12 = detector(0, sizer, time, tMz);
//Vector	tmz = detector(1, sizer, time, tMz);

Vector	time13 = detector(0, sizer, time, fPx);
//Vector	fpx = detector(1, sizer, time, fPx);

Vector	time14 = detector(0, sizer, time, fPy);
//Vector	fpy = detector(1, sizer, time, fPy);

Vector	time15 = detector(0, sizer, time, fPz);
//Vector	fpz = detector(1, sizer, time, fPz);

Vector	time16 = detector(0, sizer, time, fOx);
//Vector	fox = detector(1, sizer, time, fOx);

Vector	time17 = detector(0, sizer, time, fOy);
//Vector	foy = detector(1, sizer, time, fOy);

Vector	time18 = detector(0, sizer, time, fOz);
//Vector	foz = detector(1, sizer, time, fOz);

Vector	time19 = detector(0, sizer, time, tPx);
//Vector  tpx = detector(1, sizer, time, tPx);

Vector time20 = detector(0, sizer, time, tPy);
//Vector tpy = detector(1, sizer, time, tPy);

Vector time21 = detector(0, sizer, time, tPz);
//Vector tpz = detector(1, sizer, time, tPz);

Vector time22 = detector(0, sizer, time, tOx);
//Vector tox = detector(1, sizer, time, tOx);

Vector time23 = detector(0, sizer, time, tOy);
//Vector toy = detector(1, sizer, time, tOy);

Vector time24 = detector(0, sizer, time, tOz);
//Vector toz = detector(1, sizer, time, tOz);
*/
////////////////////////////////////////////
//ofstream resample;
//resample = ofstream(resultDir + "/ReSampler.txt", ofstream::out);
//for (int i = 0; i < ffx.size(); ++i){
//resample << time1[i] << "," << ffx[i] << endl;


//}
//resample << "end" << endl;
//for (int ii = 0; ii < ffy.size(); ++ii){
//resample << time2[ii] << "," << ffy[ii] << endl;
//}
/*
resample << "end" << endl;
for (int i1 = 0; i1 < ffz.size(); ++i1){
resample << time3[i1] << "," << ffz[i1] << endl;
}
resample << "end" << endl;
for (int i2 = 0; i2 < fmx.size(); ++i2){
resample << time4[i2] << "," << fmx[i2] << endl;
}
resample << "end" << endl;
for (int i3 = 0; i3 < fmy.size(); ++i3){
resample << time5[i3] << "," << fmy[i3] << endl;
}
resample << "end" << endl;
for (int i4 = 0; i4 < fmz.size(); ++i4){
resample << time6[i4] << "," << fmz[i4] << endl;
}
resample << "end" << endl;
for (int i5 = 0; i5 < tfx.size(); ++i5){
resample << time7[i5] << "," << tfx[i5] << endl;
}resample << "end" << endl;

for (int i6 = 0; i6 < tfy.size(); ++i6){
resample << time8[i6] << "," << tfy[i6] << endl;
}resample << "end" << endl;

for (int i7 = 0; i7 < tfz.size(); ++i7){
resample << time9[i7] << "," << tfz[i7] << endl;
}resample << "end" << endl;

for (int i8 = 0; i8 < tmx.size(); ++i8){
resample << time10[i8] << "," << tmx[i8] << endl;
}
resample << "end" << endl;
for (int i9 = 0; i9 < tmy.size(); ++i9){
resample << time11[i9] << "," << tmy[i9] << endl;
}resample << "end" << endl;

for (int ia = 0; ia < tmz.size(); ++ia){
resample << time12[ia] << "," << tmz[ia] << endl;
}resample << "end" << endl;

for (int ib = 0; ib < fpx.size(); ++ib){
resample << time13[ib] << "," << fpx[ib] << endl;
}resample << "end" << endl;

for (int ic = 0; ic < fpy.size(); ++ic){
resample << time14[ic] << "," << fpy[ic] << endl;
}resample << "end" << endl;

for (int id = 0; id < fpz.size(); ++id){
resample << time15[id] << "," << fpz[id] << endl;
}resample << "end" << endl;

for (int ie = 0; ie < fox.size(); ++ie){
resample << time16[ie] << "," << fox[ie] << endl;
}resample << "end" << endl;

for (int iz = 0; iz < foy.size(); ++iz){
resample << time17[iz] << "," << foy[iz] << endl;
}resample << "end" << endl;

for (int ig = 0; ig < foz.size(); ++ig){
resample << time18[ig] << "," << foz[ig] << endl;
}resample << "end" << endl;

for (int ih = 0; ih < tpx.size(); ++ih){
resample << time19[ih] << "," << tpx[ih] << endl;
}resample << "end" << endl;

for (int o1 = 0; o1 < tpy.size(); ++o1){
resample << time20[o1] << "," << tpy[o1] << endl;
}resample << "end" << endl;

for (int oi = 0; oi < tpz.size(); ++oi){
resample << time21[oi] << "," << tpz[oi] << endl;
}resample << "end" << endl;

for (int i22 = 0; i22 < tox.size(); ++i22){
resample << time22[i22] << "," << tox[i22] << endl;
}resample << "end" << endl;

for (int i11 = 0; i11 < toy.size(); ++i11){
resample << time23[i11] << "," << toy[i11] << endl;
}resample << "end" << endl;

for (int io = 0; io < toz.size(); ++io){
resample << time24[io] << "," << toz[io] << endl;
}resample << "end" << endl;

*/

//////////////////
/////////2//////
////////////////
////////////////////////////////////////////////////////////////////////
//Vector timenew5 = timebuilder(timenew4, detector(0, sizer, time, Gf2x));
//Vector timenew6 = timebuilder(timenew5, detector(0, sizer, time, Gf2y));
//Vector timenew7 = timebuilder(timenew6, detector(0, sizer, time, Gf2z));
//Vector timenew8 = timebuilder(timenew7, detector(0, sizer, time, Gm2x));
//Vector timenew9 = timebuilder(timenew8, detector(0, sizer, time, Gm2y));
//Vector timenew10 = timebuilder(timenew9, detector(0, sizer, time, Gm2z));
/*
//Vector timenew12 = timebuilder(detector(0, sizer, time, fPx), detector(0, sizer, time, fPy));
Vector timenew11 = timebuilder(timenew10, detector(0, sizer, time, fPx));
Vector timenew12 = timebuilder(timenew11, detector(0, sizer, time, fPy));
Vector timenew13 = timebuilder(timenew12, detector(0, sizer, time, fPz));
Vector timenew14 = timebuilder(timenew13, detector(0, sizer, time, fOx));
Vector timenew15 = timebuilder(timenew14, detector(0, sizer, time, fOy));
Vector timenew16 = timebuilder(timenew15, detector(0, sizer, time, fOz));
Vector timenew17 = timebuilder(timenew16, detector(0, sizer, time, tPx));
Vector timenew18 = timebuilder(timenew17, detector(0, sizer, time, tPy));
Vector timenew19 = timebuilder(timenew18, detector(0, sizer, time, tPz));
Vector timenew20 = timebuilder(timenew19, detector(0, sizer, time, tOx));
Vector timenew21 = timebuilder(timenew20, detector(0, sizer, time, tOy));
Vector timenew = timebuilder(timenew21, detector(0, sizer, time, tOz));
*/
/*
Vector timenew = timebuilder(timebuilder(timebuilder(timebuilder(timebuilder(timebuilder(timebuilder(timebuilder(timebuilder(timebuilder(timebuilder(timebuilder(timebuilder(timebuilder(timebuilder(timebuilder(timebuilder(timebuilder(timebuilder(timebuilder(timebuilder(timebuilder(timebuilder((detector(0, sizer, time, fFx)), detector(0, sizer, time, fFy)), detector(0, sizer, time, fFz)), detector(0, sizer, time, fMx)), detector(0, sizer, time, fMy)), detector(0, sizer, time, fMz)), detector(0, sizer, time, tFx)), detector(0, sizer, time, tFy)), detector(0, sizer, time, tFz)), detector(0, sizer, time, tMx)), detector(0, sizer, time, tMy))
, detector(0, sizer, time, tMz)), detector(0, sizer, time, fPx)), detector(0, sizer, time, fPy)), detector(0, sizer, time, fPz)), detector(0, sizer, time, fOx)), detector(0, sizer, time, fOy)), detector(0, sizer, time, fOz)), detector(0, sizer, time, tPx)), detector(0, sizer, time, tPy)), detector(0, sizer, time, tPz)), detector(0, sizer, time, tOx)), detector(0, sizer, time, tOy)), detector(0, sizer, time, tOz));
////// write time results in a file/////////////
*/

/////////////////
//////3//////////
//////////////


/*
resampletim << timenew4 << endl;
resampletim << "Sixth time values after compair with the seventh values: " << endl;
resampletim << " " << endl;
resampletim << timenew5 << endl;
resampletim << "Seventh time values after compair with the eighth values: " << endl;
resampletim << " " << endl;
resampletim << timenew6 << endl;
resampletim << "Eighth time values after compair with the nineth values: " << endl;
resampletim << " " << endl;
resampletim << timenew7 << endl;
resampletim << "Nineth time values after compair with the tenth values: " << endl;
resampletim << " " << endl;
resampletim << timenew8 << endl;
resampletim << "Tenth time values after compair with the eleventh values: " << endl;
resampletim << " " << endl;
resampletim << timenew9 << endl;
resampletim << "Eleventh time values after compair with the twoelveth values: " << endl;
resampletim << " " << endl;
resampletim << timenew10 << endl;
resampletim << "Twoelveth time values after compair with the thirteenth values: " << endl;
resampletim << " " << endl;
resampletim << timenew11 << endl;
resampletim << "Thirteenth time values after compair with the fourteenth values: " << endl;
resampletim << " " << endl;

resampletim << timenew12 << endl;
resampletim << "Fourteenth time values after compair with the fiveteenth values: " << endl;
resampletim << " " << endl;
resampletim << timenew13 << endl;
resampletim << "Fiveteenth time values after compair with the seventeenth values: " << endl;
resampletim << " " << endl;
resampletim << timenew14 << endl;
resampletim << "Sixteenth time values after compair with the eighteenth values: " << endl;
resampletim << " " << endl;
resampletim << timenew15 << endl;
resampletim << "Seventeenth time values after compair with the nineteenth values: " << endl;
resampletim << " " << endl;
resampletim << timenew16 << endl;
resampletim << "Eighteenth time values after compair with the twoenteenth values: " << endl;
resampletim << " " << endl;
resampletim << timenew17 << endl;
resampletim << "Nineteenth time values after compair with the twoenteenoneth values: " << endl;
resampletim << " " << endl;
resampletim << timenew18 << endl;
resampletim << "Twoenteenth time values after compair with the twoenteentwoth values: " << endl;
resampletim << " " << endl;
resampletim << timenew19 << endl;
resampletim << "Twoenteenoneth time values after compair with the twoenteenthreeth values: " << endl;
resampletim << " " << endl;
resampletim << timenew20 << endl;
resampletim << "Twoenteentwoth time values after compair with the last values: " << endl;
resampletim << " " << endl;
resampletim << timenew21 << endl;
resampletim << "Final time vector values: " << endl;
*/

////////////////////
//////////4////////
//////////////////
/*
//cout << timenew << endl;
////////////////////////////build the dof values//////
/// forces//////////
Vector fFxf = DoFbuilder(Gfx, time, timenew);
Vector fFyf = DoFbuilder(Gfy, time, timenew);
Vector fFzf = DoFbuilder(Gfz, time, timenew);
Vector fMxf = DoFbuilder(Gmx, time, timenew);
Vector fMyf = DoFbuilder(Gmy, time, timenew);
Vector fMzf = DoFbuilder(Gmz, time, timenew);
Vector tFxf = DoFbuilder(Gf2x, time, timenew);
Vector tFyf = DoFbuilder(Gf2y, time, timenew);
Vector tFzf = DoFbuilder(Gf2z, time, timenew);
Vector tMxf = DoFbuilder(Gm2x, time, timenew);
Vector tMyf = DoFbuilder(Gm2y, time, timenew);
Vector tMzf = DoFbuilder(Gm2z, time, timenew);

/////positions//////////////////////////
Vector fPxf = DoFbuilder(fPx, time, timenew);
Vector fPyf = DoFbuilder(fPy, time, timenew);
Vector fPzf = DoFbuilder(fPz, time, timenew);
Vector fOxf = DoFbuilder(fOx, time, timenew);
Vector fOyf = DoFbuilder(fOy, time, timenew);
Vector fOzf = DoFbuilder(fOz, time, timenew);
Vector tPxf = DoFbuilder(tPx, time, timenew);
Vector tPyf = DoFbuilder(tPy, time, timenew);
Vector tPzf = DoFbuilder(tPz, time, timenew);
Vector tOxf = DoFbuilder(tOx, time, timenew);
Vector tOyf = DoFbuilder(tOy, time, timenew);
Vector tOzf = DoFbuilder(tOz, time, timenew);


//////////////////////////// and call the structor class for write the feb file//////////////////////////////////////////////////
int endend = timenew.size()-1;
int firstsize = time.size();
cout << "" << endl;
cout << "CONCLUSION OF RESAMPLER ANALYSIS" << endl;
cout << "" << endl;
cout << "The size of DoF's samplein the beginning of ReSample phase was: " << firstsize << " elements" << endl;
cout << "The size of the ReSample time vector for the DoF is now: " << endend <<" elements"<< endl;

TSC tsc;
tsc.run(fPxf, fPyf, fPzf, fOxf, fOyf, fOzf, tPxf, tPyf, tPzf, tOxf, tOyf, tOzf, timenew);
tsc.~TSC();
char which;
cout << "" << endl;
cout << "Do you want a DoF Resample for the case of not need a DoF, of the bodies? (y/n)" << endl;
cin >> which;
cout << "" << endl;
if (which == 'y'){
DOFResample dr;
dr.detect(itter, kind, 0, endend + 1, resultDir, timenew, fPxf, fPyf, fPzf, fOxf, fOyf, fOzf, tPxf, tPyf, tPzf, tOxf, tOyf, tOzf, fFxf, fFyf, fFzf, fMxf, fMyf, fMzf, tFxf, tFyf, tFzf, tMxf, tMyf, tMzf);
//dr.~DOFResample();
}
if (which == 'n'){
cout << "" << endl;
cout << "CALL THE FEBio's XML STRUCTORS" << endl;
cout << "" << endl;
BF_structor bff;
bff.begining(itter, kind, endend, resultDir, timenew, fFxf, fFyf, fFzf, fMxf, fMyf, fMzf, tFxf, tFyf, tFzf, tMxf, tMyf, tMzf);
//bff.~BF_structor();
BK_structor bkk;
bkk.begining(itter,kind, 0, endend + 1, resultDir, timenew, fPxf, fPyf, fPzf, fOxf, fOyf, fOzf, tPxf, tPyf, tPzf, tOxf, tOyf, tOzf);
//bkk.~BK_structor();
}
*/