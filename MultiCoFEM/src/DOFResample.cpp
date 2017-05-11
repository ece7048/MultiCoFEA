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
#include "BK_structor.h"
#include "BF_structor.h"
#include "DOFResample.h"
#include "Initial_VEL.h"
#include "Visualizerfebgeo.h"

DOFResample::DOFResample(void)
{
}

DOFResample::~DOFResample(void)
{
}

void DOFResample::store(Vector fxo, int which){
	if (which == 0){ time1 = fxo; }
	if (which == 1){ fx1 = fxo; }
	if (which == 2){ fy1 = fxo; }
	if (which == 3){ fz1 = fxo; }
	if (which == 4){ fox1 = fxo; }
	if (which == 5){ foy1 = fxo; }
	if (which == 6){ foz1 = fxo; }
	if (which == 7){ tx1 = fxo; }
	if (which == 8){ ty1 = fxo; }
	if (which == 9){ tz1 = fxo; }
	if (which == 10){ tox1 = fxo; }
	if (which == 11){ toy1 = fxo; }
	if (which == 12){ toz1 = fxo; }
	if (which == 13){ fx = fxo; }
	if (which == 14){ fy = fxo; }
	if (which == 15){ fz = fxo; }
	if (which == 16){ fox = fxo; }
	if (which == 17){ foy = fxo; }
	if (which == 18){ foz = fxo; }
	if (which == 19){ tx = fxo; }
	if (which == 20){ ty = fxo; }
	if (which == 21){ tz = fxo; }
	if (which == 22){ tox = fxo; }
	if (which == 23){ toy = fxo; }
	if (which == 24){ toz = fxo; }
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


Vector DOFResample::return_vect(int which){
	if (which == 0){ return time1; }
	if (which == 1){ return fx1; }
	if (which == 2){ return fy1; }
	if (which == 3){ return fz1; }
	if (which == 4){ return fox1; }
	if (which == 5){ return foy1; }
	if (which == 6){ return foz1; }
	if (which == 7){ return tx1; }
	if (which == 8){ return ty1; }
	if (which == 9){ return tz1; }
	if (which == 10){ return tox1; }
	if (which == 11){ return toy1; }
	if (which == 12){ return toz1; }
	if (which == 13){ return fx; }
	if (which == 14){ return fy; }
	if (which == 15){ return fz; }
	if (which == 16){ return fox; }
	if (which == 17){ return foy; }
	if (which == 18){ return foz; }
	if (which == 19){ return tx; }
	if (which == 20){ return ty; }
	if (which == 21){ return tz; }
	if (which == 22){ return tox; }
	if (which == 23){ return toy; }
	if (which == 24){ return toz; }
}
void DOFResample::detect(int itter,string kind[24],  int sizer1, string resultDir1,int behavior){
	//initialized the store vectors///

	time1 = return_vect(0);
	fx1 = return_vect(1);
	/////global vector////
	fy1 = return_vect(2);
	fz1 = return_vect(3);
	fox1 = return_vect(4);
	foy1 = return_vect(5);
	foz1 = return_vect(6);
	tx1 = return_vect(7);
	ty1 = return_vect(8);
	tz1 = return_vect(9);
	tox1 = return_vect(10);
	toy1 = return_vect(11);
	toz1 = return_vect(12);
	fx = return_vect(13);
	/////global vector////
	fy = return_vect(14);
	fz = return_vect(15);
	fox = return_vect(16);
	foy = return_vect(17);
	foz = return_vect(18);
	tx = return_vect(19);
	ty = return_vect(20);
	tz = return_vect(21);
	tox = return_vect(22);
	toy = return_vect(23);
	toz = return_vect(24);

	Vector fx2=fx;
	Vector fy2=fy;
	Vector fz2=fz;
	Vector fox2=fox;
	Vector foy2=foy;
	Vector foz2=foz;
	Vector tx2=tx;
	Vector ty2=ty;
	Vector tz2=tz;
	Vector tox2=tox;
	Vector toy2=toy;
	Vector toz2=toz;

	
//////////////////////////////////////////////////////////

	int count = 0;
	int count1 = 0;
	int count2 = 0;
	int count3 = 0;
	int count4 = 0;
	int count5 = 0;
	int out = 0;
	int sizer = fx1.size();
	int sizerr = fx.size();

	if (behavior==2){
		for (int o = 0; o < sizer; ++o){
			if (fx1(o) == tx1(o)){ ++count; }
			if (fy1(o) == ty1(o)){ ++count1; }
			if (fz1(o) == tz1(o)){ ++count2; }
			if (fox1(o) == tox1(o)){ ++count3; }
			if (foy1(o) == toy1(o)){ ++count4; }
			if (foz1(o) == toz1(o)){ ++count5; }
			}
		cout << "FIRST PHASE FORCE-VELOCITY RESAMPLER" << endl;
	}
	if (behavior == 1){
		cout << fx[0] << endl;
		cout << fy[0] << endl;
		cout << fz[0] << endl;
		cout << fox[0] << endl;
		//cout << foy[0] << endl;
		//cout << foz[0] << endl;

		//cout << tx[0] << endl;
		//cout << ty[0] << endl;
		//cout << tz[0] << endl;
		//cout << tox[0] << endl;
		//cout << toy[0] << endl;
		//cout << toz[0] << endl;
		int uno=0;
		int duo=0;
		int tre=0;
		int four=0;
		int fif=0;
		int six=0;
		count = sizer;
		count1 = sizer;
		count2 = sizer;
		count3 = sizer;
		count4 = sizer;
		count5 = sizer;

		for (int o = 0; o < sizer; ++o){
			
			if (fx1(o) != tx1(o)){ count = 0; uno = 1; }
			//cout << "x" << endl; }
			if (fy1(o) != ty1(o)){ count1 = 0; duo = 1; } //cout << "y" << endl; }
			if (fz1(o) != tz1(o)){ count2 = 0;  tre = 1; }//cout << "z" << endl; }
			if (fox1(o) != tox1(o)){ count3 = 0; four = 1; } //cout << "X" << endl; }
			if (foy1(o) != toy1(o)){ count4 = 0; fif = 1; }//cout << "Y" << endl; }
			if (foz1(o) != toz1(o)){ count5 = 0; six = 1; }//cout << "Z" << endl; }
			if (uno ==1 && duo == 1 && tre == 1 && four == 1 && fif == 1 && six == 1){ o = sizer + 1; }
		}
		cout << "SECOND PHASE FORCE-VELOCITY RESAMPLER" << endl;
		Mat31 rf, rt;
		Mat33	Ef, Et;
		
			for (int i = 0; i < sizerr; ++i){
				Mat33	Ef = Tait_Bryan(fox1(i), foy1(i), foz1(i));
				Mat33	Et = Tait_Bryan(tox1(i), toy1(i), toz1(i));
				// from RFerdestone page 22 for zero translation we have rotation for both forces and moments so:
				rf[0] = fx1[i];
				rf[1] = fy1[i];
				rf[2] = fz1[i];
				rt[0] = tx1[i];
				rt[1] = ty1[i];
				rt[2] = tz1[i];

				Mat66 Cf = plucker(Ef, rf);
				Mat66 Ct = plucker(Et, rt);
				//crosproduct...
				Mat61 rff;
				Mat61 rtt;
				Mat61 rffn;
				Mat61 rttn;
				
				rff[0][0] = fx[i];
				rff[1][0] = fy[i];
				rff[2][0] = fz[i];
				rff[3][0] = fox[i];
				rff[4][0] = foy[i];
				rff[5][0] = foz[i];

				rtt[0][0] = tx[i];
				rtt[1][0] = ty[i];
				rtt[2][0] = tz[i];
				rtt[3][0] = tox[i];
				rtt[4][0] = toy[i];
				rtt[5][0] = toz[i];

				rffn = Cf*rff;
				rttn = Ct*rtt;

				
			 fx2[i]=rffn[0][0];
			 fy2[i]=rffn[1][0];
			 fz2[i]=rffn[2][0];
			 fox2[i]=rffn[3][0];
			 foy2[i] = rffn[4][0];
			 foz2[i] = rffn[5][0];

			 tx2[i] = rttn[0][0];
			 ty2[i] = rttn[1][0];
			 tz2[i] = rttn[2][0];
			 tox2[i] = rttn[3][0];
			 toy2[i] = rttn[4][0];
			 toz2[i] = rttn[5][0];
			}

			//Detect cases of dof///


	
	cout << "" << endl;
	cout << "/////////////////////////////////////////////////////////////////" << endl;
	cout << "//////////////START THE DoF RESAMPLER Second phase////////////////////////////" << endl;
	cout << "/////////////////////////////////////////////////////////////////" << endl;
	cout << "" << endl;
	//0dof
	if (count == sizer && count1 == sizer && count2 == sizer && count3 == sizer && count4 == sizer && count5 == sizer){ cout << "Error fix geometry!!!" << endl; }

	//1-2 dof
	if (count == sizer && count1 == sizer && count2 == sizer && count3 == sizer && count4 == sizer && count5 != sizer){ cout << "the final resample of dof for this model is 1 rotational dof in z-axis" << endl; if (sizerr < 2){ fx2 = 0; fy2 = 0; fz2 = 0; fox2 = 0; foy2 = 0; } }
	if (count == sizer && count1 == sizer && count2 == sizer && count3 == sizer && count4 != sizer && count5 != sizer){ cout << "the final resample of dof for this model is 2 rotational dof in z-axis and y-axis " << endl; if (sizerr < 2){ fx2 = 0; fy2 = 0; fz2 = 0; fox2 = 0;  } }
	if (count == sizer && count1 == sizer && count2 == sizer && count3 != sizer && count4 == sizer && count5 != sizer){ cout << "the final resample of dof for this model is 2 rotational dof in z-axis and x-axis" << endl;  if (sizerr < 2){ fx2 = 0; fy2 = 0; fz2 = 0; foy2 = 0; } }
	if (count == sizer && count1 == sizer && count2 != sizer && count3 == sizer && count4 == sizer && count5 != sizer){ cout << "the final resample of dof for this model is 1 rotational dof in z-axis and 1 translation dof in z-axis" << endl;  if (sizerr < 2){ fx2 = 0; fy2 = 0; fox2 = 0; foy2 = 0;  } }
	if (count == sizer && count1 != sizer && count2 == sizer && count3 == sizer && count4 == sizer && count5 != sizer){ cout << "the final resample of dof for this model is 1 rotational dof in z-axis and 1 translation dof in x-axis" << endl;  if (sizerr < 2){  fy2 = 0; fz2 = 0; fox2 = 0; foy2 = 0; } }
	if (count != sizer && count1 == sizer && count2 == sizer && count3 == sizer && count4 == sizer && count5 != sizer){ cout << "the final resample of dof for this model is 1 rotational dof in z-axis and 1 translation dof in y-axis" << endl;  if (sizerr < 2){ fx2 = 0;  fz2 = 0; fox2 = 0; foy2 = 0;  } }
	
	if (count == sizer && count1 == sizer && count2 == sizer && count3 == sizer && count4 != sizer && count5 == sizer){ cout << "the final resample of dof for this model is 1 rotational dof in y-axis" << endl;  if (sizerr < 2){ fx2 = 0; fy2 = 0; fz2 = 0; fox2 = 0;  foz2 = 0; } }
	if (count == sizer && count1 == sizer && count2 == sizer && count3 != sizer && count4 != sizer && count5 == sizer){ cout << "the final resample of dof for this model is 2 rotational dof in y-axis and x-axis" << endl;  if (sizerr < 2){ fx2 = 0; fy2 = 0; fz2 = 0;  foz2 = 0; } }
	if (count == sizer && count1 == sizer && count2 != sizer && count3 == sizer && count4 != sizer && count5 == sizer){ cout << "the final resample of dof for this model is 1 rotational dof in y-axis and 1 translation dof in z-axis" << endl;  if (sizerr < 2){ fx2 = 0; fy2 = 0;  fox2 = 0;  foz2 = 0; } }
	if (count == sizer && count1 != sizer && count2 == sizer && count3 == sizer && count4 != sizer && count5 == sizer){ cout << "the final resample of dof for this model is 1 rotational dof in y-axis and 1 translation dof in y-axis" << endl;  if (sizerr < 2){ fx2 = 0;  fz2 = 0; fox2 = 0;  foz2 = 0; } }
	if (count != sizer && count1 == sizer && count2 == sizer && count3 == sizer && count4 != sizer && count5 == sizer){ cout << "the final resample of dof for this model is 1 rotational dof in y-axis and 1 translation dof in x-axis" << endl;  if (sizerr < 2){  fy2 = 0; fz2 = 0; fox2 = 0;  foz2 = 0; } }
	
	if (count == sizer && count1 == sizer && count2 == sizer && count3 != sizer && count4 == sizer && count5 == sizer){ cout << "the final resample of dof for this model is 1 rotational dof in x-axis" << endl; if (sizerr < 2){ fy2 = 0; fz2 = 0; fox2 = 0; foy2; foz2 = 0; } }
	if (count == sizer && count1 == sizer && count2 != sizer && count3 != sizer && count4 == sizer && count5 == sizer){ cout << "the final resample of dof for this model is 1 rotational dof in x-axis and 1 translation dof in z-axis" << endl;  if (sizerr < 2){ fx2 = 0; fy2 = 0;   foy2 = 0; foz2 = 0; } }
	if (count == sizer && count1 != sizer && count2 == sizer && count3 != sizer && count4 == sizer && count5 == sizer){ cout << "the final resample of dof for this model is 1 rotational dof in x-axis and 1 translation dof in y-axis" << endl;  if (sizerr < 2){ fx2 = 0;  fz2 = 0;  foy2 = 0; foz2 = 0; } }
	if (count != sizer && count1 == sizer && count2 == sizer && count3 != sizer && count4 == sizer && count5 == sizer){ cout << "the final resample of dof for this model is 1 rotational dof in x-axis and 1 translation dof in x-axis" << endl;  if (sizerr < 2){  fy2 = 0; fz2 = 0;  foy2 = 0; foz2 = 0; } }
	
	if (count == sizer && count1 == sizer && count2 != sizer && count3 == sizer && count4 == sizer && count5 == sizer){ cout << "the final resample of dof for this model is 1 translation dof in z-axis " << endl;  if (sizerr < 2){ fx2 = 0; fy2 = 0;  fox2 = 0; foy2 = 0; foz2 = 0; } }
	if (count == sizer && count1 != sizer && count2 != sizer && count3 == sizer && count4 == sizer && count5 == sizer){ cout << "the final resample of dof for this model is 2 translation dof in z-axis and in y-axis" << endl;  if (sizerr < 2){ fx2 = 0; fox2 = 0; foy2 = 0; foz2 = 0; } }
	if (count != sizer && count1 == sizer && count2 != sizer && count3 == sizer && count4 == sizer && count5 == sizer){ cout << "the final resample of dof for this model is 2 translation dof in z-axis and in x-axis" << endl;  if (sizerr < 2){  fy2 = 0;  fox2 = 0; foy2 = 0; foz2 = 0; } }
	
	if (count == sizer && count1 != sizer && count2 == sizer && count3 == sizer && count4 == sizer && count5 == sizer){ cout << "the final resample of dof for this model is 1 translation dof in y-axis " << endl;  if (sizerr < 2){ fx2 = 0;  fz2 = 0; fox2 = 0; foy2 = 0; foz2 = 0; } }
	if (count != sizer && count1 != sizer && count2 == sizer && count3 == sizer && count4 == sizer && count5 == sizer){ cout << "the final resample of dof for this model is 2 translation dof in y-axis and x-axis" << endl;  if (sizerr < 2){ fz2 = 0; fox2 = 0; foy2 = 0; foz2 = 0; } }
	
	if (count != sizer && count1 == sizer && count2 == sizer && count3 == sizer && count4 == sizer && count5 == sizer){ cout << "the final resample of dof for this model is 1 translation dof in x-axis" << endl;  if (sizerr < 2){  fy2 = 0; fz2 = 0; fox2 = 0; foy2 = 0; foz2 = 0; } }
	//22//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	
	//3-4 dof

	if (count == sizer && count1 == sizer && count2 == sizer && count3 != sizer && count4 != sizer && count5 != sizer){ cout << "the final resample of dof for this model is 3 rotational dof in z-x-y-axis" << endl;  if (sizerr < 2){ fx2 = 0; fy2 = 0; fz2 = 0; } }
	if (count == sizer && count1 == sizer && count2 != sizer && count3 == sizer && count4 != sizer && count5 != sizer){ cout << "the final resample of dof for this model is 2 rotational dof in z-axis, y-axis and 1 translation dof in z-axis" << endl;  if (sizerr < 2){ fx2 = 0; fy2 = 0;  fox2 = 0;  } }
	if (count == sizer && count1 != sizer && count2 == sizer && count3 == sizer && count4 != sizer && count5 != sizer){ cout << "the final resample of dof for this model is 2 rotational dof in z-axis, y-axis and 1 translation dof in y-axis" << endl;  if (sizerr< 2){ fx2 = 0;  fz2 = 0; fox2 = 0;  } }
	if (count != sizer && count1 == sizer && count2 == sizer && count3 == sizer && count4 != sizer && count5 != sizer){ cout << "the final resample of dof for this model is 2 rotational dof in z-axis, y-axis and 1 translation dof in x-axis" << endl;  if (sizerr < 2){  fy2 = 0; fz2 = 0; fox2 = 0;  } }
	/*4*/ if (count == sizer && count1 == sizer && count2 != sizer && count3 != sizer && count4 != sizer && count5 != sizer){ cout << "the final resample of dof for this model is 3 rotational dof in z-x-y-axis and 1 translation dof in z-axis" << endl;  if (sizerr < 2){ fx2 = 0; fy2 = 0;   } }
	if (count == sizer && count1 != sizer && count2 == sizer && count3 != sizer && count4 != sizer && count5 != sizer){ cout << "the final resample of dof for this model is 3 rotational dof in z-x-y-axis and 1 translation dof in x-axis" << endl;  if (sizerr < 2){  fy2 = 0; fz2 = 0;  } }
	if (count != sizer && count1 == sizer && count2 == sizer && count3 != sizer && count4 != sizer && count5 != sizer){ cout << "the final resample of dof for this model is 3 rotational dof in z-x-y-axis and 1 translation dof in y-axis" << endl;  if (sizerr < 2){ fx2 = 0;  fz2 = 0;  } }
	

	if (count == sizer && count1 == sizer && count2 != sizer && count3 != sizer && count4 != sizer && count5 == sizer){ cout << "the final resample of dof for this model is 2 rotational dof in y-axis, x-axis and 1 translation dof in z-axis" << endl;  if (sizerr < 2){ fx2 = 0; fy2 = 0;   foz2 = 0; } }
	if (count == sizer && count1 != sizer && count2 == sizer && count3 != sizer && count4 != sizer && count5 == sizer){ cout << "the final resample of dof for this model is 2 rotational dof in y-axis, x-axis and 1 translation dof in y-axis" << endl;  if (sizerr < 2){ fx2 = 0;  fz2 = 0;  foz2 = 0; } }
	if (count != sizer && count1 == sizer && count2 == sizer && count3 != sizer && count4 != sizer && count5 == sizer){ cout << "the final resample of dof for this model is 2 rotational dof in y-axis, x-axis and 1 translation dof in x-axis" << endl;  if (sizerr < 2){fy2 = 0; fz2 = 0;  foz2 = 0; } }
	/*4*/ if (count == sizer && count1 != sizer && count2 != sizer && count3 != sizer && count4 != sizer && count5 == sizer){ cout << "the final resample of dof for this model is 2 rotational dof in y-axis, x-axis and 2 translation dof in z-axis, y-axis" << endl;  if (sizerr < 2){ fx2 = 0;   foz2 = 0; } }
	if (count != sizer && count1 == sizer && count2 != sizer && count3 != sizer && count4 != sizer && count5 == sizer){ cout << "the final resample of dof for this model is 2 rotational dof in y-axis, x-axis and 2 translation dof in z-axis, x-axis" << endl;  if (sizerr < 2){  fy2 = 0;   foz2 = 0; } }
	if (count != sizer && count1 != sizer && count2 == sizer && count3 != sizer && count4 != sizer && count5 == sizer){ cout << "the final resample of dof for this model is 2 rotational dof in y-axis, x-axis and 2 translation dof in y-axis, x-axis" << endl;  if (sizerr < 2){  fz2 = 0;  foz2 = 0; } }

	if (count == sizer && count1 == sizer && count2 != sizer && count3 != sizer && count4 == sizer && count5 != sizer){ cout << "the final resample of dof for this model is 2 rotational dof in z-axis, x-axis and 1 translation dof in z-axis" << endl;  if (sizerr < 2){ fx2 = 0; fy2 = 0;   foy2 = 0;  } }
	if (count == sizer && count1 != sizer && count2 == sizer && count3 != sizer && count4 == sizer && count5 != sizer){ cout << "the final resample of dof for this model is 2 rotational dof in z-axis, x-axis and 1 translation dof in y-axis" << endl;  if (sizerr < 2){ fx2 = 0; fz2 = 0;  foy2 = 0;  } }
	if (count != sizer && count1 == sizer && count2 == sizer && count3 != sizer && count4 == sizer && count5 != sizer){ cout << "the final resample of dof for this model is 2 rotational dof in z-axis, x-axis and 1 translation dof in x-axis" << endl;  if (sizerr < 2){  fy2 = 0; fz2 = 0;  foy2 = 0;  } }
	/*4*/ if (count == sizer && count1 != sizer && count2 != sizer && count3 != sizer && count4 == sizer && count5 != sizer){ cout << "the final resample of dof for this model is 2 rotational dof in z-axis, x-axis and 2 translation dof in z-axis, y-axis" << endl;  if (sizerr < 2){ fx2 = 0;   foy2 = 0;  } }
	if (count != sizer && count1 == sizer && count2 != sizer && count3 != sizer && count4 == sizer && count5 != sizer){ cout << "the final resample of dof for this model is 2 rotational dof in z-axis, x-axis and 2 translation dof in z-axis, x-axis" << endl;  if (sizerr < 2){  fy2 = 0;   foy2 = 0;  } }
	if (count != sizer && count1 != sizer && count2 == sizer && count3 != sizer && count4 == sizer && count5 != sizer){ cout << "the final resample of dof for this model is 2 rotational dof in z-axis, x-axis and 2 translation dof in y-axis, x-axis" << endl;  if (sizerr < 2){  fz2 = 0;  foy2 = 0;  } }

	if (count == sizer && count1 != sizer && count2 != sizer && count3 != sizer && count4 == sizer && count5 == sizer){ cout << "the final resample of dof for this model is 1 rotational dof in  x-axis and 2 translation dof in z-axis, y-axis" << endl;  if (sizerr < 2){ fx2 = 0;   foy2 = 0; foz2 = 0; } }
	if (count != sizer && count1 == sizer && count2 != sizer && count3 != sizer && count4 == sizer && count5 == sizer){ cout << "the final resample of dof for this model is 1 rotational dof in  x-axis and 2 translation dof in z-axis, x-axis" << endl; if (sizerr < 2){  fy2 = 0;   foy2 = 0; foz2 = 0; } }
	if (count != sizer && count1 != sizer && count2 == sizer && count3 != sizer && count4 == sizer && count5 == sizer){ cout << "the final resample of dof for this model is 1 rotational dof in  x-axis and 2 translation dof in y-axis, x-axis" << endl; if (sizerr < 2){  fz2 = 0;  foy2 = 0; foz2 = 0; } }

	/*4*/ if (count != sizer && count1 != sizer && count2 != sizer && count3 != sizer && count4 == sizer && count5 == sizer){ cout << "the final resample of dof for this model is 1 rotational dof  x-axis and 3 translation dof in z-axis, y-axis,x-axis" << endl; if (sizerr < 2){  foy2 = 0; foz2 = 0; } }
	/*4*/ if (count == sizer && count1 != sizer && count2 != sizer && count3 == sizer && count4 != sizer && count5 != sizer){ cout << "the final resample of dof for this model is 2 rotational dof in z-axis, y-axis and 2 translation dof in z-axis, y-axis" << endl;  if (sizerr < 2){  fy2 = 0; fz2 = 0; fox2 = 0; } }
	if (count != sizer && count1 == sizer && count2 != sizer && count3 == sizer && count4 != sizer && count5 != sizer){ cout << "the final resample of dof for this model is 2 rotational dof in z-axis, y-axis and 2 translation dof in z-axis, x-axis" << endl;  if (sizerr < 2){ fx2 = 0;  fz2 = 0; fox2 = 0; } }
	if (count != sizer && count1 != sizer && count2 == sizer && count3 == sizer && count4 != sizer && count5 != sizer){ cout << "the final resample of dof for this model is 2 rotational dof in z-axis, y-axis and 2 translation dof in y-axis, x-axis" << endl;  if (sizerr < 2){ fx2 = 0; fy2 = 0; fox2 = 0;  } }


	if (count == sizer && count1 != sizer && count2 != sizer && count3 == sizer && count4 != sizer && count5 == sizer){ cout << "the final resample of dof for this model is 1 rotational dof in  y-axis and 2 translation dof in z-axis, y-axis" << endl; if (sizerr < 2){ fx2 = 0;  fox2 = 0;  foz2 = 0; } }
	if (count != sizer && count1 == sizer && count2 != sizer && count3 == sizer && count4 != sizer && count5 == sizer){ cout << "the final resample of dof for this model is 1 rotational dof in  y-axis and 2 translation dof in z-axis, x-axis" << endl;  if (sizerr < 2){  fy2 = 0;  fox2 = 0;  foz2 = 0; } }
	/*4*/ if (count != sizer && count1 != sizer && count2 != sizer && count3 == sizer && count4 != sizer && count5 == sizer){ cout << "the final resample of dof for this model is 1 rotational dof  y-axis and 3 translation dof in z-axis, y-axis,x-axis" << endl;  if (sizerr < 2){  fox2 = 0; foz2 = 0; } }
	if (count != sizer && count1 != sizer && count2 == sizer && count3 == sizer && count4 != sizer && count5 == sizer){ cout << "the final resample of dof for this model is 1 rotational dof in  y-axis and 2 translation dof in y-axis, x-axis" << endl;  if (sizerr < 2){  fz2 = 0; fox2 = 0;  foz2 = 0; } }

	if (count != sizer && count1 != sizer && count2 == sizer && count3 == sizer && count4 == sizer && count5 != sizer){ cout << "the final resample of dof for this model is 1 rotational dof in  z-axis and 2 translation dof in y-axis, x-axis" << endl;  if (sizerr < 2){  fz2 = 0; fox2 = 0; foy2 = 0;  } }
	if (count == sizer && count1 != sizer && count2 != sizer && count3 == sizer && count4 == sizer && count5 != sizer){ cout << "the final resample of dof for this model is 1 rotational dof in  z-axis and 2 translation dof in z-axis, y-axis" << endl;  if (sizerr < 2){ fx2 = 0;  fox2 = 0; foy2 = 0; } }
	if (count != sizer && count1 == sizer && count2 != sizer && count3 == sizer && count4 == sizer && count5 != sizer){ cout << "the final resample of dof for this model is 1 rotational dof in  z-axis and 2 translation dof in z-axis, x-axis" << endl;  if (sizerr < 2){  fy2 = 0;  fox2 = 0; foy2 = 0;  } }
	/*4*/ if (count != sizer && count1 != sizer && count2 != sizer && count3 == sizer && count4 == sizer && count5 != sizer){ cout << "the final resample of dof for this model is 1 rotational dof  z-axis and 3 translation dof in z-axis, y-axis,x-axis" << endl;  if (sizerr < 2){  fox2 = 0; foy2 = 0;  } }


	if (count != sizer && count1 != sizer && count2 != sizer && count3 == sizer && count4 == sizer && count5 == sizer){ cout << "the final resample of dof for this model is 3 translation dof in z-x-y-axis" << endl;  if (sizer < 2){  fox2 = 0; foy2 = 0; foz2 = 0; } }

	//35//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//5dof//

	if (count == sizer && count1 != sizer && count2 != sizer && count3 != sizer && count4 != sizer && count5 != sizer){ cout << "the final resample of dof for this model is 3 rotational dof in z-x-y axis and 2 translation dof in z-y axis" << endl;  if (sizerr < 2){ fx2 = 0; } }
	if (count != sizer && count1 == sizer && count2 != sizer && count3 != sizer && count4 != sizer && count5 != sizer){ cout << "the final resample of dof for this model is 3 rotational dof in z-x-y axis and 2 translation dof in z-x axis" << endl;  if (sizerr < 2){ fy2 = 0; } }
	if (count != sizer && count1 != sizer && count2 == sizer && count3 != sizer && count4 != sizer && count5 != sizer){ cout << "the final resample of dof for this model is 3 rotational dof in z-x-y axis and 2 translation dof in y-x axis" << endl;  if (sizerr < 2){fz2 = 0; } }
	
	if (count != sizer && count1 != sizer && count2 != sizer && count3 == sizer && count4 != sizer && count5 != sizer){ cout << "the final resample of dof for this model is 3 translation dof in z-x-y axis and 2 rotation dof in z-y axis" << endl;  if (sizerr < 2){ fox2 = 0;  } }
	if (count != sizer && count1 != sizer && count2 != sizer && count3 != sizer && count4 == sizer && count5 != sizer){ cout << "the final resample of dof for this model is 3 translation dof in z-x-y axis and 2 rotation dof in z-x axis" << endl;  if (sizerr < 2){ foy2 = 0; } }
	if (count != sizer && count1 != sizer && count2 != sizer && count3 != sizer && count4 != sizer && count5 == sizer){ cout << "the final resample of dof for this model is 3 translation dof in z-x-y axis and 2 rotation dof in y-x axis" << endl;  if (sizerr < 2){ foz2 = 0; } }
	
	
//6dof//
	if (count != sizer && count1 != sizer && count2 != sizer && count3 != sizer && count4 != sizer && count5 != sizer){ cout << "the final resample of dof for this model is 3 rotational and 3translation dof " << endl;  }

	//64//
	////////////destruct the quad vectors for each of the bodies...assume fix the dof which it will change in FEBio analysis and resample the forces and motion values of them../////////////
	///For the rotation dof///
	//Used the theorem paraller for compute the new moment base the rotation and negative vector so stay in the same point//
	//moment[x] = moment[0] + (force[y] * pointz - force[z] * pointy);
	//moment[y] = moment[1] + (force[z] * pointx - force[x] * pointz);
	//moment[z] = moment[2] + (force[x] * pointy - force[y] * pointx);

	}
	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	///////////////////////////////////////////////////////////////////
	////////////// 0 DOF TRANSLATION/////////////////////////////////////
	/////////////////////////////////////////////////////////////////
if (behavior == 2){


		cout << "" << endl;
		cout << "/////////////////////////////////////////////////////////////////" << endl;
		cout << "//////////////START THE DoF RESAMPLER First phase////////////////////////////" << endl;
		cout << "/////////////////////////////////////////////////////////////////" << endl;
		cout << "" << endl;
	///////////////////////////////////////////////////////////////////
	////////////// 1 DOF ROTATION/////////////////////////////////////
	/////////////////////////////////////////////////////////////////
	//1
	Mat31 rf, rt;
	Mat33	Ef, Et;
	if (count == sizer && count1 == sizer && count2 == sizer && count3 == sizer && count4 == sizer && count5 != sizer){
		for (int i = 0; i < sizerr; ++i){

			Mat33	Ef = Tait_Bryan(-fox1(i), -foy1(i), 0);
			Mat33	Et = Tait_Bryan(-tox1(i), -toy1(i), 0);
			// from RïyFerthestone page 22 for zero translation we have rotation for both forces and moments so:

			rf[0] = -fx1[i];
			rf[1] = -fy1[i];
			rf[2] = -fz1[i];
			rt[0] = -tx1[i];
			rt[1] = -ty1[i];
			rt[2] = -tz1[i];

			

			Mat66 Cf = plucker(Ef, rf);
			Mat66 Ct = plucker(Et, rt);
			//crosproduct...
			Mat61 rff;
			Mat61 rtt;
			Mat61 rffn;
			Mat61 rttn;

			rff[0][0] = fx[i];
			rff[1][0] = fy[i];
			rff[2][0] = fz[i];
			rff[3][0] = fox[i];
			rff[4][0] = foy[i];
			rff[5][0] = foz[i];

			rtt[0][0] = tx[i];
			rtt[1][0] = ty[i];
			rtt[2][0] = tz[i];
			rtt[3][0] = tox[i];
			rtt[4][0] = toy[i];
			rtt[5][0] = toz[i];

			rffn = Cf*rff;
			rttn = Ct*rtt;


			fx2[i] = rffn[0][0];
			fy2[i] = rffn[1][0];
			fz2[i] = rffn[2][0];
			fox2[i] = rffn[3][0];
			foy2[i] = rffn[4][0];
			foz2[i] = rffn[5][0];

			tx2[i] = rttn[0][0];
			ty2[i] = rttn[1][0];
			tz2[i] = rttn[2][0];
			tox2[i] = rttn[3][0];
			toy2[i] = rttn[4][0];
			toz2[i] = rttn[5][0];
			
			fx1(i) = tx1(i) = 0;
			fy1(i) = ty1(i) = 0;
			fz1(i) = tz1(i) = 0;
			fox1(i) = tox1(i) = 0;
			foy1(i) = toy1(i) = 0;

			if (sizerr < 2){
				fx2(i) = tx2(i) = 0;
				fy2(i) = ty2(i) = 0;
				fz2(i) = tz2(i) = 0;
				fox2(i) = tox2(i) = 0;
				foy2(i) = toy2(i) = 0;
			}
		}
		cout << "For this simulation we will use one rotation DoF in z axis" << endl;

	}
	//2
	if (count == sizer && count1 == sizer && count2 == sizer && count3 == sizer && count5 == sizer  && count4 != sizer){
		for (int i = 0; i < sizerr; ++i){
			Mat33	Ef = Tait_Bryan(-fox1(i), 0, -foz1(i));
			Mat33	Et = Tait_Bryan(-tox1(i), 0, -toz1(i));
			// from RFerdestone page 22 for zero translation we have rotation for both forces and moments so:

			rf[0] = -fx1[i];
			rf[1] = -fy1[i];
			rf[2] = -fz1[i];
			rt[0] = -tx1[i];
			rt[1] = -ty1[i];
			rt[2] = -tz1[i];

			
			Mat66 Cf = plucker(Ef, rf);
			Mat66 Ct = plucker(Et, rt);
			//crosproduct...
			Mat61 rff;
			Mat61 rtt;
			Mat61 rffn;
			Mat61 rttn;

			rff[0][0] = fx[i];
			rff[1][0] = fy[i];
			rff[2][0] = fz[i];
			rff[3][0] = fox[i];
			rff[4][0] = foy[i];
			rff[5][0] = foz[i];

			rtt[0][0] = tx[i];
			rtt[1][0] = ty[i];
			rtt[2][0] = tz[i];
			rtt[3][0] = tox[i];
			rtt[4][0] = toy[i];
			rtt[5][0] = toz[i];

			rffn = Cf*rff;
			rttn = Ct*rtt;


			fx2[i] = rffn[0][0];
			fy2[i] = rffn[1][0];
			fz2[i] = rffn[2][0];
			fox2[i] = rffn[3][0];
			foy2[i] = rffn[4][0];
			foz2[i] = rffn[5][0];

			tx2[i] = rttn[0][0];
			ty2[i] = rttn[1][0];
			tz2[i] = rttn[2][0];
			tox2[i] = rttn[3][0];
			toy2[i] = rttn[4][0];
			toz2[i] = rttn[5][0];

			fx1(i) = tx1(i) = 0;
			fy1(i) = ty1(i) = 0;
			fz1(i) = tz1(i) = 0;
			fox1(i) = tox1(i) = 0;
			foz1(i) = toz1(i) = 0;
			
			if (sizerr < 2){
				fx2(i) = tx2(i) = 0;
				fy2(i) = ty2(i) = 0;
				fz2(i) = tz2(i) = 0;
				fox2(i) = tox2(i) = 0;
				foz2(i) = toz2(i) = 0;
			}

		}
		cout << "For this simulation we will use one rotation DoF in y axis" << endl;

	}
	//3    //MINE CASE////
	if (count == sizer && count1 == sizer && count2 == sizer && count4 == sizer && count5 == sizer  && count3 != sizer){
		for (int i = 0; i < sizerr; ++i){
			Mat33	Ef = Tait_Bryan(0, -foy1(i), -foz1(i));
			Mat33	Et = Tait_Bryan(0, -toy1(i), -toz1(i));
			// from RFerdestone page 22 for zero translation we have rotation for both forces and moments so:

			rf[0] = -fx1[i];
			rf[1] = -fy1[i];
			rf[2] = -fz1[i];
			rt[0] = -tx1[i];
			rt[1] = -ty1[i];
			rt[2] = -tz1[i];

			
			
			Mat66 Cf = plucker(Ef, rf);
			Mat66 Ct = plucker(Et, rt);
			//crosproduct...
			Mat61 rff;
			Mat61 rtt;
			Mat61 rffn;
			Mat61 rttn;

			rff[0][0] = fx[i];
			rff[1][0] = fy[i];
			rff[2][0] = fz[i];
			rff[3][0] = fox[i];
			rff[4][0] = foy[i];
			rff[5][0] = foz[i];

			rtt[0][0] = tx[i];
			rtt[1][0] = ty[i];
			rtt[2][0] = tz[i];
			rtt[3][0] = tox[i];
			rtt[4][0] = toy[i];
			rtt[5][0] = toz[i];

			rffn = Cf*rff;
			rttn = Ct*rtt;


			fx2[i] = rffn[0][0];
			fy2[i] = rffn[1][0];
			fz2[i] = rffn[2][0];
			fox2[i] = rffn[3][0];
			foy2[i] = rffn[4][0];
			foz2[i] = rffn[5][0];

			tx2[i] = rttn[0][0];
			ty2[i] = rttn[1][0];
			tz2[i] = rttn[2][0];
			tox2[i] = rttn[3][0];
			toy2[i] = rttn[4][0];
			toz2[i] = rttn[5][0];
			
			fx1(i) = tx1(i) = 0;
			fy1(i) = ty1(i) = 0;
			fz1(i) = tz1(i) = 0;
			foy1(i) = toy1(i) = 0;
			foz1(i) = toz1(i) = 0;
			
			if (sizerr < 2){
				fx2(i) = tx2(i) = 0;
				fy2(i) = ty2(i) = 0;
				fz2(i) = tz2(i) = 0;
				foy2(i) = toy2(i) = 0;
				foz2(i) = toz2(i) = 0;
			}

		}
		cout << "For this simulation we will use one rotation DoF in x axis" << endl;

	}

	///////////////////////////////////////////////////////////////////
	////////////// 2 DOF ROTATION/////////////////////////////////////
	////////////////////////////////////////////////////////////////////////
	//4
	if (count == sizer && count1 == sizer && count2 == sizer && count3 == sizer  && count4 != sizer  && count5 != sizer){
		for (int i = 0; i < sizerr; ++i){
			Mat33	Ef = Tait_Bryan(-fox1(i), 0, 0);
			Mat33	Et = Tait_Bryan(-tox1(i), 0, 0);
			// from RFerdestone page 22 for zero translation we have rotation for both forces and moments so:
			rf[0] = -fx1[i];
			rf[1] = -fy1[i];
			rf[2] = -fz1[i];
			rt[0] = -tx1[i];
			rt[1] = -ty1[i];
			rt[2] = -tz1[i];

			
			Mat66 Cf = plucker(Ef, rf);
			Mat66 Ct = plucker(Et, rt);
			//crosproduct...
			Mat61 rff;
			Mat61 rtt;
			Mat61 rffn;
			Mat61 rttn;

			rff[0][0] = fx[i];
			rff[1][0] = fy[i];
			rff[2][0] = fz[i];
			rff[3][0] = fox[i];
			rff[4][0] = foy[i];
			rff[5][0] = foz[i];

			rtt[0][0] = tx[i];
			rtt[1][0] = ty[i];
			rtt[2][0] = tz[i];
			rtt[3][0] = tox[i];
			rtt[4][0] = toy[i];
			rtt[5][0] = toz[i];

			rffn = Cf*rff;
			rttn = Ct*rtt;


			fx2[i] = rffn[0][0];
			fy2[i] = rffn[1][0];
			fz2[i] = rffn[2][0];
			fox2[i] = rffn[3][0];
			foy2[i] = rffn[4][0];
			foz2[i] = rffn[5][0];

			tx2[i] = rttn[0][0];
			ty2[i] = rttn[1][0];
			tz2[i] = rttn[2][0];
			tox2[i] = rttn[3][0];
			toy2[i] = rttn[4][0];
			toz2[i] = rttn[5][0];

			fx1(i) = tx1(i) = 0;
			fy1(i) = ty1(i) = 0;
			fz1(i) = tz1(i) = 0;
			fox1(i) = tox1(i) = 0;
			
			if (sizerr < 2){
				fx2(i) = tx2(i) = 0;
				fy2(i) = ty2(i) = 0;
				fz2(i) = tz2(i) = 0;
				fox2(i) = tox2(i) = 0;
			}

		}
		cout << "For this simulation we will use two rotation DoF in z and y axis" << endl;

	}
	//5
	if (count == sizer && count1 == sizer && count2 == sizer && count5 == sizer  && count3 != sizer  && count4 != sizer){
		for (int i = 0; i < sizerr; ++i){
			Mat33	Ef = Tait_Bryan(0, 0, -foz1(i));
			Mat33	Et = Tait_Bryan(0, 0, -toz1(i));
			// from RFerdestone page 22 for zero translation we have rotation for both forces and moments so:
			rf[0] = -fx1[i];
			rf[1] = -fy1[i];
			rf[2] = -fz1[i];
			rt[0] = -tx1[i];
			rt[1] = -ty1[i];
			rt[2] = -tz1[i];

			
			Mat66 Cf = plucker(Ef, rf);
			Mat66 Ct = plucker(Et, rt);
			//crosproduct...
			Mat61 rff;
			Mat61 rtt;
			Mat61 rffn;
			Mat61 rttn;

			rff[0][0] = fx[i];
			rff[1][0] = fy[i];
			rff[2][0] = fz[i];
			rff[3][0] = fox[i];
			rff[4][0] = foy[i];
			rff[5][0] = foz[i];

			rtt[0][0] = tx[i];
			rtt[1][0] = ty[i];
			rtt[2][0] = tz[i];
			rtt[3][0] = tox[i];
			rtt[4][0] = toy[i];
			rtt[5][0] = toz[i];

			rffn = Cf*rff;
			rttn = Ct*rtt;


			fx2[i] = rffn[0][0];
			fy2[i] = rffn[1][0];
			fz2[i] = rffn[2][0];
			fox2[i] = rffn[3][0];
			foy2[i] = rffn[4][0];
			foz2[i] = rffn[5][0];

			tx2[i] = rttn[0][0];
			ty2[i] = rttn[1][0];
			tz2[i] = rttn[2][0];
			tox2[i] = rttn[3][0];
			toy2[i] = rttn[4][0];
			toz2[i] = rttn[5][0];

			fx1(i) = tx1(i) = 0;
			fy1(i) = ty1(i) = 0;
			fz1(i) = tz1(i) = 0;
			foz1(i) = toz1(i) = 0;
			
			if (sizerr < 2){
				fx2(i) = tx2(i) = 0;
				fy2(i) = ty2(i) = 0;
				fz2(i) = tz2(i) = 0;
				foz2(i) = toz2(i) = 0;
			}

		}
		cout << "For this simulation we will use two rotation DoF in x and y axis" << endl;

	}
	//6
	if (count == sizer && count1 == sizer && count2 == sizer && count4 == sizer  && count5 != sizer  && count3 != sizer){
		for (int i = 0; i < sizerr; ++i){
			Mat33	Ef = Tait_Bryan(0, -foy1(i), 0);
			Mat33	Et = Tait_Bryan(0, -toy1(i), 0);
			// from RFerdestone page 22 for zero translation we have rotation for both forces and moments so:
			rf[0] = -fx1[i];
			rf[1] = -fy1[i];
			rf[2] = -fz1[i];
			rt[0] = -tx1[i];
			rt[1] = -ty1[i];
			rt[2] = -tz1[i];

			
			Mat66 Cf = plucker(Ef, rf);
			Mat66 Ct = plucker(Et, rt);
			//crosproduct...
			Mat61 rff;
			Mat61 rtt;
			Mat61 rffn;
			Mat61 rttn;

			rff[0][0] = fx[i];
			rff[1][0] = fy[i];
			rff[2][0] = fz[i];
			rff[3][0] = fox[i];
			rff[4][0] = foy[i];
			rff[5][0] = foz[i];

			rtt[0][0] = tx[i];
			rtt[1][0] = ty[i];
			rtt[2][0] = tz[i];
			rtt[3][0] = tox[i];
			rtt[4][0] = toy[i];
			rtt[5][0] = toz[i];

			rffn = Cf*rff;
			rttn = Ct*rtt;


			fx2[i] = rffn[0][0];
			fy2[i] = rffn[1][0];
			fz2[i] = rffn[2][0];
			fox2[i] = rffn[3][0];
			foy2[i] = rffn[4][0];
			foz2[i] = rffn[5][0];

			tx2[i] = rttn[0][0];
			ty2[i] = rttn[1][0];
			tz2[i] = rttn[2][0];
			tox2[i] = rttn[3][0];
			toy2[i] = rttn[4][0];
			toz2[i] = rttn[5][0];

			fx1(i) = tx1(i) = 0;
			fy1(i) = ty1(i) = 0;
			fz1(i) = tz1(i) = 0;
			foy1(i) = toy1(i) = 0;
			if (sizerr < 2){
				fx2(i) = tx2(i) = 0;
				fy2(i) = ty2(i) = 0;
				fz2(i) = tz2(i) = 0;
				foy2(i) = toy2(i) = 0;
			}

		}
		cout << "For this simulation we will use two rotation DoF in z and x axis" << endl;

	}

	///////////////////////////////////////////////////////////////////
	////////////// 3 DOF ROTATION/////////////////////////////////////
	/////////////////////////////////////////////////////////////////

	//7
	if (count == sizer && count1 == sizer && count2 == sizer  && count3 != sizer  && count5 != sizer  && count4 != sizer){
		for (int i = 0; i < sizerr; ++i){
			Mat33	Ef = Tait_Bryan(0, 0, 0);
			Mat33	Et = Tait_Bryan(0, 0, 0);
			// from RFerdestone page 22 for zero translation we have rotation for both forces and moments so:
			rf[0] = -fx1[i];
			rf[1] = -fy1[i];
			rf[2] = -fz1[i];
			rt[0] = -tx1[i];
			rt[1] = -ty1[i];
			rt[2] = -tz1[i];

			
			Mat66 Cf = plucker(Ef, rf);
			Mat66 Ct = plucker(Et, rt);
			//crosproduct...
			Mat61 rff;
			Mat61 rtt;
			Mat61 rffn;
			Mat61 rttn;

			rff[0][0] = fx[i];
			rff[1][0] = fy[i];
			rff[2][0] = fz[i];
			rff[3][0] = fox[i];
			rff[4][0] = foy[i];
			rff[5][0] = foz[i];

			rtt[0][0] = tx[i];
			rtt[1][0] = ty[i];
			rtt[2][0] = tz[i];
			rtt[3][0] = tox[i];
			rtt[4][0] = toy[i];
			rtt[5][0] = toz[i];

			rffn = Cf*rff;
			rttn = Ct*rtt;


			fx2[i] = rffn[0][0];
			fy2[i] = rffn[1][0];
			fz2[i] = rffn[2][0];
			fox2[i] = rffn[3][0];
			foy2[i] = rffn[4][0];
			foz2[i] = rffn[5][0];

			tx2[i] = rttn[0][0];
			ty2[i] = rttn[1][0];
			tz2[i] = rttn[2][0];
			tox2[i] = rttn[3][0];
			toy2[i] = rttn[4][0];
			toz2[i] = rttn[5][0];

			fx1(i) = tx1(i) = 0;
			fy1(i) = ty1(i) = 0;
			fz1(i) = tz1(i) = 0;
			if (sizerr < 2){
			fx2(i) = tx2(i) = 0;
			fy2(i) = ty2(i) = 0;
			fz2(i) = tz2(i) = 0;
			}

		}
		cout << "For this simulation we will use three rotation DoF in z x y axis" << endl;

	}
	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


	///////////////////////////////////////////////////////////////////
	//////////////  TRANSLATION/////////////////////////////////////
	/////////////////////////////////////////////////////////////////



	///////////////////////////////////////////////////////////////////
	////////////// 3 DOF TRANSLATION/////////////////////////////////////
	/////////////////////////////////////////////////////////////////
	//8
	if (count != sizer && count1 != sizer && count2 != sizer){

		if (count3 == sizer && count4 == sizer && count5 == sizer){
			for (int i = 0; i < sizerr; ++i){
				Mat33	Ef = Tait_Bryan(-fox[i], -foy1(i), -foz[i]);
				Mat33	Et = Tait_Bryan(-tox[i], -toy1(i), -toz[i]);
				// from RFerdestone page 22 for zero translation we have rotation for both forces and moments so:
				//the forces stay as they are
				rf[0] = 0;
				rf[1] = 0;
				rf[2] = 0;
				rt[0] = 0;
				rt[1] = 0;
				rt[2] = 0;

				Mat66 Cf = plucker(Ef, rf);
				Mat66 Ct = plucker(Et, rt);
				//crosproduct...
				Mat61 rff;
				Mat61 rtt;
				Mat61 rffn;
				Mat61 rttn;

				rff[0][0] = fx[i];
				rff[1][0] = fy[i];
				rff[2][0] = fz[i];
				rff[3][0] = fox[i];
				rff[4][0] = foy[i];
				rff[5][0] = foz[i];

				rtt[0][0] = tx[i];
				rtt[1][0] = ty[i];
				rtt[2][0] = tz[i];
				rtt[3][0] = tox[i];
				rtt[4][0] = toy[i];
				rtt[5][0] = toz[i];

				rffn = Cf*rff;
				rttn = Ct*rtt;


				fx2[i] = rffn[0][0];
				fy2[i] = rffn[1][0];
				fz2[i] = rffn[2][0];
				fox2[i] = rffn[3][0];
				foy2[i] = rffn[4][0];
				foz2[i] = rffn[5][0];

				tx2[i] = rttn[0][0];
				ty2[i] = rttn[1][0];
				tz2[i] = rttn[2][0];
				tox2[i] = rttn[3][0];
				toy2[i] = rttn[4][0];
				toz2[i] = rttn[5][0];

				fox1(i) = tox1(i) = 0;
				foy1(i) = toy1(i) = 0;
				foz1(i) = toz1(i) = 0;
				if (sizerr < 2){
					fox2(i) = tox2(i) = 0;
					foy2(i) = toy2(i) = 0;
					foz2(i) = toz2(i) = 0;
				}
			}
			cout << "For this simulation we will use three translation DOFin z x y axis" << endl;

		}

		///////////////////////////////////////////////////////////////////
		////////////// 1 DOF ROTATION-3 DOF TRANSLATION /////////////////////////////////////
		/////////////////////////////////////////////////////////////////
		//9
		if (count3 == sizer && count4 == sizer && count5 != sizer){
			for (int i = 0; i < sizerr; ++i){
				Mat33	Ef = Tait_Bryan(-fox1(i), -foy1(i), 0);
				Mat33	Et = Tait_Bryan(-tox1(i), -toy1(i), 0);
				// from RFerdestone page 22 for zero translation we have rotation for both forces and moments so:
				rf[0] = 0;
				rf[1] = 0;
				rf[2] = 0;
				rt[0] = 0;
				rt[1] = 0;
				rt[2] = 0;

				Mat66 Cf = plucker(Ef, rf);
				Mat66 Ct = plucker(Et, rt);
				//crosproduct...
				Mat61 rff;
				Mat61 rtt;
				Mat61 rffn;
				Mat61 rttn;

				rff[0][0] = fx[i];
				rff[1][0] = fy[i];
				rff[2][0] = fz[i];
				rff[3][0] = fox[i];
				rff[4][0] = foy[i];
				rff[5][0] = foz[i];

				rtt[0][0] = tx[i];
				rtt[1][0] = ty[i];
				rtt[2][0] = tz[i];
				rtt[3][0] = tox[i];
				rtt[4][0] = toy[i];
				rtt[5][0] = toz[i];

				rffn = Cf*rff;
				rttn = Ct*rtt;


				fx2[i] = rffn[0][0];
				fy2[i] = rffn[1][0];
				fz2[i] = rffn[2][0];
				fox2[i] = rffn[3][0];
				foy2[i] = rffn[4][0];
				foz2[i] = rffn[5][0];

				tx2[i] = rttn[0][0];
				ty2[i] = rttn[1][0];
				tz2[i] = rttn[2][0];
				tox2[i] = rttn[3][0];
				toy2[i] = rttn[4][0];
				toz2[i] = rttn[5][0];

				fox1(i) = tox1(i) = 0;
				foy1(i) = toy1(i) = 0;
				if (sizerr < 2){
					fox2(i) = tox2(i) = 0;
					foy2(i) = toy2(i) = 0;
				}
			}
			cout << "For this simulation we will use three translation DOFin z x y axis and one rotaion DoF in z axis" << endl;

		}
		//10
		if (count3 == sizer && count5 == sizer && count4 != sizer){
			for (int i = 0; i < sizerr; ++i){
				Mat33	Ef = Tait_Bryan(-fox1(i), 0, -foz1(i));
				Mat33	Et = Tait_Bryan(-tox1(i), 0, -toz1(i));
				// from RFerdestone page 22 for zero translation we have rotation for both forces and moments so:
				rf[0] = 0;
				rf[1] = 0;
				rf[2] = 0;
				rt[0] = 0;
				rt[1] = 0;
				rt[2] = 0;

				Mat66 Cf = plucker(Ef, rf);
				Mat66 Ct = plucker(Et, rt);
				//crosproduct...
				Mat61 rff;
				Mat61 rtt;
				Mat61 rffn;
				Mat61 rttn;

				rff[0][0] = fx[i];
				rff[1][0] = fy[i];
				rff[2][0] = fz[i];
				rff[3][0] = fox[i];
				rff[4][0] = foy[i];
				rff[5][0] = foz[i];

				rtt[0][0] = tx[i];
				rtt[1][0] = ty[i];
				rtt[2][0] = tz[i];
				rtt[3][0] = tox[i];
				rtt[4][0] = toy[i];
				rtt[5][0] = toz[i];

				rffn = Cf*rff;
				rttn = Ct*rtt;


				fx2[i] = rffn[0][0];
				fy2[i] = rffn[1][0];
				fz2[i] = rffn[2][0];
				fox2[i] = rffn[3][0];
				foy2[i] = rffn[4][0];
				foz2[i] = rffn[5][0];

				tx2[i] = rttn[0][0];
				ty2[i] = rttn[1][0];
				tz2[i] = rttn[2][0];
				tox2[i] = rttn[3][0];
				toy2[i] = rttn[4][0];
				toz2[i] = rttn[5][0];

				fox1(i) = tox1(i) = 0;
				foz1(i) = toz1(i) = 0;
				if (sizerr < 2){
					fox2(i) = tox2(i) = 0;
					foz2(i) = toz2(i) = 0;
				}
			}
			cout << "For this simulation we will use three translation DOFin z x y axis and one rotaion DoF in y axis" << endl;

		}
		//11
		if (count4 == sizer && count5 == sizer && count3 != sizer){
			for (int i = 0; i < sizerr; ++i){
				Mat33	Ef = Tait_Bryan(0, -foy1(i), -foz1(i));
				Mat33	Et = Tait_Bryan(0, -toy1(i), -toz1(i));
				// from RFerdestone page 22 for zero translation we have rotation for both forces and moments so:
				rf[0] = 0;
				rf[1] = 0;
				rf[2] = 0;
				rt[0] = 0;
				rt[1] = 0;
				rt[2] = 0;

				Mat66 Cf = plucker(Ef, rf);
				Mat66 Ct = plucker(Et, rt);
				//crosproduct...
				Mat61 rff;
				Mat61 rtt;
				Mat61 rffn;
				Mat61 rttn;

				rff[0][0] = fx[i];
				rff[1][0] = fy[i];
				rff[2][0] = fz[i];
				rff[3][0] = fox[i];
				rff[4][0] = foy[i];
				rff[5][0] = foz[i];

				rtt[0][0] = tx[i];
				rtt[1][0] = ty[i];
				rtt[2][0] = tz[i];
				rtt[3][0] = tox[i];
				rtt[4][0] = toy[i];
				rtt[5][0] = toz[i];

				rffn = Cf*rff;
				rttn = Ct*rtt;


				fx2[i] = rffn[0][0];
				fy2[i] = rffn[1][0];
				fz2[i] = rffn[2][0];
				fox2[i] = rffn[3][0];
				foy2[i] = rffn[4][0];
				foz2[i] = rffn[5][0];

				tx2[i] = rttn[0][0];
				ty2[i] = rttn[1][0];
				tz2[i] = rttn[2][0];
				tox2[i] = rttn[3][0];
				toy2[i] = rttn[4][0];
				toz2[i] = rttn[5][0];

				foy1(i) = toy1(i) = 0;
				foz1(i) = toz1(i) = 0;
				if (sizerr < 2){
					foy2(i) = toy2(i) = 0;
					foz2(i) = toz2(i) = 0;
				}
			}
			cout << "For this simulation we will use three translation DOFin z x y axis and one rotaion DoF in x axis" << endl;

		}


		///////////////////////////////////////////////////////////////////
		////////////// 2 DOF ROTATION-3 DOF TRANSLATION /////////////////////////////////////
		/////////////////////////////////////////////////////////////////
		//12
		if (count3 == sizer && count4 != sizer && count5 != sizer){
			for (int i = 0; i < sizerr; ++i){
				Mat33	Ef = Tait_Bryan(-fox1(i), 0, 0);
				Mat33	Et = Tait_Bryan(-tox1(i), 0, 0);
				rf[0] = 0;
				rf[1] = 0;
				rf[2] = 0;
				rt[0] = 0;
				rt[1] = 0;
				rt[2] = 0;

				Mat66 Cf = plucker(Ef, rf);
				Mat66 Ct = plucker(Et, rt);
				//crosproduct...
				Mat61 rff;
				Mat61 rtt;
				Mat61 rffn;
				Mat61 rttn;

				rff[0][0] = fx[i];
				rff[1][0] = fy[i];
				rff[2][0] = fz[i];
				rff[3][0] = fox[i];
				rff[4][0] = foy[i];
				rff[5][0] = foz[i];

				rtt[0][0] = tx[i];
				rtt[1][0] = ty[i];
				rtt[2][0] = tz[i];
				rtt[3][0] = tox[i];
				rtt[4][0] = toy[i];
				rtt[5][0] = toz[i];

				rffn = Cf*rff;
				rttn = Ct*rtt;


				fx2[i] = rffn[0][0];
				fy2[i] = rffn[1][0];
				fz2[i] = rffn[2][0];
				fox2[i] = rffn[3][0];
				foy2[i] = rffn[4][0];
				foz2[i] = rffn[5][0];

				tx2[i] = rttn[0][0];
				ty2[i] = rttn[1][0];
				tz2[i] = rttn[2][0];
				tox2[i] = rttn[3][0];
				toy2[i] = rttn[4][0];
				toz2[i] = rttn[5][0];

				fox1(i) = tox1(i) = 0;
				if (sizerr < 2){
					fox2(i) = tox2(i) = 0;
				}
			}
			cout << "For this simulation we will use three translation DOFin z x y axis and two rotaion DoF in y and z axis" << endl;

		}
		//13
		if (count5 == sizer && count4 != sizer && count3 != sizer){
			for (int i = 0; i < sizerr; ++i){
				Mat33	Ef = Tait_Bryan(0, 0, -foz1(i));
				Mat33	Et = Tait_Bryan(0, 0, -toz1(i));
				rf[0] = 0;
				rf[1] = 0;
				rf[2] = 0;
				rt[0] = 0;
				rt[1] = 0;
				rt[2] = 0;

				Mat66 Cf = plucker(Ef, rf);
				Mat66 Ct = plucker(Et, rt);
				//crosproduct...
				Mat61 rff;
				Mat61 rtt;
				Mat61 rffn;
				Mat61 rttn;

				rff[0][0] = fx[i];
				rff[1][0] = fy[i];
				rff[2][0] = fz[i];
				rff[3][0] = fox[i];
				rff[4][0] = foy[i];
				rff[5][0] = foz[i];

				rtt[0][0] = tx[i];
				rtt[1][0] = ty[i];
				rtt[2][0] = tz[i];
				rtt[3][0] = tox[i];
				rtt[4][0] = toy[i];
				rtt[5][0] = toz[i];

				rffn = Cf*rff;
				rttn = Ct*rtt;

				fx2[i] = rffn[0][0];
				fy2[i] = rffn[1][0];
				fz2[i] = rffn[2][0];
				fox2[i] = rffn[3][0];
				foy2[i] = rffn[4][0];
				foz2[i] = rffn[5][0];

				tx2[i] = rttn[0][0];
				ty2[i] = rttn[1][0];
				tz2[i] = rttn[2][0];
				tox2[i] = rttn[3][0];
				toy2[i] = rttn[4][0];
				toz2[i] = rttn[5][0];

				foz1(i) = toz1(i) = 0;
				if (sizerr < 2){
					foz2(i) = toz2(i) = 0;
				}
			}
			cout << "For this simulation we will use three translation DOFin z x y axis and two rotaion DoF in y and x axis" << endl;

		}
		//14
		if (count4 == sizer && count5 != sizer && count3 != sizer){
			for (int i = 0; i < sizerr; ++i){
				Mat33	Ef = Tait_Bryan(0, -foy1(i), 0);
				Mat33	Et = Tait_Bryan(0, -toy1(i), 0);
				rf[0] = 0;
				rf[1] = 0;
				rf[2] = 0;
				rt[0] = 0;
				rt[1] = 0;
				rt[2] = 0;

				Mat66 Cf = plucker(Ef, rf);
				Mat66 Ct = plucker(Et, rt);
				//crosproduct...
				Mat61 rff;
				Mat61 rtt;
				Mat61 rffn;
				Mat61 rttn;

				rff[0][0] = fx[i];
				rff[1][0] = fy[i];
				rff[2][0] = fz[i];
				rff[3][0] = fox[i];
				rff[4][0] = foy[i];
				rff[5][0] = foz[i];

				rtt[0][0] = tx[i];
				rtt[1][0] = ty[i];
				rtt[2][0] = tz[i];
				rtt[3][0] = tox[i];
				rtt[4][0] = toy[i];
				rtt[5][0] = toz[i];

				rffn = Cf*rff;
				rttn = Ct*rtt;


				fx2[i] = rffn[0][0];
				fy2[i] = rffn[1][0];
				fz2[i] = rffn[2][0];
				fox2[i] = rffn[3][0];
				foy2[i] = rffn[4][0];
				foz2[i] = rffn[5][0];

				tx2[i] = rttn[0][0];
				ty2[i] = rttn[1][0];
				tz2[i] = rttn[2][0];
				tox2[i] = rttn[3][0];
				toy2[i] = rttn[4][0];
				toz2[i] = rttn[5][0];

				foy1(i) = toy1(i) = 0;

				if (sizerr < 2){
					foy2(i) = toy2(i) = 0;
				}



			}
			cout << "For this simulation we will use three translation DOFin z x y axis and two rotaion DoF in z and x axis" << endl;

		}
	}

	////////////////////////////////////////////////////////////////////////2/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	/////////////////////////////////////////////////////

	///////////////////////////////////////////////////////////////////
	////////////// 2 DOF TRANSLATION /////////////////////////////////////
	/////////////////////////////////////////////////////////////////





	//15

	///////////////////////////////////////////////////////////////////
	////////////// 2 DOF TRANSLATION x y /////////////////////////////////////
	/////////////////////////////////////////////////////////////////
	if (count != sizer && count1 != sizer && count2 == sizer){

		if (count3 == sizer && count4 == sizer && count5 == sizer){
			for (int i = 0; i < sizerr; ++i){
				Mat33	Ef = Tait_Bryan(-fox1(i), -foy1(i), -foz1(i));
				Mat33	Et = Tait_Bryan(-tox1(i), -toy1(i), -toz1(i));
				rf[0] = 0;
				rf[1] = 0;
				rf[2] = -fz1[i];
				rt[0] = 0;
				rt[1] = 0;
				rt[2] = -tz1[i];
				
				Mat66 Cf = plucker(Ef, rf);
				Mat66 Ct = plucker(Et, rt);
				//crosproduct...
				Mat61 rff;
				Mat61 rtt;
				Mat61 rffn;
				Mat61 rttn;

				rff[0][0] = fx[i];
				rff[1][0] = fy[i];
				rff[2][0] = fz[i];
				rff[3][0] = fox[i];
				rff[4][0] = foy[i];
				rff[5][0] = foz[i];

				rtt[0][0] = tx[i];
				rtt[1][0] = ty[i];
				rtt[2][0] = tz[i];
				rtt[3][0] = tox[i];
				rtt[4][0] = toy[i];
				rtt[5][0] = toz[i];

				rffn = Cf*rff;
				rttn = Ct*rtt;


				fx2[i] = rffn[0][0];
				fy2[i] = rffn[1][0];
				fz2[i] = rffn[2][0];
				fox2[i] = rffn[3][0];
				foy2[i] = rffn[4][0];
				foz2[i] = rffn[5][0];

				tx2[i] = rttn[0][0];
				ty2[i] = rttn[1][0];
				tz2[i] = rttn[2][0];
				tox2[i] = rttn[3][0];
				toy2[i] = rttn[4][0];
				toz2[i] = rttn[5][0];// from RFerdestone page 22 for zero translation we have rotation for both forces and moments so:
				//the forces stay as they are

				fox1(i) = tox1(i) = 0;
				foy1(i) = toy1(i) = 0;
				foz1(i) = toz1(i) = 0;
				fz1(i) = tz1(i) = 0;
				if (sizerr < 2){
					fz2(i) = tz2(i) = 0;
					fox2(i) = tox2(i) = 0;
					foy2(i) = toy2(i) = 0;
					foz2(i) = toz2(i) = 0;
					fz2(i) = tz2(i) = 0;
				}
			}
			cout << "For this simulation we will use two translation DOFin  x y axis" << endl;

		}

		///////////////////////////////////////////////////////////////////
		////////////// 1 DOF ROTATION-2 DOF TRANSLATION /////////////////////////////////////
		/////////////////////////////////////////////////////////////////
		//16
		if (count3 == sizer && count4 == sizer  && count5 != sizer){
			for (int i = 0; i < sizerr; ++i){
				Mat33	Ef = Tait_Bryan(-fox1(i), -foy1(i), 0);
				Mat33	Et = Tait_Bryan(-tox1(i), -toy1(i), 0);
				// from RFerdestone page 22 for zero translation we have rotation for both forces and moments so:

				rf[0] = 0;
				rf[1] = 0;
				rf[2] = -fz1[i];
				rt[0] = 0;
				rt[1] = 0;
				rt[2] = -tz1[i];

				
				Mat66 Cf = plucker(Ef, rf);
				Mat66 Ct = plucker(Et, rt);
				//crosproduct...
				Mat61 rff;
				Mat61 rtt;
				Mat61 rffn;
				Mat61 rttn;

				rff[0][0] = fx[i];
				rff[1][0] = fy[i];
				rff[2][0] = fz[i];
				rff[3][0] = fox[i];
				rff[4][0] = foy[i];
				rff[5][0] = foz[i];

				rtt[0][0] = tx[i];
				rtt[1][0] = ty[i];
				rtt[2][0] = tz[i];
				rtt[3][0] = tox[i];
				rtt[4][0] = toy[i];
				rtt[5][0] = toz[i];

				rffn = Cf*rff;
				rttn = Ct*rtt;


				fx2[i] = rffn[0][0];
				fy2[i] = rffn[1][0];
				fz2[i] = rffn[2][0];
				fox2[i] = rffn[3][0];
				foy2[i] = rffn[4][0];
				foz2[i] = rffn[5][0];

				tx2[i] = rttn[0][0];
				ty2[i] = rttn[1][0];
				tz2[i] = rttn[2][0];
				tox2[i] = rttn[3][0];
				toy2[i] = rttn[4][0];
				toz2[i] = rttn[5][0];

				fox1(i) = tox1(i) = 0;
				foy1(i) = toy1(i) = 0;
				if (sizerr < 2){
					fox2(i) = tox2(i) = 0;
					foy2(i) = toy2(i) = 0;
					fz2(i) = tz2(i) = 0;
				}
				fz1(i) = tz1(i) = 0;
				

			}
			cout << "For this simulation we will use  two translation DOFin  x y axis and one rotaion DoF in z axis" << endl;

		}
		//17
		if (count3 == sizer && count5 == sizer  && count4 != sizer){
			for (int i = 0; i < sizerr; ++i){
				Mat33	Ef = Tait_Bryan(-fox1(i), 0, -foz1(i));
				Mat33	Et = Tait_Bryan(-tox1(i), 0, -toz1(i));
				// from RFerdestone page 22 for zero translation we have rotation for both forces and moments so:

				rf[0] = 0;
				rf[1] = 0;
				rf[2] = fz1[i];
				rt[0] = 0;
				rt[1] = 0;
				rt[2] = tz1[i];
				

				Mat66 Cf = plucker(Ef, rf);
				Mat66 Ct = plucker(Et, rt);
				//crosproduct...
				Mat61 rff;
				Mat61 rtt;
				Mat61 rffn;
				Mat61 rttn;

				rff[0][0] = fx[i];
				rff[1][0] = fy[i];
				rff[2][0] = fz[i];
				rff[3][0] = fox[i];
				rff[4][0] = foy[i];
				rff[5][0] = foz[i];

				rtt[0][0] = tx[i];
				rtt[1][0] = ty[i];
				rtt[2][0] = tz[i];
				rtt[3][0] = tox[i];
				rtt[4][0] = toy[i];
				rtt[5][0] = toz[i];

				rffn = Cf*rff;
				rttn = Ct*rtt;


				fx2[i] = rffn[0][0];
				fy2[i] = rffn[1][0];
				fz2[i] = rffn[2][0];
				fox2[i] = rffn[3][0];
				foy2[i] = rffn[4][0];
				foz2[i] = rffn[5][0];

				tx2[i] = rttn[0][0];
				ty2[i] = rttn[1][0];
				tz2[i] = rttn[2][0];
				tox2[i] = rttn[3][0];
				toy2[i] = rttn[4][0];
				toz2[i] = rttn[5][0];
				if (sizerr < 2){
					fox2(i) = tox2(i) = 0;
					foz2(i) = toz2(i) = 0;
					fz2(i) = tz2(i) = 0;
				}
				fz1(i) = tz1(i) = 0;
				

			}
			cout << "For this simulation we will use  two translation DOFin  x y axis and one rotaion DoF in y axis" << endl;

		}
		//18
		if (count4 == sizer && count5 == sizer  && count3 != sizer){
			for (int i = 0; i < sizerr; ++i){
				Mat33	Ef = Tait_Bryan(0, -foy1(i), -foz1(i));
				Mat33	Et = Tait_Bryan(0, -toy1(i), -toz1(i));
				// from RFerdestone page 22 for zero translation we have rotation for both forces and moments so:

				rf[0] = 0;
				rf[1] = 0;
				rf[2] = -fz1[i];
				rt[0] = 0;
				rt[1] = 0;
				rt[2] = -tz1[i];

				Mat66 Cf = plucker(Ef, rf);
				Mat66 Ct = plucker(Et, rt);
				//crosproduct...
				Mat61 rff;
				Mat61 rtt;
				Mat61 rffn;
				Mat61 rttn;

				rff[0][0] = fx[i];
				rff[1][0] = fy[i];
				rff[2][0] = fz[i];
				rff[3][0] = fox[i];
				rff[4][0] = foy[i];
				rff[5][0] = foz[i];

				rtt[0][0] = tx[i];
				rtt[1][0] = ty[i];
				rtt[2][0] = tz[i];
				rtt[3][0] = tox[i];
				rtt[4][0] = toy[i];
				rtt[5][0] = toz[i];

				rffn = Cf*rff;
				rttn = Ct*rtt;


				fx2[i] = rffn[0][0];
				fy2[i] = rffn[1][0];
				fz2[i] = rffn[2][0];
				fox2[i] = rffn[3][0];
				foy2[i] = rffn[4][0];
				foz2[i] = rffn[5][0];

				tx2[i] = rttn[0][0];
				ty2[i] = rttn[1][0];
				tz2[i] = rttn[2][0];
				tox2[i] = rttn[3][0];
				toy2[i] = rttn[4][0];
				toz2[i] = rttn[5][0];

				foy1(i) = toy1(i) = 0;
				foz1(i) = toz1(i) = 0;
				if (sizerr < 2){
					foy2(i) = toy2(i) = 0;
					foz2(i) = toz2(i) = 0;
					fz2(i) = tz2(i) = 0;
				}
				fz1(i) = tz1(i) = 0;
				
			}
			cout << "For this simulation we will use  two translation DOFin  x y axis axis and one rotaion DoF in x axis" << endl;

		}


		///////////////////////////////////////////////////////////////////
		////////////// 2 DOF ROTATION-2 DOF TRANSLATION /////////////////////////////////////
		/////////////////////////////////////////////////////////////////
		//19
		if (count3 == sizer  && count4 != sizer  && count5 != sizer){
			for (int i = 0; i < sizerr; ++i){
				Mat33	Ef = Tait_Bryan(-fox1(i), 0, 0);
				Mat33	Et = Tait_Bryan(-tox1(i), 0, 0);
				// from RFerdestone page 22 for zero translation we have rotation for both forces and moments so:
				rf[0] = 0;
				rf[1] = 0;
				rf[2] = -fz1[i];
				rt[0] = 0;
				rt[1] = 0;
				rt[2] = -tz1[i];

				Mat66 Cf = plucker(Ef, rf);
				Mat66 Ct = plucker(Et, rt);
				//crosproduct...
				Mat61 rff;
				Mat61 rtt;
				Mat61 rffn;
				Mat61 rttn;

				rff[0][0] = fx[i];
				rff[1][0] = fy[i];
				rff[2][0] = fz[i];
				rff[3][0] = fox[i];
				rff[4][0] = foy[i];
				rff[5][0] = foz[i];

				rtt[0][0] = tx[i];
				rtt[1][0] = ty[i];
				rtt[2][0] = tz[i];
				rtt[3][0] = tox[i];
				rtt[4][0] = toy[i];
				rtt[5][0] = toz[i];

				rffn = Cf*rff;
				rttn = Ct*rtt;


				fx2[i] = rffn[0][0];
				fy2[i] = rffn[1][0];
				fz2[i] = rffn[2][0];
				fox2[i] = rffn[3][0];
				foy2[i] = rffn[4][0];
				foz2[i] = rffn[5][0];

				tx2[i] = rttn[0][0];
				ty2[i] = rttn[1][0];
				tz2[i] = rttn[2][0];
				tox2[i] = rttn[3][0];
				toy2[i] = rttn[4][0];
				toz2[i] = rttn[5][0];

				fz1(i) = tz1(i) = 0;
				
				fox1(i) = tox1(i) = 0;
				if (sizerr < 2){
					fox2(i) = tox2(i) = 0;
					fz2(i) = tz2(i) = 0;
				}
			}
			cout << "For this simulation we will use  two translation DOFin  x y axis and two rotaion DoF in y and z axis" << endl;

		}
		//20
		if (count5 == sizer  && count4 != sizer  && count3 != sizer){
			for (int i = 0; i < sizerr; ++i){
				Mat33	Ef = Tait_Bryan(0, 0, -foz1(i));
				Mat33	Et = Tait_Bryan(0, 0, -toz1(i));
				// from RFerdestone page 22 for zero translation we have rotation for both forces and moments so:
				rf[0] = 0;
				rf[1] = 0;
				rf[2] = -fz1[i];
				rt[0] = 0;
				rt[1] = 0;
				rt[2] = -tz1[i];

				
				Mat66 Cf = plucker(Ef, rf);
				Mat66 Ct = plucker(Et, rt);
				//crosproduct...
				Mat61 rff;
				Mat61 rtt;
				Mat61 rffn;
				Mat61 rttn;

				rff[0][0] = fx[i];
				rff[1][0] = fy[i];
				rff[2][0] = fz[i];
				rff[3][0] = fox[i];
				rff[4][0] = foy[i];
				rff[5][0] = foz[i];

				rtt[0][0] = tx[i];
				rtt[1][0] = ty[i];
				rtt[2][0] = tz[i];
				rtt[3][0] = tox[i];
				rtt[4][0] = toy[i];
				rtt[5][0] = toz[i];

				rffn = Cf*rff;
				rttn = Ct*rtt;


				fx2[i] = rffn[0][0];
				fy2[i] = rffn[1][0];
				fz2[i] = rffn[2][0];
				fox2[i] = rffn[3][0];
				foy2[i] = rffn[4][0];
				foz2[i] = rffn[5][0];

				tx2[i] = rttn[0][0];
				ty2[i] = rttn[1][0];
				tz2[i] = rttn[2][0];
				tox2[i] = rttn[3][0];
				toy2[i] = rttn[4][0];
				toz2[i] = rttn[5][0];

				fz1(i) = tz1(i) = 0;
				
				foz1(i) = toz1(i) = 0;
				if (sizerr < 2){
					foz2(i) = toz2(i) = 0;
					fz2(i) = tz2(i) = 0;
				}
			}
			cout << "For this simulation we will use  two translation DOFin  x y axis axis and two rotaion DoF in y and x axis" << endl;

		}
		//21
		if (count4 == sizer  && count3 != sizer  && count5 != sizer){
			for (int i = 0; i < sizerr; ++i){
				Mat33	Ef = Tait_Bryan(0, -foy1(i), 0);
				Mat33	Et = Tait_Bryan(0, -toy1(i), 0);
				// from RFerdestone page 22 for zero translation we have rotation for both forces and moments so:
				rf[0] = 0;
				rf[1] = 0;
				rf[2] = -fz1[i];
				rt[0] = 0;
				rt[1] = 0;
				rt[2] = -tz1[i];
				Mat66 Cf = plucker(Ef, rf);
				Mat66 Ct = plucker(Et, rt);
				//crosproduct...
				Mat61 rff;
				Mat61 rtt;
				Mat61 rffn;
				Mat61 rttn;

				rff[0][0] = fx[i];
				rff[1][0] = fy[i];
				rff[2][0] = fz[i];
				rff[3][0] = fox[i];
				rff[4][0] = foy[i];
				rff[5][0] = foz[i];

				rtt[0][0] = tx[i];
				rtt[1][0] = ty[i];
				rtt[2][0] = tz[i];
				rtt[3][0] = tox[i];
				rtt[4][0] = toy[i];
				rtt[5][0] = toz[i];

				rffn = Cf*rff;
				rttn = Ct*rtt;


				fx2[i] = rffn[0][0];
				fy2[i] = rffn[1][0];
				fz2[i] = rffn[2][0];
				fox2[i] = rffn[3][0];
				foy2[i] = rffn[4][0];
				foz2[i] = rffn[5][0];

				tx2[i] = rttn[0][0];
				ty2[i] = rttn[1][0];
				tz2[i] = rttn[2][0];
				tox2[i] = rttn[3][0];
				toy2[i] = rttn[4][0];
				toz2[i] = rttn[5][0];

				fz1(i) = tz1(i) = 0;
				
				foy1(i) = toy1(i) = 0;

				if (sizerr < 2){
					foy2(i) = toy2(i) = 0;
					fz2(i) = tz2(i) = 0;
				}



			}
			cout << "For this simulation we will use two translation DOFin  x y axis and two rotaion DoF in z and x axis" << endl;

		}
	}
	////////////////////////////////////////////////////////////////////////////////////////////////////////////








	///////////////////////////////////////////////////////////////////
	////////////// 2 DOF TRANSLATION  x z /////////////////////////////////////
	/////////////////////////////////////////////////////////////////
	//22
	if (count != sizer && count1 == sizer && count2 != sizer){

		if (count3 == sizer && count4 == sizer && count5 == sizer){
			for (int i = 0; i < sizerr; ++i){
				Mat33	Ef = Tait_Bryan(-fox1(i), -foy1(i), -foz1(i));
				Mat33	Et = Tait_Bryan(-tox1(i), -toy1(i), -toz1(i));
				// from RFerdestone page 22 for zero translation we have rotation for both forces and moments so:
				rf[0] = 0;
				rf[1] =-fy1[i];
				rf[2] = 0;
				rt[0] = 0;
				rt[1] =-ty1[i];
				rt[2] = 0;
				
				Mat66 Cf = plucker(Ef, rf);
				Mat66 Ct = plucker(Et, rt);
				//crosproduct...
				Mat61 rff;
				Mat61 rtt;
				Mat61 rffn;
				Mat61 rttn;

				rff[0][0] = fx[i];
				rff[1][0] = fy[i];
				rff[2][0] = fz[i];
				rff[3][0] = fox[i];
				rff[4][0] = foy[i];
				rff[5][0] = foz[i];

				rtt[0][0] = tx[i];
				rtt[1][0] = ty[i];
				rtt[2][0] = tz[i];
				rtt[3][0] = tox[i];
				rtt[4][0] = toy[i];
				rtt[5][0] = toz[i];

				rffn = Cf*rff;
				rttn = Ct*rtt;


				fx2[i] = rffn[0][0];
				fy2[i] = rffn[1][0];
				fz2[i] = rffn[2][0];
				fox2[i] = rffn[3][0];
				foy2[i] = rffn[4][0];
				foz2[i] = rffn[5][0];

				tx2[i] = rttn[0][0];
				ty2[i] = rttn[1][0];
				tz2[i] = rttn[2][0];
				tox2[i] = rttn[3][0];
				toy2[i] = rttn[4][0];
				toz2[i] = rttn[5][0];//the forces stay as they are

				fox1(i) = tox1(i) = 0;
				foy1(i) = toy1(i) = 0;
				foz1(i) = toz1(i) = 0;
				fy1(i) = ty1(i) = 0;
				if (sizerr < 2){
					fy2(i) = ty2(i) = 0;
					fox2(i) = tox2(i) = 0;
					foy2(i) = toy2(i) = 0;
					foz2(i) = toz2(i) = 0;
				}

			}
			cout << "For this simulation we will use two translation DOFin  x z axis" << endl;

		}
		//moment[x] = moment[0] + (force[y] * pointz - force[z] * pointy);
		//moment[y] = moment[1] + (force[z] * pointx - force[x] * pointz);
		//moment[z] = moment[2] + (force[x] * pointy - force[y] * pointx);
		///////////////////////////////////////////////////////////////////
		////////////// 1 DOF ROTATION-2 DOF TRANSLATION /////////////////////////////////////
		/////////////////////////////////////////////////////////////////
		//23
		if (count3 == sizer && count4 == sizer  && count5 != sizer){
			for (int i = 0; i < sizerr; ++i){
				Mat33	Ef = Tait_Bryan(-fox1(i), -foy1(i), 0);
				Mat33	Et = Tait_Bryan(-tox1(i), -toy1(i), 0);
				// from RFerdestone page 22 for zero translation we have rotation for both forces and moments so:

				rf[0] = 0;
				rf[1] = -fy1[i];
				rf[2] = 0;
				rt[0] = 0;
				rt[1] = -ty1[i];
				rt[2] = 0;
				
				Mat66 Cf = plucker(Ef, rf);
				Mat66 Ct = plucker(Et, rt);
				//crosproduct...
				Mat61 rff;
				Mat61 rtt;
				Mat61 rffn;
				Mat61 rttn;

				rff[0][0] = fx[i];
				rff[1][0] = fy[i];
				rff[2][0] = fz[i];
				rff[3][0] = fox[i];
				rff[4][0] = foy[i];
				rff[5][0] = foz[i];

				rtt[0][0] = tx[i];
				rtt[1][0] = ty[i];
				rtt[2][0] = tz[i];
				rtt[3][0] = tox[i];
				rtt[4][0] = toy[i];
				rtt[5][0] = toz[i];

				rffn = Cf*rff;
				rttn = Ct*rtt;


				fx2[i] = rffn[0][0];
				fy2[i] = rffn[1][0];
				fz2[i] = rffn[2][0];
				fox2[i] = rffn[3][0];
				foy2[i] = rffn[4][0];
				foz2[i] = rffn[5][0];

				tx2[i] = rttn[0][0];
				ty2[i] = rttn[1][0];
				tz2[i] = rttn[2][0];
				tox2[i] = rttn[3][0];
				toy2[i] = rttn[4][0];
				toz2[i] = rttn[5][0];

				fox1(i) = tox1(i) = 0;
				foy1(i) = toy1(i) = 0;
				fy1(i) = ty1(i) = 0;
				if (sizerr < 2){
					fox2(i) = tox2(i) = 0;
					foy2(i) = toy2(i) = 0;
					
					fy2(i) = ty2(i) = 0;
				}
			}
			cout << "For this simulation we will use  two translation DOFin  x z axis and one rotaion DoF in z axis" << endl;

		}
		//24
		if (count3 == sizer && count5 == sizer  && count4 != sizer){
			for (int i = 0; i < sizerr; ++i){
				Mat33	Ef = Tait_Bryan(-fox1(i), 0, -foz1(i));
				Mat33	Et = Tait_Bryan(-tox1(i), 0, -toz1(i));
				// from RFerdestone page 22 for zero translation we have rotation for both forces and moments so:


				rf[0] = 0;
				rf[1] = -fy1[i];
				rf[2] = 0;
				rt[0] = 0;
				rt[1] = -ty1[i];
				rt[2] = 0;
				
				Mat66 Cf = plucker(Ef, rf);
				Mat66 Ct = plucker(Et, rt);
				//crosproduct...
				Mat61 rff;
				Mat61 rtt;
				Mat61 rffn;
				Mat61 rttn;

				rff[0][0] = fx[i];
				rff[1][0] = fy[i];
				rff[2][0] = fz[i];
				rff[3][0] = fox[i];
				rff[4][0] = foy[i];
				rff[5][0] = foz[i];

				rtt[0][0] = tx[i];
				rtt[1][0] = ty[i];
				rtt[2][0] = tz[i];
				rtt[3][0] = tox[i];
				rtt[4][0] = toy[i];
				rtt[5][0] = toz[i];

				rffn = Cf*rff;
				rttn = Ct*rtt;


				fx2[i] = rffn[0][0];
				fy2[i] = rffn[1][0];
				fz2[i] = rffn[2][0];
				fox2[i] = rffn[3][0];
				foy2[i] = rffn[4][0];
				foz2[i] = rffn[5][0];

				tx2[i] = rttn[0][0];
				ty2[i] = rttn[1][0];
				tz2[i] = rttn[2][0];
				tox2[i] = rttn[3][0];
				toy2[i] = rttn[4][0];
				toz2[i] = rttn[5][0];

				fy1(i) = ty1(i) = 0;

				

				fox1(i) = tox1(i) = 0;
				foz1(i) = toz1(i) = 0;
				if (sizerr < 2){
					fox2(i) = tox2(i) = 0;
					foz2(i) = toz2(i) = 0;
					fy2(i) = ty2(i) = 0;
				}
			}
			cout << "For this simulation we will use  two translation DOFin  x z axis and one rotaion DoF in y axis" << endl;

		}
		//25
		if (count4 == sizer && count5 == sizer  && count3 != sizer){
			for (int i = 0; i < sizerr; ++i){
				Mat33	Ef = Tait_Bryan(0, foy1(i), foz1(i));
				Mat33	Et = Tait_Bryan(0, toy1(i), toz1(i));
				// from RFerdestone page 22 for zero translation we have rotation for both forces and moments so:

				rf[0] = 0;
				rf[1] = fy1[i];
				rf[2] = 0;
				rt[0] = 0;
				rt[1] = ty1[i];
				rt[2] = 0;
				Mat66 Cf = plucker(Ef, rf);
				Mat66 Ct = plucker(Et, rt);
				//crosproduct...
				Mat61 rff;
				Mat61 rtt;
				Mat61 rffn;
				Mat61 rttn;

				rff[0][0] = fx[i];
				rff[1][0] = fy[i];
				rff[2][0] = fz[i];
				rff[3][0] = fox[i];
				rff[4][0] = foy[i];
				rff[5][0] = foz[i];

				rtt[0][0] = tx[i];
				rtt[1][0] = ty[i];
				rtt[2][0] = tz[i];
				rtt[3][0] = tox[i];
				rtt[4][0] = toy[i];
				rtt[5][0] = toz[i];

				rffn = Cf*rff;
				rttn = Ct*rtt;


				fx2[i] = rffn[0][0];
				fy2[i] = rffn[1][0];
				fz2[i] = rffn[2][0];
				fox2[i] = rffn[3][0];
				foy2[i] = rffn[4][0];
				foz2[i] = rffn[5][0];

				tx2[i] = rttn[0][0];
				ty2[i] = rttn[1][0];
				tz2[i] = rttn[2][0];
				tox2[i] = rttn[3][0];
				toy2[i] = rttn[4][0];
				toz2[i] = rttn[5][0];

				fy1(i) = ty1(i) = 0;
				

				foy1(i) = toy1(i) = 0;
				foz1(i) = toz1(i) = 0;
				if (sizerr < 2){
					foy2(i) = toy2(i) = 0;
					foz2(i) = toz2(i) = 0;
					fy2(i) = ty2(i) = 0;
				}
			}
			cout << "For this simulation we will use  two translation DOFin  x z axis axis and one rotaion DoF in x axis" << endl;

		}


		///////////////////////////////////////////////////////////////////
		////////////// 2 DOF ROTATION-2 DOF TRANSLATION /////////////////////////////////////
		/////////////////////////////////////////////////////////////////
		//26
		if (count3 == sizer  && count5 != sizer  && count4 != sizer){
			for (int i = 0; i < sizerr; ++i){
				Mat33	Ef = Tait_Bryan(-fox1(i), 0, 0);
				Mat33	Et = Tait_Bryan(-tox1(i), 0, 0);
				// from RFerdestone page 22 for zero translation we have rotation for both forces and moments so:
				rf[0] = 0;
				rf[1] = -fy1[i];
				rf[2] = 0;
				rt[0] = 0;
				rt[1] = -ty1[i];
				rt[2] = 0;
				
				Mat66 Cf = plucker(Ef, rf);
				Mat66 Ct = plucker(Et, rt);
				//crosproduct...
				Mat61 rff;
				Mat61 rtt;
				Mat61 rffn;
				Mat61 rttn;

				rff[0][0] = fx[i];
				rff[1][0] = fy[i];
				rff[2][0] = fz[i];
				rff[3][0] = fox[i];
				rff[4][0] = foy[i];
				rff[5][0] = foz[i];

				rtt[0][0] = tx[i];
				rtt[1][0] = ty[i];
				rtt[2][0] = tz[i];
				rtt[3][0] = tox[i];
				rtt[4][0] = toy[i];
				rtt[5][0] = toz[i];

				rffn = Cf*rff;
				rttn = Ct*rtt;


				fx2[i] = rffn[0][0];
				fy2[i] = rffn[1][0];
				fz2[i] = rffn[2][0];
				fox2[i] = rffn[3][0];
				foy2[i] = rffn[4][0];
				foz2[i] = rffn[5][0];

				tx2[i] = rttn[0][0];
				ty2[i] = rttn[1][0];
				tz2[i] = rttn[2][0];
				tox2[i] = rttn[3][0];
				toy2[i] = rttn[4][0];
				toz2[i] = rttn[5][0];

				fy1(i) = ty1(i) = 0;
				

				fox1(i) = tox1(i) = 0;
				if (sizerr < 2){
					fox2(i) = tox2(i) = 0;
					fy2(i) = ty2(i) = 0;
				}
			}
			cout << "For this simulation we will use  two translation DOFin  x z axis and two rotaion DoF in y and z axis" << endl;

		}
		//27
		if (count5 == sizer  && count4 != sizer  && count3 != sizer){
			for (int i = 0; i < sizerr; ++i){
				Mat33	Ef = Tait_Bryan(0, 0, -foz1(i));
				Mat33	Et = Tait_Bryan(0, 0, -toz1(i));
				// from RFerdestone page 22 for zero translation we have rotation for both forces and moments so:
				rf[0] = 0;
				rf[1] = -fy1[i];
				rf[2] = 0;
				rt[0] = 0;
				rt[1] = -ty1[i];
				rt[2] = 0;
				
				Mat66 Cf = plucker(Ef, rf);
				Mat66 Ct = plucker(Et, rt);
				//crosproduct...
				Mat61 rff;
				Mat61 rtt;
				Mat61 rffn;
				Mat61 rttn;

				rff[0][0] = fx[i];
				rff[1][0] = fy[i];
				rff[2][0] = fz[i];
				rff[3][0] = fox[i];
				rff[4][0] = foy[i];
				rff[5][0] = foz[i];

				rtt[0][0] = tx[i];
				rtt[1][0] = ty[i];
				rtt[2][0] = tz[i];
				rtt[3][0] = tox[i];
				rtt[4][0] = toy[i];
				rtt[5][0] = toz[i];

				rffn = Cf*rff;
				rttn = Ct*rtt;


				fx2[i] = rffn[0][0];
				fy2[i] = rffn[1][0];
				fz2[i] = rffn[2][0];
				fox2[i] = rffn[3][0];
				foy2[i] = rffn[4][0];
				foz2[i] = rffn[5][0];

				tx2[i] = rttn[0][0];
				ty2[i] = rttn[1][0];
				tz2[i] = rttn[2][0];
				tox2[i] = rttn[3][0];
				toy2[i] = rttn[4][0];
				toz2[i] = rttn[5][0];

				fy1(i) = ty1(i) = 0;
			

				foz1(i) = toz1(i) = 0;
				if (sizerr < 2){
					foz2(i) = toz2(i) = 0;
					fy2(i) = ty2(i) = 0;
				}
			}
			cout << "For this simulation we will use  two translation DOFin  x z axis axis and two rotaion DoF in y and x axis" << endl;

		}
		//28
		if (count4 == sizer  && count3 != sizer  && count5 != sizer){
			for (int i = 0; i < sizerr; ++i){
				Mat33	Ef = Tait_Bryan(0, -foy1(i), 0);
				Mat33	Et = Tait_Bryan(0, -toy1(i), 0);
				// from RFerdestone page 22 for zero translation we have rotation for both forces and moments so:
				rf[0] = 0;
				rf[1] = -fy1[i];
				rf[2] = 0;
				rt[0] = 0;
				rt[1] = -ty1[i];
				rt[2] = 0;
				
				Mat66 Cf = plucker(Ef, rf);
				Mat66 Ct = plucker(Et, rt);
				//crosproduct...
				Mat61 rff;
				Mat61 rtt;
				Mat61 rffn;
				Mat61 rttn;

				rff[0][0] = fx[i];
				rff[1][0] = fy[i];
				rff[2][0] = fz[i];
				rff[3][0] = fox[i];
				rff[4][0] = foy[i];
				rff[5][0] = foz[i];

				rtt[0][0] = tx[i];
				rtt[1][0] = ty[i];
				rtt[2][0] = tz[i];
				rtt[3][0] = tox[i];
				rtt[4][0] = toy[i];
				rtt[5][0] = toz[i];

				rffn = Cf*rff;
				rttn = Ct*rtt;


				fx2[i] = rffn[0][0];
				fy2[i] = rffn[1][0];
				fz2[i] = rffn[2][0];
				fox2[i] = rffn[3][0];
				foy2[i] = rffn[4][0];
				foz2[i] = rffn[5][0];

				tx2[i] = rttn[0][0];
				ty2[i] = rttn[1][0];
				tz2[i] = rttn[2][0];
				tox2[i] = rttn[3][0];
				toy2[i] = rttn[4][0];
				toz2[i] = rttn[5][0];


				fy1(i) = ty1(i) = 0;
				//
				foy1(i) = toy1(i) = 0;

				if (sizerr < 2){
					foy2(i) = toy2(i) = 0;
					fy2(i) = ty2(i) = 0;
				}


			}
			cout << "For this simulation we will use two translation DOFin  x y axis and two rotaion DoF in z and x axis" << endl;

		}
	}


	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////








	///////////////////////////////////////////////////////////////////
	////////////// 2 DOF TRANSLATION  y z /////////////////////////////////////
	/////////////////////////////////////////////////////////////////
	//29
	if (count == sizer && count1 != sizer && count2 != sizer){

		if (count3 == sizer && count4 == sizer && count5 == sizer){
			for (int i = 0; i < sizerr; ++i){
				Mat33	Ef = Tait_Bryan(-fox1(i), -foy1(i), -foz1(i));
				Mat33	Et = Tait_Bryan(-tox1(i), -toy1(i), -toz1(i));
				// from RFerdestone page 22 for zero translation we have rotation for both forces and moments so:
				//the forces stay as they are
				rf[0] = -fx1[i];
				rf[1] = 0;
				rf[2] = 0;
				rt[0] = -tx1[i];
				rt[1] = 0;
				rt[2] = 0;
				Mat66 Cf = plucker(Ef, rf);
				Mat66 Ct = plucker(Et, rt);
				//crosproduct...
				Mat61 rff;
				Mat61 rtt;
				Mat61 rffn;
				Mat61 rttn;

				rff[0][0] = fx[i];
				rff[1][0] = fy[i];
				rff[2][0] = fz[i];
				rff[3][0] = fox[i];
				rff[4][0] = foy[i];
				rff[5][0] = foz[i];

				rtt[0][0] = tx[i];
				rtt[1][0] = ty[i];
				rtt[2][0] = tz[i];
				rtt[3][0] = tox[i];
				rtt[4][0] = toy[i];
				rtt[5][0] = toz[i];

				rffn = Cf*rff;
				rttn = Ct*rtt;


				fx2[i] = rffn[0][0];
				fy2[i] = rffn[1][0];
				fz2[i] = rffn[2][0];
				fox2[i] = rffn[3][0];
				foy2[i] = rffn[4][0];
				foz2[i] = rffn[5][0];

				tx2[i] = rttn[0][0];
				ty2[i] = rttn[1][0];
				tz2[i] = rttn[2][0];
				tox2[i] = rttn[3][0];
				toy2[i] = rttn[4][0];
				toz2[i] = rttn[5][0];

				fox1(i) = tox1(i) = 0;
				foy1(i) = toy1(i) = 0;
				foz1(i) = toz1(i) = 0;
				fx1(i) = tx1(i) = 0;
				
				if (sizerr < 2){
					fx2(i) = tx2(i) = 0;
					fox2(i) = tox2(i) = 0;
					foy2(i) = toy2(i) = 0;
					foz2(i) = toz2(i) = 0;
				}

			}
			cout << "For this simulation we will use two translation DOFin  y z axis" << endl;

		}
		//moment[x] = moment[0] + (force[y] * pointz - force[z] * pointy);
		//moment[y] = moment[1] + (force[z] * pointx - force[x] * pointz);
		//moment[z] = moment[2] + (force[x] * pointy - force[y] * pointx);
		///////////////////////////////////////////////////////////////////
		////////////// 1 DOF ROTATION-2 DOF TRANSLATION /////////////////////////////////////
		/////////////////////////////////////////////////////////////////
		//30
		if (count3 == sizer && count4 == sizer && count5 != sizer){
			for (int i = 0; i < sizerr; ++i){
				Mat33	Ef = Tait_Bryan(-fox1(i), -foy1(i), 0);
				Mat33	Et = Tait_Bryan(-tox1(i), -toy1(i), 0);
				// from RFerdestone page 22 for zero translation we have rotation for both forces and moments so:

				rf[0] = -fx1[i];
				rf[1] = 0;
				rf[2] = 0;
				rt[0] = -tx1[i];
				rt[1] = 0;
				rt[2] = 0;
				
				

				fox1(i) = tox1(i) = 0;
				foy1(i) = toy1(i) = 0;
				if (sizerr < 2){
					fox2(i) = tox2(i) = 0;
					fx2(i) = tx2(i) = 0;
					foy2(i) = toy2(i) = 0;
				}
				fx1(i) = tx1(i) = 0;
				//

			}
			cout << "For this simulation we will use  two translation DOFin  y z axis and one rotaion DoF in z axis" << endl;

		}
		//31
		if (count3 == sizer && count5 == sizer && count4 != sizer){
			for (int i = 0; i < sizerr; ++i){
				Mat33	Ef = Tait_Bryan(-fox1(i), 0, -foz1(i));
				Mat33	Et = Tait_Bryan(-tox1(i), 0, -toz1(i));
				// from RFerdestone page 22 for zero translation we have rotation for both forces and moments so:
				rf[0] = -fx1[i];
				rf[1] = 0;
				rf[2] = 0;
				rt[0] = -tx1[i];
				rt[1] = 0;
				rt[2] = 0;
				
				Mat66 Cf = plucker(Ef, rf);
				Mat66 Ct = plucker(Et, rt);
				//crosproduct...
				Mat61 rff;
				Mat61 rtt;
				Mat61 rffn;
				Mat61 rttn;

				rff[0][0] = fx[i];
				rff[1][0] = fy[i];
				rff[2][0] = fz[i];
				rff[3][0] = fox[i];
				rff[4][0] = foy[i];
				rff[5][0] = foz[i];

				rtt[0][0] = tx[i];
				rtt[1][0] = ty[i];
				rtt[2][0] = tz[i];
				rtt[3][0] = tox[i];
				rtt[4][0] = toy[i];
				rtt[5][0] = toz[i];

				rffn = Cf*rff;
				rttn = Ct*rtt;


				fx2[i] = rffn[0][0];
				fy2[i] = rffn[1][0];
				fz2[i] = rffn[2][0];
				fox2[i] = rffn[3][0];
				foy2[i] = rffn[4][0];
				foz2[i] = rffn[5][0];

				tx2[i] = rttn[0][0];
				ty2[i] = rttn[1][0];
				tz2[i] = rttn[2][0];
				tox2[i] = rttn[3][0];
				toy2[i] = rttn[4][0];
				toz2[i] = rttn[5][0];

				fx1(i) = tx1(i) = 0;
				//

				fox1(i) = tox1(i) = 0;
				foz1(i) = toz1(i) = 0;
				if (sizer < 2){
					fox2(i) = tox2(i) = 0;
					foz2(i) = toz2(i) = 0;
					fx2(i) = tx2(i) = 0;
				}
			}
			cout << "For this simulation we will use  two translation DOFin  y z axis and one rotaion DoF in y axis" << endl;

		}
		//32
		if (count4 == sizer && count5 == sizer  && count3 != sizer){
			for (int i = 0; i < sizerr; ++i){
				Mat33	Ef = Tait_Bryan(0, -foy1(i), -foz1(i));
				Mat33	Et = Tait_Bryan(0, -toy1(i), -toz1(i));
				// from RFerdestone page 22 for zero translation we have rotation for both forces and moments so:

				rf[0] = -fx1[i];
				rf[1] = 0;
				rf[2] = 0;
				rt[0] = -tx1[i];
				rt[1] = 0;
				rt[2] = 0;
				
				Mat66 Cf = plucker(Ef, rf);
				Mat66 Ct = plucker(Et, rt);
				//crosproduct...
				Mat61 rff;
				Mat61 rtt;
				Mat61 rffn;
				Mat61 rttn;

				rff[0][0] = fx[i];
				rff[1][0] = fy[i];
				rff[2][0] = fz[i];
				rff[3][0] = fox[i];
				rff[4][0] = foy[i];
				rff[5][0] = foz[i];

				rtt[0][0] = tx[i];
				rtt[1][0] = ty[i];
				rtt[2][0] = tz[i];
				rtt[3][0] = tox[i];
				rtt[4][0] = toy[i];
				rtt[5][0] = toz[i];

				rffn = Cf*rff;
				rttn = Ct*rtt;


				fx2[i] = rffn[0][0];
				fy2[i] = rffn[1][0];
				fz2[i] = rffn[2][0];
				fox2[i] = rffn[3][0];
				foy2[i] = rffn[4][0];
				foz2[i] = rffn[5][0];

				tx2[i] = rttn[0][0];
				ty2[i] = rttn[1][0];
				tz2[i] = rttn[2][0];
				tox2[i] = rttn[3][0];
				toy2[i] = rttn[4][0];
				toz2[i] = rttn[5][0];

				fx1(i) = tx1(i) = 0;
				

				foy1(i) = toy1(i) = 0;
				foz1(i) = toz1(i) = 0;
				if (sizerr < 2){
					foy2(i) = toy2(i) = 0;
					foz2(i) = toz2(i) = 0;
					fx2(i) = tx2(i) = 0;
				}
			}
			cout << "For this simulation we will use  two translation DOFin  y z axis axis and one rotaion DoF in x axis" << endl;

		}


		///////////////////////////////////////////////////////////////////
		////////////// 2 DOF ROTATION-2 DOF TRANSLATION /////////////////////////////////////
		/////////////////////////////////////////////////////////////////
		//33
		if (count3 == sizer && count5 != sizer  && count4 != sizer){
			for (int i = 0; i < sizerr; ++i){
				Mat33	Ef = Tait_Bryan(-fox1(i), 0, 0);
				Mat33	Et = Tait_Bryan(-tox1(i), 0, 0);
				// from RFerdestone page 22 for zero translation we have rotation for both forces and moments so:
				rf[0] = -fx1[i];
				rf[1] = 0;
				rf[2] = 0;
				rt[0] = -tx1[i];
				rt[1] = 0;
				rt[2] = 0;
				
				Mat66 Cf = plucker(Ef, rf);
				Mat66 Ct = plucker(Et, rt);
				//crosproduct...
				Mat61 rff;
				Mat61 rtt;
				Mat61 rffn;
				Mat61 rttn;

				rff[0][0] = fx[i];
				rff[1][0] = fy[i];
				rff[2][0] = fz[i];
				rff[3][0] = fox[i];
				rff[4][0] = foy[i];
				rff[5][0] = foz[i];

				rtt[0][0] = tx[i];
				rtt[1][0] = ty[i];
				rtt[2][0] = tz[i];
				rtt[3][0] = tox[i];
				rtt[4][0] = toy[i];
				rtt[5][0] = toz[i];

				rffn = Cf*rff;
				rttn = Ct*rtt;


				fx2[i] = rffn[0][0];
				fy2[i] = rffn[1][0];
				fz2[i] = rffn[2][0];
				fox2[i] = rffn[3][0];
				foy2[i] = rffn[4][0];
				foz2[i] = rffn[5][0];

				tx2[i] = rttn[0][0];
				ty2[i] = rttn[1][0];
				tz2[i] = rttn[2][0];
				tox2[i] = rttn[3][0];
				toy2[i] = rttn[4][0];
				toz2[i] = rttn[5][0];

				fx1(i) = tx1(i) = 0;
				

				fox1(i) = tox1(i) = 0;
				if (sizerr < 2){
					fox2(i) = tox2(i) = 0;
					fx2(i) = tx2(i) = 0;
				}
			}
			cout << "For this simulation we will use  two translation DOFin  y z axis and two rotaion DoF in y and z axis" << endl;

		}
		//34
		if (count5 == sizer  && count4 != sizer  && count3 != sizer){
			for (int i = 0; i < sizerr; ++i){
				Mat33	Ef = Tait_Bryan(0, 0, -foz1(i));
				Mat33	Et = Tait_Bryan(0, 0, -toz1(i));
				// from RFerdestone page 22 for zero translation we have rotation for both forces and moments so:
				rf[0] = -fx1[i];
				rf[1] = 0;
				rf[2] = 0;
				rt[0] = -tx1[i];
				rt[1] = 0;
				rt[2] = 0;
				
				Mat66 Cf = plucker(Ef, rf);
				Mat66 Ct = plucker(Et, rt);
				//crosproduct...
				Mat61 rff;
				Mat61 rtt;
				Mat61 rffn;
				Mat61 rttn;

				rff[0][0] = fx[i];
				rff[1][0] = fy[i];
				rff[2][0] = fz[i];
				rff[3][0] = fox[i];
				rff[4][0] = foy[i];
				rff[5][0] = foz[i];

				rtt[0][0] = tx[i];
				rtt[1][0] = ty[i];
				rtt[2][0] = tz[i];
				rtt[3][0] = tox[i];
				rtt[4][0] = toy[i];
				rtt[5][0] = toz[i];

				rffn = Cf*rff;
				rttn = Ct*rtt;


				fx2[i] = rffn[0][0];
				fy2[i] = rffn[1][0];
				fz2[i] = rffn[2][0];
				fox2[i] = rffn[3][0];
				foy2[i] = rffn[4][0];
				foz2[i] = rffn[5][0];

				tx2[i] = rttn[0][0];
				ty2[i] = rttn[1][0];
				tz2[i] = rttn[2][0];
				tox2[i] = rttn[3][0];
				toy2[i] = rttn[4][0];
				toz2[i] = rttn[5][0];

				fx1(i) = tx1(i) = 0;
				//

				foz1(i) = toz1(i) = 0;
				if (sizerr < 2){
					foz2(i) = toz2(i) = 0;
					fx2(i) = tx2(i) = 0;
				}
			}
			cout << "For this simulation we will use  two translation DOFin  y z axis axis and two rotaion DoF in y and x axis" << endl;

		}
		//35
		if (count4 == sizer  && count5 != sizer  && count3 != sizer){
			for (int i = 0; i < sizerr; ++i){
				Mat33	Ef = Tait_Bryan(0, -foy1(i), 0);
				Mat33	Et = Tait_Bryan(0, -toy1(i), 0);
				// from RFerdestone page 22 for zero translation we have rotation for both forces and moments so:
				rf[0] = -fx1[i];
				rf[1] = 0;
				rf[2] = 0;
				rt[0] = -tx1[i];
				rt[1] = 0;
				rt[2] = 0;
				
				Mat66 Cf = plucker(Ef, rf);
				Mat66 Ct = plucker(Et, rt);
				//crosproduct...
				Mat61 rff;
				Mat61 rtt;
				Mat61 rffn;
				Mat61 rttn;

				rff[0][0] = fx[i];
				rff[1][0] = fy[i];
				rff[2][0] = fz[i];
				rff[3][0] = fox[i];
				rff[4][0] = foy[i];
				rff[5][0] = foz[i];

				rtt[0][0] = tx[i];
				rtt[1][0] = ty[i];
				rtt[2][0] = tz[i];
				rtt[3][0] = tox[i];
				rtt[4][0] = toy[i];
				rtt[5][0] = toz[i];

				rffn = Cf*rff;
				rttn = Ct*rtt;


				fx2[i] = rffn[0][0];
				fy2[i] = rffn[1][0];
				fz2[i] = rffn[2][0];
				fox2[i] = rffn[3][0];
				foy2[i] = rffn[4][0];
				foz2[i] = rffn[5][0];

				tx2[i] = rttn[0][0];
				ty2[i] = rttn[1][0];
				tz2[i] = rttn[2][0];
				tox2[i] = rttn[3][0];
				toy2[i] = rttn[4][0];
				toz2[i] = rttn[5][0];

				fx1(i) = tx1(i) = 0;
				//
				foy1(i) = toy1(i) = 0;

				if (sizerr < 2){
					foy2(i) = toy2(i) = 0;
					fx2(i) = tx2(i) = 0;
				}



			}
			cout << "For this simulation we will use two translation DOFin  z y axis and two rotaion DoF in z and x axis" << endl;

		}
	}


	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


	///////////////////////////////////////////////////////////////////
	////////////// 1 DOF TRANSLATION /////////////////////////////////////
	/////////////////////////////////////////////////////////////////







	///////////////////////////////////////////////////////////////////
	////////////// 1 DOF TRANSLATION x  /////////////////////////////////////
	/////////////////////////////////////////////////////////////////
	//36
	if (count != sizer && count1 == sizer && count2 == sizer){

		if (count3 == sizer && count4 == sizer && count5 == sizer){
			for (int i = 0; i < sizerr; ++i){
				Mat33	Ef = Tait_Bryan(-fox1(i), -foy1(i), foz1(i));
				Mat33	Et = Tait_Bryan(-tox1(i), -toy1(i), toz1(i));
				// from RFerdestone page 22 for zero translation we have rotation for both forces and moments so:
				//the forces stay as they are
				rf[0] = 0;
				rf[1] = -fy1[i];
				rf[2] = -fz1[i];
				rt[0] = 0;
				rt[1] = -ty1[i];
				rt[2] = -tz1[i];
				
				Mat66 Cf = plucker(Ef, rf);
				Mat66 Ct = plucker(Et, rt);
				//crosproduct...
				Mat61 rff;
				Mat61 rtt;
				Mat61 rffn;
				Mat61 rttn;

				rff[0][0] = fx[i];
				rff[1][0] = fy[i];
				rff[2][0] = fz[i];
				rff[3][0] = fox[i];
				rff[4][0] = foy[i];
				rff[5][0] = foz[i];

				rtt[0][0] = tx[i];
				rtt[1][0] = ty[i];
				rtt[2][0] = tz[i];
				rtt[3][0] = tox[i];
				rtt[4][0] = toy[i];
				rtt[5][0] = toz[i];

				rffn = Cf*rff;
				rttn = Ct*rtt;


				fx2[i] = rffn[0][0];
				fy2[i] = rffn[1][0];
				fz2[i] = rffn[2][0];
				fox2[i] = rffn[3][0];
				foy2[i] = rffn[4][0];
				foz2[i] = rffn[5][0];

				tx2[i] = rttn[0][0];
				ty2[i] = rttn[1][0];
				tz2[i] = rttn[2][0];
				tox2[i] = rttn[3][0];
				toy2[i] = rttn[4][0];
				toz2[i] = rttn[5][0];

				fox1(i) = tox1(i) = 0;
				foy1(i) = toy1(i) = 0;
				foz1(i) = toz1(i) = 0;
				fz1(i) = tz1(i) = 0;
				fy1(i) = ty1(i) = 0;
				if (sizerr < 2){
					fy2(i) = ty2(i) = 0;
					fox2(i) = tox2(i) = 0;
					foy2(i) = toy2(i) = 0;
					foz2(i) = toz2(i) = 0;
					fz2(i) = tz2(i) = 0;
				}
			}
			cout << "For this simulation we will use two translation DOFin  x  axis" << endl;

		}
		//moment[x] = moment[0] + (force[y] * pointz - force[z] * pointy);
		//moment[y] = moment[1] + ( - force[x] * pointz);
		//moment[z] = moment[2] + (force[x] * pointy );
		///////////////////////////////////////////////////////////////////
		////////////// 1 DOF ROTATION-1 DOF TRANSLATION /////////////////////////////////////
		/////////////////////////////////////////////////////////////////
		//37
		if (count3 == sizer && count4 == sizer  && count5 != sizer){
			for (int i = 0; i < sizerr; ++i){
				Mat33	Ef = Tait_Bryan(-fox1(i), -foy1(i), 0);
				Mat33	Et = Tait_Bryan(-tox1(i), -toy1(i), 0);
				// from RFerdestone page 22 for zero translation we have rotation for both forces and moments so:

				rf[0] = 0;
				rf[1] = -fy1[i];
				rf[2] = -fz1[i];
				rt[0] = 0;
				rt[1] = -ty1[i];
				rt[2] = -tz1[i];
				
				Mat66 Cf = plucker(Ef, rf);
				Mat66 Ct = plucker(Et, rt);
				//crosproduct...
				Mat61 rff;
				Mat61 rtt;
				Mat61 rffn;
				Mat61 rttn;

				rff[0][0] = fx[i];
				rff[1][0] = fy[i];
				rff[2][0] = fz[i];
				rff[3][0] = fox[i];
				rff[4][0] = foy[i];
				rff[5][0] = foz[i];

				rtt[0][0] = tx[i];
				rtt[1][0] = ty[i];
				rtt[2][0] = tz[i];
				rtt[3][0] = tox[i];
				rtt[4][0] = toy[i];
				rtt[5][0] = toz[i];

				rffn = Cf*rff;
				rttn = Ct*rtt;


				fx2[i] = rffn[0][0];
				fy2[i] = rffn[1][0];
				fz2[i] = rffn[2][0];
				fox2[i] = rffn[3][0];
				foy2[i] = rffn[4][0];
				foz2[i] = rffn[5][0];

				tx2[i] = rttn[0][0];
				ty2[i] = rttn[1][0];
				tz2[i] = rttn[2][0];
				tox2[i] = rttn[3][0];
				toy2[i] = rttn[4][0];
				toz2[i] = rttn[5][0];

				fox1(i) = tox1(i) = 0;
				foy1(i) = toy1(i) = 0;
				fy1(i) = ty1(i) = 0;
				if (sizerr < 2){
					fy2(i) = ty2(i) = 0;
					fox2(i) = tox2(i) = 0;
					foy2(i) = toy2(i) = 0;
					fz2(i) = tz2(i) = 0;
				}
				fz1(i) = tz1(i) = 0;
				//

			}
			cout << "For this simulation we will use  two translation DOFin  x  axis and one rotaion DoF in z axis" << endl;

		}
		//38
		if (count3 == sizer && count5 == sizer  && count4 != sizer){
			for (int i = 0; i < sizerr; ++i){
				Mat33	Ef = Tait_Bryan(-fox1(i), 0, -foz1(i));
				Mat33	Et = Tait_Bryan(-tox1(i), 0, -toz1(i));
				// from RFerdestone page 22 for zero translation we have rotation for both forces and moments so:

				rf[0] = 0;
				rf[1] = -fy1[i];
				rf[2] = -fz1[i];
				rt[0] = 0;
				rt[1] = -ty1[i];
				rt[2] = -tz1[i];
				
				Mat66 Cf = plucker(Ef, rf);
				Mat66 Ct = plucker(Et, rt);
				//crosproduct...
				Mat61 rff;
				Mat61 rtt;
				Mat61 rffn;
				Mat61 rttn;

				rff[0][0] = fx[i];
				rff[1][0] = fy[i];
				rff[2][0] = fz[i];
				rff[3][0] = fox[i];
				rff[4][0] = foy[i];
				rff[5][0] = foz[i];

				rtt[0][0] = tx[i];
				rtt[1][0] = ty[i];
				rtt[2][0] = tz[i];
				rtt[3][0] = tox[i];
				rtt[4][0] = toy[i];
				rtt[5][0] = toz[i];

				rffn = Cf*rff;
				rttn = Ct*rtt;


				fx2[i] = rffn[0][0];
				fy2[i] = rffn[1][0];
				fz2[i] = rffn[2][0];
				fox2[i] = rffn[3][0];
				foy2[i] = rffn[4][0];
				foz2[i] = rffn[5][0];

				tx2[i] = rttn[0][0];
				ty2[i] = rttn[1][0];
				tz2[i] = rttn[2][0];
				tox2[i] = rttn[3][0];
				toy2[i] = rttn[4][0];
				toz2[i] = rttn[5][0];

				fox1(i) = tox1(i) = 0;
				foz1(i) = toz1(i) = 0;
				fy1(i) = ty1(i) = 0;
				
				if (sizerr < 2){
					fy2(i) = ty2(i) = 0;
					fox2(i) = tox2(i) = 0;
					foz2(i) = toz2(i) = 0;
					fz2(i) = tz2(i) = 0;
				}

				fz1(i) = tz1(i) = 0;
				//

			}
			cout << "For this simulation we will use  two translation DOFin  x  axis and one rotaion DoF in y axis" << endl;

		}
		//39
		if (count4 == sizer && count5 == sizer  && count3 != sizer){
			for (int i = 0; i < sizerr; ++i){
				Mat33	Ef = Tait_Bryan(0, -foy1(i), -foz1(i));
				Mat33	Et = Tait_Bryan(0, -toy1(i), -toz1(i));
				// from RFerdestone page 22 for zero translation we have rotation for both forces and moments so:

				rf[0] = 0;
				rf[1] = -fy1[i];
				rf[2] = -fz1[i];
				rt[0] = 0;
				rt[1] = -ty1[i];
				rt[2] = -tz1[i];
				
				Mat66 Cf = plucker(Ef, rf);
				Mat66 Ct = plucker(Et, rt);
				//crosproduct...
				Mat61 rff;
				Mat61 rtt;
				Mat61 rffn;
				Mat61 rttn;

				rff[0][0] = fx[i];
				rff[1][0] = fy[i];
				rff[2][0] = fz[i];
				rff[3][0] = fox[i];
				rff[4][0] = foy[i];
				rff[5][0] = foz[i];

				rtt[0][0] = tx[i];
				rtt[1][0] = ty[i];
				rtt[2][0] = tz[i];
				rtt[3][0] = tox[i];
				rtt[4][0] = toy[i];
				rtt[5][0] = toz[i];

				rffn = Cf*rff;
				rttn = Ct*rtt;


				fx2[i] = rffn[0][0];
				fy2[i] = rffn[1][0];
				fz2[i] = rffn[2][0];
				fox2[i] = rffn[3][0];
				foy2[i] = rffn[4][0];
				foz2[i] = rffn[5][0];

				tx2[i] = rttn[0][0];
				ty2[i] = rttn[1][0];
				tz2[i] = rttn[2][0];
				tox2[i] = rttn[3][0];
				toy2[i] = rttn[4][0];
				toz2[i] = rttn[5][0];

				foy1(i) = toy1(i) = 0;
				foz1(i) = toz1(i) = 0;
				fy1(i) = ty1(i) = 0;
				if (sizerr < 2){
					fy2(i) = ty2(i) = 0;
					fz2(i) = tz2(i) = 0;
					foy2(i) = toy2(i) = 0;
					foz2(i) = toz2(i) = 0;
				}
				fz1(i) = tz1(i) = 0;
				//
			}
			cout << "For this simulation we will use  two translation DOFin  x  axis axis and one rotaion DoF in x axis" << endl;

		}


		///////////////////////////////////////////////////////////////////
		////////////// 2 DOF ROTATION-1 DOF TRANSLATION /////////////////////////////////////
		/////////////////////////////////////////////////////////////////
		//40
		if (count3 == sizer  && count4 != sizer  && count5 != sizer){
			for (int i = 0; i < sizerr; ++i){
				Mat33	Ef = Tait_Bryan(-fox1(i), 0, 0);
				Mat33	Et = Tait_Bryan(-tox1(i), 0, 0);
				// from RFerdestone page 22 for zero translation we have rotation for both forces and moments so:
				rf[0] = 0;
				rf[1] = -fy1[i];
				rf[2] = -fz1[i];
				rt[0] = 0;
				rt[1] = -ty1[i];
				rt[2] = -tz1[i];
				
				Mat66 Cf = plucker(Ef, rf);
				Mat66 Ct = plucker(Et, rt);
				//crosproduct...
				Mat61 rff;
				Mat61 rtt;
				Mat61 rffn;
				Mat61 rttn;

				rff[0][0] = fx[i];
				rff[1][0] = fy[i];
				rff[2][0] = fz[i];
				rff[3][0] = fox[i];
				rff[4][0] = foy[i];
				rff[5][0] = foz[i];

				rtt[0][0] = tx[i];
				rtt[1][0] = ty[i];
				rtt[2][0] = tz[i];
				rtt[3][0] = tox[i];
				rtt[4][0] = toy[i];
				rtt[5][0] = toz[i];

				rffn = Cf*rff;
				rttn = Ct*rtt;


				fx2[i] = rffn[0][0];
				fy2[i] = rffn[1][0];
				fz2[i] = rffn[2][0];
				fox2[i] = rffn[3][0];
				foy2[i] = rffn[4][0];
				foz2[i] = rffn[5][0];

				tx2[i] = rttn[0][0];
				ty2[i] = rttn[1][0];
				tz2[i] = rttn[2][0];
				tox2[i] = rttn[3][0];
				toy2[i] = rttn[4][0];
				toz2[i] = rttn[5][0];

				fz1(i) = tz1(i) = 0;
				//
				fy1(i) = ty1(i) = 0;
				
				fox1(i) = tox1(i) = 0;
				if (sizerr < 2){
					fox2(i) = tox2(i) = 0;
					fy2(i) = ty2(i) = 0;
					fz2(i) = tz2(i) = 0;
				}
			}
			cout << "For this simulation we will use  two translation DOFin  x  axis and two rotaion DoF in y and z axis" << endl;

		}
		//41
		if (count5 == sizer  && count4 != sizer  && count3 != sizer){
			for (int i = 0; i < sizerr; ++i){
				Mat33	Ef = Tait_Bryan(0, 0, -foz1(i));
				Mat33	Et = Tait_Bryan(0, 0, -toz1(i));
				// from RFerdestone page 22 for zero translation we have rotation for both forces and moments so:
				rf[0] = 0;
				rf[1] = -fy1[i];
				rf[2] = -fz1[i];
				rt[0] = 0;
				rt[1] = -ty1[i];
				rt[2] = -tz1[i];
				
				Mat66 Cf = plucker(Ef, rf);
				Mat66 Ct = plucker(Et, rt);
				//crosproduct...
				Mat61 rff;
				Mat61 rtt;
				Mat61 rffn;
				Mat61 rttn;

				rff[0][0] = fx[i];
				rff[1][0] = fy[i];
				rff[2][0] = fz[i];
				rff[3][0] = fox[i];
				rff[4][0] = foy[i];
				rff[5][0] = foz[i];

				rtt[0][0] = tx[i];
				rtt[1][0] = ty[i];
				rtt[2][0] = tz[i];
				rtt[3][0] = tox[i];
				rtt[4][0] = toy[i];
				rtt[5][0] = toz[i];

				rffn = Cf*rff;
				rttn = Ct*rtt;


				fx2[i] = rffn[0][0];
				fy2[i] = rffn[1][0];
				fz2[i] = rffn[2][0];
				fox2[i] = rffn[3][0];
				foy2[i] = rffn[4][0];
				foz2[i] = rffn[5][0];

				tx2[i] = rttn[0][0];
				ty2[i] = rttn[1][0];
				tz2[i] = rttn[2][0];
				tox2[i] = rttn[3][0];
				toy2[i] = rttn[4][0];
				toz2[i] = rttn[5][0];

				fz1(i) = tz1(i) = 0;
				//
				foz1(i) = toz1(i) = 0;
				fy1(i) = ty1(i) = 0;

				if (sizerr < 2){
					fy2(i) = ty2(i) = 0;

					foz2(i) = toz2(i) = 0;
					fz2(i) = tz2(i) = 0;
				}
			}
			cout << "For this simulation we will use  two translation DOFin  x  axis axis and two rotaion DoF in y and x axis" << endl;

		}
		//42
		if (count4 == sizer  && count3 != sizer  && count5 != sizer){
			for (int i = 0; i < sizerr; ++i){
				Mat33	Ef = Tait_Bryan(0, -foy1(i), 0);
				Mat33	Et = Tait_Bryan(0, -toy1(i), 0);
				// from RFerdestone page 22 for zero translation we have rotation for both forces and moments so:
				rf[0] = 0;
				rf[1] = -fy1[i];
				rf[2] = -fz1[i];
				rt[0] = 0;
				rt[1] = -ty1[i];
				rt[2] = -tz1[i];
				
				Mat66 Cf = plucker(Ef, rf);
				Mat66 Ct = plucker(Et, rt);
				//crosproduct...
				Mat61 rff;
				Mat61 rtt;
				Mat61 rffn;
				Mat61 rttn;

				rff[0][0] = fx[i];
				rff[1][0] = fy[i];
				rff[2][0] = fz[i];
				rff[3][0] = fox[i];
				rff[4][0] = foy[i];
				rff[5][0] = foz[i];

				rtt[0][0] = tx[i];
				rtt[1][0] = ty[i];
				rtt[2][0] = tz[i];
				rtt[3][0] = tox[i];
				rtt[4][0] = toy[i];
				rtt[5][0] = toz[i];

				rffn = Cf*rff;
				rttn = Ct*rtt;


				fx2[i] = rffn[0][0];
				fy2[i] = rffn[1][0];
				fz2[i] = rffn[2][0];
				fox2[i] = rffn[3][0];
				foy2[i] = rffn[4][0];
				foz2[i] = rffn[5][0];

				tx2[i] = rttn[0][0];
				ty2[i] = rttn[1][0];
				tz2[i] = rttn[2][0];
				tox2[i] = rttn[3][0];
				toy2[i] = rttn[4][0];
				toz2[i] = rttn[5][0];

				fz1(i) = tz1(i) = 0;
				
				foy1(i) = toy1(i) = 0;
				fy1(i) = ty1(i) = 0;

				if (sizerr < 2){
					fy2(i) = ty2(i) = 0;
					fz2(i) = tz2(i) = 0;
					foy2(i) = toy2(i) = 0;

				}


			}
			cout << "For this simulation we will use two translation DOFin  x  axis and two rotaion DoF in z and x axis" << endl;

		}
	}
	////////////////////////////////////////////////////////////////////////////////////////////////////////////








	///////////////////////////////////////////////////////////////////
	////////////// 1 DOF TRANSLATION  z /////////////////////////////////////
	/////////////////////////////////////////////////////////////////
	//43
	if (count == sizer && count1 == sizer && count2 != sizer){

		if (count3 == sizer && count4 == sizer && count5 == sizer){
			for (int i = 0; i < sizerr; ++i){
				Mat33	Ef = Tait_Bryan(-fox1(i), -foy1(i), -foz1(i));
				Mat33	Et = Tait_Bryan(-tox1(i), -toy1(i), -toz1(i));
				// from RFerdestone page 22 for zero translation we have rotation for both forces and moments so:
				//the forces stay as they are
				rf[0] = -fx1[i];
				rf[1] = -fy1[i];
				rf[2] = 0;
				rt[0] = -tx1[i];
				rt[1] = -ty1[i];
				rt[2] = 0;
				
				Mat66 Cf = plucker(Ef, rf);
				Mat66 Ct = plucker(Et, rt);
				//crosproduct...
				Mat61 rff;
				Mat61 rtt;
				Mat61 rffn;
				Mat61 rttn;

				rff[0][0] = fx[i];
				rff[1][0] = fy[i];
				rff[2][0] = fz[i];
				rff[3][0] = fox[i];
				rff[4][0] = foy[i];
				rff[5][0] = foz[i];

				rtt[0][0] = tx[i];
				rtt[1][0] = ty[i];
				rtt[2][0] = tz[i];
				rtt[3][0] = tox[i];
				rtt[4][0] = toy[i];
				rtt[5][0] = toz[i];

				rffn = Cf*rff;
				rttn = Ct*rtt;


				fx2[i] = rffn[0][0];
				fy2[i] = rffn[1][0];
				fz2[i] = rffn[2][0];
				fox2[i] = rffn[3][0];
				foy2[i] = rffn[4][0];
				foz2[i] = rffn[5][0];

				tx2[i] = rttn[0][0];
				ty2[i] = rttn[1][0];
				tz2[i] = rttn[2][0];
				tox2[i] = rttn[3][0];
				toy2[i] = rttn[4][0];
				toz2[i] = rttn[5][0];

				fx1(i) = tx1(i) = 0;
				//
				fox1(i) = tox1(i) = 0;
				foy1(i) = toy1(i) = 0;
				foz1(i) = toz1(i) = 0;
				fy1(i) = ty1(i) = 0;

				if (sizerr < 2){
					fy2(i) = ty2(i) = 0;
					fox2(i) = tox2(i) = 0;
					foy2(i) = toy2(i) = 0;
					foz2(i) = toz2(i) = 0;
					fx2(i) = tx2(i) = 0;
				}
			}
			cout << "For this simulation we will use one translation DOFin   z axis" << endl;

		}
		//moment[x] = moment[0] + ( - force[z] * pointy);
		//moment[y] = moment[1] + (force[z] * pointx );
		//moment[z] = moment[2] + (force[x] * pointy - force[y] * pointx);
		///////////////////////////////////////////////////////////////////
		////////////// 1 DOF ROTATION-1 DOF TRANSLATION /////////////////////////////////////
		/////////////////////////////////////////////////////////////////
		//44
		if (count3 == sizer && count4 == sizer  && count5 != sizer){
			for (int i = 0; i < sizerr; ++i){
				Mat33	Ef = Tait_Bryan(-fox1(i), -foy1(i), 0);
				Mat33	Et = Tait_Bryan(-tox1(i), -toy1(i), 0);
				// from RFerdestone page 22 for zero translation we have rotation for both forces and moments so:
				rf[0] = -fx1[i];
				rf[1] = -fy1[i];
				rf[2] = 0;
				rt[0] = -tx1[i];
				rt[1] = -ty1[i];
				rt[2] = 0;
				
				Mat66 Cf = plucker(Ef, rf);
				Mat66 Ct = plucker(Et, rt);
				//crosproduct...
				Mat61 rff;
				Mat61 rtt;
				Mat61 rffn;
				Mat61 rttn;

				rff[0][0] = fx[i];
				rff[1][0] = fy[i];
				rff[2][0] = fz[i];
				rff[3][0] = fox[i];
				rff[4][0] = foy[i];
				rff[5][0] = foz[i];

				rtt[0][0] = tx[i];
				rtt[1][0] = ty[i];
				rtt[2][0] = tz[i];
				rtt[3][0] = tox[i];
				rtt[4][0] = toy[i];
				rtt[5][0] = toz[i];

				rffn = Cf*rff;
				rttn = Ct*rtt;

				fx2[i] = rffn[0][0];
				fy2[i] = rffn[1][0];
				fz2[i] = rffn[2][0];
				fox2[i] = rffn[3][0];
				foy2[i] = rffn[4][0];
				foz2[i] = rffn[5][0];

				tx2[i] = rttn[0][0];
				ty2[i] = rttn[1][0];
				tz2[i] = rttn[2][0];
				tox2[i] = rttn[3][0];
				toy2[i] = rttn[4][0];
				toz2[i] = rttn[5][0];

				fx1(i) = tx1(i) = 0;
				//
				fox1(i) = tox1(i) = 0;
				foy1(i) = toy1(i) = 0;
				if (sizerr < 2){
					fox2(i) = tox2(i) = 0;
					foy2(i) = toy2(i) = 0;
					fy2(i) = ty2(i) = 0;
					fx2(i) = tx2(i) = 0;
				}
				fy1(i) = ty1(i) = 0;
				//

			}
			cout << "For this simulation we will use  two translation DOFin   z axis and one rotaion DoF in z axis" << endl;

		}
		//45
		if (count3 == sizer && count5 == sizer  && count4 != sizer){
			for (int i = 0; i < sizerr; ++i){
				Mat33	Ef = Tait_Bryan(-fox1(i), 0, -foz1(i));
				Mat33	Et = Tait_Bryan(-tox1(i), 0, -toz1(i));
				// from RFerdestone page 22 for zero translation we have rotation for both forces and moments so:
				rf[0] = -fx1[i];
				rf[1] = -fy1[i];
				rf[2] = 0;
				rt[0] = -tx1[i];
				rt[1] = -ty1[i];
				rt[2] = 0;
				
				Mat66 Cf = plucker(Ef, rf);
				Mat66 Ct = plucker(Et, rt);
				//crosproduct...
				Mat61 rff;
				Mat61 rtt;
				Mat61 rffn;
				Mat61 rttn;

				rff[0][0] = fx[i];
				rff[1][0] = fy[i];
				rff[2][0] = fz[i];
				rff[3][0] = fox[i];
				rff[4][0] = foy[i];
				rff[5][0] = foz[i];

				rtt[0][0] = tx[i];
				rtt[1][0] = ty[i];
				rtt[2][0] = tz[i];
				rtt[3][0] = tox[i];
				rtt[4][0] = toy[i];
				rtt[5][0] = toz[i];

				rffn = Cf*rff;
				rttn = Ct*rtt;


				fx2[i] = rffn[0][0];
				fy2[i] = rffn[1][0];
				fz2[i] = rffn[2][0];
				fox2[i] = rffn[3][0];
				foy2[i] = rffn[4][0];
				foz2[i] = rffn[5][0];

				tx2[i] = rttn[0][0];
				ty2[i] = rttn[1][0];
				tz2[i] = rttn[2][0];
				tox2[i] = rttn[3][0];
				toy2[i] = rttn[4][0];
				toz2[i] = rttn[5][0];

				fy1(i) = ty1(i) = 0;
				//
				fx1(i) = tx1(i) = 0;
				//

				fox1(i) = tox1(i) = 0;
				foz1(i) = toz1(i) = 0;
				if (sizerr < 2){
					fox2(i) = tox2(i) = 0;
					foz2(i) = toz2(i) = 0;
					fx2(i) = tx2(i) = 0;
					fy2(i) = ty2(i) = 0;
				}
			}
			cout << "For this simulation we will use  two translation DOFin  z axis and one rotaion DoF in y axis" << endl;

		}
		//46
		if (count4 == sizer && count5 == sizer  && count3 != sizer){
			for (int i = 0; i < sizerr; ++i){
				Mat33	Ef = Tait_Bryan(0, -foy1(i), -foz1(i));
				Mat33	Et = Tait_Bryan(0, -toy1(i), -toz1(i));
				// from RFerdestone page 22 for zero translation we have rotation for both forces and moments so:
				rf[0] = -fx1[i];
				rf[1] = -fy1[i];
				rf[2] = 0;
				rt[0] = -tx1[i];
				rt[1] = -ty1[i];
				rt[2] = 0;
				
				Mat66 Cf = plucker(Ef, rf);
				Mat66 Ct = plucker(Et, rt);
				//crosproduct...
				Mat61 rff;
				Mat61 rtt;
				Mat61 rffn;
				Mat61 rttn;

				rff[0][0] = fx[i];
				rff[1][0] = fy[i];
				rff[2][0] = fz[i];
				rff[3][0] = fox[i];
				rff[4][0] = foy[i];
				rff[5][0] = foz[i];

				rtt[0][0] = tx[i];
				rtt[1][0] = ty[i];
				rtt[2][0] = tz[i];
				rtt[3][0] = tox[i];
				rtt[4][0] = toy[i];
				rtt[5][0] = toz[i];

				rffn = Cf*rff;
				rttn = Ct*rtt;


				fx2[i] = rffn[0][0];
				fy2[i] = rffn[1][0];
				fz2[i] = rffn[2][0];
				fox2[i] = rffn[3][0];
				foy2[i] = rffn[4][0];
				foz2[i] = rffn[5][0];

				tx2[i] = rttn[0][0];
				ty2[i] = rttn[1][0];
				tz2[i] = rttn[2][0];
				tox2[i] = rttn[3][0];
				toy2[i] = rttn[4][0];
				toz2[i] = rttn[5][0];

				fy1(i) = ty1(i) = 0;
				//

				foy1(i) = toy1(i) = 0;
				foz1(i) = toz1(i) = 0;
				if (sizerr < 2){
					foy2(i) = toy2(i) = 0;
					foz2(i) = toz2(i) = 0;
					fx2(i) = tx2(i) = 0;
					fy2(i) = ty2(i) = 0;
				}
				fx1(i) = tx1(i) = 0;
				//
			}
			cout << "For this simulation we will use  two translation DOFin   z axis axis and one rotaion DoF in x axis" << endl;

		}


		///////////////////////////////////////////////////////////////////
		////////////// 2 DOF ROTATION-1 DOF TRANSLATION /////////////////////////////////////
		/////////////////////////////////////////////////////////////////
		//47
		if (count3 == sizer  && count5 != sizer  && count4 != sizer){
			for (int i = 0; i < sizerr; ++i){
				Mat33	Ef = Tait_Bryan(-fox1(i), 0, 0);
				Mat33	Et = Tait_Bryan(-tox1(i), 0, 0);
				// from RFerdestone page 22 for zero translation we have rotation for both forces and moments so:
				rf[0] = -fx1[i];
				rf[1] = -fy1[i];
				rf[2] = 0;
				rt[0] = -tx1[i];
				rt[1] = -ty1[i];
				rt[2] = 0;
				
				Mat66 Cf = plucker(Ef, rf);
				Mat66 Ct = plucker(Et, rt);
				//crosproduct...
				Mat61 rff;
				Mat61 rtt;
				Mat61 rffn;
				Mat61 rttn;

				rff[0][0] = fx[i];
				rff[1][0] = fy[i];
				rff[2][0] = fz[i];
				rff[3][0] = fox[i];
				rff[4][0] = foy[i];
				rff[5][0] = foz[i];

				rtt[0][0] = tx[i];
				rtt[1][0] = ty[i];
				rtt[2][0] = tz[i];
				rtt[3][0] = tox[i];
				rtt[4][0] = toy[i];
				rtt[5][0] = toz[i];

				rffn = Cf*rff;
				rttn = Ct*rtt;


				fx2[i] = rffn[0][0];
				fy2[i] = rffn[1][0];
				fz2[i] = rffn[2][0];
				fox2[i] = rffn[3][0];
				foy2[i] = rffn[4][0];
				foz2[i] = rffn[5][0];

				tx2[i] = rttn[0][0];
				ty2[i] = rttn[1][0];
				tz2[i] = rttn[2][0];
				tox2[i] = rttn[3][0];
				toy2[i] = rttn[4][0];
				toz2[i] = rttn[5][0];

				fy1(i) = ty1(i) = 0;
				//

				fox1(i) = tox1(i) = 0;

				//
				fx1(i) = tx1(i) = 0;

				if (sizerr < 2){
					fx2(i) = tx2(i) = 0;
					fox2(i) = tox2(i) = 0;
					fy2(i) = ty2(i) = 0;
				}

			}
			cout << "For this simulation we will use  one translation DOFin   z axis and two rotaion DoF in y and z axis" << endl;

		}
		//48
		if (count5 == sizer  && count4 != sizer  && count3 != sizer){
			for (int i = 0; i < sizerr; ++i){
				Mat33	Ef = Tait_Bryan(0, 0, -foz1(i));
				Mat33	Et = Tait_Bryan(0, 0, -toz1(i));
				// from RFerdestone page 22 for zero translation we have rotation for both forces and moments so:
				rf[0] = -fx1[i];
				rf[1] = -fy1[i];
				rf[2] = 0;
				rt[0] = -tx1[i];
				rt[1] = -ty1[i];
				rt[2] = 0;
				Mat66 Cf = plucker(Ef, rf);
				Mat66 Ct = plucker(Et, rt);
				//crosproduct...
				Mat61 rff;
				Mat61 rtt;
				Mat61 rffn;
				Mat61 rttn;

				rff[0][0] = fx[i];
				rff[1][0] = fy[i];
				rff[2][0] = fz[i];
				rff[3][0] = fox[i];
				rff[4][0] = foy[i];
				rff[5][0] = foz[i];

				rtt[0][0] = tx[i];
				rtt[1][0] = ty[i];
				rtt[2][0] = tz[i];
				rtt[3][0] = tox[i];
				rtt[4][0] = toy[i];
				rtt[5][0] = toz[i];

				rffn = Cf*rff;
				rttn = Ct*rtt;


				fx2[i] = rffn[0][0];
				fy2[i] = rffn[1][0];
				fz2[i] = rffn[2][0];
				fox2[i] = rffn[3][0];
				foy2[i] = rffn[4][0];
				foz2[i] = rffn[5][0];

				tx2[i] = rttn[0][0];
				ty2[i] = rttn[1][0];
				tz2[i] = rttn[2][0];
				tox2[i] = rttn[3][0];
				toy2[i] = rttn[4][0];
				toz2[i] = rttn[5][0];

				fy1(i) = ty1(i) = 0;
				//

				foz1(i) = toz1(i) = 0;
				fx1(i) = tx1(i) = 0;
				if (sizerr < 2){
					fx2(i) = tx2(i) = 0;
					foz2(i) = toz2(i) = 0;
					fy2(i) = ty2(i) = 0;
				}
			}
			cout << "For this simulation we will use  one translation DOFin   z axis axis and two rotaion DoF in y and x axis" << endl;

		}
		//49
		if (count4 == sizer  && count3 != sizer  && count5 != sizer){
			for (int i = 0; i < sizerr; ++i){
				Mat33	Ef = Tait_Bryan(0, -foy1(i), 0);
				Mat33	Et = Tait_Bryan(0, -toy1(i), 0);
				// from RFerdestone page 22 for zero translation we have rotation for both forces and moments so:
				rf[0] = -fx1[i];
				rf[1] = -fy1[i];
				rf[2] = 0;
				rt[0] = -tx1[i];
				rt[1] = -ty1[i];
				rt[2] = 0;
				
				Mat66 Cf = plucker(Ef, rf);
				Mat66 Ct = plucker(Et, rt);
				//crosproduct...
				Mat61 rff;
				Mat61 rtt;
				Mat61 rffn;
				Mat61 rttn;

				rff[0][0] = fx[i];
				rff[1][0] = fy[i];
				rff[2][0] = fz[i];
				rff[3][0] = fox[i];
				rff[4][0] = foy[i];
				rff[5][0] = foz[i];

				rtt[0][0] = tx[i];
				rtt[1][0] = ty[i];
				rtt[2][0] = tz[i];
				rtt[3][0] = tox[i];
				rtt[4][0] = toy[i];
				rtt[5][0] = toz[i];

				rffn = Cf*rff;
				rttn = Ct*rtt;

				fx2[i] = rffn[0][0];
				fy2[i] = rffn[1][0];
				fz2[i] = rffn[2][0];
				fox2[i] = rffn[3][0];
				foy2[i] = rffn[4][0];
				foz2[i] = rffn[5][0];

				tx2[i] = rttn[0][0];
				ty2[i] = rttn[1][0];
				tz2[i] = rttn[2][0];
				tox2[i] = rttn[3][0];
				toy2[i] = rttn[4][0];
				toz2[i] = rttn[5][0];

				fy1(i) = ty1(i) = 0;
				//
				foy1(i) = toy1(i) = 0;
				fx1(i) = tx1(i) = 0;
				if (sizerr < 2){
					fx2(i) = tx2(i) = 0;
					fy2(i) = ty2(i) = 0;
					foy2(i) = toy2(i) = 0;
				}



			}
			cout << "For this simulation we will use one translation DOFin  z axis and two rotaion DoF in z and x axis" << endl;

		}
	}


	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////








	///////////////////////////////////////////////////////////////////
	////////////// 1 DOF TRANSLATION  y  /////////////////////////////////////
	/////////////////////////////////////////////////////////////////
	//50
	if (count == sizer && count1 != sizer && count2 == sizer){

		if (count3 == sizer && count4 == sizer && count5 == sizer){
			for (int i = 0; i < sizerr; ++i){
				Mat33	Ef = Tait_Bryan(-fox1(i), -foy1(i), -foz1(i));
				Mat33	Et = Tait_Bryan(-tox1(i), -toy1(i), -toz1(i));
				// from RFerdestone page 22 for zero translation we have rotation for both forces and moments so:
				rf[0] = -fx1[i];
				rf[1] = 0;
				rf[2] = -fz1[i];
				rt[0] = -tx1[i];
				rt[1] = 0;
				rt[2] = -tz1[i];
				
				Mat66 Cf = plucker(Ef, rf);
				Mat66 Ct = plucker(Et, rt);
				//crosproduct...
				Mat61 rff;
				Mat61 rtt;
				Mat61 rffn;
				Mat61 rttn;

				rff[0][0] = fx[i];
				rff[1][0] = fy[i];
				rff[2][0] = fz[i];
				rff[3][0] = fox[i];
				rff[4][0] = foy[i];
				rff[5][0] = foz[i];

				rtt[0][0] = tx[i];
				rtt[1][0] = ty[i];
				rtt[2][0] = tz[i];
				rtt[3][0] = tox[i];
				rtt[4][0] = toy[i];
				rtt[5][0] = toz[i];

				rffn = Cf*rff;
				rttn = Ct*rtt;


				fx2[i] = rffn[0][0];
				fy2[i] = rffn[1][0];
				fz2[i] = rffn[2][0];
				fox2[i] = rffn[3][0];
				foy2[i] = rffn[4][0];
				foz2[i] = rffn[5][0];

				tx2[i] = rttn[0][0];
				ty2[i] = rttn[1][0];
				tz2[i] = rttn[2][0];
				tox2[i] = rttn[3][0];
				toy2[i] = rttn[4][0];
				toz2[i] = rttn[5][0];
				//the forces stay as they are
				fz1(i) = tz1(i) = 0;
				//
				fox1(i) = tox1(i) = 0;
				foy1(i) = toy1(i) = 0;
				foz1(i) = toz1(i) = 0;
				fx1(i) = tx1(i) = 0;
				if (sizerr < 2){
					fx2(i) = tx2(i) = 0;
					fox2(i) = tox2(i) = 0;
					foy2(i) = toy2(i) = 0;
					foz2(i) = toz2(i) = 0;
					fz2(i) = tz2(i) = 0;
				}

			}
			cout << "For this simulation we will use one translation DOF in  y  axis" << endl;

		}
		//moment[x] = moment[0] + (force[y] * pointz );
		//moment[y] = moment[1] + (force[z] * pointx - force[x] * pointz);
		//moment[z] = moment[2] + ( - force[y] * pointx);
		///////////////////////////////////////////////////////////////////
		////////////// 1 DOF ROTATION-1 DOF TRANSLATION /////////////////////////////////////
		/////////////////////////////////////////////////////////////////
		//51
		if (count3 == sizer && count4 == sizer && count5 != sizer){
			for (int i = 0; i < sizerr; ++i){
				Mat33	Ef = Tait_Bryan(-fox1(i), -foy1(i), 0);
				Mat33	Et = Tait_Bryan(-tox1(i), -toy1(i), 0);
				// from RFerdestone page 22 for zero translation we have rotation for both forces and moments so:

				rf[0] = -fx1[i];
				rf[1] = 0;
				rf[2] = -fz1[i];
				rt[0] = -tx1[i];
				rt[1] = 0;
				rt[2] = -tz1[i];
				
				Mat66 Cf = plucker(Ef, rf);
				Mat66 Ct = plucker(Et, rt);
				//crosproduct...
				Mat61 rff;
				Mat61 rtt;
				Mat61 rffn;
				Mat61 rttn;

				rff[0][0] = fx[i];
				rff[1][0] = fy[i];
				rff[2][0] = fz[i];
				rff[3][0] = fox[i];
				rff[4][0] = foy[i];
				rff[5][0] = foz[i];

				rtt[0][0] = tx[i];
				rtt[1][0] = ty[i];
				rtt[2][0] = tz[i];
				rtt[3][0] = tox[i];
				rtt[4][0] = toy[i];
				rtt[5][0] = toz[i];

				rffn = Cf*rff;
				rttn = Ct*rtt;


				fx2[i] = rffn[0][0];
				fy2[i] = rffn[1][0];
				fz2[i] = rffn[2][0];
				fox2[i] = rffn[3][0];
				foy2[i] = rffn[4][0];
				foz2[i] = rffn[5][0];

				tx2[i] = rttn[0][0];
				ty2[i] = rttn[1][0];
				tz2[i] = rttn[2][0];
				tox2[i] = rttn[3][0];
				toy2[i] = rttn[4][0];
				toz2[i] = rttn[5][0];

				fox1(i) = tox1(i) = 0;
				foy1(i) = toy1(i) = 0;
				fz1(i) = tz1(i) = 0;
				if (sizerr < 2){
					fz2(i) = tz2(i) = 0;
					fox2(i) = tox2(i) = 0;
					foy2(i) = toy2(i) = 0;
					fx2(i) = tx2(i) = 0;
				}
				fx1(i) = tx1(i) = 0;
				//

			}
			cout << "For this simulation we will use  one translation DOFin  y  axis and one rotaion DoF in z axis" << endl;

		}
		//52
		if (count3 == sizer && count5 == sizer && count4 != sizer){
			for (int i = 0; i < sizerr; ++i){
				Mat33	Ef = Tait_Bryan(-fox1(i), 0, -foz1(i));
				Mat33	Et = Tait_Bryan(-tox1(i), 0, -toz1(i));
				// from RFerdestone page 22 for zero translation we have rotation for both forces and moments so:

				rf[0] = -fx1[i];
				rf[1] = 0;
				rf[2] = -fz1[i];
				rt[0] = -tx1[i];
				rt[1] = 0;
				rt[2] = -tz1[i];
				
				Mat66 Cf = plucker(Ef, rf);
				Mat66 Ct = plucker(Et, rt);
				//crosproduct...
				Mat61 rff;
				Mat61 rtt;
				Mat61 rffn;
				Mat61 rttn;

				rff[0][0] = fx[i];
				rff[1][0] = fy[i];
				rff[2][0] = fz[i];
				rff[3][0] = fox[i];
				rff[4][0] = foy[i];
				rff[5][0] = foz[i];

				rtt[0][0] = tx[i];
				rtt[1][0] = ty[i];
				rtt[2][0] = tz[i];
				rtt[3][0] = tox[i];
				rtt[4][0] = toy[i];
				rtt[5][0] = toz[i];

				rffn = Cf*rff;
				rttn = Ct*rtt;


				fx2[i] = rffn[0][0];
				fy2[i] = rffn[1][0];
				fz2[i] = rffn[2][0];
				fox2[i] = rffn[3][0];
				foy2[i] = rffn[4][0];
				foz2[i] = rffn[5][0];

				tx2[i] = rttn[0][0];
				ty2[i] = rttn[1][0];
				tz2[i] = rttn[2][0];
				tox2[i] = rttn[3][0];
				toy2[i] = rttn[4][0];
				toz2[i] = rttn[5][0];

				fx1(i) = tx1(i) = 0;
				//

				fox1(i) = tox1(i) = 0;
				foz1(i) = toz1(i) = 0;
				fz1(i) = tz1(i) = 0;

				if (sizerr < 2){
					fz2(i) = tz2(i) = 0;
					fox2(i) = tox2(i) = 0;
					foz2(i) = toz2(i) = 0;
					fx2(i) = tx2(i) = 0;
				}
			}
			cout << "For this simulation we will use  one translation DOFin  y  axis and one rotaion DoF in y axis" << endl;

		}
		//53
		if (count4 == sizer && count5 == sizer  && count3 != sizer){
			for (int i = 0; i < sizerr; ++i){
				Mat33	Ef = Tait_Bryan(0, -foy1(i), -foz1(i));
				Mat33	Et = Tait_Bryan(0, -toy1(i), -toz1(i));
				// from RFerdestone page 22 for zero translation we have rotation for both forces and moments so:

				rf[0] = -fx1[i];
				rf[1] = 0;
				rf[2] = -fz1[i];
				rt[0] = -tx1[i];
				rt[1] = 0;
				rt[2] = -tz1[i];
				
				Mat66 Cf = plucker(Ef, rf);
				Mat66 Ct = plucker(Et, rt);
				//crosproduct...
				Mat61 rff;
				Mat61 rtt;
				Mat61 rffn;
				Mat61 rttn;

				rff[0][0] = fx[i];
				rff[1][0] = fy[i];
				rff[2][0] = fz[i];
				rff[3][0] = fox[i];
				rff[4][0] = foy[i];
				rff[5][0] = foz[i];

				rtt[0][0] = tx[i];
				rtt[1][0] = ty[i];
				rtt[2][0] = tz[i];
				rtt[3][0] = tox[i];
				rtt[4][0] = toy[i];
				rtt[5][0] = toz[i];

				rffn = Cf*rff;
				rttn = Ct*rtt;


				fx2[i] = rffn[0][0];
				fy2[i] = rffn[1][0];
				fz2[i] = rffn[2][0];
				fox2[i] = rffn[3][0];
				foy2[i] = rffn[4][0];
				foz2[i] = rffn[5][0];

				tx2[i] = rttn[0][0];
				ty2[i] = rttn[1][0];
				tz2[i] = rttn[2][0];
				tox2[i] = rttn[3][0];
				toy2[i] = rttn[4][0];
				toz2[i] = rttn[5][0];

				fx1(i) = tx1(i) = 0;
				//

				foy1(i) = toy1(i) = 0;
				foz1(i) = toz1(i) = 0;
				fz1(i) = tz1(i) = 0;
				if (sizerr < 2){
					fz2(i) = tz2(i) = 0;
					foy2(i) = toy2(i) = 0;
					foz2(i) = toz2(i) = 0;
					fx2(i) = tx2(i) = 0;
				}
			}
			cout << "For this simulation we will use  one translation DOFin  y  axis axis and one rotaion DoF in x axis" << endl;

		}


		///////////////////////////////////////////////////////////////////
		////////////// 2 DOF ROTATION-1 DOF TRANSLATION /////////////////////////////////////
		/////////////////////////////////////////////////////////////////
		//54
		if (count3 == sizer && count5 != sizer  && count4 != sizer){
			for (int i = 0; i < sizerr; ++i){
				Mat33	Ef = Tait_Bryan(-fox1(i), 0, 0);
				Mat33	Et = Tait_Bryan(-tox1(i), 0, 0);
				// from RFerdestone page 22 for zero translation we have rotation for both forces and moments so:
				rf[0] = -fx1[i];
				rf[1] = 0;
				rf[2] = -fz1[i];
				rt[0] = -tx1[i];
				rt[1] = 0;
				rt[2] = -tz1[i];
				
				Mat66 Cf = plucker(Ef, rf);
				Mat66 Ct = plucker(Et, rt);
				//crosproduct...
				Mat61 rff;
				Mat61 rtt;
				Mat61 rffn;
				Mat61 rttn;

				rff[0][0] = fx[i];
				rff[1][0] = fy[i];
				rff[2][0] = fz[i];
				rff[3][0] = fox[i];
				rff[4][0] = foy[i];
				rff[5][0] = foz[i];

				rtt[0][0] = tx[i];
				rtt[1][0] = ty[i];
				rtt[2][0] = tz[i];
				rtt[3][0] = tox[i];
				rtt[4][0] = toy[i];
				rtt[5][0] = toz[i];

				rffn = Cf*rff;
				rttn = Ct*rtt;


				fx2[i] = rffn[0][0];
				fy2[i] = rffn[1][0];
				fz2[i] = rffn[2][0];
				fox2[i] = rffn[3][0];
				foy2[i] = rffn[4][0];
				foz2[i] = rffn[5][0];

				tx2[i] = rttn[0][0];
				ty2[i] = rttn[1][0];
				tz2[i] = rttn[2][0];
				tox2[i] = rttn[3][0];
				toy2[i] = rttn[4][0];
				toz2[i] = rttn[5][0];

				fx1(i) = tx1(i) = 0;
				//

				fox1(i) = tox1(i) = 0;
				fz1(i) = tz1(i) = 0;
				if (sizerr < 2){
					fz2(i) = tz2(i) = 0;
					fox2(i) = tox2(i) = 0;
					fx2(i) = tx2(i) = 0;
				}
			}
			cout << "For this simulation we will use  one translation DOFin  y  axis and two rotaion DoF in y and z axis" << endl;

		}

		//55
		if (count5 == sizer  && count4 != sizer  && count3 != sizer){
			for (int i = 0; i < sizerr; ++i){
				Mat33	Ef = Tait_Bryan(0, 0, -foz1(i));
				Mat33	Et = Tait_Bryan(0, 0, -toz1(i));
				// from RFerdestone page 22 for zero translation we have rotation for both forces and moments so:
				rf[0] = -fx1[i];
				rf[1] = 0;
				rf[2] = -fz1[i];
				rt[0] = -tx1[i];
				rt[1] = 0;
				rt[2] = -tz1[i];
				
				Mat66 Cf = plucker(Ef, rf);
				Mat66 Ct = plucker(Et, rt);
				//crosproduct...
				Mat61 rff;
				Mat61 rtt;
				Mat61 rffn;
				Mat61 rttn;

				rff[0][0] = fx[i];
				rff[1][0] = fy[i];
				rff[2][0] = fz[i];
				rff[3][0] = fox[i];
				rff[4][0] = foy[i];
				rff[5][0] = foz[i];

				rtt[0][0] = tx[i];
				rtt[1][0] = ty[i];
				rtt[2][0] = tz[i];
				rtt[3][0] = tox[i];
				rtt[4][0] = toy[i];
				rtt[5][0] = toz[i];

				rffn = Cf*rff;
				rttn = Ct*rtt;


				fx2[i] = rffn[0][0];
				fy2[i] = rffn[1][0];
				fz2[i] = rffn[2][0];
				fox2[i] = rffn[3][0];
				foy2[i] = rffn[4][0];
				foz2[i] = rffn[5][0];

				tx2[i] = rttn[0][0];
				ty2[i] = rttn[1][0];
				tz2[i] = rttn[2][0];
				tox2[i] = rttn[3][0];
				toy2[i] = rttn[4][0];
				toz2[i] = rttn[5][0];

				fx1(i) = tx1(i) = 0;
				//

				foz1(i) = toz1(i) = 0;
				fz1(i) = tz1(i) = 0;
				if (sizerr < 2){
					fz2(i) = tz2(i) = 0;
					foz2(i) = toz2(i) = 0;
					fx2(i) = tx2(i) = 0;
				}

			}
			cout << "For this simulation we will use  one translation DOFin  y  axis axis and two rotaion DoF in y and x axis" << endl;

		}
		//56
		if (count4 == sizer  && count5 != sizer  && count3 != sizer){
			for (int i = 0; i < sizerr; ++i){
				Mat33	Ef = Tait_Bryan(0, -foy1(i), 0);
				Mat33	Et = Tait_Bryan(0, -toy1(i), 0);
				// from RFerdestone page 22 for zero translation we have rotation for both forces and moments so:
				rf[0] = -fx1[i];
				rf[1] = 0;
				rf[2] = -fz1[i];
				rt[0] = -tx1[i];
				rt[1] = 0;
				rt[2] = -tz1[i];
				
				Mat66 Cf = plucker(Ef, rf);
				Mat66 Ct = plucker(Et, rt);
				//crosproduct...
				Mat61 rff;
				Mat61 rtt;
				Mat61 rffn;
				Mat61 rttn;

				rff[0][0] = fx[i];
				rff[1][0] = fy[i];
				rff[2][0] = fz[i];
				rff[3][0] = fox[i];
				rff[4][0] = foy[i];
				rff[5][0] = foz[i];

				rtt[0][0] = tx[i];
				rtt[1][0] = ty[i];
				rtt[2][0] = tz[i];
				rtt[3][0] = tox[i];
				rtt[4][0] = toy[i];
				rtt[5][0] = toz[i];

				rffn = Cf*rff;
				rttn = Ct*rtt;


				fx2[i] = rffn[0][0];
				fy2[i] = rffn[1][0];
				fz2[i] = rffn[2][0];
				fox2[i] = rffn[3][0];
				foy2[i] = rffn[4][0];
				foz2[i] = rffn[5][0];

				tx2[i] = rttn[0][0];
				ty2[i] = rttn[1][0];
				tz2[i] = rttn[2][0];
				tox2[i] = rttn[3][0];
				toy2[i] = rttn[4][0];
				toz2[i] = rttn[5][0];

				fx1(i) = tx1(i) = 0;
				//
				foy1(i) = toy1(i) = 0;

				if (sizerr < 2){
					foy2(i) = toy2(i) = 0;
					fx2(i) = tx2(i) = 0;
				}


			}
			cout << "For this simulation we will use two translation DOFin   y axis and two rotaion DoF in z and x axis" << endl;

		}
	}


	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



	///////////////////////////////////////////////////////////////////
	////////////// 0 DOF ROTATION-0 DOF TRANSLATION /////////////////////////////////////
	/////////////////////////////////////////////////////////////////
	//57
	if (count == sizer && count1 == sizer && count2 == sizer && count3 == sizer && count4 == sizer && count5 == sizer){
		
		
			cout << "ERROR: Fixed Geometry " << endl;
		

	}
	///////////////////////////////////////////////////////////////////
	////////////// 3 DOF ROTATION-3 DOF TRANSLATION /////////////////////////////////////
	/////////////////////////////////////////////////////////////////
	// there are no if insert so remain as they are the DOFs////////
	///////////////////////////////////////////////////////////////////
	//58
	//TODO SOS change the 6dof if the user set 6dof from opensim case from the begin
	if (count != sizer && count1 != sizer && count2 != sizer && count3 != sizer && count4 != sizer && count5 != sizer){
		
			cout << "For this simulation we will use a six DOFs model three rotation and three translation DOF. " << endl;
			cout << "/////////////////////////////////////////////////////////////////" << endl;
		
		
	}
	//59
	if (count == sizer && count1 != sizer && count2 != sizer && count3 != sizer && count4 != sizer && count5 != sizer){
		for (int i = 0; i < sizerr; ++i){

			Mat33	Ef = Tait_Bryan(0, 0, 0);
			Mat33	Et = Tait_Bryan(0, 0, 0);
			// from RïyFerthestone page 22 for zero translation we have rotation for both forces and moments so:

			rf[0] = -fx1[i];
			rf[1] = 0;
			rf[2] = 0;
			rt[0] = -tx1[i];
			rt[1] = 0;
			rt[2] = 0;



			Mat66 Cf = plucker(Ef, rf);
			Mat66 Ct = plucker(Et, rt);
			//crosproduct...
			Mat61 rff;
			Mat61 rtt;
			Mat61 rffn;
			Mat61 rttn;

			rff[0][0] = fx[i];
			rff[1][0] = fy[i];
			rff[2][0] = fz[i];
			rff[3][0] = fox[i];
			rff[4][0] = foy[i];
			rff[5][0] = foz[i];

			rtt[0][0] = tx[i];
			rtt[1][0] = ty[i];
			rtt[2][0] = tz[i];
			rtt[3][0] = tox[i];
			rtt[4][0] = toy[i];
			rtt[5][0] = toz[i];

			rffn = Cf*rff;
			rttn = Ct*rtt;


			fx2[i] = rffn[0][0];
			fy2[i] = rffn[1][0];
			fz2[i] = rffn[2][0];
			fox2[i] = rffn[3][0];
			foy2[i] = rffn[4][0];
			foz2[i] = rffn[5][0];

			tx2[i] = rttn[0][0];
			ty2[i] = rttn[1][0];
			tz2[i] = rttn[2][0];
			tox2[i] = rttn[3][0];
			toy2[i] = rttn[4][0];
			toz2[i] = rttn[5][0];

			//fx1(i) = tx1(i) = 0;
			//fy1(i) = ty1(i) = 0;
			fx1(i) = tx1(i) = 0;
			if (sizerr < 2){
				//fox1(i) = tox1(i) = 0;
				//foy1(i) = toy1(i) = 0;
				fx2(i) = tx2(i) = 0;
				//fy(i) = ty(i) = 0;
				//fz(i) = tz(i) = 0;
				//fox(i) = tox(i) = 0;
				//foy(i) = toy(i) = 0;
			}
		}
		cout << "For this simulation we will use three rotation DoF in z-x-y axis two translation z-y" << endl;
	}
	//60//
	if (count != sizer && count1 == sizer && count2 != sizer && count3 != sizer && count4 != sizer && count5 != sizer){
		for (int i = 0; i < sizerr; ++i){

			Mat33	Ef = Tait_Bryan(0, 0, 0);
			Mat33	Et = Tait_Bryan(0, 0, 0);
			// from RïyFerthestone page 22 for zero translation we have rotation for both forces and moments so:

			rf[0] = 0;
			rf[1] = -fy1[i];
			rf[2] = 0;
			rt[0] = 0;
			rt[1] = -ty1[i];
			rt[2] = 0;



			Mat66 Cf = plucker(Ef, rf);
			Mat66 Ct = plucker(Et, rt);
			//crosproduct...
			Mat61 rff;
			Mat61 rtt;
			Mat61 rffn;
			Mat61 rttn;

			rff[0][0] = fx[i];
			rff[1][0] = fy[i];
			rff[2][0] = fz[i];
			rff[3][0] = fox[i];
			rff[4][0] = foy[i];
			rff[5][0] = foz[i];

			rtt[0][0] = tx[i];
			rtt[1][0] = ty[i];
			rtt[2][0] = tz[i];
			rtt[3][0] = tox[i];
			rtt[4][0] = toy[i];
			rtt[5][0] = toz[i];

			rffn = Cf*rff;
			rttn = Ct*rtt;


			fx2[i] = rffn[0][0];
			fy2[i] = rffn[1][0];
			fz2[i] = rffn[2][0];
			fox2[i] = rffn[3][0];
			foy2[i] = rffn[4][0];
			foz2[i] = rffn[5][0];

			tx2[i] = rttn[0][0];
			ty2[i] = rttn[1][0];
			tz2[i] = rttn[2][0];
			tox2[i] = rttn[3][0];
			toy2[i] = rttn[4][0];
			toz2[i] = rttn[5][0];
			//fx1(i) = tx1(i) = 0;
			//fy1(i) = ty1(i) = 0;
			fy1(i) = ty1(i) = 0;
			if (sizerr < 2){
				//fox1(i) = tox1(i) = 0;
				//foy1(i) = toy1(i) = 0;
				//fx(i) = tx(i) = 0;
				fy2(i) = ty2(i) = 0;
				//fz(i) = tz(i) = 0;
				//fox(i) = tox(i) = 0;
				//foy(i) = toy(i) = 0;
			}
		}
		cout << "For this simulation we will use three rotation DoF in z-x-y axis two translation z-x" << endl;

	}
	//61//
	if (count != sizer && count1 != sizer && count2 == sizer && count3 == sizer && count4 == sizer && count5 == sizer){
		for (int i = 0; i < sizerr; ++i){

			Mat33	Ef = Tait_Bryan(0, 0, 0);
			Mat33	Et = Tait_Bryan(0, 0, 0);
			// from RïyFerthestone page 22 for zero translation we have rotation for both forces and moments so:

			rf[0] = 0;
			rf[1] = 0;
			rf[2] = -fz[i];
			rt[0] = 0;
			rt[1] = 0;
			rt[2] = -fz[i];



			Mat66 Cf = plucker(Ef, rf);
			Mat66 Ct = plucker(Et, rt);
			//crosproduct...
			Mat61 rff;
			Mat61 rtt;
			Mat61 rffn;
			Mat61 rttn;

			rff[0][0] = fx[i];
			rff[1][0] = fy[i];
			rff[2][0] = fz[i];
			rff[3][0] = fox[i];
			rff[4][0] = foy[i];
			rff[5][0] = foz[i];

			rtt[0][0] = tx[i];
			rtt[1][0] = ty[i];
			rtt[2][0] = tz[i];
			rtt[3][0] = tox[i];
			rtt[4][0] = toy[i];
			rtt[5][0] = toz[i];

			rffn = Cf*rff;
			rttn = Ct*rtt;


			fx2[i] = rffn[0][0];
			fy2[i] = rffn[1][0];
			fz2[i] = rffn[2][0];
			fox2[i] = rffn[3][0];
			foy2[i] = rffn[4][0];
			foz2[i] = rffn[5][0];

			tx2[i] = rttn[0][0];
			ty2[i] = rttn[1][0];
			tz2[i] = rttn[2][0];
			tox2[i] = rttn[3][0];
			toy2[i] = rttn[4][0];
			toz2[i] = rttn[5][0];

			fz1(i) = tz1(i) = 0;
			if (sizerr < 2){
				//fox1(i) = tox1(i) = 0;
				//foy1(i) = toy1(i) = 0;
				//fx(i) = tx(i) = 0;
				//fy(i) = ty(i) = 0;
				fz2(i) = tz2(i) = 0;
				//fox(i) = tox(i) = 0;
				//foy(i) = toy(i) = 0;
			}
		}
		cout << "For this simulation we will use three rotation DoF in z-x-y axis two translation y-x" << endl;

	}

	//62
	if (count != sizer && count1 != sizer && count2 == sizer && count3 != sizer && count4 != sizer && count5 != sizer){
		for (int i = 0; i < sizerr; ++i){

			Mat33	Ef = Tait_Bryan(0, 0, 0);
			Mat33	Et = Tait_Bryan(0, 0, 0);
			// from RïyFerthestone page 22 for zero translation we have rotation for both forces and moments so:

			rf[0] = 0;
			rf[1] = 0;
			rf[2] = -fz1[i];
			rt[0] = 0;
			rt[1] = 0;
			rt[2] = -tz1[i];



			Mat66 Cf = plucker(Ef, rf);
			Mat66 Ct = plucker(Et, rt);
			//crosproduct...
			Mat61 rff;
			Mat61 rtt;
			Mat61 rffn;
			Mat61 rttn;

			rff[0][0] = fx[i];
			rff[1][0] = fy[i];
			rff[2][0] = fz[i];
			rff[3][0] = fox[i];
			rff[4][0] = foy[i];
			rff[5][0] = foz[i];

			rtt[0][0] = tx[i];
			rtt[1][0] = ty[i];
			rtt[2][0] = tz[i];
			rtt[3][0] = tox[i];
			rtt[4][0] = toy[i];
			rtt[5][0] = toz[i];

			rffn = Cf*rff;
			rttn = Ct*rtt;


			fx2[i] = rffn[0][0];
			fy2[i] = rffn[1][0];
			fz2[i] = rffn[2][0];
			fox2[i] = rffn[3][0];
			foy2[i] = rffn[4][0];
			foz2[i] = rffn[5][0];

			tx2[i] = rttn[0][0];
			ty2[i] = rttn[1][0];
			tz2[i] = rttn[2][0];
			tox2[i] = rttn[3][0];
			toy2[i] = rttn[4][0];
			toz2[i] = rttn[5][0];

			fx1(i) = tx1(i) = 0;
			fy1(i) = ty1(i) = 0;
			if (sizerr < 2){
				//fx1(i) = tx1(i) = 0;
				//fox1(i) = tox1(i) = 0;
				//foy1(i) = toy1(i) = 0;
				fx2(i) = tx2(i) = 0;
				fy2(i) = ty2(i) = 0;
				//fz(i) = tz(i) = 0;
				//fox(i) = tox(i) = 0;
				//foy(i) = toy(i) = 0;
			}
		}
		cout << "For this simulation we will use three rotation DoF in z-x-y axis two translation z" << endl;
	}
	//63//
	if (count != sizer && count1 == sizer && count2 != sizer && count3 != sizer && count4 != sizer && count5 != sizer){
		for (int i = 0; i < sizerr; ++i){

			Mat33	Ef = Tait_Bryan(0, 0, 0);
			Mat33	Et = Tait_Bryan(0, 0, 0);
			// from RïyFerthestone page 22 for zero translation we have rotation for both forces and moments so:

			rf[0] = -fx1[i];
			rf[1] = 0;
			rf[2] = -fz1[i];
			rt[0] = -tx1[i];
			rt[1] = 0;
			rt[2] = -fz1[i];


			Mat66 Cf = plucker(Ef, rf);
			Mat66 Ct = plucker(Et, rt);
			//crosproduct...
			Mat61 rff;
			Mat61 rtt;
			Mat61 rffn;
			Mat61 rttn;

			rff[0][0] = fx[i];
			rff[1][0] = fy[i];
			rff[2][0] = fz[i];
			rff[3][0] = fox[i];
			rff[4][0] = foy[i];
			rff[5][0] = foz[i];

			rtt[0][0] = tx[i];
			rtt[1][0] = ty[i];
			rtt[2][0] = tz[i];
			rtt[3][0] = tox[i];
			rtt[4][0] = toy[i];
			rtt[5][0] = toz[i];

			rffn = Cf*rff;
			rttn = Ct*rtt;


			fx2[i] = rffn[0][0];
			fy2[i] = rffn[1][0];
			fz2[i] = rffn[2][0];
			fox2[i] = rffn[3][0];
			foy2[i] = rffn[4][0];
			foz2[i] = rffn[5][0];

			tx2[i] = rttn[0][0];
			ty2[i] = rttn[1][0];
			tz2[i] = rttn[2][0];
			tox2[i] = rttn[3][0];
			toy2[i] = rttn[4][0];
			toz2[i] = rttn[5][0];

			fx1(i) = tx1(i) = 0;
			//fy1(i) = ty1(i) = 0;
			fz1(i) = tz1(i) = 0;
			if (sizerr < 2){
				//fox1(i) = tox1(i) = 0;
				//foy1(i) = toy1(i) = 0;
				fx2(i) = tx2(i) = 0;
				//fy(i) = ty(i) = 0;
				fz2(i) = tz2(i) = 0;
				//fox(i) = tox(i) = 0;
				//foy(i) = toy(i) = 0;
			}

		}
		cout << "For this simulation we will use three rotation DoF in z-x-y axis one translation y" << endl;

	}
	//64//
	if (count == sizer && count1 != sizer && count2 != sizer && count3 == sizer && count4 == sizer && count5 == sizer){
		for (int i = 0; i < sizerr; ++i){

			Mat33	Ef = Tait_Bryan(0, 0, 0);
			Mat33	Et = Tait_Bryan(0, 0, 0);
			// from RïyFerthestone page 22 for zero translation we have rotation for both forces and moments so:

			rf[0] = 0;
			rf[1] = -fy[i];
			rf[2] = -fz[i];
			rt[0] = 0;
			rt[1] = -ty[i];
			rt[2] = -fz[i];


			Mat66 Cf = plucker(Ef, rf);
			Mat66 Ct = plucker(Et, rt);
			//crosproduct...
			Mat61 rff;
			Mat61 rtt;
			Mat61 rffn;
			Mat61 rttn;

			rff[0][0] = fx[i];
			rff[1][0] = fy[i];
			rff[2][0] = fz[i];
			rff[3][0] = fox[i];
			rff[4][0] = foy[i];
			rff[5][0] = foz[i];

			rtt[0][0] = tx[i];
			rtt[1][0] = ty[i];
			rtt[2][0] = tz[i];
			rtt[3][0] = tox[i];
			rtt[4][0] = toy[i];
			rtt[5][0] = toz[i];

			rffn = Cf*rff;
			rttn = Ct*rtt;


			fx2[i] = rffn[0][0];
			fy2[i] = rffn[1][0];
			fz2[i] = rffn[2][0];
			fox2[i] = rffn[3][0];
			foy2[i] = rffn[4][0];
			foz2[i] = rffn[5][0];

			tx2[i] = rttn[0][0];
			ty2[i] = rttn[1][0];
			tz2[i] = rttn[2][0];
			tox2[i] = rttn[3][0];
			toy2[i] = rttn[4][0];
			toz2[i] = rttn[5][0];

			//fx1(i) = tx1(i) = 0;
			fy1(i) = ty1(i) = 0;
			fz1(i) = tz1(i) = 0;
			if (sizerr < 2){
				//fox1(i) = tox1(i) = 0;
				//foy1(i) = toy1(i) = 0;
				//fx(i) = tx(i) = 0;
				fy2(i) = ty2(i) = 0;
				fz2(i) = tz2(i) = 0;
				//fox(i) = tox(i) = 0;
				//foy(i) = toy(i) = 0;
			}
		}
		cout << "For this simulation we will use three rotation DoF in z-x-y axis one translation x" << endl;

	}
	
	}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	///////////////////UPDATE SECTION///////////////////////////////////////////////////
	fx = fx2;
	fy = fy2;
	fz = fz2;
	fox = fox2;
	foy = foy2;
	foz = foz2;

	tx = tx2;
	ty = ty2;
	tz = tz2;
	tox = tox2;
	toy = toy2;
	toz = toz2;
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


	///////////////////////////
	/////Force motion case/////
	///////////////////////////////
	if (behavior == 1){ cout << "END oF Force RESAMPLER" << endl; }
	if (behavior == 2){
		if (sizerr > 2){


			INIReader ini = INIReader(INI_FILE);
			cout << "Do you want a visualizer of pre motion of the FEBio geometry? (y/n)" << endl;
			string f = ini.Get("BASICSETUP", "Visualizer_premotion", "");
			cout << f << endl;
			if (f == "y"){
				//Visualizerfebgeo vl;
				//vl.XMLwritestate(time1, fx1, fy1, fz1, fox1, foy1, foz1, tx1, ty1, tz1, tox1, toy1, toz1);
				//vl.~Visualizerfebgeo();
				ofstream opengeostate;
				
				string resultDir1 = BASE_DIR + ini.Get("FEBIOSTEP", "GEO", "");
				string geof = ini.Get("FEBIOSTEP", "GEOF", "");
				string geos = ini.Get("FEBIOSTEP", "GEOS", "");
				string bd1 = ini.Get("BODYFORCES", "BDNAME1", "");
				string bd2 = ini.Get("BODYFORCES", "BDNAME2", "");
				//char itter = itteration + '0';
				////create a new folder for the analysis/////////
				int sizer = time1.size();

				opengeostate = ofstream(resultDir1 + "/motionstate.mot", ofstream::out);
				opengeostate << "Coordinates" << endl;
				opengeostate << "version=1" << endl;
				opengeostate << "nRows=" << sizer - 1 << endl;
				opengeostate << "nColumns=13" << endl;
				opengeostate << "inDegrees=no" << endl;
				opengeostate << "" << endl;
				opengeostate << "Units are S.I.units(second, meters, Newtons, ...)" << endl;
				opengeostate << "Angles are not in degrees." << endl;
				opengeostate << "" << endl;
				opengeostate << "endheader" << endl;

				opengeostate << "  time  " << bd1 << "_tilt  " << bd1 << "_list  " << bd1 << "_rotation  " << bd1 << "_tz  " << bd1 << "_tx  " << bd1 << "_ty  " << bd2 << "_tilt  " << bd2 << "_list  " << bd2 << "_rotation  " << bd2 << "_tz  " << bd2 << "_tx  " << bd2 << "_ty  " << endl;
				for (int i = 0; i < sizer; ++i){
					opengeostate << "  " << time1[i] << "  " << foz1[i] << "  " << fox1[i] << "  " << foy1[i] << "  " << fz1[i] << "  " << fx1[i] << "  " << fy1[i] << "  " << toz1[i] << "  " << tox1[i] << "  " << toy1[i] << "  " << tz1[i] << "  " << tx1[i] << "  " << ty1[i] << endl;

				}
				Run();

				/////////////////ASK to MAKE CHANGES IN THE POSITIONS OF MOTION///////////////////
				char caser4 = 'y';
				char caser5 = 'y';

				while (caser5 == 'y'){


					cout << "" << endl;
					cout << "Do you want to make changes in position vector based of the pre-FEM_motion_vision you just saw? (y/n)" << endl;
					char caser1;
					cin >> caser1;
					if (caser1 == 'n'){
						caser5 = 'n';
					}

					if (caser1 == 'y'){

						resamplernow('y');

						while (caser4 == 'y'){

							cout << "Which of the first body 6DOF you want to make changes first? (x/y/z/X/Y/Z)" << endl;
							cout << "P.S. the capital X,Y,Z are the rotation degrees of freedom" << endl;
							char L;
							cin >> L;
							cout << "Now give: first time value,second the new offset value of the dof." << endl;
							if (L == 'x'){
								char caser3 = 'y';
								while (caser3 == 'y'){
									double timee;
									double x;
									double caser2;
									cout << "You want : " << endl;
									cout << "1) a time interval with fix value? " << endl;
									cout << "2) a time interval with linear max and min value?" << endl;
									cout << "3) a point? " << endl;
									cout << "choose by but 1/2/3..." << endl;
									cin >> caser2;
									if (caser2 == 1){
										cout << "START TIME:" << endl;
										cin >> timee;
										cout << "END TIME:" << endl;
										double timel;
										cin >> timel;
										cout << "x_value:" << endl;
										cin >> x;
										int i = 0;
										while (time1[i] <= timee){
											++i;
										}
										if (timee == 0){ i = 0; }
										int o = 0;
										while (time1[o] <= timel){
											++o;
										}
										for (int run = i; run < o + 1; ++run){
											fx1[run] = fx1[run] + x;
										}
									}
									if (caser2 == 2){
										double xm;
										cout << "START TIME:" << endl;
										cin >> timee;
										cout << "END TIME:" << endl;
										double timel;
										cin >> timel;
										cout << "max_x_value:" << endl;
										cin >> x;
										cout << "min_x_value:" << endl;
										cin >> xm;

										int i = 0;
										while (time1[i] <= timee){
											++i;
										}
										if (timee == 0){ i = 0; }
										int o = 0;
										while (time1[o] <= timel){
											++o;
										}
										int dist = o - i;
										double dist2 = x - xm;
										double step = dist2 / dist;
										double xl = xm;
										for (int run = i; run < o + 1; ++run){
											xl = xl + step;
											fx1[run] = fx1[run] + xl;
										}
									}
									if (caser2 == 3){
										int i = -1;
										while (i == -1){
											cout << "TIME:" << endl;
											cin >> timee;
											cout << "x_value:" << endl;
											cin >> x;
											int k = 0;
											while (k < time1.size()){
												if (time1[k] == timee){
													i = k;
													
												}
												++k;
											}
											if (i == -1){ cout << "Error no exist value of time of the point" << endl; }
										}
										if (timee == 0){ i = 0; }
										fx1[i] = fx1[i] + x;
									}
									cout << "Do you want an other value " << L << " axis change? (y/n)" << endl;
									cin >> caser3;
								}
							}
							if (L == 'y'){
								char caser3 = 'y';
								while (caser3 == 'y'){
									double timee;
									double y;
									double caser2;
							cout << "You want : " << endl;
							cout << "1) a time interval with fix value? " << endl;
							cout << "2) a time interval with linear max and min value?" << endl;
							cout << "3) a point? " << endl;
							cout << "choose by but 1/2/3..." << endl;
							cin >> caser2;
							if (caser2 == 1){
								cout << "START TIME:" << endl;
								cin >> timee;
								cout << "END TIME:" << endl;
								double timel;
								cin >> timel;
								cout << "y_value:" << endl;
								cin >> y;
								int i = 0;
								while (time1[i] <= timee){
									++i;
								}
								if (timee == 0){ i = 0; }
								int o = 0;
								while (time1[o] <= timel){
									++o;
								}
								for (int run = i; run < o + 1; ++run){
									fy1[run] = fy1[run] + y;
								}
							}
							if (caser2 == 2){
								double ym;
								cout << "START TIME:" << endl;
								cin >> timee;
								cout << "END TIME:" << endl;
								double timel;
								cin >> timel;
								cout << "max_y_value:" << endl;
								cin >> y;
								cout << "min_y_value:" << endl;
								cin >> ym;

								int i = 0;
								while (time1[i] <= timee){
									++i;
								}
								if (timee == 0){ i = 0; }
								int o = 0;
								while (time1[o] <= timel){
									++o;
								}
								int dist = o - i;
								double dist2 = y - ym;
								double step = dist2 / dist;
								double yl =ym;
								for (int run = i; run < o + 1; ++run){
									yl = yl + step;
									fy1[run] = fy1[run] + yl;
								}
							}
							if (caser2 == 3){
								int i = -1;
								while (i == -1){
									cout << "TIME:" << endl;
									cin >> timee;
									cout << "y_value:" << endl;
									cin >> y;
									int k = 0;
									while (k < time1.size()){
										if (time1[k] == timee){
											i = k; 
										}++k;
									}
									if (i == -1){ cout << "Error no exist value of time of the point" << endl; }
								}
								if (timee == 0){ i = 0; }
								fy1[i] = fy1[i] + y;
							}
							cout << "Do you want an other value " << L << " axis change? (y/n)" << endl;
							cin >> caser3;
						}
					}

							
							if (L == 'z'){
								char caser3 = 'y';
								while (caser3 == 'y'){
									double timee;
									double z;
									double caser2;
									cout << "You want : " << endl;
									cout << "1) a time interval with fix value? " << endl;
									cout << "2) a time interval with linear max and min value?" << endl;
									cout << "3) a point? " << endl;
									cout << "choose by but 1/2/3..." << endl;
									cin >> caser2;
									if (caser2 == 1){
										cout << "START TIME:" << endl;
										cin >> timee;
										cout << "END TIME:" << endl;
										double timel;
										cin >> timel;
										cout << "z_value:" << endl;
										cin >> z;
										int i = 0;
										while (time1[i] <= timee){
											++i;
										}
										if (timee == 0){ i = 0; }
										int o = 0;
										while (time1[o] <= timel){
											++o;
										}
										for (int run = i; run < o + 1; ++run){
											fz1[run] = fz1[run] + z;
										}
									}
									if (caser2 == 2){
										double zm;
										cout << "START TIME:" << endl;
										cin >> timee;
										cout << "END TIME:" << endl;
										double timel;
										cin >> timel;
										cout << "max_z_value:" << endl;
										cin >> z;
										cout << "min_z_value:" << endl;
										cin >> zm;

										int i = 0;
										while (time1[i] <= timee){
											++i;
										}
										if (timee == 0){ i = 0; }
										int o = 0;
										while (time1[o] <= timel){
											++o;
										}
										int dist = o - i;
										double dist2 = z - zm;
										double step = dist2 / dist;
										double zl = zm;
										for (int run = i; run < o + 1; ++run){
											zl = zl + step;
											fz1[run] = fz1[run] + zl;
										}
									}
									if (caser2 == 3){
										int i = -1;
										while (i == -1){
											cout << "TIME:" << endl;
											cin >> timee;
											cout << "z_value:" << endl;
											cin >> z;
											int k = 0;
											while (k < time1.size()){
												if (time1[k] == timee){
													i = k;
												}++k;
											}
											if (i == -1){ cout << "Error no exist value of time of the point" << endl; }
										}
										if (timee == 0){ i = 0; }
										fz1[i] = fz1[i] + z;
									}
									cout << "Do you want an other value " << L << " axis change? (y/n)" << endl;
									cin >> caser3;
								}
							}
							if (L == 'X'){
								char caser3 = 'y';
								while (caser3 == 'y'){
									double timee;
									double X;
									double caser2;
									cout << "You want : " << endl;
									cout << "1) a time interval with fix value? " << endl;
									cout << "2) a time interval with linear max and min value?" << endl;
									cout << "3) a point? " << endl;
									cout << "choose by but 1/2/3..." << endl;
									cin >> caser2;
									if (caser2 == 1){
										cout << "START TIME:" << endl;
										cin >> timee;
										cout << "END TIME:" << endl;
										double timel;
										cin >> timel;
										cout << "Rx_value:" << endl;
										cin >> X;
										int i = 0;
										while (time1[i] <= timee){
											++i;
										}if (timee == 0){ i = 0; }
										int o = 0;
										while (time1[o] <= timel){
											++o;
										}
										for (int run = i; run < o + 1; ++run){
											fox1[run] = fox1[run] + X*3.14 / 180;
										}
									}
									if (caser2 == 2){
										double Rxm;
										cout << "START TIME:" << endl;
										cin >> timee;
										cout << "END TIME:" << endl;
										double timel;
										cin >> timel;
										cout << "max_Rx_value:" << endl;
										cin >> X;
										cout << "min_Rx_value:" << endl;
										cin >> Rxm;

										int i = 0;
										while (time1[i] <= timee){
											++i;
										}
										if (timee == 0){ i = 0; }
										int o = 0;
										while (time1[o] <= timel){
											++o;
										}
										int dist = o - i;
										double dist2 = X - Rxm;
										double step = dist2 / dist;
										double Rxl = Rxm;
										for (int run = i; run < o + 1; ++run){
											Rxl = Rxl + step;
											fox1[run] = fox1[run] + Rxl*3.14 / 180;
										}
									}
									if (caser2 == 3){
										int i = -1;
										while (i == -1){
											cout << "TIME:" << endl;
											cin >> timee;
											cout << "Rx_value:" << endl;
											cin >> X;
											int k = 0;
											while (k < time1.size()){
												if (time1[k] == timee){
													i = k;
													++k;
												}
											}
											if (i == -1){ cout << "Error no exist value of time of the point" << endl; }
										}
										if (timee == 0){ i = 0; }
										fox1[i] = fox1[i] + X*3.14 / 180;
									}
									cout << "Do you want an other value " << L << " axis change? (y/n)" << endl;
									cin >> caser3;
								}

							}
							if (L == 'Y'){
								char caser3 = 'y';
								while (caser3 == 'y'){
									double timee;
									double Ry;
									double caser2;
									cout << "You want : " << endl;
									cout << "1) a time interval with fix value? " << endl;
									cout << "2) a time interval with linear max and min value?" << endl;
									cout << "3) a point? " << endl;
									cout << "choose by but 1/2/3..." << endl;
									cin >> caser2;
									if (caser2 == 1){
										cout << "START TIME:" << endl;
										cin >> timee;
										cout << "END TIME:" << endl;
										double timel;
										cin >> timel;
										cout << "Ry_value:" << endl;
										cin >> Ry;
										int i = 0;
										while (time1[i] <= timee){
											++i;
										}
										if (timee == 0){ i = 0; }
										int o = 0;
										while (time1[o] <= timel){
											++o;
										}
										for (int run = i; run < o + 1; ++run){
											foy1[run] = foy1[run] + Ry*3.14 / 180;
										}
									}
									if (caser2 == 2){
										double Rym;
										cout << "START TIME:" << endl;
										cin >> timee;
										cout << "END TIME:" << endl;
										double timel;
										cin >> timel;
										cout << "max_Ry_value:" << endl;
										cin >> Ry;
										cout << "min_Ry_value:" << endl;
										cin >> Rym;

										int i = 0;
										while (time1[i] <= timee){
											++i;
										}
										if (timee == 0){ i = 0; }
										int o = 0;
										while (time1[o] <= timel){
											++o;
										}
										int dist = o - i;
										double dist2 = Ry - Rym;
										double step = dist2 / dist;
										double Ryl = Rym;
										for (int run = i; run < o + 1; ++run){
											Ryl = Ryl + step;
											foy1[run] = foy1[run] + Ryl*3.14 / 180;
										}
									}
									if (caser2 == 3){
										int i = -1;
										while (i == -1){
											cout << "TIME:" << endl;
											cin >> timee;
											cout << "Ry_value:" << endl;
											cin >> Ry;
											int k = 0;
											while (k < time1.size()){
												if (time1[k] == timee){
													i = k; 
												}++k;
											}
											if (i == -1){ cout << "Error no exist value of time of the point" << endl; }
										}
										if (timee == 0){ i = 0; }
										foy1[i] = foy1[i] + Ry*3.14 / 180;
									}
									cout << "Do you want an other value " << L << " axis change? (y/n)" << endl;
									cin >> caser3;
								}
							}
							if (L == 'Z'){
								char caser3 = 'y';
								while (caser3 == 'y'){
									double timee;
									double Rz;
									double caser2;
									cout << "You want : " << endl;
									cout << "1) a time interval with fix value? " << endl;
									cout << "2) a time interval with linear max and min value?" << endl;
									cout << "3) a point? " << endl;
									cout << "choose by but 1/2/3..." << endl;
									cin >> caser2;
									if (caser2 == 1){
										cout << "START TIME:" << endl;
										cin >> timee;
										cout << "END TIME:" << endl;
										double timel;
										cin >> timel;
										cout << "Rz_value:" << endl;
										cin >> Rz;
										int i = 0;
										while (time1[i] <= timee){
											++i;
										}if (timee == 0){ i = 0; }
										int o = 0;
										while (time1[o] <= timel){
											++o;
										}
										for (int run = i; run < o + 1; ++run){
											foz1[run] = foz1[run] + Rz*3.14 / 180;
										}
									}
									if (caser2 == 2){
										double Rzm;
										cout << "START TIME:" << endl;
										cin >> timee;
										cout << "END TIME:" << endl;
										double timel;
										cin >> timel;
										cout << "max_Rz_value:" << endl;
										cin >> Rz;
										cout << "min_Rz_value:" << endl;
										cin >> Rzm;

										int i = 0;
										while (time1[i] <= timee){
											++i;
										}
										if (timee == 0){ i = 0; }
										int o = 0;
										while (time1[o] <= timel){
											++o;
										}
										int dist = o - i;
										double dist2 = Rz - Rzm;
										double step = dist2 / dist;
										double Rzl = Rzm;
										for (int run = i; run < o + 1; ++run){
											Rzl = Rzl + step;
											foz1[run] = foz1[run] + Rzl*3.14 / 180;
										}
									}
									if (caser2 == 3){
										int i = -1;
										while (i == -1){
											cout << "TIME:" << endl;
											cin >> timee;
											cout << "Rz_value:" << endl;
											cin >> Rz;
											int k = 0;
											while (k < time1.size()){
												if (time1[k] == timee){
													i = k;
													
												}++k;
											}
											if (i == -1){ cout << "Error no exist value of time of the point" << endl; }
										}
										if (timee == 0){ i = 0; }
										foz1[i] = foz1[i] + Rz*3.14 / 180;
									}
									cout << "Do you want an other value " << L << " axis change? (y/n)" << endl;
									cin >> caser3;

								}
							}
							cout << "Do you want to change another DoF motion values? (y/n)" << endl;
							cin >> caser4;
						}
						cout << "Do you want to vision again the motion? (y/n)" << endl;
						cin >> caser5;
						if (caser5 == 'y'){

							///WRITTING THE MOTION FILE for geofem
							ofstream opengeostate;
							INIReader ini = INIReader(INI_FILE);
							string resultDir1 = BASE_DIR + ini.Get("FEBIOSTEP", "GEO", "");
							string geof = ini.Get("FEBIOSTEP", "GEOF", "");
							string geos = ini.Get("FEBIOSTEP", "GEOS", "");
							string bd1 = ini.Get("BODYFORCES", "BDNAME1", "");
							string bd2 = ini.Get("BODYFORCES", "BDNAME2", "");
							//char itter = itteration + '0';
							////create a new folder for the analysis/////////
							int sizer = time1.size();

							opengeostate = ofstream(resultDir1 + "/motionstate.mot", ofstream::out);
							opengeostate << "Coordinates" << endl;
							opengeostate << "version=1" << endl;
							opengeostate << "nRows=" << sizer - 1 << endl;
							opengeostate << "nColumns=13" << endl;
							opengeostate << "inDegrees=no" << endl;
							opengeostate << "" << endl;
							opengeostate << "Units are S.I.units(second, meters, Newtons, ...)" << endl;
							opengeostate << "Angles are not in degrees." << endl;
							opengeostate << "" << endl;
							opengeostate << "endheader" << endl;

							opengeostate << "  time  " << bd1 << "_tilt  " << bd1 << "_list  " << bd1 << "_rotation  " << bd1 << "_tz  " << bd1 << "_tx  " << bd1 << "_ty  " << bd2 << "_tilt  " << bd2 << "_list  " << bd2 << "_rotation  " << bd2 << "_tz  " << bd2 << "_tx  " << bd2 << "_ty  " << endl;
							for (int i = 0; i < sizer; ++i){
								opengeostate << "  " << time1[i] << "  " << foz1[i] << "  " << fox1[i] << "  " << foy1[i] << "  " << fz1[i] << "  " << fx1[i] << "  " << fy1[i] << "  " << toz1[i] << "  " << tox1[i] << "  " << toy1[i] << "  " << tz1[i] << "  " << tx1[i] << "  " << ty1[i] << endl;


							}
							caser4 = 'y';
						}
					}
				}
			}
		}

		/*
////////////////////CALL THE WRITERS NOW///////////////////////////////////////////////////////////

		cout << "" << endl;
		cout << "CALL THE FEBio's XML STRUCTORS" << endl;

		BF_structor bffs;
		bffs.begining(itter, kind, sizer1, resultDir1, time1, fx, fy, fz, fox, foy, foz, tx, ty, tz, tox, toy, toz);
		//bffs.~BF_structor();
		BK_structor bkks;
		bkks.begining(itter, kind, 0, sizer1, resultDir1, time1, fx1, fy1, fz1, fox1, foy1, foz1, tx1, ty1, tz1, tox1, toy1, toz1);
		//bkks.~BK_structor();
		*/
	}
	
	
///////////////////////////////////////////////////////////////////////////////////////////////////////////////

	///////////////////////////
	/////Initial velocity case/////
	///////////////////////////////
	if (behavior == 2){
		if (sizerr < 2){
			cout << "" << endl;
			cout << "CALL THE FEBio's Initial Velocities XML STRUCTORS for first time..." << endl;

			Initial_VEL vel;
			vel.write(itter, resultDir1, fx, fy, fz, fox, foy, foz, tx, ty, tz, tox, toy, toz);


		}
	}
	if (behavior == 1){
		if (sizerr < 2){
			cout << "" << endl;
			cout << "CALL THE FEBio's Initial Velocities XML STRUCTORS once again..." << endl;
			cout << fx << "," << fy << "," << fz << "," << fox << "," << foy << "," << foz << "," << tx << "," << ty << "," << tz << "," << tox << "," << toy << "," << toz  << endl;
			Initial_VEL vel;
			vel.write(itter, resultDir1, fx, fy, fz, fox, foy, foz, tx, ty, tz, tox, toy, toz);
			vel.~Initial_VEL();


		}
	}
}

void DOFResample::_JR_resampler(){
	//initialized the store vectors///

	//time1 = return_vect(0);
	fx1 = return_vect(1);
	/////global vector////
	fy1 = return_vect(2);
	fz1 = return_vect(3);
	fox1 = return_vect(4);
	foy1 = return_vect(5);
	foz1 = return_vect(6);

	fx = return_vect(13);
	/////global vector////
	fy = return_vect(14);
	fz = return_vect(15);
	fox = return_vect(16);
	foy = return_vect(17);
	foz = return_vect(18);


	Vector fx2 = fx;
	Vector fy2 = fy;
	Vector fz2 = fz;
	Vector fox2 = fox;
	Vector foy2 = foy;
	Vector foz2 = foz;



	//////////////////////////////////////////////////////////

	int sizer = fx1.size();
	int sizerr = fx.size();



	cout << "JR forces-velocities resampler" << endl;
	Mat31 rf, rt;
	Mat33	Ef, Et;

	for (int i = 0; i < sizerr; ++i){

		Mat33	Ef = Tait_Bryan(fox1[i], foy1[i], foz1[i]);
		// from RFerdestone page 22 for zero translation we have rotation for both forces and moments so:

		rf[0] = (fx1[i] );
		rf[1] = (fy1[i] );
		rf[2] = (fz1[i] );

		Mat66 Ct = plucker(Ef, rf);
		//crosproduct...
		Mat61 rtt;
		Mat61 rttn;


		rtt[0][0] = fx[i];
		rtt[1][0] = fy[i];
		rtt[2][0] = fz[i];
		rtt[3][0] = fox[i];
		rtt[4][0] = foy[i];
		rtt[5][0] = foz[i];

		rttn = Ct*rtt;


		fx2[i] = rttn[0][0];
		fy2[i] = rttn[1][0];
		fz2[i] = rttn[2][0];
		fox2[i] = rttn[3][0];
		foy2[i] = rttn[4][0];
		foz2[i] = rttn[5][0];
	}

	//Detect cases of dof///
	 fx = fx2;
	 fy = fy2;
	 fz = fz2;
     fox = fox2;
	 foy = foy2;
	 foz = foz2;
	 

}

Mat33 DOFResample::Tait_Bryan(double x, double y, double z){
// Euler wiki Tait-Bryan https://en.wikipedia.org/wiki/Euler_angles
	//ZXY case

	Mat33 orientation(0, 0, 0, 0, 0, 0, 0, 0, 0);
	orientation[0][0] = cos(y)*cos(z) - sin(x)*sin(y)*sin(z);
	orientation[0][1] = -cos(x)*sin(z);
	orientation[1][0] = cos(z)*sin(x)*sin(y) + sin(z)*cos(y);
	orientation[0][2] = cos(y)*sin(x)*sin(z) + cos(z)*sin(y);
	orientation[2][0] = -cos(x)*sin(y);
	orientation[1][1] = cos(z)*cos(x);
	orientation[1][2] =-cos(y)*cos(z)*sin(x) + sin(y)*sin(z);
	orientation[2][1] = sin(x);
	orientation[2][2] = cos(x)*cos(y);
	return orientation;
}



Mat66 DOFResample::plucker(Mat33 E, Mat31 r ){
	// Roy Featherstone 'Rigid Body Dynamics Algorithms' page 2

	Mat33 C = -E*cros(r);
	Mat66 B;

	B[0][0] = E[0][0];
	B[0][1] = E[0][1];
	B[0][2] = E[0][2];
	B[0][3] = C[0][0];
	B[0][4] = C[0][1];
	B[0][5] = C[0][2];

		B[1][0] = E[1][0];
		B[1][1] = E[1][1];
		B[1][2] = E[1][2];
		B[1][3] = C[1][0];
		B[1][4] = C[1][1];
		B[1][5] = C[1][2];


	B[2][0] = E[2][0];
	B[2][1] = E[2][1];
	B[2][2] = E[2][2];
	B[2][3] = C[2][0];
	B[2][4] = C[2][1];
	B[2][5] = C[2][2];

	B[3][3] = E[0][0];
	B[3][4] = E[0][1];
	B[3][5] = E[0][2];
	B[3][0] = 0;
	B[3][1] = 0;
	B[3][2] = 0;

	B[4][3] = E[1][0];
	B[4][4] = E[1][1];
	B[4][5] = E[1][2];
	B[4][0] = 0;
	B[4][1] = 0;
	B[4][2] = 0;

	B[5][3] = E[2][0];
	B[5][4] = E[2][1];
	B[5][5] = E[2][2];
	B[5][0] = 0;
	B[5][1] = 0;
	B[5][2] = 0;

	
	return B;
}


Mat33 DOFResample::cros(Mat31 a){

	Mat33 c;
	
	c[0][0] = 0;
	c[0][1] = -a[2][0];
	c[0][2] = a[1][0];
	c[1][0] = a[2][0];
	c[1][1] = 0;
	c[1][2] = -a[0][0];
	c[2][0] = -a[1][0];
	c[2][1] = a[0][0];
		c[2][2] = 0;
	
		return c;
}



void DOFResample::Run()
{
	INIReader ini = INIReader(INI_FILE);
	string resultDir1 = BASE_DIR + ini.Get("FEBIOSTEP", "GEO", "");
	string modelPath = resultDir1 + "/geopensim.osim";
	string programname = ini.Get("PATH", "PROGRAMMNAME", "");
	string name = ini.Get("PATH", "NAME", "");
	string arg1 = " --console suppress ";

	int count = 0;
	string arg2 = modelPath;
	cout << "" << endl;
	cout << "START THE OPENSIM GUI " << endl;

	string workingFileName = arg2;

	string arguments = arg1;

	HANDLE process = ShellExecuteHandler(programname, arguments, name);
	cout << "" << endl;
	cout << "PASS THE .osim file and the .mot FILE IN OPENSIM GUI " << endl;
	string name1 = "";
	string arguments1 = "";
	HANDLE process1 = Shellfile(resultDir1);
}//void

//}

HANDLE DOFResample::ShellExecuteHandler(string program, string args, string name)
{
	HANDLE hProcess = NULL;
	SHELLEXECUTEINFO shellInfo;
	::ZeroMemory(&shellInfo, sizeof(shellInfo));
	shellInfo.cbSize = sizeof(shellInfo);
	shellInfo.fMask = SEE_MASK_FLAG_NO_UI | SEE_MASK_NOCLOSEPROCESS;
	shellInfo.lpVerb = "open";
	shellInfo.lpFile = name.c_str();
	shellInfo.lpParameters = args.c_str();
	shellInfo.lpDirectory = program.c_str();
	shellInfo.nShow = 10;
	if (::ShellExecuteEx(&shellInfo))
	{ // success 
		hProcess = shellInfo.hProcess;
	} // success 
	return hProcess;
}

HANDLE DOFResample::Shellfile(string program)
{
	HANDLE hProcess = NULL;
	SHELLEXECUTEINFO shellInfo;
	::ZeroMemory(&shellInfo, sizeof(shellInfo));
	shellInfo.cbSize = sizeof(shellInfo);
	shellInfo.fMask = SEE_MASK_FLAG_NO_UI | SEE_MASK_NOCLOSEPROCESS;
	shellInfo.lpVerb = "open";
	shellInfo.lpDirectory = program.c_str();
	shellInfo.nShow = 10;
	if (::ShellExecuteEx(&shellInfo))
	{ // success 
		hProcess = shellInfo.hProcess;
	} // success 
	return hProcess;
}


void DOFResample::resamplernow(char x){
	re = x;
}

char DOFResample::return_resamplernow(){ return re; }
///////////////////////////////////////////////////
////////////P.S.//////////////////////
///////////////////////
////////////////
/*
*/

//NOT USED SEE THE NOTES OF FORCES WORD//
/*


*/