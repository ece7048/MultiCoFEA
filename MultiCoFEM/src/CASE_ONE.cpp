#include "CASE_ONE.h"

CASE_ONE::CASE_ONE(void)
{
}

CASE_ONE::~CASE_ONE(void)
{
}

void CASE_ONE::store(Vector fxo, int which){
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


Vector CASE_ONE::return_vect(int which){
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

void CASE_ONE::run(int itteration, string kind[24], int endend, string resultDir2, int pass, char dof){
	INIReader ini = INIReader(INI_FILE);

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
	if (pass == 1){
		TSC t;
		t.stepswriter(timefinal);
		t.~TSC();
	}

		

		cout << "" << endl;
		cout << "Do you want a DoF Resample for case it not need it a DoF of the bodies? (y/n)" << endl;
		string  which =  ini.Get("BASICSETUP", "DOF_Resample", "");
		if (which == ""){ cout << "PLEASE,answer the question!" << endl;
		cin >> which;
		}
		cout << which << endl;
		cout << "" << endl;

		////////////////////////////////////
		////A. YES FORCE-MOTION RESAMPLER CASE/////////
		//////////////////////////////////

		if (which == "y"){

			////////////////////////////////////
			//////////////// a. The initial velocity RESAMPLER  start////////////
			/////////////////////////////////////
			char which2 = which[0];
			cout << "Start the Initial velocity resample detection..." << endl;
			Initial_VEL vel;
			vel.initial(itteration, which2, 2, dof);
			//vel.~Initial_VEL();
			////////////////////////////////////
			//////////////// a. The initial velocity RESAMPLER  end////////////
			/////////////////////////////////////


			cout << "Start the DOF motion resample detection..." << endl;
			////////////////////////////////////
			//////////////// b. The FORCE-MOTION RESAMPLER  start////////////
			/////////////////////////////////////

			DOFResample dres;
			dres.store(timefinal, 0);
			dres.store(fPx, 1);
			dres.store(fPy, 2);
			dres.store(fPz, 3);
			dres.store(fOx, 4);
			dres.store(fOy, 5);
			dres.store(fOz, 6);
			dres.store(tPx, 7);
			dres.store(tPy, 8);
			dres.store(tPz, 9);
			dres.store(tOx, 10);
			dres.store(tOy, 11);
			dres.store(tOz, 12);
			dres.store(fFx, 13);
			dres.store(fFy, 14);
			dres.store(fFz, 15);
			dres.store(fMx, 16);
			dres.store(fMy, 17);
			dres.store(fMz, 18);
			dres.store(tFx, 19);
			dres.store(tFy, 20);
			dres.store(tFz, 21);
			dres.store(tMx, 22);
			dres.store(tMy, 23);
			dres.store(tMz, 24);
			dres.detect(itteration, kind, endend, resultDir2, 2);

			/////global vector////
			timefinal = dres.return_vect(0);
			fPx = dres.return_vect(1);
			fPy = dres.return_vect(2);
			fPz = dres.return_vect(3);
			fOx = dres.return_vect(4);
			fOy = dres.return_vect(5);
			fOz = dres.return_vect(6);
			tPx = dres.return_vect(7);
			tPy = dres.return_vect(8);
			tPz = dres.return_vect(9);
			tOx = dres.return_vect(10);
			tOy = dres.return_vect(11);
			tOz = dres.return_vect(12);
			/////global vector////
			fFx = dres.return_vect(13);
			fFy = dres.return_vect(14);
			fFz = dres.return_vect(15);
			fMx = dres.return_vect(16);
			fMy = dres.return_vect(17);
			fMz = dres.return_vect(18);
			tFx = dres.return_vect(19);
			tFy = dres.return_vect(20);
			tFz = dres.return_vect(21);
			tMx = dres.return_vect(22);
			tMy = dres.return_vect(23);
			tMz = dres.return_vect(24);

			resamplernow = dres.return_resamplernow();
			dres.~DOFResample();



			//////////////////////////////////
			////////////////////////////////////
			//////////////// b. The FORCE-MOTION RESAMPLER  end////////////
			/////////////////////////////////////

			////////////////////////////////////
			////////////////////////////////////
			//////////////// b1. Forces resample for the new positions by user start////////////
			/////////////////////////////////////

			if (resamplernow == 'y'){
				DOFResample dresl2;
				dresl2.store(timefinal, 0);
				dresl2.store(fPx, 1);
				dresl2.store(fPy, 2);
				dresl2.store(fPz, 3);
				dresl2.store(fOx, 4);
				dresl2.store(fOy, 5);
				dresl2.store(fOz, 6);
				dresl2.store(tPx, 7);
				dresl2.store(tPy, 8);
				dresl2.store(tPz, 9);
				dresl2.store(tOx, 10);
				dresl2.store(tOy, 11);
				dresl2.store(tOz, 12);
				dresl2.store(fFx, 13);
				dresl2.store(fFy, 14);
				dresl2.store(fFz, 15);
				dresl2.store(fMx, 16);
				dresl2.store(fMy, 17);
				dresl2.store(fMz, 18);
				dresl2.store(tFx, 19);
				dresl2.store(tFy, 20);
				dresl2.store(tFz, 21);
				dresl2.store(tMx, 22);
				dresl2.store(tMy, 23);
				dresl2.store(tMz, 24);
				dresl2.detect(itteration, kind, endend, resultDir2, 1);

				timefinal = dresl2.return_vect(0);

				/////global vector////
				fPx = dresl2.return_vect(1);
				fPy = dresl2.return_vect(2);
				fPz = dresl2.return_vect(3);
				fOx = dresl2.return_vect(4);
				fOy = dresl2.return_vect(5);
				fOz = dresl2.return_vect(6);
				tPx = dresl2.return_vect(7);
				tPy = dresl2.return_vect(8);
				tPz = dresl2.return_vect(9);
				tOx = dresl2.return_vect(10);
				tOy = dresl2.return_vect(11);
				tOz = dresl2.return_vect(12);
				/////globall vector////
				fFx = dresl2.return_vect(13);
				fFy = dresl2.return_vect(14);
				fFz = dresl2.return_vect(15);
				fMx = dresl2.return_vect(16);
				fMy = dresl2.return_vect(17);
				fMz = dresl2.return_vect(18);
				tFx = dresl2.return_vect(19);
				tFy = dresl2.return_vect(20);
				tFz = dresl2.return_vect(21);
				tMx = dresl2.return_vect(22);
				tMy = dresl2.return_vect(23);
				tMz = dresl2.return_vect(24);
				dresl2.~DOFResample();

				cout << "Resample the initial velocities base the new prescribe trajectory" << endl;
				vel.store(timefinal, 0);
				vel.store(fPx, 1);
				vel.store(fPy, 2);
				vel.store(fPz, 3);
				vel.store(fOx, 4);
				vel.store(fOy, 5);
				vel.store(fOz, 6);
				vel.store(tPx, 7);
				vel.store(tPy, 8);
				vel.store(tPz, 9);
				vel.store(tOx, 10);
				vel.store(tOy, 11);
				vel.store(tOz, 12);
				char which2 = which[0];
				vel.initial(itteration, which2, 1, dof);
				vel.~Initial_VEL();

			}
			////////////////////////////////////
			//////////////// b1. Forces resample for the new positions by user end////////////
			/////////////////////////////////////
		

			//////////////// c. The STRUCTOR SECTION start////////////
			/////////////////////////////////////
			BF_structor bff;
			bff.begining(itteration, kind, endend, resultDir2, timefinal, fFx, fFy, fFz, fMx, fMy, fMz, tFx, tFy, tFz, tMx, tMy, tMz);
			//bff.~BF_structor();
			

			BK_structor bkk;
			bkk.begining(itteration, kind, 0, endend, resultDir2, timefinal, fPx, fPy, fPz, fOx, fOy, fOz, tPx, tPy, tPz, tOx, tOy, tOz);
			//bkk.~BK_structor();

		} // end of yes force resampler



		////////////////////////////////////
		////B. NO MOTION RESAMPLER CASE/////////
		//////////////////////////////////


		if (which == "n"){
			//////////////////////////////////

			////////////////////////////////////
			//////////////// a. The initial velocity RESAMPLER  start////////////
			/////////////////////////////////////
			Initial_VEL vel;
			char which2 = which[0];
			vel.initial(itteration, which2, 2,  dof);
			//vel.~Initial_VEL();
			////////////////////////////////////
			//////////////// a. The initial velocity RESAMPLER end////////////
			/////////////////////////////////////



			cout << "Do you want a visualizer of pre motion of the FEBio geometry? (y/n)" << endl;
			string f = ini.Get("BASICSETUP", "Visualizer_premotion", "");
			if (f == ""){
				cout << "PLEASE,answer the question!" << endl;
				cin >> f;
			}
			cout << f << endl;
			if (f == "y"){

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
				int sizer = timefinal.size();

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

				opengeostate << "  time  " << bd1 << "_tilt  " << bd1 << "_list  " << bd1 << "_rotation  " << bd1 << "_tx  " << bd1 << "_ty  " << bd1 << "_tz  " << bd2 << "_tilt  " << bd2 << "_list  " << bd2 << "_rotation  " << bd2 << "_tx  " << bd2 << "_ty  " << bd2 << "_tz  " << endl;
				for (int i = 0; i < sizer; ++i){
					opengeostate << "  " << timefinal[i] << "  " << fOz[i] << "  " << fOx[i] << "  " << fOy[i] << "  " << fPz[i] << "  " << fPx[i] << "  " << fPy[i] << "  " << tOz[i] << "  " << tOx[i] << "  " << tOy[i] << "  " << tPz[i] << "  " << tPx[i] << "  " << tPy[i] << endl;

				}
				Run();

				////////////////////////////////////
				//////////////// b. The VISION SECTION start////////////
				/////////////////////////////////////

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
						resamplernow = 'y';
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
										while (timefinal[i] <= timee){
											++i;
										}
										if (timee == 0){ i = 0; }
										int o = 0;
										while (timefinal[o] <= timel){
											++o;
										}
										for (int run = i; run < o + 1; ++run){
											fPx[run] = fPx[run] + x;
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
										while (timefinal[i] <= timee){
											++i;
										}
										if (timee == 0){ i = 0; }
										int o = 0;
										while (timefinal[o] <= timel){
											++o;
										}
										int dist = o - i;
										double dist2 = x - xm;
										double step = dist2 / dist;
										double xl = xm;
										for (int run = i; run < o + 1; ++run){
											xl = xl + step;
											fPx[run] = fPx[run] + xl;
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
											while (k < timefinal.size()){
												if (timefinal[k] == timee){
													i = k;
												}++k;
											}
											if (i == -1){ cout << "Error no exist value of time of the point" << endl; }
										}
										if (timee == 0){ i = 0; }
										fPx[i] = fPx[i] + x;
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
										while (timefinal[i] <= timee){
											++i;
										}
										if (timee == 0){ i = 0; }
										int o = 0;
										while (timefinal[o] <= timel){
											++o;
										}
										for (int run = i; run < o + 1; ++run){
											fPy[run] = fPy[run] + y;
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
										while (timefinal[i] <= timee){
											++i;
										}
										if (timee == 0){ i = 0; }
										int o = 0;
										while (timefinal[o] <= timel){
											++o;
										}
										int dist = o - i;
										double dist2 = y - ym;
										double step = dist2 / dist;
										double yl = ym;
										for (int run = i; run < o + 1; ++run){
											yl = yl + step;
											fPy[run] = fPy[run] + yl;
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
											while (k < timefinal.size()){
												if (timefinal[k] == timee){
													i = k;
												}++k;
											}
											if (i == -1){ cout << "Error no exist value of time of the point" << endl; }
										}
										if (timee == 0){ i = 0; }
										fPy[i] = fPy[i] + y;
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
										while (timefinal[i] <= timee){
											++i;
										}
										if (timee == 0){ i = 0; }
										int o = 0;
										while (timefinal[o] <= timel){
											++o;
										}
										for (int run = i; run < o + 1; ++run){
											fPz[run] = fPz[run] + z;
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
										while (timefinal[i] <= timee){
											++i;
										}
										if (timee == 0){ i = 0; }
										int o = 0;
										while (timefinal[o] <= timel){
											++o;
										}
										int dist = o - i;
										double dist2 = z - zm;
										double step = dist2 / dist;
										double zl = zm;
										for (int run = i; run < o + 1; ++run){
											zl = zl + step;
											fPz[run] = fPz[run] + zl;
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
											while (k < timefinal.size()){
												if (timefinal[k] == timee){
													i = k;
												}++k;
											}
											if (i == -1){ cout << "Error no exist value of time of the point" << endl; }
										}
										if (timee == 0){ i = 0; }
										fPz[i] = fPz[i] + z;
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
										while (timefinal[i] <= timee){
											++i;
										}if (timee == 0){ i = 0; }
										int o = 0;
										while (timefinal[o] <= timel){
											++o;
										}
										for (int run = i; run < o + 1; ++run){
											fOx[run] = fOx[run] + X*3.14 / 180;
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
										while (timefinal[i] <= timee){
											++i;
										}
										if (timee == 0){ i = 0; }
										int o = 0;
										while (timefinal[o] <= timel){
											++o;
										}
										int dist = o - i;
										double dist2 = X - Rxm;
										double step = dist2 / dist;
										double Rxl = Rxm;
										for (int run = i; run < o + 1; ++run){
											Rxl = Rxl + step;
											fOx[run] = fOx[run] + Rxl*3.14 / 180;
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
											while (k < timefinal.size()){
												if (timefinal[k] == timee){
													i = k;
												}++k;
											}
											if (i == -1){ cout << "Error no exist value of time of the point" << endl; }
										}
										if (timee == 0){ i = 0; }
										fOx[i] = fOx[i] + X*3.14 / 180;
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
										while (timefinal[i] <= timee){
											++i;
										}
										if (timee == 0){ i = 0; }
										int o = 0;
										while (timefinal[o] <= timel){
											++o;
										}
										for (int run = i; run < o + 1; ++run){
											fOy[run] = fOy[run] + Ry*3.14 / 180;
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
										while (timefinal[i] <= timee){
											++i;
										}
										if (timee == 0){ i = 0; }
										int o = 0;
										while (timefinal[o] <= timel){
											++o;
										}
										int dist = o - i;
										double dist2 = Ry - Rym;
										double step = dist2 / dist;
										double Ryl = Rym;
										for (int run = i; run < o + 1; ++run){
											Ryl = Ryl + step;
											fOy[run] = fOy[run] + Ryl*3.14 / 180;
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
											while (k < timefinal.size()){
												if (timefinal[k] == timee){
													i = k;

												}++k;
											}
											if (i == -1){ cout << "Error no exist value of time of the point" << endl; }
										}
										if (timee == 0){ i = 0; }
										fOy[i] = fOy[i] + Ry*3.14 / 180;
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
										while (timefinal[i] <= timee){
											++i;
										}if (timee == 0){ i = 0; }
										int o = 0;
										while (timefinal[o] <= timel){
											++o;
										}
										for (int run = i; run < o + 1; ++run){
											fOz[run] = fOz[run] + Rz*3.14 / 180;
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
										while (timefinal[i] <= timee){
											++i;
										}
										if (timee == 0){ i = 0; }
										int o = 0;
										while (timefinal[o] <= timel){
											++o;
										}
										int dist = o - i;
										double dist2 = Rz - Rzm;
										double step = dist2 / dist;
										double Rzl = Rzm;
										for (int run = i; run < o + 1; ++run){
											Rzl = Rzl + step;
											fOz[run] = fOz[run] + Rzl*3.14 / 180;
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
											while (k < timefinal.size()){
												if (timefinal[k] == timee){
													i = k;
												}++k;
											}
											if (i == -1){ cout << "Error no exist value of time of the point" << endl; }
										}
										if (timee == 0){ i = 0; }
										fOz[i] = fOz[i] + Rz*3.14 / 180;
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
							int sizer = timefinal.size();

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

							opengeostate << "  time  " << bd1 << "_tilt  " << bd1 << "_list  " << bd1 << "_rotation  " << bd1 << "_tx  " << bd1 << "_ty  " << bd1 << "_tz  " << bd2 << "_tilt  " << bd2 << "_list  " << bd2 << "_rotation  " << bd2 << "_tx  " << bd2 << "_ty  " << bd2 << "_tz  " << endl;
							for (int i = 0; i < sizer; ++i){
								opengeostate << "  " << timefinal[i] << "  " << fOz[i] << "  " << fOx[i] << "  " << fOy[i] << "  " << fPz[i] << "  " << fPx[i] << "  " << fPy[i] << "  " << tOz[i] << "  " << tOx[i] << "  " << tOy[i] << "  " << tPz[i] << "  " << tPx[i] << "  " << tPy[i] << endl;


							}
							caser4 = 'y';
						}
					}
				}
			}


			////////////////////////////////////
			//////////////// b. The VISION SECTION end////////////
			/////////////////////////////////////

			////////////////////////////////////
			//////////////// b1. Forces resample for the new positions by user start////////////
			/////////////////////////////////////

			if (resamplernow == 'y'){
				DOFResample dresl;
				dresl.store(timefinal, 0);
				dresl.store(fPx, 1);
				dresl.store(fPy, 2);
				dresl.store(fPz, 3);
				dresl.store(fOx, 4);
				dresl.store(fOy, 5);
				dresl.store(fOz, 6);
				dresl.store(tPx, 7);
				dresl.store(tPy, 8);
				dresl.store(tPz, 9);
				dresl.store(tOx, 10);
				dresl.store(tOy, 11);
				dresl.store(tOz, 12);
				dresl.store(fFx, 13);
				dresl.store(fFy, 14);
				dresl.store(fFz, 15);
				dresl.store(fMx, 16);
				dresl.store(fMy, 17);
				dresl.store(fMz, 18);
				dresl.store(tFx, 19);
				dresl.store(tFy, 20);
				dresl.store(tFz, 21);
				dresl.store(tMx, 22);
				dresl.store(tMy, 23);
				dresl.store(tMz, 24);
				dresl.detect(itteration, kind, endend, resultDir2, 1);

				timefinal = dresl.return_vect(0);

				/////global vector////
				fPx = dresl.return_vect(1);
				fPy = dresl.return_vect(2);
				fPz = dresl.return_vect(3);
				fOx = dresl.return_vect(4);
				fOy = dresl.return_vect(5);
				fOz = dresl.return_vect(6);
				tPx = dresl.return_vect(7);
				tPy = dresl.return_vect(8);
				tPz = dresl.return_vect(9);
				tOx = dresl.return_vect(10);
				tOy = dresl.return_vect(11);
				tOz = dresl.return_vect(12);
				/////globall vector////
				fFx = dresl.return_vect(13);
				fFy = dresl.return_vect(14);
				fFz = dresl.return_vect(15);
				fMx = dresl.return_vect(16);
				fMy = dresl.return_vect(17);
				fMz = dresl.return_vect(18);
				tFx = dresl.return_vect(19);
				tFy = dresl.return_vect(20);
				tFz = dresl.return_vect(21);
				tMx = dresl.return_vect(22);
				tMy = dresl.return_vect(23);
				tMz = dresl.return_vect(24);
				dresl.~DOFResample();

				cout << "Resample the initial velocities base the new prescribe trajectory" << endl;
				vel.store(timefinal, 0);
				vel.store(fPx, 1);
				vel.store(fPy, 2);
				vel.store(fPz, 3);
				vel.store(fOx, 4);
				vel.store(fOy, 5);
				vel.store(fOz, 6);
				vel.store(tPx, 7);
				vel.store(tPy, 8);
				vel.store(tPz, 9);
				vel.store(tOx, 10);
				vel.store(tOy, 11);
				vel.store(tOz, 12);
				char which2 = which[0];
				vel.initial(itteration, which2, 1, dof);
				vel.~Initial_VEL();

			}
			////////////////////////////////////
			//////////////// b1. Forces resample for the new positions by user end////////////
			/////////////////////////////////////

			////////////////////////////////////
			//////////////// c. The STRUCTOR SECTION start////////////
			/////////////////////////////////////
			////////////////////
			
		
			BF_structor bff;
			bff.begining(itteration, kind, endend, resultDir2, timefinal, fFx, fFy, fFz, fMx, fMy, fMz, tFx, tFy, tFz, tMx, tMy, tMz);
			//bff.~BF_structor();
			
			BK_structor bkk;
			bkk.begining(itteration, kind, 0, endend, resultDir2, timefinal, fPx, fPy, fPz, fOx, fOy, fOz, tPx, tPy, tPz, tOx, tOy, tOz);
			//bkk.~BK_structor();
			////////////////////////////////////
			//////////////// c. The STRUCTOR SECTION end////////////
			/////////////////////////////////////

		}//end of no force resampler
		//cout << "debug" << endl;
	
}

void CASE_ONE::Run()
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
	cout << "PASS THE .osim FILE AND THE .mot FILE IN OPENSIM GUI " << endl;
	string name1 = "";
	string arguments1 = "";
	HANDLE process1 = Shellfile(resultDir1);


}//void


//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

HANDLE CASE_ONE::ShellExecuteHandler(string program, string args, string name)
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
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

HANDLE CASE_ONE::Shellfile(string program)
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
