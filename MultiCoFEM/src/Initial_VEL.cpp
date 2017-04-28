#include <direct.h>
#include "Settings.h"
#include "INIReader.h"
#include <iostream>
#include <fstream>
#include "Initial_VEL.h"
#include "DOFResample.h"

Initial_VEL::Initial_VEL(void)
{
}

Initial_VEL::~Initial_VEL(void)
{
}
void Initial_VEL::store(Vector fxo, int which){
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

}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


Vector Initial_VEL::return_vect(int which){
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

}

void Initial_VEL::initial(int itteration, char kase, int behavior,char dof){

	
		INIReader ini = INIReader(INI_FILE);
		ofstream valuenon;
		ofstream valueinn;
		ofstream valpo;
		ofstream valpi;
		//ofstream valueno;
		string modelPath = BASE_DIR + ini.Get("PATH", "MODELS", "");
		string resultDir = BASE_DIR + ini.Get("PATH", "RESULT_DIR", "");
		string resultDir1 = BASE_DIR + ini.Get("PATH", "RESULT_DIR2", "");
		double t0 = ini.GetReal("BODYFORCES", "START_TIME", 0);
		double tf = ini.GetReal("BODYFORCES", "END_TIME", 0);
		string stateFile = BASE_DIR + ini.Get("BODYFORCES", "STATE", "");
		string xmlpath = BASE_DIR + ini.Get("PATH", "XML", "");
		string bodyname1 = ini.Get("BODYFORCES", "BDNAME1", "");
		string bodyname2 = ini.Get("BODYFORCES", "BDNAME2", "");
		string bodyname3 = ini.Get("BODYFORCES", "BDNAME1", "");
		string bodyname4 = ini.Get("BODYFORCES", "BDNAME2", "");
		string joint12 = ini.Get("BODYFORCES", "JOINT", "");
		string joint1 = ini.Get("BODYFORCES", "JOINT1", "");
		string joint2 = ini.Get("BODYFORCES", "JOINT2", "");
		string joint3 = ini.Get("BODYFORCES", "JOINT3", "");
		string joint4 = ini.Get("BODYFORCES", "JOINT4", "");
		string joint5 = ini.Get("BODYFORCES", "JOINT5", "");
		string joint6 = ini.Get("BODYFORCES", "JOINT6", "");
		//ofstream valueno;
		Model model2(modelPath);
		char itter = itteration + '0';
		////create a new folder for the analysis/////////
		String newfolder = resultDir1 + itter;
		string resultDir2 = newfolder;

		//////////////////////////////////////END//////////////////////////////////////////
		////////////////////////////////////////////////////////////////////////

		Vector fx(1, 1);
		Vector tx(1, 1);
		Vector fy(1, 1);
		Vector ty(1, 1);
		Vector fz(1, 1);
		Vector tz(1, 1);
		Vector fox(1, 1);
		Vector tox(1, 1);
		Vector foy(1, 1);
		Vector toy(1, 1);
		Vector foz(1, 1);
		Vector toz(1, 1);
		if (behavior == 2){

		//////////////////////////////////////////////////////////////////////////////////////////////////
		///////////////////////body forces//////////////////////////////////////////////
		////////////////////////////////////////////////////////////////

		char num = itteration + '0';

		String path = resultDir + "/_BodyForceAnalysis" + num;



		Array<double> fempx;
		Array<double> tibpx;
		Array<double> fempy;
		Array<double> tibpy;
		Array<double> fempz;
		Array<double> tibpz;
		Array<double> femox;
		Array<double> tibox;
		Array<double> femoy;
		Array<double> tiboy;
		Array<double> femoz;
		Array<double> tiboz;

		Array<double> femux;
		Array<double> tibux;
		Array<double> femuy;
		Array<double> tibuy;
		Array<double> femuz;
		Array<double> tibuz;

		Array<double> femùx;
		Array<double> tibùx;
		Array<double> femùy;
		Array<double> tibùy;
		Array<double> femùz;
		Array<double> tibùz;
		Array<double> tim;



		Storage  vel(path + "/_BodyForceAnalysis.sto");

		vel.getTimeColumn(tim);
		vel.getDataColumn(bodyname3 + "_px", fempx);
		vel.getDataColumn(bodyname4 + "_px", tibpx);
		vel.getDataColumn(bodyname3 + "_py", fempy);
		vel.getDataColumn(bodyname4 + "_py", tibpy);
		vel.getDataColumn(bodyname3 + "_pz", fempz);
		vel.getDataColumn(bodyname4 + "_pz", tibpz);
		vel.getDataColumn(bodyname3 + "_ox", femox);
		vel.getDataColumn(bodyname4 + "_ox", tibox);
		vel.getDataColumn(bodyname3 + "_oy", femoy);
		vel.getDataColumn(bodyname4 + "_oy", tiboy);
		vel.getDataColumn(bodyname3 + "_oz", femoz);
		vel.getDataColumn(bodyname4 + "_oz", tiboz);

		vel.getDataColumn(bodyname3 + "_ux", femux);
		vel.getDataColumn(bodyname4 + "_ux", tibux);
		vel.getDataColumn(bodyname3 + "_uy", femuy);
		vel.getDataColumn(bodyname4 + "_uy", tibuy);
		vel.getDataColumn(bodyname3 + "_uz", femuz);
		vel.getDataColumn(bodyname4 + "_uz", tibuz);
		vel.getDataColumn(bodyname3 + "_ùx", femùx);
		vel.getDataColumn(bodyname4 + "_ùx", tibùx);
		vel.getDataColumn(bodyname3 + "_ùy", femùy);
		vel.getDataColumn(bodyname4 + "_ùy", tibùy);
		vel.getDataColumn(bodyname3 + "_ùz", femùz);
		vel.getDataColumn(bodyname4 + "_ùz", tibùz);



		int endend = tim.size();
		Vector time(endend, 1);
		Vector fx1(endend, 1);
		Vector tx1(endend, 1);
		Vector fy1(endend, 1);
		Vector ty1(endend, 1);
		Vector fz1(endend, 1);
		Vector tz1(endend, 1);
		Vector fox1(endend, 1);
		Vector tox1(endend, 1);
		Vector foy1(endend, 1);
		Vector toy1(endend, 1);
		Vector foz1(endend, 1);
		Vector toz1(endend, 1);


		fox = femùz[0];
		foy = femùx[0];
		foz = femùy[0];
		tox = tibùz[0];
		toy = tibùx[0];
		toz = tibùy[0];
		fx = femuz[0];
		fy = femux[0];
		fz = femuy[0];
		tx = tibuz[0];
		ty = tibux[0];
		tz = tibuy[0];

		for (int i = 0; i < endend; ++i){

			fx1[i] = fempz[i];
			tx1[i] = tibpz[i];
			fy1[i] = fempx[i];
			ty1[i] = tibpx[i];
			fz1[i] = fempy[i];
			tz1[i] = tibpy[i];
			fox1[i] = femoz[i];
			tox1[i] = tiboz[i];
			foy1[i] = femox[i];
			toy1[i] = tibox[i];
			foz1[i] = femoy[i];
			toz1[i] = tiboy[i];
		}
		//DC offset of position filter!
		double mean1 = 0;
		double mean2 = 0;
		double mean3 = 0;
		double mean4 = 0;
		double mean5 = 0;
		double mean6 = 0;
		for (int i = 0; i < endend; ++i){


			fx1[i] = fx1[i] * 10000;
			int parast1 = (int)fx1[i];
			fx1[i] = parast1 * 0.0001;
			fy1[i] = fy1[i] * 10000;
			int parasty1 = (int)fy1[i];
			fy1[i] = parasty1 * 0.0001;
			fz1[i] = fz1[i] * 10000;
			int parastz1 = (int)fz1[i];
			fz1[i] = parastz1 * 0.0001;

			tx1[i] = tx1[i] * 10000;
			int parast11 = (int)tx1[i];
			tx1[i] = parast11 * 0.0001;
			ty1[i] = ty1[i] * 10000;
			int parasty11 = (int)ty1[i];
			ty1[i] = parasty11 * 0.0001;
			tz1[i] = tz1[i] * 10000;
			int parastz11 = (int)tz1[i];
			tz1[i] = parastz11 * 0.0001;


			mean1 = fz1[i] + mean1;

			mean2 = fy1[i] + mean2;

			mean3 = fx1[i] + mean3;

			mean4 = tz1[i] + mean4;

			mean5 = tx1[i] + mean5;

			mean6 = ty1[i] + mean6;

		}
		int mean11 = (mean1 / endend) * 10000;
		int mean21 = (mean2 / endend) * 10000;
		int mean31 = (mean3 / endend) * 10000;
		int mean41 = (mean4 / endend) * 10000;
		int mean51 = (mean5 / endend) * 10000;
		int mean61 = (mean6 / endend) * 10000;

		double mean12 = mean11 * 0.0001;
		double mean22 = mean21 * 0.0001;
		double mean32 = mean31 * 0.0001;
		double mean42 = mean41 * 0.0001;
		double mean52 = mean51 * 0.0001;
		double mean62 = mean61 * 0.0001;
		cout << " DC filter of mean values of position translation vector... " << endl;

		for (int i = 0; i < endend; ++i){

			fz1[i] = (fz1[i] - (mean12)) ;

			fy1[i] = (fy1[i] - (mean22)) ;

			fx1[i] = (fx1[i] - (mean32)) ;

			tz1[i] = (tz1[i] - (mean42)) ;

			tx1[i] = (tx1[i] - (mean52)) ;

			ty1[i] = (ty1[i] - (mean62)) ;

		}

		
		if (dof == 'P'){
			cout << "we will continue with point of interest detection motion for the " << bodyname1 << " and " << bodyname2 << " now..." << endl;
		}
		if (dof == 'J'){
			//for (int i = 0; i < endend; ++i){


			if (joint6 == "0"){
				fx1 = 0;
				tx1 = 0;
			}


			if (joint4 == "0"){
				fy1 = 0;
				ty1 = 0;
			}


			if (joint5 == "0"){
				fz1 = 0;
				tz1 = 0;
			}


			if (joint3 == "0"){
				fox1 = 0;
				tox1 = 0;
			}


			if (joint1 == "0"){
				foy1 = 0;
				toy1 = 0;
			}


			if (joint2 == "0"){
				foz1 = 0;
				toz1 = 0;
			}



		}
		//////////////////////////////////////////////////////////////////////////////

		string kind[24] = { "l", "l", "l", "l", "l", "l", "l", "l", "l", "l", "l", "l", "l", "l", "l", "l", "l", "l", "l", "l", "l", "l", "l", "l" };
		//DOFResample df;
		//inverse the angular with the linear for the corect resample of initial velocity

		if (kase == 'y'){
			DOFResample dres;
			dres.store(time, 0);
			dres.store(fx1, 1);
			dres.store(fy1, 2);
			dres.store(fz1, 3);
			dres.store(fox1, 4);
			dres.store(foy1, 5);
			dres.store(foz1, 6);
			dres.store(tx1, 7);
			dres.store(ty1, 8);
			dres.store(tz1, 9);
			dres.store(tox1, 10);
			dres.store(toy1, 11);
			dres.store(toz1, 12);
			dres.store(fx, 13);
			dres.store(fy, 14);
			dres.store(fz, 15);
			dres.store(fox, 16);
			dres.store(foy, 17);
			dres.store(foz, 18);
			dres.store(tx, 19);
			dres.store(ty, 20);
			dres.store(tz, 21);
			dres.store(tox, 22);
			dres.store(toy, 23);
			dres.store(toz, 24);
			dres.detect(itteration, kind, endend, resultDir2, 2);
			dres.~DOFResample();

		}
		if (kase == 'n'){
			write(itteration, resultDir2, fx, fy, fz, fox, foy, foz, tx, ty, tz, tox, toy, toz);
		}
		if (kase != 'n'&& kase != 'y'){
			cout << "ERROR..no correct answer" << endl;
		}
	}

	if (behavior == 3){

		//////////////////////////////////////////////////////////////////////////////////////////////////
		///////////////////////body forces//////////////////////////////////////////////
		////////////////////////////////////////////////////////////////

		Array<double> jux;
		Array<double> juy;
		Array<double> juz;
		Array<double> jùx;
		Array<double> jùy;
		Array<double> jùz;
		Array<double> tim;
		Storage  motion(stateFile);
		motion.getTimeColumn(tim);
		//not used
		/*
		Array<double> jpx;
		Array<double> jpy;
		Array<double> jpz;
		Array<double> jox;
		Array<double> joy;
		Array<double> joz;
		
		*/
		
		

		if (joint1 != "0"){
			//motion.getDataColumn(joint1, jox);
			motion.getDataColumn(joint1 + "_u", jùx);
		}
		if (joint2 != "0"){
			//motion.getDataColumn(joint2, joy);
			motion.getDataColumn(joint2 + "_u", jùy);

		}
		if (joint3 != "0"){
			//motion.getDataColumn(joint3, joz);
			motion.getDataColumn(joint3 + "_u", jùz);

		}
		if (joint4 != "0"){
			//motion.getDataColumn(joint4, jpx);
			motion.getDataColumn(joint4 + "_u", jux);

		}
		if (joint5 != "0"){
			//motion.getDataColumn(joint5, jpy);
			motion.getDataColumn(joint5 + "_u", juy);

		}
		if (joint6 != "0"){
			//motion.getDataColumn(joint6, jpz);
			motion.getDataColumn(joint6 + "_u", juz);

		}



		


		int endend = tim.size();
		Vector time(endend, 1);
		Vector fx(endend, 1);
		Vector tx(endend, 1);
		Vector fy(endend, 1);
		Vector ty(endend, 1);
		Vector fz(endend, 1);
		Vector tz(endend, 1);
		Vector fox(endend, 1);
		Vector tox(endend, 1);
		Vector foy(endend, 1);
		Vector toy(endend, 1);
		Vector foz(endend, 1);
		Vector toz(endend, 1);
		//not used not need of resamle in this face
		/*
		Vector fx1(endend, 1);
		Vector tx1(endend, 1);
		Vector fy1(endend, 1);
		Vector ty1(endend, 1);
		Vector fz1(endend, 1);
		Vector tz1(endend, 1);
		Vector fox1(endend, 1);
		Vector tox1(endend, 1);
		Vector foy1(endend, 1);
		Vector toy1(endend, 1);
		Vector foz1(endend, 1);
		Vector toz1(endend, 1);
		*/
		for (int i = 0; i < endend; ++i){
			time[i] = tim[i]*100000;
			int parast1 = (int)time[i];
			time[i] = parast1 * 0.00001;
			/*
			tx1[i] = 0;
			ty1[i] = 0;
			tz1[i] = 0;
			tox1[i] = 0;
			toy1[i] = 0;
			toz1[i] = 0;

			if (joint3 == "0"){
				fox1[i] = 0;

			}
			if (joint3 != "0"){
				fox1[i] = joz[i];
			}

			if (joint1 == "0"){
				foy1[i] = 0;

			}
			if (joint1 != "0"){
				foy1[i] = jox[i];
			}

			if (joint2 == "0"){
				foz1[i] = 0;

			}
			if (joint2 != "0"){
				foz1[i] = joy[i];
			}

			if (joint6 == "0"){
				fx1[i] = 0;

			}
			if (joint6 != "0"){
				fx1[i] = jpz[i];
			}

			if (joint4 == "0"){
				fy1[i] = 0;

			}
			if (joint4 != "0"){
				fy1[i] = jpx[i];
			}

			if (joint5 == "0"){
				fz1[i] = 0;

			}
			if (joint5 != "0"){
				fz1[i] = jpy[i];
			}
			*/
		}
		int f = 0;
		for (int i=0; i < endend; ++i){
			if (time[i] > t0 || time[i] == t0){ f = i; i = endend + 1; }
		}
		tx[0] = 0;
		ty[0] = 0;
		tz[0] = 0;
		tox[0] = 0;
		toy[0] = 0;
		toz[0] = 0;

		cout << t0 << endl;
		cout << "the time number is: " << f << endl;
		cout << "and the time is: " << time[f] << endl;

		if (joint3 == "0"){
			fox = 0;

		}
		if (joint3 != "0"){
			fox = jùz[f];
		}

		if (joint1 == "0"){
			foy = 0;

		}
		if (joint1 != "0"){
			foy = jùx[f];
		}

		if (joint2 == "0"){
			foz = 0;

		}
		if (joint2 != "0"){
			foz = jùy[f];
		}

		if (joint6 == "0"){
			fx = 0;

		}
		if (joint6 != "0"){
			fx = juz[f];
		}

		if (joint4 == "0"){
			fy = 0;

		}
		if (joint4 != "0"){
			fy = jux[f];
		}

		if (joint5 == "0"){
			fz = 0;

		}
		if (joint5 != "0"){
			fz = juy[f];
		}


		//Not used
		/*
		for (int i = 0; i < endend; ++i){


			fx1[i] = fx1[i] * 10000;
			int parast1 = (int)fx1[i];
			fx1[i] = parast1 * 0.0001;
			fy1[i] = fy1[i] * 10000;
			int parasty1 = (int)fy1[i];
			fy1[i] = parasty1 * 0.0001;
			fz1[i] = fz1[i] * 10000;
			int parastz1 = (int)fz1[i];
			fz1[i] = parastz1 * 0.0001;

			tx1[i] = tx1[i] * 10000;
			int parast11 = (int)tx1[i];
			tx1[i] = parast11 * 0.0001;
			ty1[i] = ty1[i] * 10000;
			int parasty11 = (int)ty1[i];
			ty1[i] = parasty11 * 0.0001;
			tz1[i] = tz1[i] * 10000;
			int parastz11 = (int)tz1[i];
			tz1[i] = parastz11 * 0.0001;


			
		}
		*/
		fx[0] = fx[0] * 1000;
	
		fy[0] = fy[0] * 1000;
	
		fz[0] = fz[0] * 1000;
		

		tx[0] = tx[0] * 1000;
	
		ty[0] = ty[0] * 1000;

		tz[0] = tz[0] * 1000;
		

		//not used
		/*
		
		string kind[24] = { "l", "l", "l", "l", "l", "l", "l", "l", "l", "l", "l", "l", "l", "l", "l", "l", "l", "l", "l", "l", "l", "l", "l", "l" };
		//DOFResample df;
		//inverse the angular with the linear for the corect resample of initial velocity

		if (kase == 'y'){
			DOFResample dres;
			dres.store(time, 0);
			dres.store(fx1, 1);
			dres.store(fy1, 2);
			dres.store(fz1, 3);
			dres.store(fox1, 4);
			dres.store(foy1, 5);
			dres.store(foz1, 6);
			dres.store(tx1, 7);
			dres.store(ty1, 8);
			dres.store(tz1, 9);
			dres.store(tox1, 10);
			dres.store(toy1, 11);
			dres.store(toz1, 12);
			dres.store(fx, 13);
			dres.store(fy, 14);
			dres.store(fz, 15);
			dres.store(fox, 16);
			dres.store(foy, 17);
			dres.store(foz, 18);
			dres.store(tx, 19);
			dres.store(ty, 20);
			dres.store(tz, 21);
			dres.store(tox, 22);
			dres.store(toy, 23);
			dres.store(toz, 24);
			dres.detect(itteration, kind, endend, resultDir2, 2);
			dres.~DOFResample();

		}
		*/
		if (kase == 'n'){
			write(itteration, resultDir2, fx, fy, fz, fox, foy, foz, tx, ty, tz, tox, toy, toz);
		}
		if (kase != 'n'&& kase != 'y'){
			cout << "ERROR..no correct answer" << endl;
		}
	}

	if (behavior == 1){
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
	//ifstream number;
	//number = ifstream(resultDir1 + "/initialvelnumber" + itter + ".txt");
	//Vector d(13,1);
	std::fstream number(resultDir1 + itter + "/initialvelnumber" + itter + ".txt", std::ios_base::in);
	double a, b, c, d, e, f, g, h, k, l, m, n;
	if (number.is_open()){
		

		number >> a >> b >> c >> d >> e >> f >> g >> h >> k >> l >> m >> n;

		printf("%f\t%f\t%f\t%f\t%f\t%f\%f\t%f\t%f\t%f\t%f\t%f\n", a, b, c, d, e, f, g, h, k, l, m, n);

		getchar();
	}
	//string line;
	//for (int i = 0; i < 13; i++)
	//{
		//if (number.is_open()){
			//PASS Load data/////////
			//getline(number, line);
			 //istringstream buffer(line);
			 //buffer >> d[i];
			 //cout << d[i] << endl;
		//}

	//}
	
	fx[0] = a;// d[0];
	fy[0] = b;// d[1];
	fz[0] = c;// d[2];
	fox[0] = d;// d[3];
	foy[0] = e;// d[4];
	foz[0] = f;// d[5];
	tx[0] = g;// d[6];
	ty[0] = h;// d[7];
	tz[0] = k;// d[8];
	tox[0] = l;// d[9];
	toy[0] = m;// d[10];
	toz[0] = n;// d[11];


	DOFResample dres2;
	dres2.store(time1, 0);
	dres2.store(fx1, 1);
	dres2.store(fy1, 2);
	dres2.store(fz1, 3);
	dres2.store(fox1, 4);
	dres2.store(foy1, 5);
	dres2.store(foz1, 6);
	dres2.store(tx1, 7);
	dres2.store(ty1, 8);
	dres2.store(tz1, 9);
	dres2.store(tox1, 10);
	dres2.store(toy1, 11);
	dres2.store(toz1, 12);
	dres2.store(fx, 13);
	dres2.store(fy, 14);
	dres2.store(fz, 15);
	dres2.store(fox, 16);
	dres2.store(foy, 17);
	dres2.store(foz, 18);
	dres2.store(tx, 19);
	dres2.store(ty, 20);
	dres2.store(tz, 21);
	dres2.store(tox, 22);
	dres2.store(toy, 23);
	dres2.store(toz, 24);
	string kind[24] = { "l", "l", "l", "l", "l", "l", "l", "l", "l", "l", "l", "l", "l", "l", "l", "l", "l", "l", "l", "l", "l", "l", "l", "l" };
	dres2.detect(itteration, kind, time1.size(), resultDir2, 1);
	dres2.~DOFResample();

	}

	
}


void Initial_VEL::write(int iteration, string resultDir1, Vector fxf, Vector fyf, Vector fzf, Vector foxf, Vector foyf, Vector fozf, Vector txf, Vector tyf, Vector tzf, Vector toxf, Vector toyf, Vector tozf)
{
	INIReader ini = INIReader(INI_FILE);
	cout << "Writting the initial_velocities..." << endl;
	char itter = iteration + '0';
	ofstream velocity;
	ofstream number;
	velocity = ofstream(resultDir1 + "/initialvel" + itter + ".txt", ofstream::out);
	//cout << fxf* 1000 << endl;

	//////dof detection///////////////
	string presc11 = ini.Get("BASICSETUP", "Prescribed_DOF_1", "");


	string presc21 = ini.Get("BASICSETUP", "Prescribed_DOF_2", "");
	int fox, foy, tox, toy, foz, toz, tz, tx, ty, fx, fy, fz, fox1, foy1, tox1, toy1, foz1, toz1, tx1, ty1, tz1, fx1, fy1, fz1;
	fox = foy = tox = toy = foz = toz = tz, tx, ty = fx = fy = fz = fox1 = foy1 = tox1 = toy1 = foz1 = toz1 = tx1 = ty1 = tz1 = fx1 = fy1 = fz1=1;

	if (presc11 == ""){
		cout << "Give now the DOF velocity  of prescribed dof first traslation, second rotation seperated by _ for the first Rigid Body" << endl;
		cout << "If you do not want any DOF Prescribed set NAN, you can set only rotation if you want." << endl;
		string forc;
		cin >> forc;
		presc11 = forc;
	}
	if (presc21 == ""){
		cout << "give now the DOF velocity  of prescribed dof first traslation, second rotation seperated by _ for the second Rigid Body" << endl;
		cout << "If you do not want any DOF Prescribed set NAN, you can set only rotation if you want." << endl;
		string forc;
		cin >> forc;
		presc21 = forc;
	}

	//////////////////first RB body/////////////////////

	if (presc11 == "NAN"){ fx1 = fy1 = fz1 = fox1 = foy1 = foz1 = 0; }
	if (presc11 != "NAN"){
		if (presc11[0] != 'r'){
			if (presc11[0] != 'x' && presc11[1] != 'x' && presc11[2] != 'x'){ fx1 = 0; }
			if (presc11[0] != 'y' && presc11[1] != 'y' && presc11[2] != 'y'){ fy1 = 0; }
			if (presc11[0] != 'z' && presc11[1] != 'z' && presc11[2] != 'z'){ fz1 = 0; }
			if (presc11[1] == '_' || presc11[2] == '_' || presc11[3] == '_'){

				if (presc11[3] != 'x'&& presc11[4] != 'x' && presc11[5] != 'x' && presc11[6] != 'x' && presc11[7] != 'x'&& presc11[8] != 'x'&& presc11[9] != 'x'&& presc11[10] != 'x'&& presc11[11] != 'x'){ fox1 = 0; }
				if (presc11[3] != 'y'&& presc11[4] != 'y' && presc11[5] != 'y' && presc11[6] != 'y' && presc11[7] != 'y'&& presc11[8] != 'y'&& presc11[9] != 'y'&& presc11[10] != 'y'&& presc11[11] != 'y'){ foy1 = 0; }
				if (presc11[3] != 'z'&& presc11[4] != 'z' && presc11[5] != 'z' && presc11[6] != 'z' && presc11[7] != 'z'&& presc11[8] != 'z'&& presc11[9] != 'z'&& presc11[10] != 'z'&& presc11[11] != 'z'){ foz1 = 0; }
			}
			if (presc11[1] != '_' && presc11[2] != '_' && presc11[3] != '_'){
				fox1 = 0; foy1 = 0; foz1 = 0;
			}
		}
		if (presc11[0] == 'r'){
			if (presc11[1] != 'x' && presc11[3] != 'x' && presc11[5] != 'x'){ fox1 = 0; }
			if (presc11[1] != 'y' && presc11[3] != 'y' && presc11[5] != 'y'){ foy1 = 0; }
			if (presc11[1] != 'z' && presc11[3] != 'z' && presc11[5] != 'z'){ foz1 = 0; }
			fx1 = 0;
			fy1 = 0;
			fz1 = 0;
		}
	}
	//////////////////////////////second RB////////////////////////////////////////////////////////
	if (presc21 == "NAN"){ tx1 = ty1 = tz1 = tox1 = toy1 = toz1 = 0; }
	if (presc21 != "NAN"){
		if (presc21[0] != 'r'){
			if (presc21[0] != 'x' && presc21[1] != 'x' && presc21[2] != 'x'){ tx1 = 0; }
			if (presc21[0] != 'y' && presc21[1] != 'y' && presc21[2] != 'y'){ ty1 = 0; }
			if (presc21[0] != 'z' && presc21[1] != 'z' && presc21[2] != 'z'){ tz1 = 0; }
			if (presc21[1] == '_' || presc21[2] == '_' || presc21[3] == '_'){

				if (presc21[3] != 'x'&& presc21[4] != 'x' && presc21[5] != 'x' && presc21[6] != 'x' && presc21[7] != 'x'&& presc21[8] != 'x'&& presc21[9] != 'x'&& presc21[10] != 'x'&& presc21[11] != 'x'){ tox1 = 0; }
				if (presc21[3] != 'y'&& presc21[4] != 'y' && presc21[5] != 'y' && presc21[6] != 'y' && presc21[7] != 'y'&& presc21[8] != 'y'&& presc21[9] != 'y'&& presc21[10] != 'y'&& presc21[11] != 'y'){ toy1 = 0; }
				if (presc21[3] != 'z'&& presc21[4] != 'z' && presc21[5] != 'z' && presc21[6] != 'z' && presc21[7] != 'z'&& presc21[8] != 'z'&& presc21[9] != 'z'&& presc21[10] != 'z'&& presc21[11] != 'z'){ toz1 = 0; }
			}
			if (presc21[1] != '_' && presc21[2] != '_' && presc21[3] != '_'){
				tox1 = 0; toy1 = 0; toz1 = 0;
			}
		}
		if (presc21[0] == 'r'){
			if (presc21[1] != 'x' && presc21[3] != 'x' && presc21[5] != 'x'){ tox1 = 0; }
			if (presc21[1] != 'y' && presc21[3] != 'y' && presc21[5] != 'y'){ toy1 = 0; }
			if (presc21[1] != 'z' && presc21[3] != 'z' && presc21[5] != 'z'){ toz1 = 0; }
			tx1 = 0;
			ty1 = 0;
			tz1 = 0;
		}
	}

	string presc1 = ini.Get("BASICSETUP", "Forced_DOF_1", "");
	
	string presc2 = ini.Get("BASICSETUP", "Forced_DOF_2", "");
	
	if (presc1 == ""){
		cout << "give now the DOF velocity  of  Forced dof first traslation, second rotation seperated by _ for the first Rigid Body" << endl;
		cout << "If you do not want any DOF Forced set NAN, you can set only rotation if you want." << endl;
		string forc;
		cin >> forc;
		presc1 = forc;
	}
	if (presc2 == ""){
		cout << "give now the DOF velocity  of  Forced dof first traslation, second rotation seperated by _ for the second Rigid Body" << endl;
		cout << "If you do not want any DOF Forced set NAN, you can set only rotation if you want." << endl;
		string forc;
		cin >> forc;
		presc2 = forc;
	}


	if (presc1 == "NAN"){ fx = fy = fz = fox = foy = foz = 0; }
	if (presc1 != "NAN"){
		if (presc1[0] != 'r'){
			if (presc1[0] != 'x' && presc1[1] != 'x' && presc1[2] != 'x'){ fx = 0; }
			if (presc1[0] != 'y' && presc1[1] != 'y' && presc1[2] != 'y'){ fy = 0; }
			if (presc1[0] != 'z' && presc1[1] != 'z' && presc1[2] != 'z'){ fz = 0; }
			if (presc1[1] == '_' || presc1[2] == '_' || presc1[3] == '_'){

				if (presc1[3] != 'x'&& presc1[4] != 'x' && presc1[5] != 'x' && presc1[6] != 'x' && presc1[7] != 'x'&& presc1[8] != 'x'&& presc1[9] != 'x'&& presc1[10] != 'x'&& presc1[11] != 'x'){ fox = 0; }
				if (presc1[3] != 'y'&& presc1[4] != 'y' && presc1[5] != 'y' && presc1[6] != 'y' && presc1[7] != 'y'&& presc1[8] != 'y'&& presc1[9] != 'y'&& presc1[10] != 'y'&& presc1[11] != 'y'){ foy = 0; }
				if (presc1[3] != 'z'&& presc1[4] != 'z' && presc1[5] != 'z' && presc1[6] != 'z' && presc1[7] != 'z'&& presc1[8] != 'z'&& presc1[9] != 'z'&& presc1[10] != 'z'&& presc1[11] != 'z'){ foz = 0; }
			}
			if (presc1[1] != '_' && presc1[2] != '_' && presc1[3] != '_'){
				fox = 0; foy = 0; foz = 0;
			}
		}
		if (presc1[0] == 'r'){
			if (presc1[1] != 'x' && presc1[3] != 'x' && presc1[5] != 'x'){ fox = 0; }
			if (presc1[1] != 'y' && presc1[3] != 'y' && presc1[5] != 'y'){ foy = 0; }
			if (presc1[1] != 'z' && presc1[3] != 'z' && presc1[5] != 'z'){ foz = 0; }
			fx = 0;
			fy = 0;
			fz = 0;
		}
	}

	if (presc2 == "NAN"){ tx = ty = tz = tox = toy = toz = 0; }
	if (presc2 != "NAN"){
		if (presc2[0] != 'r'){
			if (presc2[0] != 'x' && presc2[1] != 'x' && presc2[2] != 'x'){ tx = 0; }
			if (presc2[0] != 'y' && presc2[1] != 'y' && presc2[2] != 'y'){ ty = 0; }
			if (presc2[0] != 'z' && presc2[1] != 'z' && presc2[2] != 'z'){ tz = 0; }
			if (presc2[1] == '_' || presc2[2] == '_' || presc2[3] == '_'){

				if (presc2[3] != 'x'&& presc2[4] != 'x' && presc2[5] != 'x' && presc2[6] != 'x' && presc2[7] != 'x'&& presc2[8] != 'x'&& presc2[9] != 'x'&& presc2[10] != 'x'&& presc2[11] != 'x'){ tox = 0; }
				if (presc2[3] != 'y'&& presc2[4] != 'y' && presc2[5] != 'y' && presc2[6] != 'y' && presc2[7] != 'y'&& presc2[8] != 'y'&& presc2[9] != 'y'&& presc2[10] != 'y'&& presc2[11] != 'y'){ toy = 0; }
				if (presc2[3] != 'z'&& presc2[4] != 'z' && presc2[5] != 'z' && presc2[6] != 'z' && presc2[7] != 'z'&& presc2[8] != 'z'&& presc2[9] != 'z'&& presc2[10] != 'z'&& presc2[11] != 'z'){ toz = 0; }
			}
			if (presc2[1] != '_' && presc2[2] != '_' && presc2[3] != '_'){
				tox = 0; toy = 0; toz = 0;
			}
		}
		if (presc2[0] == 'r'){
			if (presc2[1] != 'x' && presc2[3] != 'x' && presc2[5] != 'x'){ tox = 0; }
			if (presc2[1] != 'y' && presc2[3] != 'y' && presc2[5] != 'y'){ toy = 0; }
			if (presc2[1] != 'z' && presc2[3] != 'z' && presc2[5] != 'z'){ toz = 0; }
			tx = 0;
			ty = 0;
			tz = 0;
		}
	}

	if (fx1 == 0 && fx == 0){ fxf[0] = 0; }
	if (fy1 == 0 && fy == 0){ fyf[0] = 0; }
	if (fz1 == 0 && fz == 0){ fzf[0] = 0; }
	if (fox1 == 0 && fox == 0){ foxf[0] = 0; }
	if (foy1 == 0 && foy == 0){ foyf[0] = 0; }
	if (foz1 == 0 && foz == 0){ fozf[0] = 0; }
	if (tx1 == 0 && tx == 0){ txf[0] = 0; }
	if (ty1 == 0 && ty == 0){ tyf[0] = 0; }
	if (tz1 == 0 && tz == 0){ tzf[0] = 0; }
	if (tox1 == 0 && tox == 0){ toxf[0] = 0; }
	if (toy1 == 0 && toy == 0){ toyf[0] = 0; }
	if (toz1 == 0 && toz == 0){ tozf[0] = 0; }
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	velocity << "<rigid_body mat = " << '"' << "1" << '"' << ">" << endl;
	velocity << "<initial_velocity>" << fxf[0] * 1000 << "," << fyf[0] * 1000 << "," << fzf[0] * 1000 << " </initial_velocity>" << endl;
	velocity << "<initial_angular_velocity>" << foxf[0] << "," << foyf[0] << "," << fozf[0] << " </initial_angular_velocity>" << endl;
	velocity << " </rigid_body>" << endl;
	
	velocity << "<rigid_body mat = " << '"' << "2" << '"' << ">" << endl;
	velocity << "<initial_velocity>" << txf[0] * 1000 << "," << tyf[0] * 1000 << "," << tzf[0] * 1000 << " </initial_velocity>" << endl;
	velocity << "<initial_angular_velocity>" << toxf[0] << "," << toyf[0] << "," << tozf[0] << " </initial_angular_velocity>" << endl;
	velocity << " </rigid_body>" << endl;


	number = ofstream(resultDir1 + "/initialvelnumber" + itter + ".txt", ofstream::out);
	number << fxf[0]  << endl;
	number << fyf[0]  << endl;
	number << fzf[0]  << endl;
	number << foxf[0] << endl;
	number << foyf[0] << endl;
	number << fozf[0] << endl;
	number << txf[0]  << endl;
	number << tyf[0]  << endl;
	number << tzf[0]  << endl;
	number << toxf[0] << endl;
	number << toyf[0] << endl;
	number << tozf[0] << endl;
	
	//number << fxf[0] << fyf[0] << fzf[0] << foxf[0]	<< foyf[0] << fozf[0] << txf[0] << tyf[0] << tzf[0] << toxf[0]<< toyf[0]<< tozf[0] << endl;



}




//////////////////////
/////PS///////////
//////////////////
/*
////////
//1///
///
//1B..
if (body == 'y'){
	//compute relative positions and forces//
	DOFResample dresq;
	dresq.store(time, 0);
	dresq.store(fx1, 1);
	dresq.store(fy1, 2);
	dresq.store(fz1, 3);
	dresq.store(fox1, 4);
	dresq.store(foy1, 5);
	dresq.store(foz1, 6);
	dresq.store(tx1, 7);
	dresq.store(ty1, 8);
	dresq.store(tz1, 9);
	dresq.store(tox1, 10);
	dresq.store(toy1, 11);
	dresq.store(toz1, 12);
	dresq.store(fx, 13);
	dresq.store(fy, 14);
	dresq.store(fz, 15);
	dresq.store(fox, 16);
	dresq.store(foy, 17);
	dresq.store(foz, 18);
	dresq.store(tx, 19);
	dresq.store(ty, 20);
	dresq.store(tz, 21);
	dresq.store(tox, 22);
	dresq.store(toy, 23);
	dresq.store(toz, 24);
	dresq._1B_resampler();

	time = dresq.return_vect(0);

	/////global vector////
	tx = dresq.return_vect(19);
	ty = dresq.return_vect(20);
	tz = dresq.return_vect(21);
	tox = dresq.return_vect(22);
	toy = dresq.return_vect(23);
	toz = dresq.return_vect(24);
	dresq.~DOFResample();
	//(because we will use the velocity of femur acting we take the distance base them first fx-tx)
	fx = fx - tx;

	fy = fy - ty;

	fz = fz - tz;

	tx = 0;

	ty = 0;

	tz = 0;

	fox = fox - tox;

	foy = foy - toy;

	foz = foz - toz;

	tox = 0;

	toy = 0;

	toz = 0;
}
*/