# MULTI-Co-FEA

	cout << "///////////////////////////////////////////////////////////////////////////////////////////////////////////////////" << endl;
	cout << "///////////////////////////ooooo///////////////////////////////////////////////////ooooo///////////////////////////" << endl;
	cout << "///////////////////////////ooooo///////////////////////////////////////////////////ooooo///////////////////////////" << endl;
	cout << "///////////////////////////oo/ OpenSource C++ Program coding from Michail Mamalakis/oo///////////////////////////" << endl;
	cout << "///////////////////////////ooooo/////////////oo//////////////////oo////////////////ooooo///////////////////////////" << endl;
	cout << "///////////////////////////ooooo/////////////oo/////////////////oo/////////////////ooooo///////////////////////////" << endl;
	cout << "/////////////////////////////////////////////oo//////oooo//////oo//////////////////////////////////////////////////" << endl;
	cout << "//////////////////////////////////////////////oo/////oooo/////oo///////////////////////////////////////////////////" << endl;
	cout << "///////////////////////////////////////////////oo////oooo////oo////////////////////////////////////////////////////" << endl;
	cout << "////////////////////////////////////////////////oo////oo///oo//////////////////////////////////////////////////////" << endl;
	cout << "//////////////////////////////////////////////////ooooooooo////////////////////////////////////////////////////////" << endl;
	cout << "////////////////////////////////////////////////////ooo////////////////////////////////////////////////////////////" << endl;
	cout << "////////////////////////////////////////////////////ooo////////////////////////////////////////////////////////////" << endl;
	cout << "////////////////////////////////////////////////////ooo////////////////////////////////////////////////////////////" << endl;
	cout << "//////////////////////////////////////////////////oooooo///////////////////////////////////////////////////////////" << endl;
	cout << "////////////////////////////////////////////////////oo/////////////////////////////////////////////////////////////" << endl;
	cout << "////////////////////////////////////////////////oo////oo///////////////////////////////////////////////////////////" << endl;
	cout << "///////////////////////////////////////////////oo//////oo//////////////////////////////////////////////////////////" << endl;
	cout << "/////////////////////////////////////////////oo/////////oo/////////////////////////////////////////////////////////" << endl;
	cout << "////////////////////////////////////////////oo//////////oo/////////////////////////////////////////////////////////" << endl;
	cout << "//////////////////////////////////////////oo///////////oo//////////////////////////////////////////////////////////" << endl;
	cout << "////////////////////////////////////////oo////////////oo///////////////////////////////////////////////////////////" << endl;
	cout << "///////////////////////////////////////oo////////////oo////////////////////////////////////////////////////////////" << endl;
	cout << "//////////////////////////////////////o/////////////ooooo//////////////////////////////////////////////////////////" << endl;
	cout << "/////////////////////////////////////o///////////////////oooo///oooo/////////////ooooo/////o/////o/////o////////////" << endl;
	cout << "////////////////////////////////////////////////////////o//////o//o/////////////oo////////o/////o/o/o/o////////////" << endl;
	cout << "///////////////////////////////////////////////////////o//////o//o/////ooo////////oo/////o/////o//o//o/////////////" << endl;
	cout << "Copyright (c) all rights reserved/////////////////////oooo///oooo/////////////ooooo/////o/////o/////o/////////////" << endl;
	cout << "///////////////////////////////////////////////////////////////////////////////////////////////////////////////////" << endl;



"For this co-simulation it will need the states after the two bodies of interest's CMC analysis. Do you have them? (y/n)"

 "The analysis you have to do are:  "
"Inverse Kinematics (IK) Inverse Dynamic (ID) Reduce Residual Actuator (RRA) Compute Muscle Control (CMC) " 
"In setup.ini you have to set ALL the variables of these analysis in IK in variable GRFNAME you have to set "
"the name before the _grf.trc file because it used as output in RRA and CMC and need it for these program..."

cout << "#####################################################################################################################################################################" << endl;
	cout << "SAME BASIC INITIAL OPTIONS FOR THE ANALYSIS:" << endl;
	cout << "1. Before run the exe file be sure that the Center Of Mass (COM) of the two bodies in FEBio are in (0,0,0) point" << endl;
	cout << "2. In the [BODYFORCES] section of setup.ini and setup1.ini you have to set the Joints from 1-6 with first rotations dof and then translations with axis order x-y-z" << endl;
	cout << "3. If a DoF of joint is fixed set it as 0!!!" << endl;
	cout << "4. In the set up file in the [FEBIOSTEP] in GEO, GEOF, GEOS  set the two .stl or .vpl or .obj Rigid Geometries of the FEBio file. " << endl;
	cout << "   The Rigid Geometries have to be in millimeter. " << endl;
	cout << "5. Be sure that in [INVERSEKINEMATICSTATIC] section of setup.ini file in variabe RESULT_DIR, you have named the static motion " << endl;
	cout << "   file with | stmotion.mot | name or the analysis will not start!" << endl;
	cout << "6. If you want to set the FEM stragety DOF set in setup.ini file the field Forced_DOF, Prescribed_DOF,Fixed_DOF " << endl;
	cout << "   with the traslation dof and the rotation in case. You have to have in mind that if a rotation DOF set prescrided " << endl;
	cout << "   then the other rotation dof have to be prescribed or fixed not free, same with the forced case (forced or fixed)." << endl;
	cout << "   Set first the traslation dof with xyz (no space) and then the rotation with rxryrz (no space) seperate by _" << endl;
	cout << "   If you want nan dof set forced fixed or prescribed set it with NAN." << endl; 
	cout << "   The DOFs you want to be free, without acting force or prescribed motion or be fixed, just do not entry them in the variables." << endl;
	cout << "   Finaly, if you are not sure for the stragety wait and set the DOF at the end of the program and leave the variables " << endl;
	cout << "   which already mention with no character, empty (Forced_DOF, Prescribed_DOF,Fixed_DOF). " << endl;


	cout << "#####################################################################################################################################################################" << endl;




# THE SETUP FILE:

setup.ini is a set up file where the user have to set same basic input paths and variables. If the user has already the state file of motion and the model (after RRA analysis) can set them in [BODYFORCE] STATE and the [PATH] MODEL variable respectivly.
The fields {INVERSEKINEMATIC], [RRA], [CMC] is not nessecery in that case. The [InVERSEKINEMATICSTATIC] is necessery for the calibration step for sure if the analysis of FEBIO will be the forces or motion in bothe bodies of joint (RB).
If the user has not the CMC states has to set all the field above for do the program these analysis before set the FEBio. The [PATH] MODEL and [BODYFORCE] STATE do not need to set it will be automatically from the programm.