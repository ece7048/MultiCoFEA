# MULTI-Co-FEA

"For this co-simulation it will need the states after the two bodies of interest's CMC analysis. Do you have them? (y/n)"

 "The analysis you have to do are:  "
"Inverse Kinematics (IK) Inverse Dynamic (ID) Reduce Residual Actuator (RRA) Compute Muscle Control (CMC) " 
"In setup.ini you have to set ALL the variables of these analysis in IK in variable GRFNAME you have to set "
"the name before the _grf.trc file because it used as output in RRA and CMC and need it for these program..."

"#####################################################################################################################################################################"
"SAME BASIC INITIAL OPTIONS FOR THE ANALYSIS " 
"1. Before run the exe file be sure that the Center Of Mass (COM) of the two bodies in FEBio are in (0,0,0) point" 
"2. In the [BODYFORCES] section of setup.ini and setup1.ini you have to set the Joints from 1-6 with first rotations dof and then translations with axis order x-y-z" 
"3. If a DoF of joint is fixed set it as 0!!!" 
"4. In the set up file in the [FEBIOSTEP] in GEO, GEOF, GEOS  set the two .stl or .vpl or .obj Rigid Geometries of the FEBio file. " 
 "   The Rigid Geometries have to be in millimeter. " 
"5. Be sure that in [INVERSEKINEMATICSTATIC] section of setup.ini file in variabe RESULT_DIR, you have named the static motion " 
"   file with | stmotion.mot | name or the analysis will not start!" 
 "6. If you want to set the FEM stragety DOF set in setup.ini file the field Forced_DOF, Prescribed_DOF,Fixed_DOF " 
 "   with the traslation dof and the rotation in case. You have to have in mind that if a rotation DOF set prescrided " 
 "   then the other rotation dof have to be prescribed or fixed not free, same with the forced case (forced or fixed)." 
 "   Set first the traslation dof with xyz (no space) and then the rotation with rxryrz (no space) seperate by _" 
 "   If you want nan dof set forced fixed or prescribed set it with NAN."  
 "   The DOFs you want to be free (translation or all the rotation), without acting force or prescribed motion or be fixed, just do not entry them in the variables." 
"   Finaly, if you are not sure for the stragety wait and set the DOF at the end of the program and leave the variables "
 "   which already mention with no character, empty (Forced_DOF, Prescribed_DOF,Fixed_DOF). " 
"   SOS: If you want to set a rotation dof prescribed and you want to set another rotation dof free set in setup.ini file the variable: Joint_free = y ." 
 "   The free dof you want just not set it in prescribed or fixed variabel. If you want the free dof to set a force lc then set it in Forced_DOF varible. " 
 "7. You have to set the two Rigid bodies as material 1 and 2, specially if you set the Joint_free = y. " 
 "8. You have to set in model path the FEBio model. This model has to have the Globals,Material,Geometry,Boundary,Contact,Costraints,Discrete sections" 
"   These section are not modified by this program so you have to set them by your self before start the analysis in MODELFeb variable in PATH section (setup.ini)" 
"THANKS FOR YOUR PATIENCE!!" 
"#####################################################################################################################################################################"




# THE SETUP FILE:

setup.ini is a set up file where the user have to set same basic input paths and variables. If the user has already the state file of motion and the model (after RRA analysis) can set them in [BODYFORCE] STATE and the [PATH] MODEL variable respectivly.
The fields {INVERSEKINEMATIC], [RRA], [CMC] is not nessecery in that case. The [InVERSEKINEMATICSTATIC] is necessery for the calibration step for sure if the analysis of FEBIO will be the forces or motion in bothe bodies of joint (RB).
If the user has not the CMC states has to set all the field above for do the program these analysis before set the FEBio. The [PATH] MODEL and [BODYFORCE] STATE do not need to set it will be automatically from the programm.
