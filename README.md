# MultiCoFEA

  A co-operation software between a muskuloskeletal multibody (OpenSim) and finite element (FEBio) analysis software.
The MultiCoFEA is implemented  in C++ and it is an opensource code.There are two choices concerning the musculoskeletal analysis. In both analysis, the trajectory motion of joint’s coordinates are applied as input data. The velocities and forces-moment vectors of joint are computed in first analysis as output data,  contrary to second analysis where the velocities and forces-moment vectors of bones are computed. The user has the choice to regulate some basic characteristics of the musculoskeletal and finite element analysis. Moreover, there is the potentiality for the user to implement a resample of output vector with respect to time so the complexity of the input data of finite element analysis can be reduced, to apply the displacement boundaries of rigid bodies in finite element model manualy or automatically, to tract an efficient time step for the finite element analysis and to have a vision of the motion which the finite element analysis will be reproduced before it solves the geometry. In addition, there is the option by the user to set the finite element analysis be partitioned and solved parallel. These setup file has a .txt format. Lastly, the opensource code creates the XML file of finite element solver and starts the simulation.

![alt text](https://github.com/ece7048/MultiCoFEA/blob/master/MultiCoFEM/models/knee.gif "Logo Title Text 1")

  Initial step: set the OpenSim model (.osim) and the FEBio model in setup.ini file in .../data/setup.ini.
  
  Main step: the user have to set the other variables of the setup.ini file (like the CMC states of motion for the OpenSim model under  observation).
  
  Final step: build and run the .exe
  
# Dependency
It is based on OpenSim 3.3. The user must define the OPENSIM_HOME variable and to add OPENSIM_HOME/bin to path.
It is based on FEBio 2.5 The user must define the FEBio variable and to add FEBio/bin to path.
#  Building
Run CMake in the root directory.
# Running
You can configure your simulation through data/setup.ini, where the path configured by CMake through src/Setting.h. 

