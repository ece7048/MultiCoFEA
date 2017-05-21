# MultiCoFEA

  A co-operation program between multibody (OpenSim) and finite element method (FEBio) analysis programs.
  Used by set the OpenSim model (.osim) and the FEBio model in setup.ini file in .../data/setup.ini.
  Also the user have to set the other variables of the setup.ini file (like the CMC states of motion for the OpenSim model under  observation).
  
# Dependency
It is based on OpenSim 3.3. The user must define the OPENSIM_HOME variable and to add OPENSIM_HOME/bin to path.
It is based on FEBio 2.5 The user must define the FEBio variable and to add FEBio/bin to path.
#  Building
Run CMake in the root directory.
# Running
You can configure your simulation through data/setup.ini, where the path configured by CMake through src/Setting.h. 

# Note 
If someone needs a manual of software just send me an email and ask it please (mixalis.mamalakis@gmail.com).
