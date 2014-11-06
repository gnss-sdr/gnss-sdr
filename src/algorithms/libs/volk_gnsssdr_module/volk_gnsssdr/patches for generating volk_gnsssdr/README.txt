########################################################################
# Patching original volk module
########################################################################
In order to fit the GNSS-SDR needs, the original volk module must be patched.

The folder containing this file has some patches to automatize the process and
modify the files quickly. To apply them you will need to run the following command:
$ patch -p5 < /Path/Of/The/Patch/nameOfThePatch.patch

The number after “-p” may change, read the patch documentation for more help.

You may need this information if you want to recreate the volk_gnsssdr module again
or you want to update the volk_gnsssdr module with the improvements introduced by GNURadio.

########################################################################
########################################################################
# Operations apply by the patches and other information (not needed if you know how to apply the patches!!!)
########################################################################
########################################################################

To create the volk module you will need to follow the following steps:
In order to understand and follow the creation and setup of the volk_gnsssdr module I will use some absolute paths:/Users/andres/Github/gnuradio => a cloned repository of the GNURadio project./Users/andres/Github/gnss-sdr => a cloned repository of the GNSS- SDR project.

########################################################################
#FIRST STEP: using volk_modtool to create a new volk module
########################################################################
GNURadio offers a tool called volk_modtool to create and manage new volk modules and their proto-kernels. The steps to create the volk_gnsssdr module are:

1) Export the PYTHONPATH, that indicates where volk_modtool is:
$ export PYTHONPATH=/Users/andres/Github/gnuradio/volk/python

2) Go to the folder where volk_modtool executable is:$ cd /Users/andres/Github/gnuradio/volk/python/volk_modtool

3) Execute volk_modtool indicating that we want to create a new volk module (-i):$ ./volk_modtool -i

4) volk_modtool will ask us about the name of the newly created module, the destination folder where you want to store it and the base module (the base module is the volk module inside the GNURadio project):name: gnsssdrdestination: /Users/andres/Github/gnss-sdr/src/algorithms/libsbase: /Users/andres/github/gnuradio/volk

########################################################################
#SECOND STEP: add proto-kernels to the module
########################################################################
After creating the module you will need to add some proto-kernels to it. To accomplish it you will need to:1) Copy your proto-kernels inside the /kernels folder. Copy the ORC implementations inside the /orc folder. Copy the macros implementations inside the /kernels/CommonMacros folder. (those folders are found in the root of the volk_gnsssdr module)
2) Add one profiling line for each of the proto-kernels inside the /apps/volk_gnsssdr_profile.cc file. 

3) Add one test line for each of the proto-kernels inside the /lib/testqa.cc file.########################################################################
#THIRD STEP: modifications to allow profiling of some proto-kernels with special parameters
########################################################################Some of the proto-kernels that GNSS-SDR needs are not supported by the profiling environment of the volk_gnsssdr module. In order to profile them some modifications need to be done to two files:1) Modify /src/algorithms/libs/volk_gnsssdr/lib/qa_utils.cc At the first part of this file there are defined the parameters supported by the environment. The number after run_cast_test indicates the total number of parameters passed to the proto-kernel (input +output parameters). The other part indicates the type of the data passed. Inside func(....) you will need to add the same number of buffs[ ] that the one specified after run_cast_test.

2) Modify /src/algorithms/libs/volk_gnsssdr/lib/qa_utils.h In the header you will need to add typedefs for the new definitions made in the .cc file. Take care: you will need to add the same number of void * that the one specified after run_cast_test.

########################################################################
#FOURTH STEP: optional modifications
########################################################################
1) Modify /src/algorithms/libs/volk_gnsssdr/lib/CMakeLists.txt in order to see kernel files, ORC files and macros when generating the IDE project.

2) To be able to use volk_gnsssdr and default volk functions at the same time i n the same file you will need to modify the template files that volk_gnsssdr module uses at build time to generate some headers.
The files modified are found inside /tmpl:volk_gnsssdr.tmpl.h 
volk_gnsssdr_typedefs.tmpl.h 
volk_gnsssdr_machines.tmpl.h 
volk_gnsssdr_cpu.tmpl.h 
volk_gnsssdr_config_fixed.tmpl.hThe modifications consist of changing the defines of those files to different ones to allow the definition of the volk_gnsssdr functions although the default volk functions are already defined.

########################################################################
#FIFTH STEP: add volk_gnsssdr module to the GNSS-SDR project
########################################################################
In order to add the volk_gnsssdr module to the GNSS-SDR project the CMakeLists.txt global file needs to be edited.

########################################################################
#SIXTH STEP: using volk_gnsssdr functions
########################################################################
To use the proto-kernels inside volk_gnsssdr project two steps are needed:1) in the CMakeFiles.txt you will need to add $ {VOLK_GNSSSDR_INCLUDE_DIRS} inside the include_directories function, and also add $ {VOLK_GNSSSDR_LIBRARIES} inside the target_link_libraries function.
2) Add the line #include “volk_gnsssdr.h” at the top of the file.