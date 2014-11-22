
########################################################################
# Adding proto-kernels to the module
########################################################################
1) Add your proto-kernels inside the kernels/ folder, and the ORC implementations inside the orc/ folder. Add the macros implementations inside the /kernels/CommonMacros folder. (those folders are found in the root of the volk_gnsssdr module)
2) Add one profiling line for each of the proto-kernels inside the /apps/volk_gnsssdr_profile.cc file. 

3) Add one test line for each of the proto-kernels inside the /lib/testqa.cc file.########################################################################
# Modifications to allow profiling of some proto-kernels with special parameters
########################################################################Some of the proto-kernels that GNSS-SDR needs are not supported by the profiling environment of the volk_gnsssdr module. In order to profile them some modifications need to be done to two files:1) src/algorithms/libs/volk_gnsssdr/lib/qa_utils.cc At the first part of this file there are defined the parameters supported by the environment. The number after run_cast_test indicates the total number of parameters passed to the proto-kernel (input +output parameters). The other part indicates the type of the data passed. Inside func(....) you will need to add the same number of buffs[ ] that the one specified after run_cast_test.

2) src/algorithms/libs/volk_gnsssdr/lib/qa_utils.h In the header you will need to add typedefs for the new definitions made in the .cc file. Take care: you will need to add the same number of void * that the one specified after run_cast_test.

3) To be able to use volk_gnsssdr and default volk functions at the same time in the same file, it is required to add the template files that volk_gnsssdr module uses at build time to generate some headers.
The files are found inside tmpl/:volk_gnsssdr.tmpl.h 
volk_gnsssdr_typedefs.tmpl.h 
volk_gnsssdr_machines.tmpl.h 
volk_gnsssdr_cpu.tmpl.h 
volk_gnsssdr_config_fixed.tmpl.h
