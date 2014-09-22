####################################################################
Common Macros inside volk_gnsssdr module
####################################################################

First of all, sorry for making you need to read this: macros are evil, they can not be debugged, you do not know where the errors come from, syntax is annoying.. BUT this is the only way I found that allows to share one piece of code between various proto-kernels without performance penalties.
Inline functions have been tested, and they introduce a really small time penalty, but it becomes huge because of long loops, with thousands of samples.

####################################################################
Syntax
####################################################################

In order to allow better understanding of the code I created the macros with an specific syntax.

1) Inside CommonMacros.h you will find macros for common operations. I will explain the syntax with an example:

example: CM_16IC_X4_SCALAR_PRODUCT_16IC_X2_U_SSE2(realx, imagx, realy, imagy, realx_mult_realy, imagx_mult_imagy, realx_mult_imagy, imagx_mult_realy, real_output, imag_output)

First of all, you find the characters “CM”, which means CommonMacros. After that the type and the amount of inputs is placed: “_16IC_X4” (16 bits complex integers, four inputs). The syntax for type is the same as the one used with volk protokernels, refer to GNURadio documentation for more help. The it comes the name of the macro (“_SCALAR_PRODUCT”), and after that the type and the amount of outputs (“_16IC_X2”). Finally it is placed the SSE minimum version needed to run (“_U_SSE2”). In the arguments you will find (from left to right) the inputs (four inputs: realx, imagx, realy, imagy), some variables that the macro needs to work (realx_mult_realy, imagx_mult_imagy, realx_mult_imagy, imagx_mult_realy) and finally the outputs (two outputs: real_output, imag_output).
The variables that the macro needs are specified when calling it in order to avoid after-compile problems: if you want to use a macro you will need to declare all the variables it needs before, or you will not be able to compile.

2) Inside all the other headers, CommonMacros_XXXXXX.h you will find macros for a specific group of proto-kernels. The syntax is the same as the CommonMacros.h

####################################################################
Workflow
####################################################################

In order to use the macros easily, I usually test the code without macros inside a testing proto-kernel, where you are able to test it, debug it and use breakpoints.
When it works I place code inside a macro an I test it again.

####################################################################
Why macros
####################################################################
1) They are the only way I could find for sharing code between proto-kernels without performance penalty.
2) It is true that they are really difficult to debug, but if you work with them responsibly it is not so hard. Volk_gnsssdr checks all the SSE proto-kernels implementations results against the generic implementation results, so if your macro is not working you will appreciate it after profiling it.