#!/usr/bin/octave
arg_list = argv ();
x=read_float_binary (arg_list{1});
figure(1);
plot(x);
figure(2)
input "press any key to end..."