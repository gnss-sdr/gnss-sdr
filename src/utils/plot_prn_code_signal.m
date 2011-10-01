# Load number of used satellites
satellites = load("./data/satellites.dat")

# Plot prn codes
for i = [0:satellites-1]
	figure(i+1);
	file_sufix = strcat("_", num2str(i), ".dat");
	prn_code = read_float_binary(strcat("./data/prn_code_signal", file_sufix));
	prn_code = prn_code(1:99);
	prn_code = [prn_code; -2; +2];
	plot(prn_code, "1*");

endfor
figure(satellites+1);
input "press any key to end..."