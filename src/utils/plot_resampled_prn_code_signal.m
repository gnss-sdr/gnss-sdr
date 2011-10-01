# Load number of used satellites
satellites = load("./data/satellites.dat")

# Plot resampled prn codes
for i = [0:satellites-1]
	figure(i+1);
	file_sufix = strcat("_", num2str(i), ".dat");
	resampled_prn_code = read_float_binary(strcat("./data/resampled_prn_code_signal", file_sufix));
	resampled_prn_code = resampled_prn_code(1:599);
	resampled_prn_code = [resampled_prn_code; -2; +2];
	plot(resampled_prn_code, "1*");

endfor
figure(satellites+1);
input "press any key to end..."