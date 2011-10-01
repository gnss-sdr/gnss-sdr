# Load number of used satellites
satellites = load("./data/satellites.dat")

# Plot delayed and resampled prn codes
for i = [0:satellites-1]
	figure(i+1);
	file_sufix = strcat("_", num2str(i), ".dat");
	delay = read_float_binary(strcat("./data/delay_signal", file_sufix));
	delay = prn_code(1:999);
	delay = [prn_code; -0.5; +0.5];
	plot(prn_code, "1*");

endfor
figure(satellites+1);
input "press any key to end..."