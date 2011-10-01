# Load number of used satellites
satellites = load("./data/satellites.dat")
samples = load("./data/samples.dat")
sampling_frequency = load("./data/sampling_frequency.dat")
signal_duration = 1/(sampling_frequency/samples)

# Plot delayed and resampled prn codes spectrum
for i = [0:satellites-1]
	figure(i+1);
	file_sufix = strcat("_", num2str(i), ".dat");
	delay = read_float_binary(strcat("./data/delay_signal", file_sufix));
	plot_spectrum(delay, sampling_frequency, signal_duration, ";delayed and resampled prn code spectrum;");
endfor
figure(satellites+1);
input "press any key to end..."