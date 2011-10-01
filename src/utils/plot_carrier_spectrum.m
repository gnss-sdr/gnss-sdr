# Load number of used satellites
satellites = load("./data/satellites.dat")
samples = load("./data/samples.dat")
sampling_frequency = load("./data/sampling_frequency.dat")
signal_duration = 1/(sampling_frequency/samples)

# Plot carrier spectrum
for i = [0:satellites-1]
	figure(i+1);
	file_sufix = strcat("_", num2str(i), ".dat");
	carrier = read_float_binary(strcat("./data/carrier_signal", file_sufix));
	plot_spectrum(carrier, sampling_frequency, signal_duration, ";carrier spectrum;");
endfor
figure(satellites+1);
input "press any key to end..."