# Load number of used satellites
satellites = load("./data/satellites.dat")
samples = load("./data/samples.dat")
sampling_frequency = load("./data/sampling_frequency.dat")
signal_duration = 1/(sampling_frequency/samples)

# Plot resampled prn codes spectrum
for i = [0:satellites-1]
	figure(i+1);
	file_sufix = strcat("_", num2str(i), ".dat");
	resampled_prn_code = read_float_binary(strcat("./data/resampled_prn_code_signal", file_sufix));
	plot_spectrum(resampled_prn_code, sampling_frequency, signal_duration, ";resampled prn code spectrum;");
endfor
figure(satellites+1);
input "press any key to end..."