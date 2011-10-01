# Load number of used satellites
satellites = load("./data/satellites.dat")
samples = load("./data/samples.dat")
sampling_frequency = load("./data/sampling_frequency.dat")
signal_duration = 1/(sampling_frequency/samples)

# Plot gps signal spectrum
figure(1);
gps_signal = read_float_binary("./data/gps_signal.dat");
plot_spectrum(prn_code, sampling_frequency, signal_duration, ";gps signal spectrum;");
figure(2);
input "press any key to end..."