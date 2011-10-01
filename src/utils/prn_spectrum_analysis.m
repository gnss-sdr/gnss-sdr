phase=load("./data/prn_code_phase.dat");
samples_per_code=load("./data/prn_code_samples_per_code.dat");
fs=load("./data/prn_code_fs.dat");
signal=read_float_binary("./data/prn_code_signal.dat");

phase
samples_per_code
fs

if( phase == -1 )
	for i = [0:1022]
		i
		prn_code=signal([(i*samples_per_code)+1:(i+1)*samples_per_code]);
		spectrum_analysis (prn_code,fs,0.001);
	endfor
else
	spectrum_analysis(signal, fs,0.001);
endif