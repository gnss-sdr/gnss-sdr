% /*!
%  * \file beidou_B1I_generator.m

%  create a file containing the desired B1I code sequence of the selected BeiDou satellite 
%
%  * \author Giorgio Savastano, 2015. giorgio.savastano(at)uniroma1.it
%  * -------------------------------------------------------------------------

clc;
clear all;
close all;
%% ------------Settings---------------------------------------------------------
fileName ='/Users/enricjuan/code_repos/gnss-sdr/src/tests/signal_samples/test_beidou_60s.dat';

f_rf   = 1561.098e6;    %[Hz] BeiDou B1 nominal frequency;
f_if   = 0;%0.098e6;       %[Hz] IF nominal frequency;      
f_prn  = 2.046e6;       %[Hz] Nominal PRN-generator clock frequency;
f_nh   = 1e3;           %[Hz] Nominal Neiman-Huffman-generator clock frequency;
f_data = 50;            %[Hz] Nominal data rate of BeiDou D1 signal
f_data_NH = 1e3;        %[Hz] Data bit rate after modulation by NH code

phi0_if   = 0;      %[rad] Initial phase of RF signal;
phi0_prn1 = 0;      %[rad] Initial phase of PRN signal;
phi0_nh   = 0;      %[rad] Initial phase of NH signal;
phi0_bk   = 0;      %[rad] Initial phase of BK signal;
phi0_data = 0;      %[rad] Initial phase of data signal;

fs = 16.000e6;      %[Hz] Sampling frequency;                % think about fs = 16.368e6 [Hz]
ts = 1/fs;          %[sec]

f_d                = 1650.0;                %[Hz] Initial Doppler frequeny for RF-signal;
code_delay         = ts* 3767.0;            %[sec] the delay in seconds rounded to a multiple of ts
code_delay_samples = code_delay*fs;         %[samples] the delay in samples

%df  = -0.55;        %[Hz/sec] Initail Doppler frequency change rate for RF-signal; 

T  = 100e-3;                 %[sec] Signal length to be generated;
n_samples = floor(fs * T);

prn_num  = 20;          % PRN number [0;37];
prn_len  = 2046;        % [chips] PRN-code length in chips (bits);
nh_len   = 20;          % Neumann-Hoffman code length in chips (bits);
 
data_len = T * f_data;  %[bits] Navigation message length in bits;

%Noise generation is under development...
%SNR_dB = -3;             %SNR of the generating signal;
%SNR = 10^(SNR_dB/10);    %SNR in times;
%noise_pwr = 1/SNR;       %noise power for predefined SNR;
%noise_pwr = noise_pwr * ( (fs/2) / (2*f_prn) ); %Take into accound sampling frequency and signal bandwidth!

%% -----------Signal_generator--------------------------------------------------

t=(1:n_samples).*(1/fs);    % time samples for generating signal of T_elem length;

PRN1        = generateB1Icode(prn_num);                                % Generate PRN-code;
NH_original = [0 0 0 0 0  1 0 0  1  1 0  1 0  1 0 0  1  1  1  0];      % Generate Neiman-Huffman-code
NH          = [1 1 1 1 1 -1 1 1 -1 -1 1 -1 1 -1 1 1 -1 -1 -1  1];      % 0=>1, 1=>-1 
DATA        = (2 * round(rand(1, data_len))) - 1;                      % Temporary Stub for data bits after convolutional coder; (pseudorandom values of 1 and -1, of data_len values)

signal_I = [];

%Calculate phase for carrier;
phi_if = (phi0_if) + (2*pi*f_if*t) + (2*pi*f_d*t);                 % INITIAL PHASE + IF FREQ + DOPPLER FREQ
phi_if = mod(phi_if, (2*pi));                                      % Convert carrier phase to the range [0..2*pi];  

%Calculate phase for PRN1;
phi_prn1  = (phi0_prn1) + (f_prn*(t-code_delay));                  % (t-code_delay)
prn1_indx = (fix(mod(phi_prn1, prn_len)));					       

%Calculte phase for Neiman-Huffman;
phi_nh  = (phi0_nh) + (f_nh*(t-code_delay));                       % (t-code_delay) 
nh_indx = (fix( mod(phi_nh, nh_len) ));
 
%Calculate phase for data-message;
phi_data  = (phi0_data) + (f_data*(t-code_delay));                 % (t-code_delay)
data_indx = (fix(mod(phi_data, data_len)));

%Generate carrier;
carr_cos = cos(phi_if);      	% generate carrier (I)

%Generate PRN;
prn1 = PRN1(prn1_indx+1);    	% generate PRN for I-channel;

%Generate NH;
nh = NH(nh_indx+1);          	% generate NH-code;

%Generate DATA;
data = DATA(data_indx+1);  		% generate DATA;

s_I         = ((prn1 .* nh).* data);
%s_I         = ((prn1).* data);
signal_test = ((s_I ).*carr_cos);
  
%Let's add some noise:
%Noise generation is under development...
%noise = grand(1,length(signal_RSLT),"nor",0,noise_pwr);
%signal_RSLT = signal_RSLT + noise;
%signal_RSLT = signal_RSLT / max(abs(signal_RSLT)); %Normalization after adding noise;

%Next step is to write signal to file;
write_apend_complex_binary(signal_test, fileName, n_samples);

