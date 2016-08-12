function [v] = sampledCode(code,delay,t,Tc,Ts)

% SAMPLECODE returns the delayed PRN code.
%
% code:     the PRN code (it can be a matrix with codes of several sats)
% delay:    not necessarily integer delay suffered by the code [s]
% t:  time vector
% Tc:       chip period [s]
% Ts:       sample period [s]
%
% P. Closas (2009) pclosas@cttc.cat

[nsat,lc] = size(code);     % number of satellites
N_total = length(t);        % total number of samples

tg = kron(ones(nsat,1),t) - delay; %new delayed time vector for all sats
tnor = floor(tg/Tc); %new ROUNDED normalized and delayed time vector

nchip = mod(tnor,lc)+1; % corresponding chip for a given sample

%ARRIBAS: OK, remanent of the division between time vector normalized and
% code length in chips mod(tnos/lc) plus one because matlab array start in
% 1.
% Time vector need to contain all the timestamps for all the samples at Fs

for i = 1:nsat
    v(i,:) = code(i,nchip(i,:));
end