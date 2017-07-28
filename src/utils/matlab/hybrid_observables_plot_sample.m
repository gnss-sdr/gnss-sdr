% Read observables dump

%clear all;

samplingFreq       = 2600000;     %[Hz]
channels=2;
path='/home/javier/git/gnss-sdr/build/';
observables_log_path=[path 'observables.dat'];
GNSS_observables= read_hybrid_observables_dump(channels,observables_log_path);

%optional:
%search all channels having good satellite simultaneously
min_idx=1;
for n=1:1:channels
    idx=find(GNSS_observables.valid(n,:)>0,1,'first');
    if min_idx<idx
        min_idx=idx;
    end
end

%plot observables from that index
figure;
plot(GNSS_observables.RX_time(1,min_idx+1:end),GNSS_observables.Pseudorange_m(:,min_idx+1:end)');
title('Pseudoranges [m]')
xlabel('TOW [s]')
ylabel('[m]');

figure;
plot(GNSS_observables.RX_time(1,min_idx+1:end),GNSS_observables.Carrier_phase_hz(:,min_idx+1:end)');
title('Accumulated carrier phase')
xlabel('TOW [s]')
ylabel('[cycles]');

figure;
plot(GNSS_observables.RX_time(1,min_idx+1:end),GNSS_observables.Carrier_Doppler_hz(:,min_idx+1:end)');
title('Doppler frequency')
xlabel('TOW [s]')
ylabel('[Hz]');

