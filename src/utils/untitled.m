0 %a = SpirentSatData2struct('C:\Users\juanc\OneDrive\Documentos\Spirent_data\log_spirent\2024_03_08_recorded_max2771_110dBm\sat_data_V1A1.csv');

len = length(a.GPS.SIM_time);
for i = 1:length(b.PRN(:,1))+2; for j = 1:len; dop(i,j) = a.GPS.series(j).doppler_shift(i); end; end
clear len

d = 25000/13091;

% Crear una figura
figure;

% Primer subplot
subplot(2, 3, 1);
plot(dop(1,1:d:end),'b','LineWidth',1.4);
hold on
plot(b.Carrier_Doppler_hz(1,:),'*')
hold on
plot(dop(1,1:d:end),'b','LineWidth',1.4);
title('Doppler NSat = 1')
ylabel('$f_D$ [Hz]')
legend('Spirent','SDR')
grid on

% Segundo subplot
subplot(2, 3, 2);
plot(dop(2,1:d:end),'b','LineWidth',1.4);
hold on
plot(b.Carrier_Doppler_hz(2,:),'*')
hold on
plot(dop(2,1:d:end),'b','LineWidth',1.4);
title('Doppler NSat = 2')
ylabel('$f_D$ [Hz]')
legend('Spirent','SDR')
grid on

% Tercer subplot
subplot(2, 3, 3);
plot(dop(3,1:d:end),'b','LineWidth',1.4);
hold on
plot(b.Carrier_Doppler_hz(3,:),'*')
hold on
plot(dop(3,1:d:end),'b','LineWidth',1.4);
title('Doppler NSat = 3')
ylabel('$f_D$ [Hz]')
legend('Spirent','SDR')
grid on

% Cuarto subplot
subplot(2, 3, 4);
plot(dop(4,1:d:end),'b','LineWidth',1.4);
hold on
plot(b.Carrier_Doppler_hz(4,:),'*')
hold on
plot(dop(4,1:d:end),'b','LineWidth',1.4);
title('Doppler NSat = 4')
ylabel('$f_D$ [Hz]')
legend('Spirent','SDR')
grid on

% Quinto subplot
subplot(2, 3, 5);
plot(dop(5,1:d:end),'b','LineWidth',1.4);
hold on
plot(b.Carrier_Doppler_hz(5,:),'*')
hold on
plot(dop(5,1:d:end),'b','LineWidth',1.4);
title('Doppler NSat = 5')
ylabel('$f_D$ [Hz]')
legend('Spirent','SDR')
grid on

% Sexto subplot
subplot(2, 3, 6);
plot(dop(6,1:d:end),'b','LineWidth',1.4);
hold on
plot(b.Carrier_Doppler_hz(6,:),'*')
hold on
plot(dop(6,1:d:end),'b','LineWidth',1.4);
ylabel('$f_D$ [Hz]')
title('Doppler NSat = 6')
legend('Spirent','SDR')
grid on

sgtitle('Doppler frequency Spirent vs GNSS-SDR');