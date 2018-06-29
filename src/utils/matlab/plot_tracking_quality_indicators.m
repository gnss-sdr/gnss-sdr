%plot tracking quality indicators
figure;
hold on;
title('Carrier lock test output for all the channels');
for n=1:1:length(GNSS_tracking)
    plot(GNSS_tracking(n).carrier_lock_test)
    plotnames{n}=['SV ' num2str(round(mean(GNSS_tracking(n).PRN)))];
end
legend(plotnames);

figure;
hold on;
title('Carrier CN0 output for all the channels');
for n=1:1:length(GNSS_tracking)
    plot(GNSS_tracking(n).CN0_SNV_dB_Hz)
    plotnames{n}=['SV ' num2str(round(mean(GNSS_tracking(n).PRN)))];
end
legend(plotnames);