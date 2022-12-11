


time_reference_spirent_obs=129780;%s
time_vtl_dump_file=linspace(38,157,length(vtlSolution.filt_dop_sat));
%  rx_PRN=[28 4 17 15 27 9]; % for SPF_LD_05.
%%
    Rx_Dopp_28=figure('Name','RX_Carrier_Doppler_hz');plot(RX_time(1,:)-time_reference_spirent_obs, Carrier_Doppler_hz(1,:)','s')
    xlim([0,140]);
    ylim([-2340,-2220]);
    xlabel('')
    ylabel('Doppler (Hz)')
    xlabel('time from simulation init (seconds)')
    grid on
    hold on
    legend('PRN 28 GNSS-SDR','Location','eastoutside')
    plot(refSatData.GPS.SIM_time/1000, Carrier_Doppler_hz_sim(:,1)','.','DisplayName','reference')
    plot(time_vtl_dump_file,vtlSolution.filt_dop_sat(1,:),'o','DisplayName','filtered VTL')
    hold off;grid minor

    Rx_Dopp_4=figure('Name','RX_Carrier_Doppler_hz');
    plot(RX_time(1,:)-time_reference_spirent_obs, Carrier_Doppler_hz(2,:)','s')
    xlim([0,140]);  
    ylim([2540,2640]);
    xlabel('')
    ylabel('Doppler (Hz)')
    xlabel('time from simulation init (seconds)')
    grid on
    hold on
    legend('PRN 4 GNSS-SDR','Location','eastoutside')
    plot(refSatData.GPS.SIM_time/1000, Carrier_Doppler_hz_sim(:,2)','.','DisplayName','reference')
    plot(time_vtl_dump_file,vtlSolution.filt_dop_sat(2,:),'o','DisplayName','filtered VTL')
    hold off;grid minor

        Rx_Dopp_17=figure('Name','RX_Carrier_Doppler_hz');plot(RX_time(1,:)-time_reference_spirent_obs, Carrier_Doppler_hz(3,:)','s')
    xlim([0,140]);  
    ylim([-1800,-1730]);
    xlabel('')
    ylabel('Doppler (Hz)')
    xlabel('time from simulation init (seconds)')
    grid on
    hold on
    legend('PRN 17 GNSS-SDR','Location','eastoutside')
    plot(refSatData.GPS.SIM_time/1000, Carrier_Doppler_hz_sim(:,3)','.','DisplayName','reference')
    plot(time_vtl_dump_file,vtlSolution.filt_dop_sat(3,:),'o','DisplayName','filtered VTL')
    hold off;grid minor

        Rx_Dopp_15=figure('Name','RX_Carrier_Doppler_hz');plot(RX_time(1,:)-time_reference_spirent_obs, Carrier_Doppler_hz(4,:)','s')
    xlim([0,140]);  
    ylim([-2680,-2620]);
    xlabel('')
    ylabel('Doppler (Hz)')
    xlabel('time from simulation init (seconds)')
    grid on
    hold on
    legend('PRN 15 GNSS-SDR','Location','eastoutside')
    plot(refSatData.GPS.SIM_time/1000, Carrier_Doppler_hz_sim(:,4)','.','DisplayName','reference')
    plot(time_vtl_dump_file,vtlSolution.filt_dop_sat(4,:),'o','DisplayName','filtered VTL')
    hold off;grid minor
