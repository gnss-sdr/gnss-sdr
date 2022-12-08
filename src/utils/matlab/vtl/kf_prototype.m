%% vtl KF
%%
sat_number=5;
%% ################## Kalman filter initialization ######################################
% covariances (static)
kf_P_x  = eye(8); %TODO: use a real value.
kf_x    = zeros(8, 1);
kf_R    = zeros(2*sat_number, 2*sat_number);
kf_dt=0.1;
% kf_F = eye(8, 8);
% kf_F(1, 4) = kf_dt;
% kf_F(2, 5) = kf_dt;
% kf_F(3, 6) = kf_dt;
% kf_F(7, 8) = kf_dt;

kf_F=[      1     0     0     kf_dt     0           0     0     0
            0     1     0     0         kf_dt       0     0     0
            0     0     1     0         0           kf_dt 0     0
            0     0     0     1         0           0     0     0
            0     0     0     0         1           0     0     0
            0     0     0     0         0           1     0     0
            0     0     0     0         0           0     1     kf_dt
            0     0     0     0         0           0     0     1];
 
kf_H = zeros(2*sat_number,8);
kf_y = zeros(2*sat_number, 1);
kf_yerr = zeros(2*sat_number, 1);
% kf_xerr = zeros(8, 1);
kf_S = zeros(2*sat_number, 2*sat_number); % kf_P_y innovation covariance matrix

%% State error Covariance Matrix Q (PVT)
kf_Q = [    1     0     0     0     0     0     0     0
            0     1     0     0     0     0     0     0
            0     0     1     0     0     0     0     0
            0     0     0     1     0     0     0     0
            0     0     0     0     1     0     0     0
            0     0     0     0     0     1     0     0
            0     0     0     0     0     0     1e-9     0
            0     0     0     0     0     0     0     1e-7];%new_data.rx_pvt_var(i); %careful, values for V and T could not be adecuate.
%%

% % Measurement error Covariance Matrix R assembling
% for chan=1:5 %neccesary quantities
%     % It is diagonal 2*NSatellite x 2*NSatellite (NSat psudorange error;NSat pseudo range rate error)
%     kf_R(chan, chan) = 40.0; %TODO: fill with real values.
%     kf_R(chan+sat_number, chan+sat_number) = 10.0;
% end

kf_R=[  3     0     0     0     0     0     0     0     0     0
        0     3     0     0     0     0     0     0     0     0
        0     0     3     0     0     0     0     0     0     0
        0     0     0     3     0     0     0     0     0     0
        0     0     0     0     3     0     0     0     0     0
        0     0     0     0     0     30     0     0     0     0
        0     0     0     0     0     0     30     0     0     0
        0     0     0     0     0     0     0     30     0     0
        0     0     0     0     0     0     0     0     30     0
        0     0     0     0     0     0     0     0     0     30];
    
        d = zeros(sat_number, 1);
    rho_pri = zeros(sat_number, 1);
    rhoDot_pri = zeros(sat_number, 1);
    a_x = zeros(sat_number, 1);
    a_y = zeros(sat_number, 1);
    a_z = zeros(sat_number, 1);
    c_pr_m=zeros(sat_number,length(navSolution.RX_time));
%% ################## Kalman Tracking ######################################
% receiver solution from rtklib_solver
for t=1:length(navSolution.RX_time)
    
    if (t<length(navSolution.RX_time)-point_of_closure)
        kf_x(1,t) = navSolution.X(t);
        kf_x(2,t) = navSolution.Y(t);
        kf_x(3,t) = navSolution.Z(t);
        kf_x(4,t) = navSolution.vX(t);
        kf_x(5,t) = navSolution.vY(t);
        kf_x(6,t) = navSolution.vZ(t);
        kf_x(7,t) = clk_bias_s(t)*SPEED_OF_LIGHT_M_S;
        kf_x(8,t) = clk_drift(t)*SPEED_OF_LIGHT_M_S;%new_data.rx_dts(1);
        
        x_u=kf_x(1,t);
        y_u=kf_x(2,t);
        z_u=kf_x(3,t);
        xDot_u=kf_x(4,t);
        yDot_u=kf_x(5,t);
        zDot_u=kf_x(6,t);
        cdeltat_u(t)=kf_x(7,t);
        cdeltatDot_u=kf_x(8,t);
        kf_P_x  = eye(8)*100; %TODO: use a real value.

    else
        kf_x(:,t)=corr_kf_state(:,t-1);    
        x_u=kf_x(1,t);
        y_u=kf_x(2,t);
        z_u=kf_x(3,t);
        xDot_u=kf_x(4,t);
        yDot_u=kf_x(5,t);
        zDot_u=kf_x(6,t);
        cdeltat_u(t)=kf_x(7,t);% 
        cdeltatDot_u=kf_x(8,t);% 
    end
    % Kalman state prediction (time update)
    kf_x(:,t) = kf_F * kf_x(:,t);                        % state prediction
    kf_P_x= kf_F * kf_P_x * kf_F' + kf_Q;  % state error covariance prediction
    
    for chan=1:5 %neccesary quantities
            d(chan)=(sat_posX_m(chan,t)-kf_x(1,t))^2;
            d(chan)=d(chan)+(sat_posY_m(chan,t)-kf_x(2,t))^2;
            d(chan)=d(chan)+(sat_posZ_m(chan,t)-kf_x(3,t))^2;
            d(chan)=sqrt(d(chan));
            
            c_pr_m(chan,t)=d(chan)+cdeltat_u(t);
            
            a_x(chan,t)=-(sat_posX_m(chan,t)-kf_x(1,t))/d(chan);
            a_y(chan,t)=-(sat_posY_m(chan,t)-kf_x(2,t))/d(chan);
            a_z(chan,t)=-(sat_posZ_m(chan,t)-kf_x(3,t))/d(chan);
            
            rhoDot_pri(chan,t)=(sat_velX(chan,t)-kf_x(4,t))*a_x(chan,t)...
                +(sat_velY(chan,t)-kf_x(5,t))*a_y(chan,t)...
                +(sat_velZ(chan,t)-kf_x(6,t))*a_z(chan,t)+cdeltatDot_u;
    end
    
    
    
    for chan=1:5 % Measurement matrix H assembling
            % It has 8 columns (8 states) and 2*NSat rows (NSat psudorange error;NSat pseudo range rate error)
            kf_H(chan, 1) = a_x(chan,t); kf_H(chan, 2) = a_y(chan,t); kf_H(chan, 3) = a_z(chan,t); kf_H(chan, 7) = 1.0;
            kf_H(chan+sat_number, 4) = a_x(chan,t); kf_H(chan+sat_number, 5) = a_y(chan,t); kf_H(chan+sat_number, 6) = a_z(chan,t); kf_H(chan+sat_number, 8) = 1.0;
    end
    
    % Kalman estimation (measurement update)
    for chan=1:5 % Measurement matrix H assembling
            kf_yerr(chan,t)=c_pr_m(chan,t)-sat_prg_m(chan,t);%-0.000157*SPEED_OF_LIGHT_M_S;
            kf_yerr(chan+sat_number,t)=(sat_dopp_hz(chan,t)*Lambda_GPS_L1+cdeltatDot_u)-rhoDot_pri(chan,t);
    end
    
    % DOUBLES DIFFERENCES
    % kf_yerr = zeros(2*sat_number, 1);
    % for (int32_t i = 1; i < sat_number; i++) % Measurement vector
    % {
    %     kf_y(i)=new_data.pr_m(i)-new_data.pr_m(i-1);
    %     kf_yerr(i)=kf_y(i)-(rho_pri(i)+rho_pri(i-1));
    %     kf_y(i+sat_number)=(rhoDot_pri(i)-rhoDot_pri(i-1))/Lambda_GPS_L1;
    %     kf_yerr(i+sat_number)=kf_y(i+sat_number)-(new_data.doppler_hz(i)-new_data.doppler_hz(i-1));
    % }
    % kf_yerr.print("DOUBLES DIFFERENCES");
    
    % Kalman filter update step
    kf_S = kf_H * kf_P_x* kf_H' + kf_R;                      % innovation covariance matrix (S)
    kf_K = (kf_P_x * kf_H') * inv(kf_S);               % Kalman gain
    kf_xerr(:,t) = kf_K * (kf_yerr(:,t));                                 % Error state estimation
    kf_P_x = (eye(length(kf_P_x)) - kf_K * kf_H) * kf_P_x;  % update state estimation error covariance matrix
    corr_kf_state(:,t)=kf_x(:,t)-kf_xerr(:,t); %updated state estimation
      
    err_clk_b=kf_xerr(7,t)
    err_clk_d=kf_xerr(8,t)
    % States related tu USER clock adjust from m/s to s (by /SPEED_OF_LIGHT_M_S)
    % kf_x(6) =kf_x(6);
    % kf_x(7) =kf_x(7);
end