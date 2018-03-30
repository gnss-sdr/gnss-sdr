function [pos, el, az, dop] = leastSquarePos(satpos, obs, settings)
% Function calculates the Least Square Solution.
%
% [pos, el, az, dop] = leastSquarePos(satpos, obs, settings);
%
%   Inputs:
%       satpos      - Satellites positions (in ECEF system: [X; Y; Z;] -
%                   one column per satellite)
%       obs         - Observations - the pseudorange measurements to each
%                   satellite:
%                   (e.g. [20000000 21000000 .... .... .... .... ....])
%       settings    - receiver settings
%
%   Outputs:
%       pos         - receiver position and receiver clock error
%                   (in ECEF system: [X, Y, Z, dt])
%       el          - Satellites elevation angles (degrees)
%       az          - Satellites azimuth angles (degrees)
%       dop         - Dilutions Of Precision ([GDOP PDOP HDOP VDOP TDOP])

%--------------------------------------------------------------------------
%                           SoftGNSS v3.0
%--------------------------------------------------------------------------
%Based on Kai Borre
%Copyright (c) by Kai Borre
%Updated by Darius Plausinaitis, Peter Rinder and Nicolaj Bertelsen
%==========================================================================

%=== Initialization =======================================================
nmbOfIterations = 7;

dtr     = pi/180;
pos     = zeros(4, 1);
X       = satpos;
nmbOfSatellites = size(satpos, 2);

A       = zeros(nmbOfSatellites, 4);
omc     = zeros(nmbOfSatellites, 1);
az      = zeros(1, nmbOfSatellites);
el      = az;

%=== Iteratively find receiver position ===================================
for iter = 1:nmbOfIterations
    
    for i = 1:nmbOfSatellites
        if iter == 1
            %--- Initialize variables at the first iteration --------------
            Rot_X = X(:, i);
            trop = 2;
        else
            %--- Update equations -----------------------------------------
            rho2 = (X(1, i) - pos(1))^2 + (X(2, i) - pos(2))^2 + ...
                (X(3, i) - pos(3))^2;
            traveltime = sqrt(rho2) / settings.c ;
            
            %--- Correct satellite position (do to earth rotation) --------
            Rot_X = e_r_corr(traveltime, X(:, i));
            
            %--- Find the elevation angel of the satellite ----------------
            [az(i), el(i), dist] = topocent(pos(1:3, :), Rot_X - pos(1:3, :));
            
            if (settings.useTropCorr == 1)
                %--- Calculate tropospheric correction --------------------
                trop = tropo(sin(el(i) * dtr), ...
                    0.0, 1013.0, 293.0, 50.0, 0.0, 0.0, 0.0);
            else
                % Do not calculate or apply the tropospheric corrections
                trop = 0;
            end
        end % if iter == 1 ... ... else
        
        %--- Apply the corrections ----------------------------------------
        omc(i) = (obs(i) - norm(Rot_X - pos(1:3), 'fro') - pos(4) - trop);
        
        %--- Construct the A matrix ---------------------------------------
        A(i, :) =  [ (-(Rot_X(1) - pos(1))) / obs(i) ...
            (-(Rot_X(2) - pos(2))) / obs(i) ...
            (-(Rot_X(3) - pos(3))) / obs(i) ...
            1 ];
    end % for i = 1:nmbOfSatellites
    
    % These lines allow the code to exit gracefully in case of any errors
    if rank(A) ~= 4
        pos     = zeros(1, 4);
        return
    end
    
    %--- Find position update ---------------------------------------------
    x   = A \ omc;
    
    %--- Apply position update --------------------------------------------
    pos = pos + x;
    
end % for iter = 1:nmbOfIterations

pos = pos';

%=== Calculate Dilution Of Precision ======================================
if nargout  == 4
    %--- Initialize output ------------------------------------------------
    dop     = zeros(1, 5);
    
    %--- Calculate DOP ----------------------------------------------------
    Q       = inv(A'*A);
    
    dop(1)  = sqrt(trace(Q));                       % GDOP
    dop(2)  = sqrt(Q(1,1) + Q(2,2) + Q(3,3));       % PDOP
    dop(3)  = sqrt(Q(1,1) + Q(2,2));                % HDOP
    dop(4)  = sqrt(Q(3,3));                         % VDOP
    dop(5)  = sqrt(Q(4,4));                         % TDOP
end
