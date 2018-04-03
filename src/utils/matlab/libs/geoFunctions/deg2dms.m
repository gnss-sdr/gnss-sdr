function dmsOutput = deg2dms(deg)
% DEG2DMS  Conversion of degrees to degrees, minutes, and seconds.
% The output format (dms format) is: (degrees*100 + minutes + seconds/100)

% Written by Kai Borre
% February 7, 2001
% Updated by Darius Plausinaitis

%%% Save the sign for later processing
neg_arg = false;
if deg < 0
    % Only positive numbers should be used while splitting into deg/min/sec
    deg     = -deg;
    neg_arg = true;
end

%%% Split degrees minutes and seconds
int_deg   = floor(deg);
decimal   = deg - int_deg;
min_part  = decimal*60;
min       = floor(min_part);
sec_part  = min_part - floor(min_part);
sec       = sec_part*60;

%%% Check for overflow
if sec == 60
    min     = min + 1;
    sec     = 0;
end
if min == 60
    int_deg = int_deg + 1;
    min     = 0;
end

%%% Construct the output
dmsOutput = int_deg * 100 + min + sec/100;

%%% Correct the sign
if neg_arg == true
    dmsOutput = -dmsOutput;
end

%%%%%%%%%%%%%%%%%%% end deg2dms.m %%%%%%%%%%%%%%%%
