clear; clc; 
% This program decodes the navigation bits obtained from the telemetry
% block for SBAS. The only input required is the filename of the .mat with
% the telemetry output. 
%
% The message types available for decoding are: 0, 17, 18, 26 and 27.
%
% Written by Juan Alfaro

filename = ''; % Path to the telemetry .mat output file.
load(filename)

% SBAS preamble types
preamble1 = '01010011';
preamble2 = '10011010';
preamble3 = '11000110';

% Prepare the incoming bit data for the program
bitLog = nav_symbol == 1;

preLog1 = preamble1 == '1';
preLog2 = preamble2 == '1';
preLog3 = preamble3 == '1';

preLen = length(preamble1);
preStart = 0;
counter = 0;

% Main Loop
for i=1:(length(nav_symbol) - preLen + 1)
    if all(bitLog(i:i + preLen -1) == preLog1) || all(bitLog(i:i + preLen -1) == preLog2) || all(bitLog(i:i + preLen -1) == preLog3)
        if (i+249 <= length(nav_symbol))
            preStart = i;
            binMsg = strcat(strjoin(cellstr(char(nav_symbol(preStart:preStart+249) + '0')'),''),'000000');
            check = paritycheck(binMsg);

            if check == 1
                counter = counter +1;
                hex = convert2hex(binMsg)
                data{counter} = hex; 
                preamble    = dec2hex(bin2dec(binMsg(1:8))); % Preamble bits 0-7
                msgType     = bin2dec(binMsg(9:14));         % Message type bits 8-13
                
                if msgType == 0
                    disp('Message type 0: don`t use for safety applications (for WAAS testing)'); NavData = MT0(binMsg,preamble);
                elseif msgType == 17
                    disp('Message type 17: satellite almanacs'); NavData = MT17(binMsg,preamble);
                elseif msgType == 18
                    disp('Message type 18: Ionospheric grid point mask'); NavData = MT18(binMsg,preamble);
                elseif msgType == 26
                    disp('Message type 26: Ionospheric delay corrections'); NavData = MT26(binMsg,preamble);
                elseif msgType == 27
                    disp('Message type 27: WAAS Service Message'); NavData = MT27(binMsg,preamble);
                else
                    fprintf('Message type %d not supported! \n', msgType);
                end
            end
        end
    end
end

fprintf('There were found a total of %d messages in %d bits of data. \n', counter, length(nav_symbol))


% Functions
function check = paritycheck(binMsg)
    if strcmp(crc24q(binMsg),'000000000000000000000000') && strcmp(binMsg(251:256),'000000')% Check CRC-24 of message (parity)
        disp('CRC-24 parity valid!');
        check = 1;
    else 
        disp('CRC-24 parity not valid, error!');
        check = 0;
    end
end

function NavData = MT0(binMsg,preamble)

    NavData.dataID(1)      = bin2dec(binMsg(15:16));        % Data ID bits 14-15 (satellite 1)
    NavData.dataID(2)      = bin2dec(binMsg(82:83));        % Data ID bits 81-82 (satellite 2)
    NavData.dataID(3)      = bin2dec(binMsg(149:150));      % Data ID bits 148-149 (satellite 3)
    %NavData.parity      = bin2dec(binMsg(1:250));
    
    fprintf('Preamble: %s \n',preamble)
    fprintf('Parity: %d \n',0)
    fprintf('Data ID sat. 1: %d \n', NavData.dataID(1));
    fprintf('Data ID sat. 2: %d \n', NavData.dataID(2));
    fprintf('Data ID sat. 3: %d \n', NavData.dataID(3));


end

function NavData = MT17(binMsg,preamble)

    NavData.dataID(1)      = bin2dec(binMsg(15:16));        % Data ID bits 14-15 (satellite 1)
    NavData.dataID(2)      = bin2dec(binMsg(82:83));        % Data ID bits 81-82 (satellite 2)
    NavData.dataID(3)      = bin2dec(binMsg(149:150));      % Data ID bits 148-149 (satellite 3)

    NavData.PRN(1)         = bin2dec(binMsg(17:24));        % PRN bits 16-23 (satellite 1)
    NavData.PRN(2)         = bin2dec(binMsg(84:91));        % PRN bits 83-90 (satellite 2)
    NavData.PRN(3)         = bin2dec(binMsg(151:158));      % PRN bits 150-157 (satellite 3)

    NavData.ranging(1)     = bin2dec(binMsg(32:32));        % Ranging bits 31-31 (satellite 1)
    NavData.ranging(2)     = bin2dec(binMsg(99:99));        % Ranging bits 98-98 (satellite 2)
    NavData.ranging(3)     = bin2dec(binMsg(166:166));      % Ranging bits 165-165 (satellite 3)

    NavData.corrections(1) = bin2dec(binMsg(31:31));        % Corrections bits 30-30 (satellite 1)
    NavData.corrections(2) = bin2dec(binMsg(98:98));        % Corrections bits 97-97 (satellite 2)
    NavData.corrections(3) = bin2dec(binMsg(165:165));      % Corrections bits 164-164 (satellite 3)

    NavData.broadInteg(1)  = bin2dec(binMsg(30:30));        % Broadcast integrity bits 29-29 (satellite 1)
    NavData.broadInteg(2)  = bin2dec(binMsg(97:97));        % Broadcast integrity bits 96-96 (satellite 2)
    NavData.broadInteg(3)  = bin2dec(binMsg(164:164));      % Broadcast integrity bits 163-163 (satellite 3)

    NavData.ServProvID(1)  = bin2dec(binMsg(25:28));        % Service provider ID bits 24-27 (satellite 1)
    NavData.ServProvID(2)  = bin2dec(binMsg(92:95));        % Service provider ID bits 91-94 (satellite 2)
    NavData.ServProvID(3)  = bin2dec(binMsg(159:162));      % Service provider ID bits 158-161 (satellite 3)
    
    if binMsg(33) == '0'
        NavData.Xg(1)          = bin2dec(binMsg(33:47))*2600;  % Xg ECEF (m) bits 32-46 (satellite 1)
    else
        NavData.Xg(1)          = (bin2dec(binMsg(33:47))-2^15)*2600;
    end

    if binMsg(100) == '0'
        NavData.Xg(2)          = bin2dec(binMsg(100:114))*2600;  % Xg ECEF (m) bits 99-113 (satellite 2)
    else
        NavData.Xg(2)          = (bin2dec(binMsg(100:114))-2^15)*2600;
    end

    if binMsg(167) == '0'
        NavData.Xg(3)          = bin2dec(binMsg(167:181))*2600;  % Xg ECEF (m) bits 166-180 (satellite 3)
    else
        NavData.Xg(3)          = (bin2dec(binMsg(167:181))-2^15)*2600;
    end
    
    if binMsg(48) == '0'
        NavData.Yg(1)          = bin2dec(binMsg(48:62))*2600;  % Yg ECEF (m) bits 47-61 (satellite 1)
    else
        NavData.Yg(1)          = (bin2dec(binMsg(48:62))-2^15)*2600;
    end

    if binMsg(115) == '0'
        NavData.Yg(2)          = bin2dec(binMsg(115:129))*2600;  % Yg ECEF (m) bits 114-128 (satellite 2)
    else
        NavData.Yg(2)          = (bin2dec(binMsg(115:129))-2^15)*2600;
    end

    if binMsg(182) == '0'
        NavData.Yg(3)          = bin2dec(binMsg(182:196))*2600;  % Yg ECEF (m) bits 181-197 (satellite 3)
    else
        NavData.Yg(3)          = (bin2dec(binMsg(182:196))-2^15)*2600;
    end
    
    if binMsg(63) == '0'
        NavData.Zg(1)          = bin2dec(binMsg(63:71))*26000;  % Zg ECEF (m) bits 62-70 (satellite 1)
    else
        NavData.Zg(1)          = (bin2dec(binMsg(63:71))-2^9)*26000;
    end
    
    if binMsg(130) == '0'
        NavData.Zg(2)          = bin2dec(binMsg(130:138))*26000;  % Zg ECEF (m) bits 129-137 (satellite 2)
    else
        NavData.Zg(2)          = (bin2dec(binMsg(130:138))-2^9)*26000;
    end

    if binMsg(197) == '0'
        NavData.Zg(3)          = bin2dec(binMsg(197:205))*26000;  % Zg ECEF (m) bits 196-204 (satellite 3)
    else
        NavData.Zg(3)          = (bin2dec(binMsg(197:205))-2^9)*26000;
    end

    if binMsg(72) == '0'
        NavData.Delta_Xg(1)          = bin2dec(binMsg(72:74))*10;  % Rate of change Xg (m/s) bits 71-73 (satellite 1)
    else
        NavData.Delta_Xg(1)          = (bin2dec(binMsg(72:74))-2^3)*10;
    end
    
    if binMsg(139) == '0'
        NavData.Delta_Xg(2)          = bin2dec(binMsg(139:141))*10;  % Rate of change Xg (m/s) bits 138-140 (satellite 2)
    else
        NavData.Delta_Xg(2)          = (bin2dec(binMsg(139:141))-2^3)*10;
    end

    if binMsg(206) == '0'
        NavData.Delta_Xg(3)          = bin2dec(binMsg(206:208))*10;  % Rate of change Xg (m/s) bits 205-207 (satellite 3)
    else
        NavData.Delta_Xg(3)          = (bin2dec(binMsg(206:208))-2^3)*10;
    end

    if binMsg(75) == '0'
        NavData.Delta_Yg(1)          = bin2dec(binMsg(75:77))*10;  % Rate of change Yg (m/s) bits 74-76 (satellite 1)
    else
        NavData.Delta_Yg(1)          = (bin2dec(binMsg(75:77))-2^3)*10;
    end
    
    if binMsg(142) == '0'
        NavData.Delta_Yg(2)          = bin2dec(binMsg(142:144))*10;  % Rate of change Yg (m/s) bits 141-143 (satellite 2)
    else
        NavData.Delta_Yg(2)          = (bin2dec(binMsg(142:144))-2^3)*10;
    end

    if binMsg(209) == '0'
        NavData.Delta_Yg(3)          = bin2dec(binMsg(209:211))*10;  % Rate of change Yg (m/s) bits 208-210 (satellite 3)
    else
        NavData.Delta_Yg(3)          = (bin2dec(binMsg(209:211))-2^3)*10;
    end

    if binMsg(78) == '0'
        NavData.Delta_Zg(1)          = bin2dec(binMsg(78:81))*60;  % Rate of change Zg (m/s) bits 77-80 (satellite 1)
    else
        NavData.Delta_Zg(1)          = (bin2dec(binMsg(78:81))-2^4)*60;
    end

    if binMsg(145) == '0'
        NavData.Delta_Zg(2)          = bin2dec(binMsg(145:148))*60;  % Rate of change Zg (m/s) bits 144-147 (satellite 2)
    else
        NavData.Delta_Zg(2)          = (bin2dec(binMsg(145:148))-2^4)*60;
    end

    if binMsg(212) == '0'
        NavData.Delta_Zg(3)          = bin2dec(binMsg(212:215))*60;  % Rate of change Zg (m/s) bits 211-214 (satellite 3)
    else
        NavData.Delta_Zg(3)          = (bin2dec(binMsg(212:215))-2^4)*60;
    end
    
    NavData.t0          = bin2dec(binMsg(216:226))*64;  % Time of day (s) bits 215-225
    %NavData.parity      = bin2dec(binMsg(1:250));
    
    fprintf('Preamble: %s \n',preamble)
    fprintf('Time of the day: %d s \n',NavData.t0);
    fprintf('Parity: %d \n',0)
    disp('-----------------------------------')
    disp('Satellite [ 1 ]')
    fprintf('PRN number: %d \n', NavData.PRN(1));
    fprintf('Data ID: %d \n', NavData.dataID(1));
    fprintf('Ranging: %d \n',NavData.ranging(1))
    fprintf('Corrections: %d \n', NavData.corrections(1));
    fprintf('Broadcast integrity: %d \n', NavData.broadInteg(1));
    fprintf('Service provider ID: %d \n', NavData.ServProvID(1));

    fprintf('Xg (ECEF): %d m \n',NavData.Xg(1));
    fprintf('Yg (ECEF): %d m \n',NavData.Yg(1));
    fprintf('Zg (ECEF): %d m \n',NavData.Zg(1));

    fprintf('Xg rate-of-change: %d m/s \n',NavData.Delta_Xg(1));
    fprintf('Yg rate-of-change: %d m/s \n',NavData.Delta_Yg(1));
    fprintf('Zg rate-of-change: %d m/s \n',NavData.Delta_Zg(1));

    disp('-----------------------------------')
    disp('Satellite [ 2 ]')
    fprintf('PRN number: %d \n', NavData.PRN(2));
    fprintf('Data ID: %d \n', NavData.dataID(2));
    fprintf('Ranging: %d \n',NavData.ranging(2))
    fprintf('Corrections: %d \n', NavData.corrections(2));
    fprintf('Broadcast integrity: %d \n', NavData.broadInteg(2));
    fprintf('Service provider ID: %d \n', NavData.ServProvID(2));

    fprintf('Xg (ECEF): %d m \n',NavData.Xg(2));
    fprintf('Yg (ECEF): %d m \n',NavData.Yg(2));
    fprintf('Zg (ECEF): %d m \n',NavData.Zg(2));

    fprintf('Xg rate-of-change: %d m/s \n',NavData.Delta_Xg(2));
    fprintf('Yg rate-of-change: %d m/s \n',NavData.Delta_Yg(2));
    fprintf('Zg rate-of-change: %d m/s \n',NavData.Delta_Zg(2));

    disp('-----------------------------------')
    disp('Satellite [ 3 ]')
    fprintf('PRN number: %d \n', NavData.PRN(3));
    fprintf('Data ID: %d \n', NavData.dataID(3));
    fprintf('Ranging: %d \n',NavData.ranging(3))
    fprintf('Corrections: %d \n', NavData.corrections(3));
    fprintf('Broadcast integrity: %d \n', NavData.broadInteg(3));
    fprintf('Service provider ID: %d \n', NavData.ServProvID(3));

    fprintf('Xg (ECEF): %d m \n',NavData.Xg(3));
    fprintf('Yg (ECEF): %d m \n',NavData.Yg(3));
    fprintf('Zg (ECEF): %d m \n',NavData.Zg(3));

    fprintf('Xg rate-of-change: %d m/s \n',NavData.Delta_Xg(3));
    fprintf('Yg rate-of-change: %d m/s \n',NavData.Delta_Yg(3));
    fprintf('Zg rate-of-change: %d m/s \n',NavData.Delta_Zg(3));

end

function NavData = MT18(binMsg,preamble)

    NavData.Nbands      = bin2dec(binMsg(15:18));        % Number of bands being broadcast bits 14-17
    NavData.BandNumber  = bin2dec(binMsg(19:22));        % Band number bits 18-21
    NavData.IODI        = bin2dec(binMsg(23:24));        % Issue of data - Ionosphere bits 22-23
    NavData.IgpMask     = binMsg(25:255) == '1';         % Igp Mask bits 24-224 (logic)
    %NavData.parity      = bin2dec(binMsg(1:250));
    
    fprintf('Preamble: %s \n',preamble)
    fprintf('Parity: %d \n',0)
    fprintf('Number of bands being broadcast: %d \n', NavData.Nbands);
    fprintf('Band number: %d \n', NavData.BandNumber);
    fprintf('Issue of data - Ionosphere: %d \n',NavData.IODI)
    disp('Igp Mask: LOGIC ARRAY');


end

function NavData = MT26(binMsg,preamble)
    
    idxIGP              = [23, 31, 36, 44, 49, 57, 62, 70, 75, 83, 88, 96, 101, 109, 114, 122, 127, 135, 140, 148, 153, 161, 166, 174, 179, 187, 192, 200, 205, 213];
    idxGIVEI            = [32, 35, 45, 48, 58, 61, 71, 74, 84, 87, 97, 100, 110, 113, 123, 126, 136, 139, 149, 152, 162, 165, 175, 178, 188, 191, 201, 204, 214, 217];
    NavData.BandNumber  = bin2dec(binMsg(15:18));        % Band number bits 14-17
    NavData.BlockID     = bin2dec(binMsg(19:22));        % Block ID bits 18-21
    NavData.IODI        = bin2dec(binMsg(218:219));      % Issue of data - Ionosphere bits 217-218
    %NavData.parity      = bin2dec(binMsg(1:250));
    
    counter = 1;
    for i = 1:15
        NavData.GIVEI(i) = bin2dec(binMsg(idxGIVEI(counter):idxGIVEI(counter+1)));
        NavData.IGP(i) = bin2dec(binMsg(idxIGP(counter):idxIGP(counter+1)))*0.125;
        counter = counter +2;
    end
    
    
    fprintf('Preamble: %s \n',preamble)
    fprintf('Parity: %d \n',0)
    fprintf('Band number: %d \n', NavData.BandNumber);
    fprintf('Block ID: %d \n',NavData.BlockID)
    fprintf('Issue of data - Ionosphere: %d \n',NavData.IODI);
    for i = 1:15
        fprintf('Satellite IGP [ %d ]: ',i+15*NavData.BlockID)
        fprintf('Grid Ionospheric Vertical Error Index: %d \t',NavData.GIVEI(i))
        fprintf('IGP Vertical Delay Estimate: %fm \n',NavData.IGP(i));
    end

end

function NavData = MT27(binMsg,preamble)
    
    idxcoor1lat = [37 44 72 79 107 114 142 149 177 184];
    idxcoor1lon = [45 53 80 88 115 123 150 158 185 193];
    idxcoor2lat = [54 61 89 96 124 131 159 166 194 201];
    idxcoor2lon = [62 70 97 105 132 140 167 175 202 210];
    idxregshape = [71 106 141 176 211];
    
    NavData.IODS         = bin2dec(binMsg(15:17));        % IODS bits 14-16
    NavData.NumSerMsg    = bin2dec(binMsg(18:20))+1;      % Number of service messages bits 17-19
    NavData.SerMsgNum    = bin2dec(binMsg(21:23))+1;      % Service message number bits 20-22
    NavData.NumReg       = bin2dec(binMsg(24:26));        % Number of regions bits 23-25
    NavData.PrioCode     = bin2dec(binMsg(27:28));        % Priority Code bits 26-27
    NavData.dUDREin      = bin2dec(binMsg(29:32));        % delta UDRE indicator inside bits 28-31
    NavData.dUDREout     = bin2dec(binMsg(33:36));        % delta UDRE indicator outside bits 32-35
    %NavData.parity      = bin2dec(binMsg(1:250));
    
    if NavData.NumReg > 0
        counter = 1;
        for i = 1:NavData.NumReg
            NavData.coor1lat(i) = bin2dec(binMsg(idxcoor1lat(counter):idxcoor1lat(counter+1)));
            NavData.coor1lon(i) = bin2dec(binMsg(idxcoor1lon(counter):idxcoor1lon(counter+1)));
            NavData.coor2lat(i) = bin2dec(binMsg(idxcoor2lat(counter):idxcoor2lat(counter+1)));
            NavData.coor2lon(i) = bin2dec(binMsg(idxcoor2lon(counter):idxcoor2lon(counter+1)));
            NavData.regShape(i) = bin2dec(binMsg(idxregshape(i):idxregshape(i)));
            counter = counter +2;
        end
    end
    
    
    fprintf('Preamble: %s \n',preamble)
    fprintf('Parity: %d \n',0)
    fprintf('IODS: %d \n', NavData.IODS);
    fprintf('Number of service messages: %d \n',NavData.NumSerMsg)
    fprintf('Service message number: %d \n',NavData.SerMsgNum);
    fprintf('Number of regions: %d \n',NavData.NumReg)
    fprintf('Priority Code: %d \n', NavData.PrioCode);
    fprintf('Delta UDRE indicator inside: %d \n',NavData.dUDREin)
    fprintf('Delta UDRE indicator outside: %d \n',NavData.dUDREout);
    % for i = 1:5
    %     fprintf('Satellite IGP [ %d ]: ',i+15*NavData.BlockID)
    %     fprintf('Grid Ionospheric Vertical Error Index: %d \t',NavData.GIVEI(i))
    %     fprintf('IGP Vertical Delay Estimate: %fm \n',NavData.IGP(i));
    % end

end

function [parity] = crc24q(msg)

    % SYNTAX:
    %   [parity] = crc24q(msg);
    %
    % INPUT:
    %   msg = binary message
    %
    % OUTPUT:
    %   parity = crc parity (24 bits)
    %
    % DESCRIPTION:
    %   Applies CRC-24Q QualComm algorithm.
    
    %----------------------------------------------------------------------------------------------
    %                           goGPS v0.4.3
    %
    % Copyright (C) 2009-2014 Mirko Reguzzoni, Eugenio Realini
    %
    % ('rtcm3torinex.c', by Dirk Stoecker, BKG Ntrip Client (BNC) Version 1.6.1
    %  was used as a reference)
    %----------------------------------------------------------------------------------------------
    %
    %    This program is free software: you can redistribute it and/or modify
    %    it under the terms of the GNU General Public License as published by
    %    the Free Software Foundation, either version 3 of the License, or
    %    (at your option) any later version.
    %
    %    This program is distributed in the hope that it will be useful,
    %    but WITHOUT ANY WARRANTY; without even the implied warranty of
    %    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    %    GNU General Public License for more details.
    %
    %    You should have received a copy of the GNU General Public License
    %    along with this program.  If not, see <http://www.gnu.org/licenses/>.
    %----------------------------------------------------------------------------------------------
    
    parity = uint32(0);
    
    %check the length of the input string, in case make it splittable byte-wise
    remainder = rem(length(msg),8);
    if (remainder ~= 0)
        fill = char(ones(1,8-remainder)*48); %fill string of zeroes
        msg = [fill msg];
    end
    
    Nbits = length(msg);
    
    %pre-allocate to increase speed
    Nbytes = Nbits / 8;
    bytes = cell(1,Nbytes);
    k = 1;
    for j = 1 : 8 : Nbits
        bytes{k} = msg(j:j+7);
        k = k + 1;
    end
    %call 'fbin2dec' and 'bitshift' only once (to optimize speed)
    bytes = bitshift(fbin2dec(bytes), 16);
    bytes = uint32(bytes);
    for i = 1 : Nbytes
        parity = bitxor(parity, bytes(i));
        for j = 1 : 8
            parity = bitshift(parity, 1);
            if bitand(parity, 16777216)
                parity = bitxor(parity, 25578747);
            end
        end
    end
    
    parity = dec2bin(parity, 24);
end

function x=fbin2dec(s)
    %FBIN2DEC (fast bin2dec) Convert binary string to decimal integer.
    %   X = FBIN2DEC(B) interprets the binary string B and returns in X the
    %   equivalent decimal number. It is a stripped version of "bin2dec", with
    %   a minimal check on input.
    %
    %   If B is a character array, or a cell array of strings, each row is
    %   interpreted as a binary string. 
    %
    %   Example
    %       fbin2dec('010111') returns 23
    
    %----------------------------------------------------------------------------------------------
    %                           goGPS v0.4.3
    %
    % Copyright (C) 2009-2014 Mirko Reguzzoni, Eugenio Realini
    %----------------------------------------------------------------------------------------------
    %
    %    This program is free software: you can redistribute it and/or modify
    %    it under the terms of the GNU General Public License as published by
    %    the Free Software Foundation, either version 3 of the License, or
    %    (at your option) any later version.
    %
    %    This program is distributed in the hope that it will be useful,
    %    but WITHOUT ANY WARRANTY; without even the implied warranty of
    %    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    %    GNU General Public License for more details.
    %
    %    You should have received a copy of the GNU General Public License
    %    along with this program.  If not, see <http://www.gnu.org/licenses/>.
    %----------------------------------------------------------------------------------------------
    
    % handle input
    s = char(s);
    
    [m,n] = size(s);
    
    % Convert to numbers
    v = s - '0';
    twos = pow2(n-1:-1:0);
    x = sum(v .* twos(ones(m,1),:),2);
end

function hexStr = convert2hex(binMsg)
    % Convertir la cadena binaria a hexadecimal
    hexStr = '';
    for i = 1:4:length(binMsg)
        % Tomar grupos de 4 bits
        binSegment = binMsg(i:i+3);
        % Convertir el segmento binario a decimal
        decValue = bin2dec(binSegment);
        % Convertir el valor decimal a hexadecimal y concatenar
        hexStr = [hexStr, dec2hex(decValue)];
    end
end