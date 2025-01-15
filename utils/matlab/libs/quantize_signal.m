% This function opens a binary file using the ibyte format, reads and
% quantizes the signal using the most significant nbits, and stores the
% quantized signal into an output file. The output file also uses the
% ibyte format.
%
% Usage: quantize_signal (infile, outfile, nbits)
%
%   Inputs:
%       infile          - Input file name
%       outfile         - Output file name
%       nbits           - number of quantization bits
%

% -------------------------------------------------------------------------
%
% GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
% This file is part of GNSS-SDR.
%
% SPDX-FileCopyrightText: Marc Majoral, 2020. mmajoral(at)cttc.es
% SPDX-License-Identifier: GPL-3.0-or-later
%
% -------------------------------------------------------------------------
%

function quantize_signal (infile, outfile, nbits)

%% usage: quantize_signal (infile, outfile, nbits)
%%
%% open input signal file infile, quantize the signal using the most
%% significant nbits and write the quantized signal to outfile
%%

fileID = fopen(infile, 'rb');
fileID2 = fopen(outfile, 'wb');

% initialize loop
BlockSize = 20000000;   % processing block size
NumBitsPerSample = 8;   % num. bits per sample ibyte format.
NumBitsSHift = NumBitsPerSample - nbits;
DivVal = 2^NumBitsSHift;
Lim2sCompl = 2^(NumBitsPerSample - 1) - 1;
Base2sCompl = 2^NumBitsPerSample;
do = true;
data_bytes = fread(fileID, BlockSize);

while do

    % 2's complement
    for k=1:length(data_bytes)
        if (data_bytes(k) > Lim2sCompl)
            data_bytes(k) = -Base2sCompl + data_bytes(k);
        end
    end

    % take the nbits most significant bits
    data_bytes = floor(data_bytes/DivVal);

    % quantization correction
    data_bytes = data_bytes*2 + 1;

    % 2's complement
    for k=1:length(data_bytes)
        if (data_bytes(k) < 0)
            data_bytes(k) = Base2sCompl + data_bytes(k);
        end
    end

    % write result
    fwrite(fileID2, data_bytes);

    if (size(data_bytes) < BlockSize)
        do = false;
    else
        data_bytes = fread(fileID,BlockSize);
    end
end

% close files
fclose(fileID);
fclose(fileID2);
