% -------------------------------------------------------------------------
%
% GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
% This file is part of GNSS-SDR.
%
% SPDX-FileCopyrightText: Javier Arribas 2011
% SPDX-License-Identifier: GPL-3.0-or-later
%
% -------------------------------------------------------------------------

function [pvt_raw] = gps_l1_ca_read_pvt_raw_dump (channels, filename, count)

%% usage: read_tracking_dat (filename, [count])
%%
%% open GNSS-SDR pvt binary log file .dat and return the contents
%%

m = nargchk (1,2,nargin);
num_double_vars=3;
double_size_bytes=8;
skip_bytes_each_read=double_size_bytes*num_double_vars*channels;
bytes_shift=0;
if (m)
    usage (m);
end

if (nargin < 3)
    count = Inf;
end
%loops_counter = fread (f, count, 'uint32',4*12);
f = fopen (filename, 'rb');
if (f < 0)
else
    for N=1:1:channels
        pvt_raw.Pseudorange_m(N,:) = fread (f, count, 'float64',skip_bytes_each_read-double_size_bytes);
        bytes_shift=bytes_shift+double_size_bytes;
        fseek(f,bytes_shift,'bof'); % move to next interleaved
        pvt_raw.Pseudorange_symbol_shift(N,:) = fread (f, count, 'float64',skip_bytes_each_read-double_size_bytes);
        bytes_shift=bytes_shift+double_size_bytes;
        fseek(f,bytes_shift,'bof'); % move to next interleaved
        pvt_raw.tx_time(N,:) = fread (f, count, 'float64',skip_bytes_each_read-double_size_bytes);
        bytes_shift=bytes_shift+double_size_bytes;
        fseek(f,bytes_shift,'bof'); % move to next interleaved
    end

    fclose (f);

    %%%%%%%% output vars %%%%%%%%
    %     for (unsigned int i=0; i<d_nchannels ; i++)
    %         {
    %             tmp_double = in[i][0].Pseudorange_m;
    %             d_dump_file.write((char*)&tmp_double, sizeof(double));
    %             tmp_double = in[i][0].Pseudorange_symbol_shift;
    %             d_dump_file.write((char*)&tmp_double, sizeof(double));
    %             d_dump_file.write((char*)&d_tx_time, sizeof(double));
    %         }
end
