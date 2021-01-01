% -------------------------------------------------------------------------
%
% GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
% This file is part of GNSS-SDR.
%
% SPDX-FileCopyrightText: Javier Arribas 2011
% SPDX-License-Identifier: GPL-3.0-or-later
%
% -------------------------------------------------------------------------

function [telemetry] = gps_l1_ca_read_telemetry_dump (filename, count)

%%
%% open GNSS-SDR tracking binary log file .dat and return the contents
%%

m = nargchk (1,2,nargin);
num_double_vars=3;
double_size_bytes=8;
num_int_vars=2;
int_size_bytes=4;
skip_bytes_each_read=double_size_bytes*num_double_vars+num_int_vars*int_size_bytes;
bytes_shift=0;
if (m)
    usage (m);
end

if (nargin < 3)
    count = Inf;
end
f = fopen (filename, 'rb');
if (f < 0)
else
    [x, loops_counter] = fread (f,skip_bytes_each_read);
    fseek(f,0,-1);
    for i=1:min(count, loops_counter),
        telemetry(i).tow_current_symbol_ms = fread (f, count, 'float64',skip_bytes_each_read-double_size_bytes);
        bytes_shift=bytes_shift+double_size_bytes;
        fseek(f,bytes_shift,'bof'); % move to next interleaved
        telemetry(i).tracking_sample_counter = fread (f, count, 'uint64',skip_bytes_each_read-double_size_bytes);
        bytes_shift=bytes_shift+double_size_bytes;
        fseek(f,bytes_shift,'bof'); % move to next interleaved
        telemetry(i).tow = fread (f, count, 'float64',skip_bytes_each_read-double_size_bytes);
        bytes_shift=bytes_shift+double_size_bytes;
        fseek(f,bytes_shift,'bof'); % move to next interleaved
        telemetry(i).nav_simbols = fread (f, count, 'int32',skip_bytes_each_read-int_size_bytes);
        bytes_shift=bytes_shift+int_size_bytes;
        fseek(f,bytes_shift,'bof'); % move to next interleaved
        telemetry(i).prn = fread (f, count, 'int32',skip_bytes_each_read-int_size_bytes);
        bytes_shift=bytes_shift+int_size_bytes;
        fseek(f,bytes_shift,'bof'); % move to next interleaved
    end
    fclose (f);
end
