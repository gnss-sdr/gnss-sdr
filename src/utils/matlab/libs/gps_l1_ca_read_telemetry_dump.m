% Javier Arribas 2011             
function [telemetry] = gps_l1_ca_read_telemetry_dump (filename, count)

  %% usage: read_tracking_dat (filename, [count])
  %%
  %% open GNSS-SDR tracking binary log file .dat and return the contents
  %%

  m = nargchk (1,2,nargin);
  num_double_vars=3;
  double_size_bytes=8;
  skip_bytes_each_read=double_size_bytes*num_double_vars;
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
        telemetry.preamble_delay_ms = fread (f, count, 'float64',skip_bytes_each_read-double_size_bytes);
        bytes_shift=bytes_shift+double_size_bytes;
        fseek(f,bytes_shift,'bof'); % move to next interleaved
        telemetry.prn_delay_ms = fread (f, count, 'float64',skip_bytes_each_read-double_size_bytes);
        bytes_shift=bytes_shift+double_size_bytes;
        fseek(f,bytes_shift,'bof'); % move to next interleaved
        telemetry.Preamble_symbol_counter = fread (f, count, 'float64',skip_bytes_each_read-double_size_bytes);
        bytes_shift=bytes_shift+double_size_bytes;
        fseek(f,bytes_shift,'bof'); % move to next interleaved

    fclose (f);
    
    %%%%%%%% output vars %%%%%%%%
%           {
% 				double tmp_double;
% 				tmp_double = current_synchro_data.Preamble_delay_ms;
% 				d_dump_file.write((char*)&tmp_double, sizeof(double));
% 				tmp_double = current_synchro_data.Prn_delay_ms;
% 				d_dump_file.write((char*)&tmp_double, sizeof(double));
% 				tmp_double = current_synchro_data.Preamble_symbol_counter;
% 				d_dump_file.write((char*)&tmp_double, sizeof(double));
%             }
  end
  
