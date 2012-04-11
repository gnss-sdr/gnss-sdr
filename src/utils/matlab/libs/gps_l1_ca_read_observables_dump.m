% Javier Arribas 2011             
function [observables] = gps_l1_ca_read_observables_dump (channels, filename, count)

  %% usage: read_tracking_dat (filename, [count])
  %%
  %% open GNSS-SDR tracking binary log file .dat and return the contents
  %%

  m = nargchk (1,2,nargin);
  num_double_vars=5;
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
        observables.preamble_delay_ms(N,:) = fread (f, count, 'float64',skip_bytes_each_read-double_size_bytes);
        bytes_shift=bytes_shift+double_size_bytes;
        fseek(f,bytes_shift,'bof'); % move to next interleaved
        observables.prn_delay_ms(N,:) = fread (f, count, 'float64',skip_bytes_each_read-double_size_bytes);
        bytes_shift=bytes_shift+double_size_bytes;
        fseek(f,bytes_shift,'bof'); % move to next interleaved
        observables.Pseudorange_m(N,:) = fread (f, count, 'float64',skip_bytes_each_read-double_size_bytes);
        bytes_shift=bytes_shift+double_size_bytes;
        fseek(f,bytes_shift,'bof'); % move to next interleaved
        observables.Pseudorange_symbol_shift(N,:) = fread (f, count, 'float64',skip_bytes_each_read-double_size_bytes);
        bytes_shift=bytes_shift+double_size_bytes;
        fseek(f,bytes_shift,'bof'); % move to next interleaved
        observables.PRN(N,:) = fread (f, count, 'float64',skip_bytes_each_read-double_size_bytes);
        bytes_shift=bytes_shift+double_size_bytes;
        fseek(f,bytes_shift,'bof'); % move to next interleaved
    end

    fclose (f);
    
    %%%%%%%% output vars %%%%%%%%
%     for (unsigned int i=0; i<d_nchannels ; i++)
%         {
%             tmp_double = current_gnss_synchro[i].Preamble_delay_ms;
%             d_dump_file.write((char*)&tmp_double, sizeof(double));
%             tmp_double = current_gnss_synchro[i].Prn_delay_ms;
%             d_dump_file.write((char*)&tmp_double, sizeof(double));
%             tmp_double = current_gnss_synchro[i].Pseudorange_m;
%             d_dump_file.write((char*)&tmp_double, sizeof(double));
%             tmp_double = current_gnss_synchro[i].Pseudorange_symbol_shift;
%             d_dump_file.write((char*)&tmp_double, sizeof(double));
%             tmp_double = current_gnss_synchro[i].PRN;
%             d_dump_file.write((char*)&tmp_double, sizeof(double));
%         }
  end
  
