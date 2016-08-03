% write complex binary file wich can be used with the GNURadio
% "read_complex_binary" function. GNURadio complex file sink format
% compatible. @ Javier Arribas 2011
function v=write_complex_binary (x,filename, count)

  %% usage: read_complex_binary (x, filename, [count])
  %%
  %%  open filename and return the contents as a column vector, 
  %%  treating them as 32 bit complex numbers
  %%

  m = nargchk (1,3,nargin);
  if (m)
    usage (m);
  end

  if (nargin < 3)
    count = length(x);
  end

  f = fopen (filename, 'ab');
  if (f < 0)
    v = -1;
  else
    %for n=1:1:count
    %    fwrite (f, real(x(n)), 'float');
    %    fwrite (f, imag(x(n)), 'float');
    %end
    y=interleave(real(x(1:count)),imag(x(1:count)));
    v=fwrite (f, y, 'float');
    fclose (f);
  end