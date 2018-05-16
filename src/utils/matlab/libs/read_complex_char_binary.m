% Usage: read_complex_binary (filename, [count])
%
% Opens filename and returns the contents as a column vector,
%  treating them as 32 bit complex numbers
%

% -------------------------------------------------------------------------
%
% Copyright (C) 2010-2018  (see AUTHORS file for a list of contributors)
%
% GNSS-SDR is a software defined Global Navigation
%           Satellite Systems receiver
%
% This file is part of GNSS-SDR.
%
% GNSS-SDR is free software: you can redistribute it and/or modify
% it under the terms of the GNU General Public License as published by
% the Free Software Foundation, either version 3 of the License, or
% at your option) any later version.
%
% GNSS-SDR is distributed in the hope that it will be useful,
% but WITHOUT ANY WARRANTY; without even the implied warranty of
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
% GNU General Public License for more details.
%
% You should have received a copy of the GNU General Public License
% along with GNSS-SDR. If not, see <https://www.gnu.org/licenses/>.
%
% -------------------------------------------------------------------------
%

function v = read_complex_char_binary (filename, count)

m = nargchk (1,2,nargin);
if (m)
    usage (m);
end

if (nargin < 2)
    count = Inf;
end

f = fopen (filename, 'rb');
if (f < 0)
    v = 0;
else
    t = fread (f, [2, count], 'int8');
    fclose (f);
    v = t(1,:) + t(2,:)*i;
    [r, c] = size (v);
    v = reshape (v, c, r);
end
