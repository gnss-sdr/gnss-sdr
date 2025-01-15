% Usage: read_complex_binary (filename, [count])
%
% Opens filename and returns the contents as a column vector,
%  treating them as 32 bit complex numbers
%

% -------------------------------------------------------------------------
%
% GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
% This file is part of GNSS-SDR.
%
% Copyright (C) 2010-2019  (see AUTHORS file for a list of contributors)
% SPDX-License-Identifier: GPL-3.0-or-later
%
% -------------------------------------------------------------------------
%

function v = read_complex_short_binary (filename, count)

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
    t = fread (f, [2, count], 'short');
    fclose (f);
    v = t(1,:) + t(2,:)*i;
    [r, c] = size (v);
    v = reshape (v, c, r);
end
