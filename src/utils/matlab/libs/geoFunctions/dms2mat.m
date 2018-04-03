function [dout,mout,sout] = dms2mat(dms,n)

% DMS2MAT Converts a dms vector format to a [deg min sec] matrix
%
%  [d,m,s] = DMS2MAT(dms) converts a dms vector format to a
%  deg:min:sec matrix.  The vector format is dms = 100*deg + min + sec/100.
%  This allows compressed dms data to be expanded to a d,m,s triple,
%  for easier reporting and viewing of the data.
%
%  [d,m,s] = DMS2MAT(dms,n) uses n digits in the accuracy of the
%  seconds calculation.  n = -2 uses accuracy in the hundredths position,
%  n = 0 uses accuracy in the units position.  Default is n = -5.
%  For further discussion of the input n, see ROUNDN.
%
%  mat = DMS2MAT(...) returns a single output argument of mat = [d m s].
%  This is useful only if the input dms is a single column vector.
%
%  See also MAT2DMS

%  Copyright 1996-2002 Systems Planning and Analysis, Inc. and The MathWorks, Inc.
%  Written by:  E. Byrns, E. Brown
%  Revision: 1.10    $Date: 2002/03/20 21:25:06


if nargin == 0
    error('Incorrect number of arguments')
elseif nargin == 1
    n = -5;
end

%  Test for empty arguments

if isempty(dms); dout = []; mout = []; sout = []; return; end

%  Test for complex arguments

if ~isreal(dms)
    warning('Imaginary parts of complex ANGLE argument ignored')
    dms = real(dms);
end

%  Don't let seconds be rounded beyond the tens place.
%  If you did, then 55 seconds rounds to 100, which is not good.

if n == 2;  n = 1;   end

%  Construct a sign vector which has +1 when dms >= 0 and -1 when dms < 0.

signvec = sign(dms);
signvec = signvec + (signvec == 0);   %  Ensure +1 when dms = 0

%  Decompress the dms data vector

dms = abs(dms);
d = fix(dms/100);                      %  Degrees
m = fix(dms) - abs(100*d);             %  Minutes
[s,msg] = roundn(100*rem(dms,1),n);    %  Seconds:  Truncate to roundoff error
if ~isempty(msg);   error(msg);   end

%  Adjust for 60 seconds or 60 minutes.
%  Test for seconds > 60 to allow for round-off from roundn,
%  Test for minutes > 60 as a ripple effect from seconds > 60


indx = find(s >= 60);
if ~isempty(indx);   m(indx) = m(indx) + 1;   s(indx) = s(indx) - 60;   end
indx = find(m >= 60);
if ~isempty(indx);   d(indx) = d(indx) + 1;   m(indx) =  m(indx) - 60;   end

%  Data consistency checks

if any(m > 59) | any (m < 0)
    error('Minutes must be >= 0 and <= 59')
    
elseif any(s >= 60) | any( s < 0)
    error('Seconds must be >= 0 and < 60')
end

%  Determine where to store the sign of the angle.  It should be
%  associated with the largest nonzero component of d:m:s.

dsign = signvec .* (d~=0);
msign = signvec .* (d==0 & m~=0);
ssign = signvec .* (d==0 & m==0 & s~=0);

%  In the application of signs below, the comparison with 0 is used so that
%  the sign vector contains only +1 and -1.  Any zero occurrences causes
%  data to be lost when the sign has been applied to a higher component
%  of d:m:s.  Use fix function to eliminate potential round-off errors.

d = ((dsign==0) + dsign).*fix(d);      %  Apply signs to the degrees
m = ((msign==0) + msign).*fix(m);      %  Apply signs to minutes
s = ((ssign==0) + ssign).*s;           %  Apply signs to seconds

%  Set the output arguments

if nargout <= 1
    dout = [d m s];
elseif nargout == 3
    dout = d;   mout = m;   sout = s;
else
    error('Invalid number of output arguments')
end
