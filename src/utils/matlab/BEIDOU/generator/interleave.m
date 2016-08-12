function z = interleave(x,y)
% Interleave 2 vectors.
%
%   z = interleave(x,y)
%   If the vectors are of different lengths, the remaining elements are 
%   appended to the end of the resulting vector.  x and y can be column or 
%   row vectors (they need not both be the same), and the output will be 
%   column or row based on the configuration of x. Interleaving begins with
%   the first element of x.
%
%   INPUT:
%       x: vector to interleave 
%       y: vector to interleave
%
%   OUTPUT:
%       z: interleaved vector
%
%   EXAMPLE:
%       >>interleave([1 2 3 4], [5 6 7 8 9 10])
%
%       ans = 
%   
%           1     5     2     6     3     7     4     8     9    10
%
%   AUTHOR:
%       Jason Blackaby
%       jblackaby@gmail.com
%%
   
    [M,N] = size(x);
    [P,Q] = size(y);
    
    %Error checking
    if(M>1 && N>1)
        error('x must be a vector (1xM or Mx1)');
    end
    if(P>1 && Q>1)
        error('y must be a vector (1xM or Mx1)');
    end
    if(isempty(x) && isempty(y))
        z = []; return;
    elseif(isempty(x))
        z = y; return;
    elseif(isempty(y))
        z = x; return;
    end
    
    %Determine whether x is a column vector or not.
    if(M==1)
        colvec = false;
    else
        colvec = true;
    end

    %Make sure x is longer than y, otherwise, switch them
    if(length(x)<length(y))
        tmp=x;
        x=y;
        y=tmp;
        swtch = true;
    else
        swtch = false;
    end

    %Initialize z
    len = length(x) + length(y);
    z = zeros(1,len);
    
    %Build indices into z for x and y.
    %If the vectors were switched earlier, we still want z to start with
    %the first element of x. 
    if(~swtch)
        idy = 2:2:2*length(y);
        lasty = idy(end);
        idx = 1:2:lasty;
        
        %We know x is longer or the same length as y, so append the rest of
        %x at the end.
        idx = [idx lasty+1:len];
    else
        idy = 1:2:2*length(y);
        lasty = idy(end);
        idx = 2:2:lasty;
        
        %We know x is longer or the same length as y, so append the rest of
        %x at the end.
        idx = [idx lasty+1:len];
    end
    
    %Form z using the indices.
    z(idy) = y;
    z(idx) = x;
    
    %Make z a column vector if x was
    if(colvec)
        z=z(:);
    end
