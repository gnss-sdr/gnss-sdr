function B1code = generateB1Icode(No)
% generateB1code.m generates one of the 37 BeiDou satellite ranging codes.
%
% B1code = generateB1code(No)
%
%   Inputs:
%       No         - No number of the satellite.
%
%   Outputs:
%       B1code      - a vector containing the desired B1 ranging code(PRN)
%                   (chips).  
%                           Test
%
% * \author Giorgio Savastano, 2015. giorgio.savastano(at)uniroma1.it
%
% Based on Darius Plausinaitis,  Dennis M. Akos, Peter Rinder and 
% Nicolaj Bertelsen
% Updated and converted to matlab by Giorgio Savastano, 2015. giorgio.savastano(at)uniroma1.it
% -------------------------------------------------------------------------


% Generate G1 code -----------------------------------------------------

% Initialize G1 output to speed up the function ---
g1 = zeros(1, 2046);
% Load 11-bit linear shift register ---
reg = -1*[-1 1 -1 1 -1 1 -1 1 -1 1 -1];  

% Generate all G1 signal chips based on the G1 feedback polynomial -----
for i = 1:2046,
    g1(i)       = reg(11);
    saveBit     = reg(1)*reg(7)*reg(8)*reg(9)*reg(10)*reg(11);
    reg(2:11)   = reg(1:10);
    reg(1)      = saveBit;
end,

% Generate G2 code -----------------------------------------------------

% Initialize G2 output to speed up the function ---
g2 = zeros(1, 2046);
% Load shift register ---
reg2 = -1*[-1 1 -1 1 -1 1 -1 1 -1 1 -1];

% Generate all G2 signal chips based on the G2 feedback polynomial -----
for i=1:2046
    
    if     No == 1,  g2(i) = reg2(1)*reg2(3);
    elseif No == 2,  g2(i) = reg2(1)*reg2(4);
    elseif No == 3,  g2(i) = reg2(1)*reg2(5);
    elseif No == 4,  g2(i) = reg2(1)*reg2(6);
    elseif No == 5,  g2(i) = reg2(1)*reg2(8);
    elseif No == 6,  g2(i) = reg2(1)*reg2(9);
    elseif No == 7,  g2(i) = reg2(1)*reg2(10);
    elseif No == 8,  g2(i) = reg2(1)*reg2(11);
    elseif No == 9,  g2(i) = reg2(2)*reg2(7);
    elseif No == 10, g2(i) = reg2(3)*reg2(4);
    elseif No == 11, g2(i) = reg2(3)*reg2(5);
    elseif No == 12, g2(i) = reg2(3)*reg2(6);
    elseif No == 13, g2(i) = reg2(3)*reg2(8);
    elseif No == 14, g2(i) = reg2(3)*reg2(9);
    elseif No == 15, g2(i) = reg2(3)*reg2(10);
    elseif No == 16, g2(i) = reg2(3)*reg2(11);
    elseif No == 17, g2(i) = reg2(4)*reg2(5);
    elseif No == 18, g2(i) = reg2(4)*reg2(6);
    elseif No == 19, g2(i) = reg2(4)*reg2(8);
    elseif No == 20, g2(i) = reg2(4)*reg2(9);
    elseif No == 21, g2(i) = reg2(4)*reg2(10);
    elseif No == 22, g2(i) = reg2(4)*reg2(11);
    elseif No == 23, g2(i) = reg2(5)*reg2(6);
    elseif No == 24, g2(i) = reg2(5)*reg2(8);
    elseif No == 25, g2(i) = reg2(5)*reg2(9);
    elseif No == 26, g2(i) = reg2(5)*reg2(10);
    elseif No == 27, g2(i) = reg2(5)*reg2(11);
    elseif No == 28, g2(i) = reg2(6)*reg2(8);
    elseif No == 29, g2(i) = reg2(6)*reg2(9);
    elseif No == 30, g2(i) = reg2(6)*reg2(10);
    elseif No == 31, g2(i) = reg2(6)*reg2(11);
    elseif No == 32, g2(i) = reg2(8)*reg2(9);
    elseif No == 33, g2(i) = reg2(8)*reg2(10);
    elseif No == 34, g2(i) = reg2(8)*reg2(11);
    elseif No == 35, g2(i) = reg2(9)*reg2(10);
    elseif No == 36, g2(i) = reg2(9)*reg2(11);
    elseif No == 37, g2(i) = reg2(10)*reg2(11);
      
    end
    
    saveBit     = reg2(1)*reg2(2)*reg2(3)*reg2(4)*reg2(5)*reg2(8)*reg2(9)*reg2(11);
    reg2(2:11)   = reg2(1:10);
    reg2(1)      = saveBit;
end,
  
% Form single sample B1 code by multiplying G1 and G2 -----------------
B1code = -(g1 .* g2);
  
