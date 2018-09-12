function [u, v] = uvfromidx(idx,width)
if nargin < 2
    width = 346;
end
u = rem(idx-1,width)+1;
v = floor((idx-1)/width)+1;
end