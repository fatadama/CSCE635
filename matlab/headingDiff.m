function dh = headingDiff(h1,h2)
% function dh = headingDiff(h1,h2)
%
% TDW
% 2016-05-06
%
% Compute the h1-h2, always making sure the
% angles are as close to each other as possible in the 0->2*pi sense

while h1-h2 > pi
    h1 = h1-2*pi;
end
while h1-h2 < -pi
    h1 = h1+2*pi;
end

dh=h1-h2;

end