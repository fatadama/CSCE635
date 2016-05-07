function dx = minAngleErr(x1,x2)
% Find the error in two arrays of planar angles, x1-x2. Correct for
% differences of two*pi in recorded values to get the minimum error for any
% given values
%
%TDW
%2016-05-07

dx = x1-x2;
while any(dx > pi)
    dx(dx>pi) = dx(dx>pi)-2*pi;
end
while any(dx < -pi)
    dx(dx<-pi) = dx(dx<-pi)+2*pi;
end

end