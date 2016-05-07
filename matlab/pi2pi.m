function x = pi2pi(x)
% make values in the array x be in the range [-pi, pi]

twopi = 2*pi;
while any(x > pi)
    x(x>pi) = x(x>pi)-twopi;
end
while any(x < -pi)
    x(x<-pi) = x(x<-pi)+twopi;
end

end