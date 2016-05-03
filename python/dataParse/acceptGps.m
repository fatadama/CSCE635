function [f] = acceptGps(state,statelast)
 if state(1) < statelast(1)
     f=0;return;
 end
 if abs(state(1)-statelast(1)) > 100.0
     f=0;return;
 end
 if abs(norm(state(2:3)) - norm(statelast(2:3))) > 10.0
     f=0;return;
 end
 if abs(state(4)-statelast(4)) > 10.0
     f=0;return;
 end
 if abs(state(5)-statelast(5)) > 0.5*pi
     f=0;return;
 end
 if abs(state(5)) > 2*pi
     f=0;return;
 end
 if abs(state(4)) > 100.0
     f=0;return;
 end
 f=1;return;
 
end