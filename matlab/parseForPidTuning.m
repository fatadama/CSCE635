%% parseForPidTuning
%
% Load a specific file with light damping to determine parameters for
% Ziegler-Nichols tuning
% 
% Tim Woodbury
% 2016-06-08
% woodbury.tim@gmail.com

clear variables;
close all;

% angle conversion factors
d2r = pi/180.0;
r2d = 180.0/pi;
% conversion factor knots to m/s
knots2ms = 0.514444;

% June 6 logs from Lake Bryan testing
folder = '../python/xbee_bridge/20160606_143005/';% badly damped straight-line trials

try
    bridge = csvread( [folder 'bridgeStateLog.csv'] ,1,0);
catch err
    disp(err);%probably the file is empty
    bridge = nan(1,51);
end
% control log:
% time, rudder, throttle
control = csvread( [folder 'controlLog.csv'] ,1,0);
% control object log:
% time(sec)	x(m)	y(m)	v(m/s)	hdg(rads)	rangeRef(m)	headingRef(rad)	rudder	throttle
try
    controlObj = csvread( [folder 'controlObjectLog.csv'] ,1,0);
catch err
    disp(err);%probably the file is empty
    controlObj = nan(1,9);
end
% GPS log: systime, gpstime, lon(int), lat(int), speed, heading, status
gps = csvread([folder 'gpsLog.csv'],1,0);
% make the gps on -pi, pi for comparison
gps(:,6) = pi2pi(gps(:,6));
% synthetic waypoint log
synWp = csvread([folder 'synWaypointLog.csv'],1,0);
% grab and print the control settings
FID = fopen([folder 'settings_copy.ini'],'r+');
% save the ultimate gain from this run
Ku = 0;
while ~feof(FID)
    line = fgets(FID);
    % check for comment marker
    [tok,rem] = strtok(line,'#');
    % parse everything before the comment marker ==> tok
    [tok,rem] = strtok(tok,'=');
    switch tok(1:end-1)
        case 'kph'
            fprintf('      Kp: %8.3g\n',str2num(rem(2:end)));
            Ku = str2num(rem(2:end));
        case 'kih'
            fprintf('      Ki: %8.3g\n',str2num(rem(2:end)));
        case 'kdh'
            fprintf('      Kd: %8.3g\n',str2num(rem(2:end)));
        case 'cruisethrottle'
            fprintf('throttle: %8.3g\n',str2num(rem(2:end)));
    end
end
fclose(FID);

%% 

%find breaks in the time history of controlObj(:,1)
inbr = find(abs(diff(controlObj(:,1)) > 2.0));
inbr = [inbr;length(controlObj)];

kcount = 2;

inu = (inbr(kcount-1)+1):inbr(kcount);

% interpolate gps for this time
inr = find ( diff(gps(:,1))==0.0);
inp = setdiff(1:length(gps),inr);
gps2 = gps(inp,:);
gpsi = interp1(gps2(:,1),gps2(:,6),controlObj(inu,1));

% heading error
err = minAngleErr(controlObj(inu,5),controlObj(inu,7));
% smooth error
errf = filter(0.2*[1 1 1 1 1],1,err);
% time
t = controlObj(inu,1);

% find every local minimum in err
n = 0;
m = 0;
lmi = [];
lma = [];
for k = 2:length(errf)-1
    % local min
    if errf(k) < errf(k+1) && errf(k) < errf(k-1)
        n = n+1;
        lmi = [lmi;k];
        fprintf('%8.3g %8.3g\n',t(k),errf(k));
    end
    % local max
    if errf(k) > errf(k+1) && errf(k) > errf(k-1)
        m = n+1;
        lma = [lma;k];
        fprintf('%8.3g %8.3g\n',t(k),errf(k));
    end
end

%% ignore the first maximum; compute the period based on time between extrema

Tps = [diff(t(lma(2:end)));diff(t(lmi))];
Tu = mean(Tps);

% multiple Kpv by Ku to get the P gain
Kpv = [0.5;0.45;0.8;0.60;0.7;0.33;0.2];
% multiply Tiv by Tu to get the integral time constant
Tiv = [Inf;1/1.2;Inf;1/2;1/2.5;1/2;1/2];
Tdv = [0;0;1/8;1/8;3/20;1/3;1/3];

Kp = Ku*Kpv;
Ti = Tiv*Tu;
Td = Tdv*Tu;
Kd = Td.*Kp;
Ki = Kp./Ti;

% print out for TeX
labels = {'P','PI','PD','PID','Pessen','some overshoot','no overshoot'};
for k = 1:length(labels)
    fprintf('%s',labels{k});
    fprintf('& %.3g ',Kp(k),Ki(k),Kd(k),Ti(k),Td(k));
    fprintf('\\\\ \n');
end