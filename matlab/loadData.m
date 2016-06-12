%% loadData
%
% TDW
% 2016-06-06
%
% Load data files into workspace. I made it a script for code reuse.
%
% INPUTS: NULL, you must set the folder to load in the script
%
% OUTPUTS:
%   bridge: array containing the bridgeStateLog
%   control: array containing the controlLog
%   controlObj: array containing the controlObjectLog
%   gps: array containing gps load
%   synWp: array containing the synthetic waypoint log, which records the
%   X-Y position of waypoints as functions of time

% angle conversion factors
d2r = pi/180.0;
r2d = 180.0/pi;
% conversion factor knots to m/s
knots2ms = 0.514444;

% load file from location
%folder = '../python/xbee_bridge/20160506_100753/';
%folder = '../python/xbee_bridge/20160506_093329/';
%folder = '../python/xbee_bridge/20160506_094356/';
%folder = '../python/xbee_bridge/20160430_Bush_Lake/';

% May 7 logs with waypoint naviagation 
%folder = '../python/xbee_bridge/20160507_141654/'; % only one trial here
%folder = '../python/xbee_bridge/20160507_141917/'; % 4x trials in this one, clearly overcorrecting in PID
%folder = '../python/xbee_bridge/20160507_143130/'; % one trial here
%folder = '../python/xbee_bridge/20160507_143254/'; % one trial, does not complete
%folder = '../python/xbee_bridge/20160507_143407/'; % 2x trial, looks OK but overcorrects
%folder = '../python/xbee_bridge/20160507_143717/'; % 2x trial, looks good but maybe underdamped alightly. Might be good sysid candidate 
%folder = '../python/xbee_bridge/20160507_143924/'; % one trial, looks good. I think this is the one where I sent it back into auto mode and it beached itself
%folder = '../python/xbee_bridge/20160507_144045/'; % one trial, looks good, might be sysid candidate
%folder = '../python/xbee_bridge/20160507_144251/'; % 2x trial, underdamped but gets there
%folder = '../python/xbee_bridge/20160507_144625/';% 2x trial, definitely underdamped but gets there
%folder = '../python/xbee_bridge/20160507_144856/';% one trial, meandering not good
%folder = '../python/xbee_bridge/20160507_145007/';% one trial, didn't get there
%folder = '../python/xbee_bridge/20160507_145139/';% one trial, looks OK, might be sysid candidate

% May 10 logs from Lake Bryan testing
%folder = '../python/xbee_bridge/20160510_103505/';
%folder = '../python/xbee_bridge/20160510_105224/';% 3/4 of sqaure path
%folder = '../python/xbee_bridge/20160510_110129/';% loops through square path 3 times
%folder = '../python/xbee_bridge/20160510_110723/';% loops through sqaure path 1 time
%folder = '../python/xbee_bridge/20160510_111027/';% loops through sqaure path 1.5 times
%folder = '../python/xbee_bridge/20160510_111410/';% loops through sqaure path 1.5 times. Performance of the heading PID is not good.
%folder = '../python/xbee_bridge/20160510_133017/';% bunch of repeated point-to-point trials

% June 6 logs from Lake Bryan testing
%folder = '../python/xbee_bridge/20160606_143005/';% badly damped
%straight-line trials - may be good for Ziegler-Nichols
%folder = '../python/xbee_bridge/20160606_143453/';% single trial, badly damped but decaying
%folder = '../python/xbee_bridge/20160606_145521/';% hits one point, seems ok but stopped
%folder = '../python/xbee_bridge/20160606_150749/';% hits each waypoint once then stops. Damping seems ok.
%folder = '../python/xbee_bridge/20160606_151239/';%hits two waypoints, gets turned around
%folder = '../python/xbee_bridge/20160606_152040/';% beached self
%folder = '../python/xbee_bridge/20160606_153409/';% beached self again?
%folder = '../python/xbee_bridge/20160606_153525/';% does circuits, damping is OK
folder = '../python/xbee_bridge/20160606_154110/';% Does circuits, looks like D gain high?
%folder = '../python/xbee_bridge/20160606_154821/';% does half circuit
%folder = '../python/xbee_bridge/20160606_160027/';% hits two waypoints, got confused
%folder = '../python/xbee_bridge/20160606_160256/';% does one circuit then battery dies

% parse name for handling some earlier files with errors
foldermonth = str2num(folder(27:28));
folderday = str2num(folder(29:30));
folderhour = str2num(folder(32:33));

% load bridge log: time, X measured, Y measured, V measured, hdg measured,
% X est, Y est, VX est, VY est, AX est, AY est, X measured, Y measured, V
% measured, hdg measured, then covariance 6 x 6 matrix entries
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
while ~feof(FID)
    line = fgets(FID);
    % check for comment marker
    [tok,rem] = strtok(line,'#');
    % parse everything before the comment marker ==> tok
    [tok,rem] = strtok(tok,'=');
    switch tok(1:end-1)
        case 'kph'
            fprintf('      Kp: %8.3g\n',str2num(rem(2:end)));
        case 'kih'
            fprintf('      Ki: %8.3g\n',str2num(rem(2:end)));
        case 'kdh'
            fprintf('      Kd: %8.3g\n',str2num(rem(2:end)));
        case 'cruisethrottle'
            fprintf('throttle: %8.3g\n',str2num(rem(2:end)));
    end
end
fclose(FID);

% for these cases, the logged headings are in degrees - convert to radians
% the velocities are converted from knots to m/s twice
if foldermonth==5 && folderday==6 && folderhour < 12
    gps(:,6) = gps(:,6).*d2r;
    controlObj(:,5)=controlObj(:,5).*d2r;
    gps(:,5) = gps(:,5)./knots2ms;    
end
% for this log, the logged headings are converted from degrees to radians
% twice
if strcmp(folder, '../python/xbee_bridge/20160430_Bush_Lake/')
    gps(:,6) = gps(:,6).*r2d;
    gps(:,5) = gps(:,5)./knots2ms;
end