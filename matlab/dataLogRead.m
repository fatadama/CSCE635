clear variables;
close all;

d2r = pi/180.0;
r2d = 180.0/pi;
% conversion factor knots to m/s
knots2ms = 0.514444;

% load file from location
%folder = '../python/xbee_bridge/20160506_100753/';
%folder = '../python/xbee_bridge/20160506_093329/';
folder = '../python/xbee_bridge/20160506_094356/';
%folder = '../python/xbee_bridge/20160430_Bush_Lake/';

% parse name
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

%% plot the GPS vel and heading and the corresponding control actions

figure(1);
clf;

subplot(211);
plot(controlObj(:,1),controlObj(:,4));
hold on;
plot(gps(:,1),gps(:,5),'r--');
set(gca,'xlim',[gps(1,1),gps(end,1)]);
set(gca,'ylim',[0 12.0]);
ylabel('v(m/s)');
grid on;

subplot(212);
plot(controlObj(:,1),controlObj(:,5),'b-','linewidth',2);
hold on;
plot(gps(:,1),gps(:,6),'r--');
%set(gca,'ylim',[0 2*pi])
ylabel('hdg(rad)');
grid on;