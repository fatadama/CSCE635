clear variables;
close all;

d2r = pi/180.0;
r2d = 180.0/pi;
% conversion factor knots to m/s
knots2ms = 0.514444;

% load file from location
%folder = '../python/xbee_bridge/20160506_100753/';
%folder = '../python/xbee_bridge/20160506_093329/';
%folder = '../python/xbee_bridge/20160506_094356/';
%folder = '../python/xbee_bridge/20160430_Bush_Lake/';

% logs with waypoint naviagation
%folder = '../python/xbee_bridge/20160507_141654/'; % only one trial here
%folder = '../python/xbee_bridge/20160507_141917/'; % three trials in this one
%folder = '../python/xbee_bridge/20160507_143130/'; % one trial here
%folder = '../python/xbee_bridge/20160507_143254/'; % one trial
%folder = '../python/xbee_bridge/20160507_143407/'; % one trial
%folder = '../python/xbee_bridge/20160507_143717/'; % one trial, looks quite good
%folder = '../python/xbee_bridge/20160507_143924/'; % one trial, looks good. I think this is the one where I sent it back into auto mode and it beached itself
folder = '../python/xbee_bridge/20160507_144045/'; % one trial, looks good
%folder = '../python/xbee_bridge/20160507_144251/'; % one trial, meandering but looks OK
%folder = '../python/xbee_bridge/20160507_144625/';% one trial, meandering but OK
%folder = '../python/xbee_bridge/20160507_144856/';% one trial, meandering not good
%folder = '../python/xbee_bridge/20160507_145007/';% one trial, didn't get there
%folder = '../python/xbee_bridge/20160507_145139/';% one trial, looks OK

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
% make the gps on -pi, pi for comparison
gps(:,6) = pi2pi(gps(:,6));

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
plot(controlObj(:,1),controlObj(:,4),'b-','linewidth',2);
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

%% figure of how the waypoint nav performs

% plot each trial individually

%find breaks in the time history of controlObj(:,1)
inbr = find(abs(diff(controlObj(:,1)) > 2.0));
if isempty (inbr)
    inbr = length(controlObj);
end

kcount = 2;

for kcount = 1:length(inbr)
    %array of indices to use
    if kcount == 1
        inu = 1:inbr(kcount);
    else
        inu = (inbr(kcount-1)+1):inbr(kcount);
    end
    % interpolate gps for this time
    inr = find ( diff(gps(:,1))==0.0);
    inp = setdiff(1:length(gps),inr);
    gps2 = gps(inp,:);
    gpsi = interp1(gps2(:,1),gps2(:,6),controlObj(inu,1));
    
    figure(1+kcount);
    clf;
    
    subplot(231);
    plot(controlObj(inu,3),controlObj(inu,2),'r-','linewidth',2);
    % draw where the target should be
    thet = linspace(-pi,pi,100)';
    circ = 5.0.*[cos(thet) sin(thet)];
    hold on;
    plot(circ(:,1),circ(:,2),'k--');
    xlabel('Y (m)');
    ylabel('X (m)');
    grid on;
    axis equal
    
    subplot(232);
    plot(controlObj(inu,1),controlObj(inu,5),'b-','linewidth',2);
    hold on;
    plot(controlObj(inu,1),controlObj(inu,7),'r-','linewidth',2);
    plot(gps(:,1),gps(:,6),'r--');
    set(gca,'xlim',[controlObj(inu(1),1) controlObj(inu(end),1)]);
    grid on;
    ylabel('heading (rad)');
    legend('Measured','Reference','Raw GPS');
    title( sprintf('Trial %d',kcount));
    
    subplot(233);
    plot(controlObj(inu,1),controlObj(inu,2:3),'linewidth',2);
    ylabel('X-Y(m)');
    legend('X','Y');
    grid on;
    
    subplot(236);
    plot(controlObj(inu,1),controlObj(inu,4),'linewidth',2);
    hold on;
    plot(gps(:,1),gps(:,5),'r--');
    set(gca,'xlim',[controlObj(inu(1),1) controlObj(inu(end),1)]);
    ylabel('Ground speed (m/s)');
    legend('Offboard filter','Raw from GPS');
    grid on;
    
    subplot(235);
    plot(controlObj(inu,1),controlObj(inu,8),'linewidth',2);
    grid on;
    ylabel('Rudder (normalized)');
    
    subplot(234);
    % heading error
    plot(controlObj(inu,1),minAngleErr(controlObj(inu,5),controlObj(inu,7)),'b-','linewidth',2);
    hold on;
    plot(controlObj(inu,1),minAngleErr(gpsi,controlObj(inu,7)),'r--');
    set(gca,'xlim',[controlObj(inu(1),1) controlObj(inu(end),1)]);
    grid on;
    ylabel('heading error (rad)');
    legend('measured-ref','GPS-ref');
    
    % throttle plot: this may be important later but not now
%     subplot(236);
%     plot(controlObj(inu,1),controlObj(inu,9),'linewidth',2);
%     %hold on;
%     %plot(control(:,1),control(:,3),'r-x','linewidth',2); % plot the EMILY
%     %TX control
%     set(gca,'xlim',[controlObj(inu(1),1) controlObj(inu(end),1)]);
%     grid on;
%     ylabel('Throttle command (normalized)');

end