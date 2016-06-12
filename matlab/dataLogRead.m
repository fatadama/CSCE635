clear variables;
close all;

% angle conversion factors
d2r = pi/180.0;
r2d = 180.0/pi;
% conversion factor knots to m/s
knots2ms = 0.514444;

loadData;

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
inbr = [inbr;length(controlObj)];

kcount = 2;

% target figure position
figpos = [100 300 1500 675];

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
    hold on;
    plot(controlObj(inu(1),3),controlObj(inu(1),2),'ro','linewidth',2,'markersize',6);
    %quiver(controlObj(inu(1),3),controlObj(inu(1),2),sin(gpsi(1)),cos(gpsi(1)),10,'linewidth',2);
    %quiver(controlObj(inu,3),controlObj(inu,2),sin(gpsi),cos(gpsi),'linewidth',2);
    %hold on;
    % draw where the target should be
    thet = linspace(-pi,pi,100)';
    circ = 5.0.*[cos(thet) sin(thet)];
    %hold on;
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
    plot(controlObj(inu,1),controlObj(inu,2:3),'.','linewidth',2);
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

    set(gcf,'position',figpos);

end

%% figure for write up

figpos2 = [100 300 830 675];

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
    
    figure(1+length(inbr)+kcount);
    clf;
    
    subplot(311);
    plot(controlObj(inu,3),controlObj(inu,2),'r-','linewidth',2);
    hold on;
    plot(controlObj(inu(1),3),controlObj(inu(1),2),'ro','linewidth',2,'markersize',6);
    %quiver(controlObj(inu(1),3),controlObj(inu(1),2),sin(gpsi(1)),cos(gpsi(1)),10,'linewidth',2);
    %quiver(controlObj(inu,3),controlObj(inu,2),sin(gpsi),cos(gpsi),'linewidth',2);
    %hold on;
    % draw where the target should be
    thet = linspace(-pi,pi,100)';
    circ = 5.0.*[cos(thet) sin(thet)];
    %hold on;
    %plot(circ(:,1),circ(:,2),'k--');
    % draw the synthetic waypoints
    wps = unique(synWp(:,2:3),'rows');
    for kin = 1:size(wps,1)
        plot(wps(kin,2),wps(kin,1),'bx','markersize',6);
        plot(wps(kin,2)+circ(:,1),wps(kin,1)+circ(:,2),'k--');
    end
    xlabel('Y (m)');
    ylabel('X (m)');
    grid on;
    axis equal
    
    subplot(313);
    plot(controlObj(inu,1),controlObj(inu,8),'linewidth',2);
    grid on;
    ylabel('Rudder (normalized)');
    
    subplot(312);
    % heading error
    plot(controlObj(inu,1),minAngleErr(controlObj(inu,5),controlObj(inu,7)),'b-','linewidth',2);
    hold on;
    plot(controlObj(inu,1),minAngleErr(gpsi,controlObj(inu,7)),'r--');
    set(gca,'xlim',[controlObj(inu(1),1) controlObj(inu(end),1)]);
    grid on;
    ylabel('heading error (rad)');
    legend('measured-ref','GPS-ref');

    set(gcf,'position',figpos2);

end