%% sysid2
% use the filtered data
% t = 560-640 target

clear variables;
close all;
addpath('../');

% sysid input sets
data = {};
dcounter = 0;
for k = 1:3
    if k == 1
        folder = '../../python/xbee_bridge/20160507_143717/';% use 2x trials
    end
    if k == 2
        folder = '../../python/xbee_bridge/20160507_144045/';% use 1x trial
    end
    if k == 3
        folder = '../../python/xbee_bridge/20160507_145139/';% use 1x trial
    end
    % time(sec)	x(m)	y(m)	v(m/s)	hdg(rads)	rangeRef(m)	headingRef(rad)	rudder	throttle
    controlObj = csvread( [folder 'controlObjectLog.csv'] ,1,0);
    % GPS log: systime, gpstime, lon(int), lat(int), speed, heading, status
    gps = csvread([folder 'gpsLog.csv'],1,0);
    % make the gps on -pi, pi for comparison
    gps(:,6) = pi2pi(gps(:,6));
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
        gpsi = interp1(gps2(:,1),gps2(:,5:6),controlObj(inu,1));
        
        % make continuous heading
        hdgc = gpsi(:,2);
        for kk = 2:length(hdgc)
            while hdgc(kk)-hdgc(kk-1) > pi
                hdgc(kk:end) = hdgc(kk:end)-2*pi;
            end
            while hdgc(kk)-hdgc(kk-1) < -pi
                hdgc(kk:end) = hdgc(kk:end)+2*pi;
            end
        end
        gpsi(:,2) = hdgc;
        
        dcounter = dcounter+1;
        data{dcounter} = [controlObj(inu,:) gpsi];
        
        figure(kcount+dcounter);
        clf;
        
        subplot(221);
        plot(controlObj(inu,1),controlObj(inu,5),'b-','linewidth',2);
        hold on;
        plot(controlObj(inu,1),controlObj(inu,7),'r-','linewidth',2);
        plot(gps(:,1),gps(:,6),'r--');
        set(gca,'xlim',[controlObj(inu(1),1) controlObj(inu(end),1)]);
        grid on;
        ylabel('heading (rad)');
        legend('Measured','Reference','Raw GPS');
        title( sprintf('Trial %d',kcount));
        
        subplot(224);
        plot(controlObj(inu,1),controlObj(inu,4),'linewidth',2);
        hold on;
        plot(gps(:,1),gps(:,5),'r--');
        set(gca,'xlim',[controlObj(inu(1),1) controlObj(inu(end),1)]);
        ylabel('Ground speed (m/s)');
        legend('Offboard filter','Raw from GPS');
        grid on;
        
        subplot(223);
        plot(controlObj(inu,1),controlObj(inu,8),'linewidth',2);
        grid on;
        ylabel('Rudder (normalized)');
        
        subplot(222);
        % heading error
        plot(controlObj(inu,1),minAngleErr(controlObj(inu,5),controlObj(inu,7)),'b-','linewidth',2);
        hold on;
        plot(controlObj(inu,1),minAngleErr(gpsi(:,2),controlObj(inu,7)),'r--');
        set(gca,'xlim',[controlObj(inu(1),1) controlObj(inu(end),1)]);
        grid on;
        ylabel('heading error (rad)');
        legend('measured-ref','GPS-ref');
        
        set(gcf,'position',figpos);
        
    end
end
%% Model fit

global T_TARG Y_TARG UC EQOM;

% x = (rx ry V hdg]
eqom = @(t,y,K,u) [y(3)*[cos(y(4));sin(y(4))];K(2)*u(1);K(1)*u(1)];

k = 1;

EQOM = eqom;
T_TARG = data{k}(:,1);
Y_TARG = [data{k}(:,2:3) data{k}(:,end-1:end)];% X Y V hdg(rad)
UC = data{k}(:,8);

%x0 = [xUr(1)/mean(V);xUt(1);0.5];
x0 = [1.0;1.0]';
% x > 0.0 req'd: -eye*x < -0.0
lb = [-5.0;-5.0];
ub = [5.0;5.0];
xsol = fmincon(@fsolfun,x0,[],[],[],[],lb,ub,[],optimset('disp','iter'));

% propagate
YE = zeros(size(data{k}(:,2:5)));
y0 = Y_TARG(1,:)';
YE(1,:) = y0';
for kk = 1:length(T_TARG)-1
    [~,Y1] = ode45(EQOM,[T_TARG(kk) T_TARG(kk+1)],y0,odeset(),xsol,UC(kk,:));
    YE(kk+1,:) = Y1(end,:);
    y0 = Y1(end,:)';
end

%
figure;
clf;
% plot histories
lbls = {'X(m)','Y(m)','V(m/s)','hdg(rad)','u_r'};
Y = [data{k}(:,2:3) data{k}(:,end-1:end) data{k}(:,8)];
for j = 1:5
    subplot(5,1,j);
    plot(data{k}(:,1),Y(:,j));
    if j < 5
        hold on;
        plot(data{k}(:,1),YE(:,j),'r--');
    end
    ylabel(lbls{j});
    grid on;
end