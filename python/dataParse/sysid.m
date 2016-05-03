%% sysid
% t = 560-640 target

clear variables;
close all;

control = csvread('../joystick/20160430/controlLog.csv',1,0);
controlOut = csvread('../joystick/20160430/controlOutLog.csv',1,0);
gps = csvread('../joystick/20160430/gpsLog.csv',1,0);

t0 = controlOut(1);
gps(:,1) = gps(:,1) - t0;
controlOut(:,1) = controlOut(:,1)-t0;
control(:,1) = control(:,1)-t0;
% scale to target time
gi = find(gps(:,1) <=640 & gps(:,1) >=560);
ci = find(control(:,1) <=640 & control(:,1) >=560);
gps = gps(gi,:);
control = control(ci,:);
% rescale GPS to doubles
gps(:,3) = gps(:,3)*1.0e-7;
gps(:,4) = gps(:,4)*1.0e-7;
% fix GPS heading
gps(:,6) = gps(:,6).*(1.0/0.0174533);
% home GPS
gpsHome = gps(1,3:4);
% GPS XY
gpsXY = [(gps(:,4)-gpsHome(2)) (gps(:,3)-gpsHome(1))]*111318.845;

state = [gps(:,1) gpsXY gps(:,5:6)];
statelast = state(1,:);
controlGps = zeros(length(gps),2);
controlGps(1,:) = control(1,2:3);
% parse out which GPS to accept
rej = [];
for k = 2:length(gps)
    f = acceptGps(state(k,:),statelast);
    if f 
        statelast = state(k,:);
    else
        rej = [rej;k];
    end
    in = find(control(:,1) <= gps(k,1),1,'last');
    controlGps(k,:) = control(in,2:3);
end
fprintf('Reject %d gps\n',length(rej));
acc = setdiff(1:length(gps),rej);

% plot histories
lbls = {'X(m)','Y(m)','V(m/s)','hdg(rad)','u_t','u_r'};
for k = 1:6
    subplot(3,2,k);
    if k < 5
        plot(state(acc,1),state(acc,k+1));
    else
        plot(state(acc,1),controlGps(acc,k-4));
    end
    ylabel(lbls{k});
    grid on;
end

%% Model fit

global T_TARG Y_TARG UC EQOM;

eqom = @(t,y,K,u) [y(3)*[cos(y(4));sin(y(4))];K(2)*(u(2)-K(3));K(1)*y(3)*u(1)];

EQOM = eqom;
T_TARG = state(acc,1);
Y_TARG = state(acc,2:5);
UC = controlGps(acc,:);

x0 = [0.1;0.1;0.0;0.0;0.0;0.0;0.0];
% x > 0.0 req'd: -eye*x < -0.0
lb = [1e-4;1e-4;0.0;-10;-10;-10;-10];
ub = [10.0;10.0;1.0;10;10;10;10];
xsol = fmincon(@fsolfun,x0,[],[],[],[],lb,ub,[],optimset('disp','iter'));

%% propagate
YE = zeros(size(state(acc,2:5)));
y0 = Y_TARG(1,:)'-xsol(4:7);
YE(1,:) = y0';
for k = 1:length(YE)-1
    [~,Y1] = ode45(eqom,[T_TARG(k) T_TARG(k+1)],y0,odeset(),xsol,UC(k,:));
    YE(k+1,:) = Y1(end,:);
    y0 = Y1(end,:)';
end
%
figure;
clf;
% plot histories
lbls = {'X(m)','Y(m)','V(m/s)','hdg(rad)','u_t','u_r'};
for k = 1:6
    subplot(3,2,k);
    if k < 5
        plot(state(acc,1),state(acc,k+1));
        hold on;
        plot(state(acc,1),YE(:,k),'r--');
    else
        plot(state(acc,1),controlGps(acc,3-(k-4)));
    end
    ylabel(lbls{k});
    grid on;
end