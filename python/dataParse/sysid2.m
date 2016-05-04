%% sysid2
% use the filtered data
% t = 560-640 target

clear variables;
close all;

data = csvread('../joystick/20160430/jerkOut.csv',1,0);

tUse = [560 630];

in = find(data > tUse(1) & data < tUse(2));
data = data(in,:);

V = sqrt(sum( data(:,4:5).^2,2));
hdg = atan2( data(:,5),data(:,4));
hdg(hdg<0) = hdg(hdg<0)+2*pi;
% make continuous heading
hdgc = hdg;
for k = 2:length(hdgc)
    while hdgc(k)-hdgc(k-1) > pi
        hdgc(k:end) = hdgc(k:end)-2*pi;
    end
    while hdgc(k)-hdgc(k-1) < -pi
        hdgc(k:end) = hdgc(k:end)+2*pi;
    end
end

% plot histories
figure(1);
clf;
lbls = {'X(m)','Y(m)','V(m/s)','hdg(rad)','u_t','u_r'};
Y = [data(:,2:3) V hdgc data(:,13:-1:12)];
for k = 1:6
    subplot(3,2,k);
    plot(data(:,1),Y(:,k));
    ylabel(lbls{k});
    grid on;
end

%% do rudder throttle correlate with heading speed derivatives?

dV = (data(:,4).*data(:,6)+data(:,5).*data(:,7))./V;%[0;diff(V)./diff(data(:,1))];
dhdg = (data(:,4).*data(:,7)-data(:,5).*data(:,6))./(V.^2);%[0;diff(hdgc)./diff(data(:,1))];

figure(2);
clf;
subplot(221);
plot(data(:,13),dV,'x');
ylabel('~dV/dt');
xlabel('u_t');
grid on;

subplot(222);
plot(data(:,12),dhdg,'x');
ylabel('~dhdg/dt');
xlabel('u_r');
grid on;

% fit data for throttle
% find u_t in 0.3,0.8
tlim = [0.3 0.8];
inut = find(data(:,13) > tlim(1) & data(:,18) < tlim(2) & abs(dV) < 10.0);
C = [data(inut,13) ones(length(inut),1)];
d = dV(inut);
xUt = lsqlin(C,d,[],[]);
yfit = xUt(1)*tlim + xUt(2);
subplot(221);
hold on;
plot(tlim,yfit,'r-','linewidth',2);

% fit data for rudder
% find u_r in -0.5,0.5
rlim = 0.25.*[-1 1];
inur = find(data(:,12) > rlim(1) & data(:,12) < rlim(2));
C = [data(inur,12) ones(length(inur),1)];
d = dhdg(inur);
xUr = lsqlin(C,d,[],[]);
yfit = xUr(1).*rlim + xUr(2);
subplot(222);
hold on;
plot(rlim,yfit,'r-','linewidth',2);

subplot(223);
plot(data(:,1),data(:,13));
hold on;
plot(data(inut,1),data(inut,13),'rd');
ylabel('u_t');
grid on;

subplot(224);
plot(data(:,1),data(:,12));
hold on;
plot(data(inur,1),data(inur,12),'rd');
ylabel('u_r');
grid on;

fprintf('Rudder fit: dot(hdg) = %g*u_r + %g\n',xUr);
fprintf('Throttle fit: dot(V) = %g*u_t + %g\n',xUt);

%% Model fit

global T_TARG Y_TARG UC EQOM;

% x = (rx ry V hdg]
eqom = @(t,y,K,u) [y(3)*[cos(y(4));sin(y(4))];K(2)*(u(2)-K(3));K(1)*y(3)*u(1)];

EQOM = eqom;
T_TARG = data(:,1);
Y_TARG = [data(:,2:3) V hdgc];
UC = data(:,12:13);

%x0 = [xUr(1)/mean(V);xUt(1);0.5];
x0 = [0.478743414912847          1.03416253003404         0.542837924376463]';
% x > 0.0 req'd: -eye*x < -0.0
lb = [1e-4;1e-4;0.0];
ub = [10.0;10.0;1.0];
xsol = fmincon(@fsolfun,x0,[],[],[],[],lb,ub,[],optimset('disp','iter'));

%% propagate
YE = zeros(size(data(:,2:5)));
y0 = Y_TARG(1,:)';
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
Y = [data(:,2:3) V hdgc data(:,13:-1:12)];
for k = 1:6
    subplot(3,2,k);
    plot(data(:,1),Y(:,k));
    if k < 5
        hold on;
        plot(data(:,1),YE(:,k),'r--');
    end
    ylabel(lbls{k});
    grid on;
end