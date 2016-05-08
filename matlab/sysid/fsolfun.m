function [f] = fsolfun(x)
% x(1) = heading rudder derivative
% x(2) = speed rudder derivative
% u(1) = rudder command

global T_TARG Y_TARG UC EQOM;

f = 0.0;
y0 = Y_TARG(1,:)';
for k = 1:length(T_TARG)-1
    [~,Y1] = ode45(EQOM,[T_TARG(k) T_TARG(k+1)],y0,odeset(),x,UC(k,:));
    err = Y1(end,:)-Y_TARG(k+1,:);
    err(4) = minAngleErr(Y1(end,4),Y_TARG(k+1,4));
    f = f + sum(err(4).^2);
    y0 = Y1(end,:)';
end

end