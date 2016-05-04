function [f] = fsolfun(x)
% x(1) = gain on heading
% x(2) = gain on velocity
% x(3) = velocity trim?

global T_TARG Y_TARG UC EQOM;

f = 0.0;
y0 = Y_TARG(1,:)';
for k = 1:length(T_TARG)-1
    [~,Y1] = ode45(EQOM,[T_TARG(k) T_TARG(k+1)],y0,odeset(),x,UC(k,:));
    err = Y1(end,:)-Y_TARG(k+1,:);
    f = f + sum(err(1:4).^2);
    y0 = Y1(end,:)';
end

end