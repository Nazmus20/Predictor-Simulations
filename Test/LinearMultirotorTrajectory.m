function [xdot] = LinearMultirotorTrajectory(t, x, A, AD, B, BD, Klqr, XD, UD, XDdot, ...
    t_vec, in_delay)

idx = find(t>=t_vec,1, 'last');
xD = XD(:,idx);
uD = UD(:,idx);
xdotD = XDdot(:,idx);

x=x(1:12);
if t<in_delay
    xdot=zeros(12,12)*x;
else
    xdot = (AD-BD*Klqr)*x + BD*Klqr*xD+  BD*uD;
end

end

