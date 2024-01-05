function [xdot] = LinearMultirotorTrajectoryDelay(t, x, A, AD, B, BD, Klqr, XD, UD, XDdot, ...
    t_vec, in_delay)

idx = find(t>=t_vec,1, 'last');
xD = XD(:,idx);
uD = UD(:,idx);
xdotD = XDdot(:,idx);

x=x(1:12);

if t>in_delay
xdot = (A-B*Klqr)*x + B*Klqr*xD + B*uD;
else 
    xdot = zeros(12,12)*x;
end

end
