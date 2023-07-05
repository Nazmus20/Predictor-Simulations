function Eq=Equilibrium(x)
%To determine equilbrium of the system set 
% xdot=0 when evaluated at x=x* and u=u*
%
% For steady, constant altitude, wings level flight we have
% phi*=0, psi=psi*, v*=0, p*=0, r*=0
% 
% The set of equations becomes
% xdot= c(theta)u -s(theta)w;
% ydot= 0 ;
% zdot= s(theta)u + c(theta)w;
% phidot=0 ;
% thetadot= q ; From here we get q=0
% psidot= 0;
% udot= (1/m)*(1/2 *rho*V^2*S*Cx+Thrust-m*g*sin(theta)) ;
% vdot= (1/m)*(1/2 *rho*V^2*S*Cy);
% wdot= (1/m)*(1/2 *rho*V^2*S*Cz +m*g*cos(theta));
% pdot= Iyy/det *(Ixz(N) + Izz(L));
% qdot= (Ixx Izz -Ixz^2)/det *(M); This is equiv. to Cm=0;
% rdot= Iyy/det *( Ixx(N) + Ixz(L) );
% 
% 

global m rho g S ...
    Cxalpha Cxde Cxalpha2 Cx0 ...
    Czalpha Czde Czalpha2 Cz0 ...
    Cmalpha Cmde Cmalpha2 Cmalpha3 Cm0 ...
    VEq


Thr=x(1);
theta=x(2);
de=x(3);

alpha=theta;

Cx=Cxalpha*alpha + Cxde*de + Cxalpha2*alpha^2 +Cx0;
Cz=Czalpha*alpha + Czde*de + Czalpha2*alpha^2 +Cz0;
Cm=Cmalpha*alpha + Cmde*de +Cmalpha2*alpha^2 ...
    + Cmalpha3*alpha^3+Cm0;

Eq(1)= Cx -2*m*g/(rho*S*VEq^2) *sin(theta) + 2*Thr/(rho*S*VEq^2);
Eq(2)= Cz + 2*m*g/(rho*S*VEq^2) *cos(theta);
Eq(3)= Cm;