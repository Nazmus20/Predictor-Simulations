function Eq=Equilibrium(x)
%To determine equilbrium of the system set 
% xdot=0 when evaluated at x=x* and u=u*
%
% For steady, constant altitude, wings level flight we have
% phi*=0, theta=theta*, psi=psi*, v*=0, p*=0, q*=0, r*=0, Vw*=zeros(3,1)
% 
% The set of equations becomes
% xdot= c(psi)*c(theta)*u + c(psi)*s(theta)*w;
% ydot= s(psi)*c(theta)*u + s(psi)*s(theta)*w;
% zdot= -s(theta)u + c(theta)w;
% phidot=0 ;
% thetadot= 0 ; From here we get q=0
% psidot= 0;
% udot= (1/m)*(1/2 *rho*V^2*S*Cx+PropDia^4*rho*nProp*etaProp*drps^2*CJ -m*g*s(theta);
% vdot= (1/m)*(1/2 *rho*V^2*S*Cy); This is equiv. to Cy=0;
% wdot= (1/m)*(1/2 *rho*V^2*S*Cz +m*g*cos(theta));
% pdot= Iyy/det *(Ixz(N) + Izz(L));
% qdot= (Ixx Izz -Ixz^2)/det *(M); This is equiv. to Cm=0;
% rdot= Iyy/det *( Ixx(N) + Ixz(L) );
% 
% 

global e1 e2 e3 m rho g Inertia b c S ...
    Cj2 Cj Cj0 ...
    Cxalpha Cxde Cx0 Cxq Cxalpha2...
    Czalpha Czq Cz0 Czde...
    Cmalpha Cmq Cmde Cm0 ...
    Cybeta Cyp Cyr Cyda Cydr Cy0 ...
    Clbeta Clp Clda Clr Cldr Cl0...
    Cnbeta Cnr Cnda Cndr Cnp Cn0...
    drpsEq PropDia nProp etaProp VEq deEq thetaEq

V=x(1);
theta=x(2);
de=x(3);

drps=drpsEq;
alpha=theta; 

qhat=0;

dynpres=0.5*rho*V^2;

J=V/(drps*PropDia);
drps_star=2100*2*pi/60;
Jstar=18/(drps_star*PropDia);


CJ= Cj0+ Cj*J + Cj2*J^2;
Cx = Cx0 + Cxq*qhat + Cxde*de + Cxalpha*alpha + Cxalpha2*alpha^2;
Cz = Cz0 + Czq*qhat + Czde*de + Czalpha*alpha ;
Cm = Cm0 + Cmq*qhat + Cmde*de + Cmalpha*alpha;

FX=dynpres*S*Cx- m*g*sin(theta) + PropDia^4*rho*etaProp*nProp*drps^2*CJ ; %From Mekonen. Obtain the paper

FZ=dynpres*S*Cz + m*g*cos(theta);

MM= dynpres*S*c*Cm;


Eq(1)= FX;
Eq(2)= FZ;
Eq(3)= MM;