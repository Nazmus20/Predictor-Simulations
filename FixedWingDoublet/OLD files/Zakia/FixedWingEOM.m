function dxdt=FixedWingEOM(t,x)
%nonlinear EOMs for Fixed Wing Aircraft
% State vector:
% x=[x;y;z;
%    phi;theta;psi;
%    u;v;w;
%    p;q;r;];
global e1 e2 e3 m rho g Inertia b c S ...
    Cxalpha Cxde Cxalpha2 Cx0 ...
    Czalpha Czde Czalpha2 Cz0 ...
    Cmalpha Cmq Cmde Cmalpha2 Cmalpha3 Cm0 ...
    Cybeta Cyp Cyr Cyda Cydr Cybeta3 ...
    Clbeta Clp Clr Clda ...
    Cnbeta Cnp Cnr Cnda Cndr Cnbeta3 ...
     dTEq deEq

%Defining states
X=x(1:3);
Theta=x(4:6);
    phi=Theta(1);
    theta=Theta(2);
    psi=Theta(3);
V=x(7:9);
    u=V(1);
    v=V(2);
    w=V(3);
omega=x(10:12);
    p=omega(1);
    q=omega(2);
    r=omega(3);

RotMat=expm(psi*hat(e3))*expm(theta*hat(e2))*expm(phi*hat(e1));
LMat=[1 sin(phi)*tan(theta) cos(phi)*tan(theta);
    0 cos(phi) -sin(phi);
    0 sin(phi)*sec(theta) cos(phi)*sec(theta);];


%Control Inputs
da=0; %aileron
de=deEq;
dr=0; %rudder
dT=dTEq; %thrust

%Air speed, alpha, beta
Vt=sqrt(u^2 + v^2 + w^2);
alpha=atan(w/u);
beta=asin(v/Vt);

phat= p*b/(2*Vt);
qhat= q*c/(2*Vt);
rhat=r*b/(2*Vt);

%Aerodynamic Forces
Cx=Cxalpha*alpha + Cxde*de + Cxalpha2*alpha^2 +Cx0;
Cy=Cybeta*beta + Cyp*phat + Cyr*rhat + Cyda*da + Cydr*dr + Cybeta3*beta^3; 
Cz=Czalpha*alpha + Czde*de + Czalpha2*alpha^2 +Cz0;

dynpres=(1/2)*rho*Vt^2;
X=dynpres*S*Cx +dT;
Y=dynpres*S*Cy;
Z=dynpres*S*Cz;

Gravity=m*g*[-sin(theta); cos(theta)*sin(phi); cos(theta)*cos(phi)];

Force=[X;Y;Z] + Gravity;

%Aerodynamic Moments
Cl= Clbeta*beta + Clp*phat + Clr*rhat + Clda*da; 
Cm=Cmalpha*alpha + Cmq*qhat +Cmde*de +Cmalpha2*alpha^2 ...
    + Cmalpha3*alpha^3+Cm0;
Cn=Cnbeta*beta + Cnp*phat + Cnr*rhat + Cnda*da + Cndr*dr ...
    +Cnbeta3*beta^3;

L=dynpres*S*b*Cl;
M=dynpres*S*c*Cm;
N=dynpres*S*b*Cn; 

Moment=[L;M;N;];

%Equations of Motion

%Kinematic Equations
Xdot=RotMat*V;
Thetadot=LMat*omega;

%Dynamic Equations
Vdot=1/m*(Force + cross(m*V,omega));
omegadot=Inertia\(Moment+ cross(Inertia*omega, omega));


dxdt=[Xdot; Thetadot; Vdot; omegadot;];
