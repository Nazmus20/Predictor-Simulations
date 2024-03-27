clc; clear all
syms vx vy vz X Y Z p q r del_e del_r del_a del_rps phi theta psi real

%Physical Parameters
m= 3.311;
Ixx= 0.319;
Iyy= 0.267;
Izz= 0.471;
Ixz= 0.024;
b=1.8;
c=0.254;
S=0.4571;
rho=1.237;
g=9.81;
eta_e=0.9;
eta_n=2;
D=0.254;

Inertia=[Ixx 0 -Ixz; 0 Iyy 0; -Ixz 0 Izz;];

e1=[1;0;0;];
e2=[0;1;0;];
e3=[0;0;1;];

%Rotation matrix
RIB=expm(psi*hat(e3))*expm(theta*hat(e2))*expm(phi*hat(e1));

%L_IB
LIB=[1, sin(phi)*tan(theta), cos(phi)*tan(theta);
    0, cos(phi), -sin(phi);
    0, sin(phi)*sec(theta), cos(phi)*sec(theta)];

%states
vr=[vx;vy;vz;];
omega=[p;q;r];

%Aerodynamic Force and Moment Models
%define non-dimensional terms
Vt=sqrt(vx^2 + vy^2 + vz^2);
alpha=atan(vz/vx);
beta=asin(vy/Vt);
pHat=p*b/2/Vt; 
qHat=q*c/2/Vt;
rHat=r*b/2/Vt;
Veq=18.1608; 
del_rps=220;
J=Vt/(del_rps*D);

%define parameters
%Longitudinal parameter estimates 
Cj2= -0.13096;
Cj= -0.04005;
Cj0= 0.115918;

% Cx: Aerodynamic coefficients
Cx0 = 0.009-0.4374; % + 0.026;the -0.4374 is the thrust nominal contribution
Cxde = +0.051; % + 0.025;
Cxalpha = +0.282 + 0.000;
Cxalpha2 = +3.173 + 0.119;

% Cz: Aerodynamic coefficients
Cz0 = -0.255; % - 0.074;
Czq = -12.54; % - 0.030;
Czde = 0; % + 0.000;
Czalpha = -4.436 - 0.015;

% Cm: Aerodynamic coefficients
Cm0 = +0.008; % + 0.006;
Cmq = -14.019; % - 3.363;
Cmde = -0.415; % - 0.058;
Cmalpha = -0.444 - 0.027;


%laterial-directional parameter estimates
% Cy: Aerodynamic coefficients
Cyp = +0.221; % + 0.133;
Cyr = +0.230; % + 0.547;
Cyda = +0.118; % + 0.011;
Cydr = +0.136; % + 0.030;
Cybeta = -0.410;

% Cl: Aerodynamic coefficients
Clp = -0.386; % - 0.041;
Clda = -0.137; % - 0.010;
Clbeta = -0.035 - 0.004;

% Cn: Aerodynamic coefficients
Cnr = -0.119; % - 0.029;
Cnda = +0.013; % + 0.009;
Cndr = -0.068; % - 0.006;
Cnbeta = +0.083 + 0.020;

%Define models
CJ= Cj0 + Cj*J + Cj2*J^2; 
Cx= Cx0+ Cxde*del_e + Cxalpha*alpha + Cxalpha2*alpha^2;
Cy= Cyp*pHat + Cyr*rHat + Cyda*del_a + Cydr*del_r + Cybeta*beta;
Cz= Cz0 + Czq*qHat + Czalpha*alpha;
Cl= Clp*pHat + Clda*del_a + Clbeta*beta;
Cm= Cm0 + Cmq*qHat + Cmde*del_e + Cmalpha*alpha;
Cn= Cnr*rHat + Cnda*del_a + Cndr*del_r + Cnbeta*beta;

FA= 1/2*rho*Vt^2*S*[Cx; Cy; Cz] + D^4*rho*eta_e*eta_n*del_rps^2*[CJ;0;0];
MA= 1/2*rho*Vt^2*S*[b*Cl; c*Cm; b*Cn];

Vi=RIB*vr;
Thetadot= LIB*omega;
vrdot= cross(vr,omega) + RIB'*[0;0;g] + FA/m;
omegadot= inv(Inertia)*cross(Inertia*omega, omega)+inv(Inertia)*MA;

state=[X,Y,Z,phi, theta, psi, vx, vy, vz, p,q,r];
vars=[X,Y,Z,phi, theta, psi, vx, vy, vz, p, q, r, del_a, del_e, del_r, del_rps];
newvars=str2sym({'x(1)', 'x(2)', 'x(3)', 'x(4)', 'x(5)', 'x(6)', 'x(7)', 'x(8)', 'x(9)', 'x(10)', 'x(11)', 'x(12)', 'u(1)', 'u(2)', 'u(3)','u(4)'});

f= vpa(simplify([Vi;Thetadot;vrdot;omegadot]),5)
Fsmall=vpa(simplify(jacobian([Thetadot;vrdot;omegadot],state(4:12))),5)
F=[vpa(simplify(jacobian([f],state)),5)];

h=[X;Y;Z;phi; theta; psi; vr; p;q;r;];
H=[jacobian([h],state)];



f=vpa(subs(f,vars, newvars),5)
F=vpa(subs(F,vars, newvars),5)
h=subs(h,vars, newvars)
H=subs(H,vars, newvars)

