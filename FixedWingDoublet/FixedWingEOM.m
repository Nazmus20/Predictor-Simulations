function dxdt=FixedWingEOM(t,x,tmaneuver,devec, Ts)
%Nonlinear EOMs for Fixed Wing Aircraft
% State vector:
% x=[x;y;z;
%    phi;theta;psi;
%    u;v;w;
%    p;q;r];
global e1 e2 e3 m rho g Inertia b c S ...
    Cj2 Cj Cj0 ...
    Cxalpha Cxde Cx0 Cxq Cxalpha2...
    Czalpha Czq Cz0 Czde...
    Cmalpha Cmq Cmde Cm0 ...
    Cybeta Cyp Cyr Cyda Cydr Cy0 ...
    Clbeta Clp Clda Clr Cldr Cl0...
    Cnbeta Cnr Cnda Cndr Cnp Cn0...
    drpsEq PropDia nProp etaProp VEq deEq thetaEq

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
if t>=tmaneuver(1)+Ts
    de=interp1(tmaneuver+Ts,devec,t); %Take the perturbed input, use zero order hold. Add one sampling time delay due to discretization
else
    de = devec(1);
end
da=0; % elevator
dr=0; %interp1(tmaneuver,devec,t); % rudder
drps=drpsEq; % rps

% Air speed, alpha, beta, etc
Vt=sqrt(u^2 + v^2 + w^2);
alpha=atan(w/u);
beta=asin(v/Vt);

phat= p*b/(2*Vt);
qhat= q*c/(2*Vt);
rhat= r*b/(2*Vt);

J=Vt/(drps*PropDia);

%Thrust model
CJ = Cj0 + Cj*(J) + Cj2*(J)^2;
     
%Aerodynamic Forces
Cx = Cx0 + Cxq*qhat + Cxde*de + Cxalpha*alpha + Cxalpha2*alpha^2;
Cy = Cy0 + Cyp*phat + Cyr*rhat + Cyda*da + Cybeta*beta + Cydr*dr; 
Cz = Cz0 + Czq*qhat + Czde*de + Czalpha*alpha ;

dynpres=(1/2)*rho*Vt^2;
X = dynpres*S*Cx +PropDia^4*rho*etaProp*nProp*drps^2*CJ;
Y = dynpres*S*Cy;
Z = dynpres*S*Cz;

%Aerodynamic Moments
Cl = Cl0 + Clp*phat + Clr*rhat + Clda*da + Cldr*dr + Clbeta*beta; 
Cm = Cm0 + Cmq*qhat + Cmde*de + Cmalpha*alpha;
Cn = Cn0 + Cnp*phat +Cnr*rhat + Cnda*da + Cndr*dr + Cnbeta*beta ;

Gravity=RotMat'*[0; 0; m*g];

Force=[X;Y;Z;] + Gravity;

L=dynpres*S*b*Cl;
M=dynpres*S*c*Cm;
N=dynpres*S*b*Cn; 

Moment=[L;M;N;];

% Equations of Motion

% Kinematic Equations
Xdot=RotMat*V;
Thetadot=LMat*omega;

% Dynamic Equations
Vdot=1/m*(Force) + cross(V,omega);
omegadot=Inertia\(Moment+ cross(Inertia*omega, omega));


dxdt=[Xdot; Thetadot; Vdot; omegadot;];
