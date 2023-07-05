function dxdt = FixedWingEOM(t,x,t_vec,de_vec)

% DoubletFixedWingEOM.m
%
% For HW 2 of AOE X224 Vehicle Model ID class 


global e1 e2 e3 rho m g S b c Inertia ...
Cx0 Cxalpha Cxalpha2 Cxde ...
Cz0 Czalpha Czde Czalpha2 ...
Cmalpha Cmq Cmde Cmalpha2 Cmalpha3 Cm0 ...
Cybeta Cyp Cyr Cyda Cydr Cybeta3 ...
Clbeta Clp Clr Clda ...
Cnbeta Cnp Cnr Cnda Cndr Cnbeta3 ...
ThrustEq thetaEq deEq VEq

% Aircraft 12 states
X = x(1:3);
Theta = x(4:6);
    phi = Theta(1);
    theta = Theta(2);
    psi = Theta(3);
V = x(7:9);
    u = V(1);
    v = V(2);
    w = V(3);    
omega = x(10:12);
    p = omega(1);
    q = omega(2);
    r = omega(3);    

%Rotation matrix and attitude kinematic equations
RIB = expm(psi*hat(e3))*expm(theta*hat(e2))*expm(phi*hat(e1));
LIB = [1, sin(phi)*tan(theta), cos(phi)*tan(theta);
    0, cos(phi), -sin(phi);
    0, sin(phi)/cos(theta), cos(phi)/cos(theta)];

%%% AERODYNAMIC FORCES AND MOMENTS %%%

% Aerodynamic angles 
beta = asin(V(2)/norm(V));
alpha = atan2(V(3),V(1));

de = interp1(t_vec,de_vec,t);
da = 0;
dr = 0;

%normalized values
phat = p*b/2/VEq;
rhat = r*b/2/VEq;
qhat = q*c/2/VEq;
PDyn = .5*rho*VEq^2; %Dynamic pressure, lb/ft^2

% Aerodynamic force coefficients
Cx = Cxalpha*alpha + Cxde*de + Cxalpha2*(alpha^2) + Cx0; 
Cz = Czalpha*alpha + Czde*de + Czalpha2*(alpha^2) + Cz0;
Cy = Cybeta*beta + Cyp*phat + Cyr*rhat + Cyda*da + Cydr*dr + Cybeta3*(beta^3); 

%Aerodynamic forces vectors
X = PDyn*S*Cx + ThrustEq;
Y = PDyn*S*Cy;
Z = PDyn*S*Cz;
Force_Aero = [X; Y; Z];

% Components of aerodynamic moment 
Cl = Clbeta*beta + Clp*phat + Clr*rhat + Clda*da;
Cm = Cmalpha*alpha + Cmq*qhat + Cmde*de + Cmalpha2*(alpha^2) + ...
    Cmalpha3*(alpha^3) + Cm0;
Cn = Cnbeta*beta + Cnp*phat + Cnr*rhat + Cnda*da + Cndr*dr + (Cnbeta3*beta^3);

%Aerodynamic moments vectors
L = PDyn*S*b*Cl;
M = PDyn*S*c*Cm;
N = PDyn*S*b*Cn;
Moment_Aero = [L; M; N];

% Sum of forces and moments
Force = RIB'*(m*g*e3) + Force_Aero;
Moment = Moment_Aero;

%%% EQUATIONS OF MOTION %%%


%%% KINEMATIC EQUATIONS %%%
dXdt = RIB*V;
dThetadt = LIB*omega;

%%% DYNAMIC EQUATIONS %%%
dVdt = (1/m)*(cross(m*V,omega) + Force);
domegadt = inv(Inertia)*(cross(Inertia*omega,omega) + Moment);
%{
%Inputs
Thrust_prev = ThrustEq; de_prev = de; da_prev = 0; dr_prev = 0;

Thrust_in = Thrust_prev - (X - PDyn*S*Cx);
de_in = de - (Cm - (Cmalpha*alpha + Cmq*qhat + Cmalpha2*(alpha^2) + ...
    Cmalpha3*(alpha^3) + Cm0)/Cmde);
da_in = da_prev - ((Cl - (Clbeta*beta + Clp*phat + Clr*rhat))/Clda);
dr_in = dr_prev - ((Cn - (Cnbeta*beta + Cnp*phat + Cnr*rhat + ...
    Cnda*da_in + (Cnbeta3*beta^3)))/Cndr);
%{
Thrust_in = (X - PDyn*S*Cx);
de_in = (Cm - (Cmalpha*alpha + Cmq*qhat + Cmalpha2*(alpha^2) + ...
    Cmalpha3*(alpha^3) + Cm0)/Cmde);
da_in = ((Cl - (Clbeta*beta + Clp*phat + Clr*rhat))/Clda);
dr_in = ((Cn - (Cnbeta*beta + Cnp*phat + Cnr*rhat + ...
    Cnda*da_in + (Cnbeta3*beta^3)))/Cndr);
%}

Thrust_prev = Thrust_in; de_prev = de_in; da_prev = da_in; dr_prev = dr_in;
d_in = [Thrust_in; de_in; da_in; dr_in];
%}
d_in = [0;0; 0; 0];
dxdt = [dXdt; dThetadt; dVdt; domegadt; d_in];
