function [dXdt] = DoubletFixedWingEOMLinear3(t,x, t_vec, Theta, V)

% DoubletFixedWingEOMLinear.m
%
% To show that the Linear EOMs work similar to the non-linear EOMs


global e1 e2 e3 rho m g S b c Inertia ...
Cx0 Cxalpha Cxalpha2 Cxde ...
Cz0 Czalpha Czde Czalpha2 ...
Cmalpha Cmq Cmde Cmalpha2 Cmalpha3 Cm0 ...
Cybeta Cyp Cyr Cyda Cydr Cybeta3 ...
Clbeta Clp Clr Clda ...
Cnbeta Cnp Cnr Cnda Cndr Cnbeta3 ...
ThrustEq thetaEq deEq VEq

% Aircraft 12 states
phi = interp1(t_vec, Theta(:,1), t); %Interpolate at the ode45 steps
theta = interp1(t_vec, Theta(:,2), t);
psi = interp1(t_vec, Theta(:,3), t);

u = interp1(t_vec, V(:,1), t);
v = interp1(t_vec, V(:,2), t);
w = interp1(t_vec, V(:,3), t);    

%Rotation matrix and attitude kinematic equations
RIB = expm(psi*hat(e3))*expm(theta*hat(e2))*expm(phi*hat(e1));

%%% KINEMATIC EQUATIONS %%%
dXdt = RIB*[u;v;w];