function [dxdt, delta_U] = FixedWingEOMLinear(t,x,A, B, t_vec,de_vec, Ts)

% FixedWingEOMLinear.m
%
% To show that the Linear EOMs work similar to the non-linear EOMs


global e1 e2 e3 m rho g Inertia b c S ...
    Cj2 Cj Cj0 ...
    Cxalpha Cxde Cx0 Cxq Cxalpha2...
    Czalpha Czq Cz0 Czde...
    Cmalpha Cmq Cmde Cm0 ...
    Cybeta Cyp Cyr Cyda Cydr Cy0 ...
    Clbeta Clp Clda Clr Cldr Cl0...
    Cnbeta Cnr Cnda Cndr Cnp Cn0...
    drpsEq PropDia nProp etaProp VEq deEq thetaEq

% Aircraft 12 states
X = x(1:3); %They are the actual states (not deltas from nominal states)
Theta = x(4:6); %They are the perturbed states from nominal states
    phi = Theta(1);
    theta = Theta(2);
    psi = Theta(3);
V = x(7:9); %They are the perturbed states from nominal states
    u = V(1);
    v = V(2);
    w = V(3);    
omega = x(10:12); %They are the perturbed states from nominal states
    p = omega(1);
    q = omega(2);
    r = omega(3);    

%Rotation matrix and attitude kinematic equations
RIB = expm(psi*hat(e3))*expm(theta*hat(e2))*expm(phi*hat(e1));

%Control Inputs
if t>=t_vec(1)+Ts
    de=interp1(t_vec+Ts,de_vec,t); %Take the perturbed input, use zero order hold. Add one sampling time delay due to discretization
else
    de = de_vec(1);
end
da = 0;
dr = 0;

%%% EQUATIONS OF MOTION %%%

%Obtained by solving Vss^2 = uss^2+vss^2+wss^2 and alpha_ss = arctan(wss/uss)
uss = VEq*cos(thetaEq); wss = VEq*sin(thetaEq); vss = 0;

phiss = 0; thetass = thetaEq; psiss = 0; pss = 0; qss = 0; rss = 0;

%%% KINEMATIC EQUATIONS %%%
dXdt = RIB*V;

delta_Theta = Theta - [phiss; thetass; psiss];
delta_V = V - [uss;vss;wss];
delta_omega = omega - [pss;qss;rss];

delta_X = [delta_Theta; delta_V; delta_omega];
delta_U = [da; de-deEq; dr];
delta_xdt = A*delta_X + B*delta_U;

X_dotdt = delta_xdt + [phiss; thetass; psiss; uss; vss; wss; pss; qss; rss];

dxdt = [dXdt; delta_xdt];
        