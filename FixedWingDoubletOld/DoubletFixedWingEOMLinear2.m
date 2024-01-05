function [delta_xdt, delta_U] = DoubletFixedWingEOMLinear2(t,x,A, B, t_vec,de_vec)

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

% Aircraft 9 states
Theta = x(1:3); %They are the perturbed states from nominal states
    phi = Theta(1);
    theta = Theta(2);
    psi = Theta(3);
V = x(4:6); %They are the perturbed states from nominal states
    u = V(1);
    v = V(2);
    w = V(3);    
omega = x(7:9); %They are the perturbed states from nominal states
    p = omega(1);
    q = omega(2);
    r = omega(3);    

de = interp1(t_vec,de_vec,t); %Take the perturbed input
da = 0;
dr = 0;

%%% EQUATIONS OF MOTION %%%

delta_Theta = Theta;
delta_V = V;
delta_omega = omega;

delta_X = [delta_Theta; delta_V; delta_omega];
delta_U = [da; de-deEq; dr];
delta_xdt = A*delta_X + B*delta_U;