function F = WingsLevelEquilibrium(x)

% WingsLevelEquilibrium.m
%
% Static force and moment balance for equilibrium speed, pitch, and
% elevator deflection corresponding to steady, wings level flight at
% constant altitude. (Constant power aircraft -- adjust throttle setting.)

global rho m g S Power ...
Cx0 Cxu Cxw Cxw2 ...
Cz0 Czw Czw2 Czde ...
Cm0 Cmw Cmde dTEq 

% Solving for speed, pitch, and elevator deflection 
V = x(1); % m/s
theta = x(2); % rad
de = x(3); % rad

% Nondimensional force and moment coefficients in equilibrium flight
Cx = Cx0 + Cxu*cos(theta) + Cxw*sin(theta) + Cxw2*(sin(theta))^2; 
Cz = Cz0 + Czw*sin(theta) + Czw2*(sin(theta))^2 + Czde*de;
Cm = Cm0 + Cmw*sin(theta) + Cmde*de;

% Nondimensional forces and moments balance to zero. (Find the roots.)
F(1) = Cx + (2*(dTEq*Power))/(rho*S*V^3) - (2*m*g*sin(theta))/(rho*S*V^2);
F(2) = Cz + (2*m*g*cos(theta))/(rho*S*V^2);
F(3) = Cm;
