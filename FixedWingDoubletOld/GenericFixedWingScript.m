clear
close all

% GenericFixedWingScript.m solves the nonlinear equations of motion for a 
% fixed-wing aircraft flying in ambient wind with turbulence. 

global e1 e2 e3 rho m g S b c Inertia Power ...
Cx0 Cxu Cxw Cxw2 ...
Cz0 Czw Czw2 Czq Czde ...
Cm0 Cmw Cmq Cmde ...
Cy0 Cyv Cyp Cyr Cyda Cydr ...
Cl0 Clv Clp Clr Clda Cldr ...
Cn0 Cnv Cnv2 Cnp Cnr Cnda Cndr ...
dTEq deEq ...
VonKarmanFlag Amp Phase Omega Phase2 Omega2 

% Basis vectors
e1 = [1;0;0];
e2 = [0;1;0];
e3 = [0;0;1];

% Atmospheric and gravity parameters (Constant altitude: Sea level)
a = 340.3;          % Speed of sound (m/s)
rho = 1.225;        % Density (kg/m^3)
g = 9.80665;        % Gravitational acceleration (m/s^2)

% Aircraft parameters (Bix3)
m = 1.202;          % Mass (kg)
W = m*g;            % Weight (Newtons)
Ix = 0.095;         % Roll inertia (kg-m^2)
Iy = 215e3;         % Pitch inertia (kg-m^2)
Iz = 447e3;         % Yaw inertia (kg-m^2)
Ixz = 0;            % Roll/Yaw product of inertia (kg-m^2)
Inertia = [Ix, 0, -Ixz; 0, Iy, 0; -Ixz, 0, Iz];

S = 0.285;         % Wing area (m^2)
b = 1.54;           % Wing span (m)
c = 0.188;          % Wing chord (m)
AR = (b^2)/S;       % Aspect ratio

% Longitudinal nondimensional stability and control derivatives
Cx0 = 0; % Define CT0 = 0.197 below. (Cx0 = 0.197 in [Simmons et al, 2019])
Cxu = -0.156;
Cxw = 0.297;
Cxw2 = 0.960;

Cz0 = -0.179;
Czw = -5.32;
Czw2 = 7.02;
Czq = -8.20;
Czde = -0.308;

Cm0 = 0.0134;
Cmw = -0.240;
Cmq = -4.49;
Cmde = -0.364;

% Lateral-directional nondimensional stability and control derivatives
Cy0 = 0;
Cyv = -0.251;
Cyp = 0.170;
Cyr = 0.350;
Cyda = 0.103;
Cydr = 0.0157;

Cl0 = 0;
Clv = -0.0756;
Clp = -0.319;
Clr = 0.183;
Clda = 0.170;
Cldr = -0.0117;

Cn0 = 0;
Cnv = 0.0408;
Cnv2 = 0.0126;
Cnp = -0.242;
Cnr = -0.166;
Cnda = 0.0416;
Cndr = -0.0618;

% Solve for wings-level, constant-altitude equilibrium flight condition
VEq = 12;               % Initial guess for speed (m/s)
thetaEq = 2*(pi/180);	% Initial guess for pitch (rad)
deEq = -(Cm0 + Cmw*sin(thetaEq))/Cmde; % Initial guess for elevator (rad)
Power = 200;            % Maximum Power (W)
CT0 = 0.197;            % CT0 = Cx0 in [Simmons et al, 2019]
dTEq = (CT0*rho*S*(VEq^3))/(2*Power); % Nominal throttle setting

Roots = fsolve('WingsLevelEquilibrium',[VEq;thetaEq;deEq]);
VEq = Roots(1);
thetaEq = Roots(2);
deEq = Roots(3);

%{
% Generate amplitude, phase, and frequency parameters for 1D or 2D von Karman turbulence
VonKarmanFlag = 1

% 1D von Karman turbulence
if VonKarmanFlag == 1

[Amp,Phase,Omega] = OneDVonKarmanTurbulence(-4,0,100);
Omega = Omega/0.3048; % (rad/m)
disp('One Dimensional Turbulence Spectrum')

% 2D von Karman turbulence
elseif VonKarmanFlag == 2

[Amp,Phase,Phase2,Omega,Omega2] = TwoDVonKarmanTurbulence(-4,0,100);
Omega = Omega/0.3048; % (rad/m)
Omega2 = Omega2/0.3048; % (rad/m)
disp('Two Dimensional Turbulence Spectrum')

end

Amp = Amp*0.3048; % (m/s)
% Amp = 0*Amp; % Zero out the turbulence.
%}
% Define initial state
X0 = zeros(3,1); % (m)
Theta0 = [0;thetaEq;0]; % (rad)
V0 = [VEq*cos(thetaEq);0;VEq*sin(thetaEq)]; % (m/s)
omega0 = zeros(3,1); % (rad/s)

y0 = [X0; Theta0; V0; omega0];
t_final = 30;
[t,y] = ode45('GenericFixedWingEOM',[0:0.1:t_final]',y0);

%%%% CODE TO COMPUTE FLOW RELATIVE VELOCITY, ETC %%%%

figure(1)
subplot(2,1,1)
plot(t,y(:,1:3))
legend('X','Y','Z')
ylabel('Position (m)')
subplot(2,1,2)
plot(t,y(:,4:6)*(180/pi))
legend('\phi','\theta','\psi')
ylabel('Attitude (deg)')

figure(2)
subplot(2,1,1)
plot(t,y(:,7:9))
legend('u','v','w')
ylabel('Velocity (m/s)')
subplot(2,1,2)
plot(t,y(:,10:12)*(180/pi))
ylabel('Angular Velocity (deg/s)')
legend('p','q','r')
