clear all; clc; close all;

% FixedWingScript.m solves the nonlinear equations of motion for a 
% fixed-wing aircraft flying in ambient wind. 

% For HW 2 of AOE X224 Vehicle Model ID class 

global e1 e2 e3 rho m g S b c Inertia ...
Cx0 Cxalpha Cxalpha2 Cxde ...
Cz0 Czalpha Czde Czalpha2 ...
Cmalpha Cmq Cmde Cmalpha2 Cmalpha3 Cm0 ...
Cybeta Cyp Cyr Cyda Cydr Cybeta3 ...
Clbeta Clp Clr Clda ...
Cnbeta Cnp Cnr Cnda Cndr Cnbeta3 ...
ThrustEq thetaEq deEq VEq

% Basis vectors
e1 = [1;0;0];
e2 = [0;1;0];
e3 = [0;0;1];

% Atmospheric and gravity parameters (Constant altitude: Sea level)
rho = 1.225*0.00194032; % Density (slug/ft^3)
g = 32.174;        % Gravitational acceleration (ft/s^2)

% Aircraft parameters (MTD)
m = .211;          % Mass (slug)
W = m*g;            % Weight (lb)
Ix = 0.2163;         % Roll inertia (slug-ft^2)
Iy = .1823;         % Pitch inertia (slug-ft^2)
Iz = .3396;         % Yaw inertia (slug-ft^2)
Ixz = .0364;        % Roll/Yaw product of inertia (slug-ft^2)
Inertia = [Ix, 0, -Ixz; 0, Iy, 0; -Ixz, 0, Iz];

S = 4.92;         % Wing area (ft^2)
b = 5.91;           % Wing span (ft)
c = 0.833;          % Wing chord (ft)
AR = (b^2)/S;       % Aspect ratio

% Longitudinal nondimensional stability and control derivatives
Cxalpha = .44; % Define CT0 = 0.197 below. (Cx0 = 0.197 in [Simmons et al, 2019])
Cxde = -0.01398;
Cxalpha2 = 2.59;
Cx0 = -.061;

Czalpha = -3.947;
Czde = -21.19;
Czalpha2 = -.8207;
Cz0 = -0.206;

Cmalpha = -.8068;
Cmq = -4.937;
Cmde = -.7286;
Cmalpha2 = -1.251;
Cmalpha3 = -29.92%-7.932; %-29.92;
Cm0 = -.061;

% Lateral-directional nondimensional stability and control derivatives
Cybeta = -.3761;
Cyp = .6323;
Cyr = .1951;
Cyda = .2491;
Cydr = .1544;
Cybeta3 = -.2294;

Clbeta = -.0529;
Clp = -.6586;
Clr = .1365;
Clda = -.2729;

Cnbeta = .104;
Cnp = .0427;
Cnr = -.1511;
Cnda = .0522;
Cndr = -.0726;
Cnbeta3 = .1777;

% Solve for wings-level, constant-altitude equilibrium flight condition
VEq = 45;               % Given airspeed (ft/s)
theta = 0*(pi/180);	% Initial guess for pitch (rad)
%Solving Cm = 0 for equilibrium flight we get deEq. As there is no wind,
%alphaEq = thetaEq, and qhatEq = 0 for wings level steady flight;
de = 0; % Initial guess for elevator (rad)
Thrust = 0;%Initial guess for Thrust, lb

Roots = fsolve('EquilibriumEOM',[Thrust;theta;de]);

ThrustEq = Roots(1);
thetaEq = Roots(2);
deEq = Roots(3);
%%

% Define initial state
X0 = zeros(3,1); % (m)
Theta0 = [0;thetaEq;0]; % (rad)
V0 = [VEq*cos(thetaEq);0;VEq*sin(thetaEq)]; % (m/s)
omega0 = zeros(3,1); % (rad/s)
input_vec0 = [ThrustEq; deEq; 0; 0];

y0 = [X0; Theta0; V0; omega0; input_vec0];
t_final = 10; %seconds
t_vec = [0:0.01:t_final];

maneuver = 'None';

if strcmp('None', maneuver)
    [t,y] = ode45('FixedWingEom',t_vec',y0);
    input_vec(:,1:4) = y(:,13:16);
    
elseif strcmp('Doublet', maneuver)
    
    %Doublet start time, amplitude and duration
    tstart = 2; doubletAmp = .1*pi/180; duration = .5;
    for n=1:length(t_vec)
        if t_vec(n) >= tstart && t_vec(n) < tstart+duration 
            de_vec(n) = deEq + doubletAmp; %rad
        elseif t_vec(n) >= tstart+duration && t_vec(n) < tstart+2*duration 
            de_vec(n) = deEq - doubletAmp; %rad
        else
            de_vec(n) = deEq;
        end
    end
    
    [t,y] = ode45(@(t,y) DoubletFixedWingEom(t,y,t_vec,de_vec), t_vec, y0);
    %[t,y] = ode45('DoubletFixedWingEom',t_vec',y0);
    input_vec(:,1:4) = y(:,13:16);    
end        
%%
%%%% CODE TO COMPUTE FLOW RELATIVE VELOCITY, ETC %%%%

figure(1)
subplot(2,1,1)
plot(t,y(:,1:3))
legend('X','Y','Z')
xlabel('Time (sec)')
ylabel('Position (m)')
subplot(2,1,2)
plot(t,y(:,4:6)*(180/pi))
legend('\phi','\theta','\psi')
ylabel('Attitude (deg)')
xlabel('Time (sec)')

figure(2)
subplot(2,1,1)
plot(t,y(:,7:9))
legend('u','v','w')
xlabel('Time (sec)')
ylabel('Velocity (m/s)')
subplot(2,1,2)
plot(t,y(:,10:12)*(180/pi))
xlabel('Time (sec)')
ylabel('Angular Velocity (deg/s)')
legend('p','q','r')

figure(3)
subplot(2,1,1)
plot(t,input_vec(:,1))
xlabel('Time (sec)')
ylabel('Thrust (lb)')
subplot(2,1,2)
plot(t,input_vec(:,2)*180/pi, t,input_vec(:,3)*180/pi, t, ...
    input_vec(:,4)*180/pi)
xlabel('Time (sec)')
ylabel('Control surface deflection (deg)')
legend('de','da','dr')
