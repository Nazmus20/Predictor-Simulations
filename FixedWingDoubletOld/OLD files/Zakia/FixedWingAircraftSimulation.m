%   Fixed-wing Aircraft simulation code
%   Zakia Ahmed
%   2022

%   Aircraft model from the following paper:  
%   [1] J. L. Gresham, J. Fahmi, B. M. Simmons, J. W. Hopwood, W. Foster, and C. A.
%   Woolsey. Flight test approach for modeling and control law validation for unmanned
%   aircraft. In AIAA Scitech 2022 Forum, 2022.

clc
clear all
close all

global e1 e2 e3 m rho g Inertia b c S ...
    Cxalpha Cxde Cxalpha2 Cx0 ...
    Czalpha Czde Czalpha2 Cz0 ...
    Cmalpha Cmq Cmde Cmalpha2 Cmalpha3 Cm0 ...
    Cybeta Cyp Cyr Cyda Cydr Cybeta3 ...
    Clbeta Clp Clr Clda ...
    Cnbeta Cnp Cnr Cnda Cndr Cnbeta3 ...
    ThrustEq VEq dTEq thetaEq deEq tau1 tau0 ...

%basis vectors
e1=[1;0;0;];
e2=[0;1;0;];
e3=[0;0;1;];

%gravity and density
g=32.1740; %ft/s^2
rho=0.763/32.2; %lb/ft^3

%aircraft physical parameters
m= 0.211; %slug
c= 0.833; %ft
b= 5.91; %ft
S=4.92; %ft^2
Ixx=0.2163; %slug-ft^2
Iyy=0.1823; %slug-ft^2
Izz=0.3396; %slug-ft^2
Ixz=0.0364; %slug-ft^2
Ixy=0;
Iyz=0;
Inertia=[Ixx Ixy -Ixz; Ixy Iyy Iyz; -Ixz Iyz Izz;];
%% Parameter estimates

%Longitudinal parameter estimates 
Cxalpha= 0.440;
Cxde= -0.01398;
Cxalpha2= 2.59;
Cx0= -0.061;

Czalpha= -3.947;
Czde= -21.19;
Czalpha2= -0.8207;
Cz0= -0.206;

Cmalpha= -0.8068;
Cmq= -4.937;
Cmde= -0.7286;
Cmalpha2= -1.251;
Cmalpha3= -7.932; %-29.920;
Cm0= -0.061;

%laterial-directional parameter estimates
Cybeta= -0.3761;
Cyp= 0.6323;
Cyr= 0.1951;
Cyda= 0.2491;
Cydr= 0.1544;
Cybeta3= -0.2294;

Clbeta= -0.0529;
Clp= -0.6586;
Clr= 0.1365;
Clda= -0.2729;

Cnbeta= 0.1040;
Cnp= 0.0427;
Cnr= -0.1511;
Cnda= 0.0522;
Cndr= -0.0726;
Cnbeta3= 0.1777;

%% Determining equilbrium

%Finding an Equilbrium condition
VEq=45;
thetaEq=5*pi/180 %0.25 *(pi/180);
deEq=-(Cmalpha*thetaEq +Cmalpha2*thetaEq^2 + Cmalpha3*thetaEq^3+Cm0)/Cmde;
Thrust=320;

EqSol=fsolve('Equilibrium',[Thrust;thetaEq;deEq]);
dTEq=EqSol(1)
thetaEq=EqSol(2)
deEq=EqSol(3)

%Initial Conditions
x0=[0;0;0;];
Theta0=[0;thetaEq;0;];
v0=[VEq*cos(thetaEq);0;VEq*sin(thetaEq);];
omega0=[0;0;0;];
IC=[x0;Theta0;v0;omega0;];

tf=100;
tvec=[0:0.1:tf]';

%% ODE solver and plotting

Manuever= 'none';

if strcmp('none',Manuever)
    %no doublet
    [t,x]=ode45(@(t,x) FixedWingEOM(t,x),tvec,IC);

    figure(1)
    da=0;
    dr=0;
    subplot(2,1,1)
    plot(t, da*ones(length(t),1)*180/pi, t, deEq*ones(length(t),1)*180/pi, t, dr*ones(length(t),1)*180/pi)
    legend('\delta_a', '\delta_e', '\delta_r')
    xlabel('time, seconds')
    ylabel('Control Inputs (deg)')
    ylim([-2 deEq*180/pi+2])
    subplot(2,1,2)
    plot(t, dTEq*ones(length(t),1))
    xlabel('time, seconds')
    ylabel('\delta T, (lbs)')
    ylim([0 dTEq+25])
    
elseif strcmp('doublet',Manuever)
    tstart=2.5;
    duration=0.5;
    doubletamp=0.25*pi/180;
    [usw,tdoublet]=mksqw(doubletamp,duration,[1 1],tstart,0.05,tf);
    devec=deEq*ones(length(usw),1)+usw;
    
    [t,x]=ode45(@(t,x) doubletFixedWingEOM(t,x,tdoublet, devec),tvec,IC); %doublet


    figure(1)
    da=0;
    dr=0;
    subplot(2,1,1)
    plot(t, da*ones(length(t),1), tdoublet, devec*180/pi, t, dr*ones(length(t),1))
    legend('\delta_a', '\delta_e', '\delta_r')
    xlabel('time, seconds')
    ylabel('Control Inputs (deg)')
    maxang=max(devec*180/pi);
    minang=min(devec*180/pi);
    ylim([minang-2 maxang+2])
    subplot(2,1,2)
    plot(t, dTEq*ones(length(t),1))
    xlabel('time, seconds')
    ylabel('\delta T, (lbs)')
    ylim([0 dTEq+25])
    
elseif strcmp('doubletdelay',Manuever)
    tstart=2.5;
    duration=0.5;
    doubletamp=0.25*pi/180;
    [usw,tdoublet]=mksqw(doubletamp,duration,[1 1],tstart,0.05,tf);
    devec=deEq*ones(length(usw),1)+usw;
    tau0=1.5;
    tau1=1;
    
    
    [t,x]=ode45(@(t,x) actuatordelaydoubletFixedWingEOM(t,x,tdoublet, devec),tvec,IC); %doublet
    deplot=zeros(length(t),1);
    for n=1:length(t)
    deplot(n)=interp1(tdoublet,devec,t(n))*heaviside(t(n)-tau0)*(1-exp(-1/tau1 *(t(n)-tau0)));
    end
    
    figure(1)
    da=0;
    dr=0;
    subplot(2,1,1)
    plot(t, da*ones(length(t),1), t, deplot*180/pi, tdoublet, devec*180/pi, t, dr*ones(length(t),1))
    legend('\delta_a', '\delta_e (actual)','\delta_e (commanded)', '\delta_r')
    xlabel('time, seconds')
    ylabel('Control Inputs (deg)')
    maxang=max(devec*180/pi);
    minang=min(devec*180/pi);
    ylim([minang-2 maxang+2])
    subplot(2,1,2)
    plot(t, dTEq*ones(length(t),1))
    xlabel('time, seconds')
    ylabel('\delta T, (lbs)')
    ylim([0 dTEq+25])

end


figure(2)
subplot(2,1,1)
plot(t, x(:,1), t, x(:,2), t, x(:,3))
legend('x','y','z')
xlabel('time, seconds')
ylabel('Position, ft')

subplot(2,1,2)
plot(t, x(:,4)*180/pi, t, x(:,5)*180/pi, t, x(:,6)*180/pi)
legend('\phi','\theta','\psi')
xlabel('time, seconds')
ylabel('Attitude, deg')
minth=min(min([x(:,4)*180/pi x(:,5)*180/pi x(:,6)*180/pi]));
maxth=max(max([x(:,4)*180/pi x(:,5)*180/pi x(:,6)*180/pi]));
ylim([minth-2 maxth+2])

figure(3)
subplot(2,1,1)
plot(t, x(:,7), t, x(:,8), t, x(:,9))
legend('u','v','w')
xlabel('time, seconds')
ylabel('Velocity, ft/s')
minv=min(min([x(:,7) x(:,8) x(:,9)]));
maxv=max(max([x(:,7) x(:,8) x(:,9)]));
ylim([minv-5 maxv+5])

subplot(2,1,2)
plot(t, x(:,10)*180/pi, t, x(:,11)*180/pi, t, x(:,12)*180/pi)
legend('p','q','r')
xlabel('time, seconds')
ylabel('Angular Velocity, deg/s')
minomega=min(min([x(:,10)*180/pi x(:,11)*180/pi x(:,12)*180/pi]));
maxomega=max(max([x(:,10)*180/pi x(:,11)*180/pi x(:,12)*180/pi]));
ylim([minomega-2 maxomega+2])


