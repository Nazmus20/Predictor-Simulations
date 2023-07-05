function [Amp,Phase1,Phase2,Omega1,Omega2] = TwoDVonKarmanTurbulence(OmegaMinExp,OmegaMaxExp,NFreqs)

% TwoDVonKarmanTurbulence.m
%
% 2-dimensional von Karman turbulence. The function produces velocity 
% amplitudes (in US Customary Units) and random phase shifts corresponding 
% to spatial frequencies determined from the input parameters. For simplicity, 
% we assume the same spatial frequencies in both directions: Omega1 = Omega2
% Example: OmegaMinExp = -4; OmegaMaxExp = 0; 
%
% [Amp,Phase,Omega1,Omega2] = TwoDVonKarmanTurbulence(OmegaMinExp,OmegaMaxExp,NFreqs)
% For i in {x,y,z}:
% W_i = Sum_j Amp_i(j)*cos(Omega1(j)*x + Omega2(j)*y + Phase_i(j)) (ft/s)
% Grad(W) = [dW_x/dx, dW_x/dy, 0; dW_y/dx, dW_y/dy, 0; dW_z/dx, dW_z/dy, 0]; ((ft/s)/ft)

% Define (spatial) frequency range of interest
Omegamin = 10^OmegaMinExp; % rad/ft
Omegamax = 10^OmegaMaxExp; % rad/ft
Omega = logspace(OmegaMinExp,OmegaMaxExp,1000)';

% Construct and plot von Karman energy distribution for Clear Air Turbulence
a = 1.339; % Empirical parameter
sigma = 10; % Square root of turbulent kinetic energy
L = 500; % Relevant turbulent length scale (ft)

% Construct a uniform distribution of N-1 frequencies that define N frequency bins.
% Select and order N-1 random frequencies between omegamin and omegamax:
Omegainterior_exp = OmegaMinExp + (OmegaMaxExp - OmegaMinExp)*rand(NFreqs-1,1);
Omegainterior_exp = sort(Omegainterior_exp);
Omegainterior = 10.^Omegainterior_exp;

% Select N random frequencies as the center frequencies for each bin.
clear Omega
dOmega = [Omegainterior;Omegamax]-[Omegamin;Omegainterior];
Omega1 = [Omegamin;Omegainterior] + dOmega/2;
Omega2 = Omega1;

% Choose an amplitude at each random frequency according to some
% spectrum function containing the frequency range of interest.

% Along-track 
Psi11 = ((sigma^2)/(6*pi))*((a*L)^2)*(1+((a*L*Omega1).^2)+(11/3)*((a*L*Omega2).^2))./((1+((a*L).^2)*(Omega1.^2+Omega2.^2)).^(7/3));
Amp11 = sqrt(2*Psi11.*dOmega);

% Lateral
Psi22 = ((sigma^2)/(6*pi))*((a*L)^2)*(1+(11/3)*((a*L*Omega1).^2)+((a*L*Omega2).^2))./((1+((a*L).^2)*(Omega1.^2+Omega2.^2)).^(7/3));
Amp22 = sqrt(2*Psi22.*dOmega);

% Vertical
Psi33 = (4*(sigma^2)/(9*pi))*((a*L)^4)*(Omega1.^2+Omega2.^2)./((1+((a*L).^2)*(Omega1.^2+Omega2.^2)).^(7/3));
Amp33 = sqrt(2*Psi33.*dOmega);

Amp = [Amp11,Amp22,Amp33];
Phase1 = 2*pi*rand(length(Omega1),3);
Phase2 = 2*pi*rand(length(Omega2),3);


