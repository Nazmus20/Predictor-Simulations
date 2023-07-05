function [Amp,Phase,Omega] = OneDVonKarmanTurbulence(OmegaMinExp,OmegaMaxExp,NFreqs)

% TwoDVonKarmanTurbulence.m
%
% 1-dimensional von Karman turbulence. The function produces velocity 
% amplitudes (in US Customary Units) and random phase shifts corresponding 
% to spatial frequencies determined from the input parameters. 
% Example: OmegaMinExp = -4; OmegaMaxExp = 0; 
%
% [Amp,Phase,Omega] = TwoDVonKarmanTurbulence(OmegaMinExp,OmegaMaxExp,NFreqs)
% For i in {x,y,z}:
% W_i = Sum_j Amp_i(j)*cos(Omega1(j)*x + Omega2(j)*y + Phase_i(j)) (ft/s)
% Grad(W) = [dW_x/dx, 0, 0; dW_y/dx, 0, 0; dW_z/dx, 0, 0]; ((ft/s)/ft)

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
Omega = [Omegamin;Omegainterior] + dOmega/2;

% Choose an amplitude at each random frequency according to some
% spectrum function containing the frequency range of interest.

% Along-track 
Phi11 = ((L*sigma^2)/pi)*(1+(a*L*Omega).^2).^(-5/6);
Amp1 = sqrt(2*Phi11.*dOmega);
Phase1 = 2*pi*rand(size(Omega));

% Lateral
Phi22 = ((L*sigma^2)/(2*pi))*(1+(8/3)*(a*L*Omega).^2).*(1+(a*L*Omega).^2).^(-11/6);
Amp2 = sqrt(2*Phi22.*dOmega);
Phase2 = 2*pi*rand(size(Omega));

% Vertical
Phi33 = Phi22;
Amp3 = sqrt(2*Phi33.*dOmega);
Phase3 = 2*pi*rand(size(Omega));

Amp = [Amp1,Amp2,Amp3];
Phase = [Phase1,Phase2,Phase3];
