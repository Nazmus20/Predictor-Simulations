function [Amp,Phase,Omega] = OneDVonKarmanTurbulence(OmegaMinExp,OmegaMaxExp,NFreqs)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Generate one-dimensional (northward) turbulence spectrum (US Customary Units)
% Example: OmegaMinExp = -4; OmegaMaxExp = 0; 
%
% [Amp,Phase] = OneDVonKarmanTurbulence(OmegaMinExp,OmegaMaxExp,NFreqs)
% W_i = Sum_j Amp_i(j)*cos(Omega(j)*x + Phase_i(j)) for i \in {1,2,3} (ft/s)
% Grad(W) = [dW_1/dx, 0, 0; dW_2/dx, 0, 0; dW_3/dx, 0, 0]; ((ft/s)/ft)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

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
