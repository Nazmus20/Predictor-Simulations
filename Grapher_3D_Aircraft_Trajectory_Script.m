% Aircraft trajectory plotter (3D)
%
% This script the 3D trajectory
%
% (C) 2022 Mekonen Haileselassie Halefom <hmeko13@vt.edu>
%
% Script to run the aircraft 3D trajectory from TTman files
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear all; clc; close all;
% 
% % Adding "Functions" folder to path
% addpath('/Users/mekonen/Desktop/Code/Functions')
% 
% % Adding "Data" folder to path
% addpath('/Users/mekonen/Desktop/Code/Data/20220929_MTD')
% 
% % Load data: fdata file from flight data
% load('20220929_MTD2_Flt1_Estim_Grid_Pattern_TTman.mat')
% load('20220929_MTD2_Flt1_Estim_Grid_Pattern_Tman.mat')
load('states.mat')
%% All figure parameters 

%plot_latex
font_size = 12;
plot_size = 1; 
figure_size = [10 4];

%% Plotting processed aircraft trajectory 

north       = x_hist_delayed; 
east        = y_hist_delayed; 
down        = z_hist_delayed; 
roll        = phi_hist_delayed;
pitch       = theta_hist_delayed;
yaw         = psi_hist_delayed;
scaleFactor = .5;
var         = 20;
aircraft    = 'helicopter';
trajectory(north,east,down,roll,pitch,yaw,scaleFactor,var,aircraft)
axis equal
% plot_axis(font_size)
grid on
ylabel('North [m]'); 
xlabel('East [m]');  
zlabel('Altitude [m]');
% zlim([0 250])

% Figure Confiuration
% plot_figure(figure_size)
% plot_save('Flt1 3D Profile')
