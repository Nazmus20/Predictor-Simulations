clear all; clc; close all;
load dxdtActual.mat

syms x y z phi_r theta_r psi_r u v w p q r u_T u_phi u_theta u_psi Ixx ...
    Iyy Izz Ixy Iyz Izx m g rho W Dx Dy Dz real

syms xD yD zD phi_rD theta_rD psi_rD uD vD wD pD qD rD real

syms Klqr [4 12] real
syms des_input [4 1] real

% Multirotor parameters numerical values (Replace with the right UAV)
mass = 1.5;          % Mass [kg]
Inertia_xx = .0348;         % Roll inertia [kg-m^2]
Inertia_yy = .0459;         % Pitch inertia [kg-m^2]
Inertia_zz = .0977;         % Yaw inertia [kg-m^2]
Inertia_xy = 0; Inertia_yz = 0; Inertia_zx = 0; % Coupled inertia, [kg-m^2]
Inertia = [Inertia_xx, Inertia_xy Inertia_zx; Inertia_xy, Inertia_yy, ...
    Inertia_yz; Inertia_zx, Inertia_yz, Inertia_zz]; % Inertia matrix 
%All the UAV parameters are defined in a structure called 'Param'
% Atmospheric and gravity parameters (Constant altitude: Sea level)
density = 1.225; % Density [kg/m^3]
gravity = 9.81;        % Gravitational acceleration [m/s^2]
Weight = mass*gravity; % Weight [N]

states = [x y z phi_r theta_r psi_r u v w p q r]'
des_states = [xD yD zD phi_rD theta_rD psi_rD uD vD wD pD qD rD]'
error = states - des_states

inp = [u_T u_phi u_theta u_psi]'

input = des_input - Klqr * error

fxuCL = subs(fxuActual, inp, input)
    
A = jacobian(fxuActual, states);

B = jacobian(fxuActual, inp);

A_CL = A - B*Klqr
ACL = jacobian(fxuCL, states)


B_CL = B*Klqr
BCL = jacobian(fxuCL, des_states)

norm(ACL-A_CL)
norm(BCL-B_CL)



