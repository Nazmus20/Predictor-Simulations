clear all; clc; close all;

%Define the symbolic terms to be used
syms x y z phi_r theta_r psi_r u v w p q r u u_T u_phi u_theta u_psi rho W Dx Dy Dz real

%Define the symbolic terms to be used
syms xD yD zD phi_rD theta_rD psi_rD uD vD wD pD qD rD real

syms Klqr [4 12] real 

load dxdtActual.mat

Dx_val = .3265; Dy_val = .3265; Dz_val = 2*.3265; 
density = 1.225; % Density [kg/m^3]
gravity = 9.81;        % Gravitational acceleration [m/s^2]
mass = 1.5;
Weight = mass*gravity; % Weight [N]

states = [x y z phi_r theta_r psi_r u v w p q r]';
des_states = [xD yD zD phi_rD theta_rD psi_rD uD vD wD pD qD rD]';
inp = [u_T u_phi u_theta u_psi]';

error = states - des_states;

inputs = -Klqr*error;
%u_T = inputs(1); u_phi = inputs(2); u_theta = inputs(3); u_psi = inputs(4);

fxu = subs(fxuActual, [rho W Dx Dy Dz], [density, Weight, Dx_val, Dy_val, Dz_val]);

A = jacobian(fxu, states)
B = jacobian(fxu, inp)

fxuCL = subs(fxuActual, [u_T u_phi u_theta u_psi rho W Dx Dy Dz], [inputs(1) inputs(2) ...
    inputs(3) inputs(4), density, Weight, Dx_val, Dy_val, Dz_val]);

    
ACL = jacobian(fxuCL, states)
A_CL = A - B*Klqr

BCL = jacobian(fxuCL, des_states)
B_CL = B*Klqr