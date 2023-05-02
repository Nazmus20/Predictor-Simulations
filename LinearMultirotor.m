function [A,B, u_ss] = LinearMultirotor(varargin)

% LinearMultirotor.m
%
% Linearizes multirotor EOM at some specified steady values

%%%INPUTS%%%
% dxdt: The 12 nonlinear symbolic EOMs, obtained from MultirotorEOM()
% val: substituting the symbolic steady-state values with actual numbers
% the first 12 elements will be the steady-state values of the states 
% the middle 4 elements will be the steady-state values of the inputs and
% the last 8 elements will be the mass and inertia properties
% val = [x_ss; y_ss; z_ss; phi_ss; theta_ss; psi_ss; u_ss; v_ss; w_ss; 
% p_ss; q_ss; r_ss; u_T_ss u_phi_ss u_theta_ss u_psi_ss; m; g; Ixx; Iyy;
% Izz; Ixy; Iyz; Izx]

%%%OUTPUTS%%%
% A: Linearized state matrix in symbolic variables of u_T_ss, u_phi_ss,
% u_theta_ss, and u_psi_ss as they will be solved for later
% B: Linearized input matrix in symbolic variables of u_T_ss, u_phi_ss,
% u_theta_ss, and u_psi_ss as they will be solved for later
% u_ss: Steady-state numerical values of the input

%Defining the 12 states, the inertias, the mass and the gravity in symbolic
syms x y z phi_r theta_r psi_r u v w p q r u_T u_phi u_theta u_psi Ixx ...
    Iyy Izz Ixy Iyz Izx m g real

%Defining the 12 states steady state values in symbolic, to be evaluated
syms x_ss y_ss z_ss phi_ss theta_ss psi_ss u_ss v_ss w_ss p_ss q_ss ...
    r_ss u_T_ss u_phi_ss u_theta_ss u_psi_ss real

if nargin==1
    fxu=varargin{1};
    
    %A = dfxu/dx @ x=xss, u=uss
    A = jacobian(fxu, [x; y; z; phi_r; theta_r; psi_r; u; v; w; p; q; r]);
    A = subs(A, [x; y; z; phi_r; theta_r; psi_r; u; v; w; p; q; r; 
        u_phi; u_theta; u_psi], [x_ss; y_ss; z_ss; phi_ss; theta_ss; ...
        psi_ss; u_ss; v_ss; w_ss; p_ss; q_ss; r_ss, u_T_ss; u_phi_ss; ...
        u_theta_ss; u_psi_ss]);
    
    %B = dfxu/du @ x=val(1:12), u=val(13:16)
    B = jacobian(fxu, [u_T; u_phi; u_theta; u_psi]);
    B = subs(B, [x; y; z; phi_r; theta_r; psi_r; u; v; w; p; q; r; 
        u_phi; u_theta; u_psi], [x_ss; y_ss; z_ss; phi_ss; theta_ss; ...
        psi_ss; u_ss; v_ss; w_ss; p_ss; q_ss; r_ss, u_T_ss; u_phi_ss; ...
        u_theta_ss; u_psi_ss]);

elseif nargin==2
    fxu=varargin{1}; val=varargin{2};
    
    %Obtaining numerical values of steady-state inputs
    u_ss = subs(val(13:16), [x_ss; y_ss; z_ss; phi_ss; theta_ss; ...
        psi_ss; u_ss; v_ss; w_ss; p_ss; q_ss; r_ss; m; g; Ixx; Iyy; ...
        Izz; Ixy; Iyz; Izx], [val(1:12);val(17:end)]);
    u_ss = double(u_ss);

    %A = dfxu/dx @ x=val(1:12), u=val(13:16)
    A = jacobian(fxu, [x; y; z; phi_r; theta_r; psi_r; u; v; w; p; q; r]);
    A = subs(A, [x; y; z; phi_r; theta_r; psi_r; u; v; w; p; q; r; ...
        u_T; u_phi; u_theta; u_psi; m; g; Ixx; Iyy; Izz; Ixy; Iyz; ...
        Izx], val);
    A = double(A);
    
    %B = dfxu/du @ x=val(1:12), u=val(13:16)
    B = jacobian(fxu, [u_T; u_phi; u_theta; u_psi]);
    B = subs(B, [x; y; z; phi_r; theta_r; psi_r; u; v; w; p; q; r; ...
        u_T; u_phi; u_theta; u_psi; m; g; Ixx; Iyy; Izz; Ixy; Iyz; ...
        Izx], val);
    B = double(B);
else
    disp('LinearMultirotor.m accepts up to two input arguments')
end