function [dx, ucntrl] = LinearMultirotorTrajectoryGeneratorOLD(tdum, ...
    xdum, A, B, Klqr, t_vec, x_ref, val)
%!!!!!!!!!!!!!!!!!DEPRECATED!!!!!!!!!!!
% LinearMultirotorTrajectoryGenerator.m
%
% Linear multirotor trajectory generator at some specified values.
% Initially starts from steady-state. If an LQR controller is used then the
% input is u=uSS - Klqr*(x - xref) which replaces the inputs u in f(x,u)
% val: substituting the symbolic steady-state values with actual numbers
% the first 4 elements will be the steady-state values of the inputs and
% the last 8 elements will be the mass and inertia properties
% val = [u_T_ss; u_phi_ss; u_theta_ss; u_psi_ss; m; g; Ixx; Iyy;
% Izz; Ixy; Iyz; Izx]

%%%INPUTS%%%
% dxdt: The 12 linear symbolic EOMs, obtained from LinearMultirotor()
% uSS: Numerical values of the input at the steady-state
% Klqr: The LQR gain matrix obtained from the linearized EOMs, applied to
% control a nonlinear system
% x_ref: The reference vector
% t_vec: Time vector at which points the values are needed to be found
% tdum: ode45() evaluation time steps
% xdum: ode45() evaluated solutions 

%%%OUTPUTS%%%
% dx: State derivatives of the 12-states at each t_vec step
% ucntrl: The control input of the controller to the UAV at each t_vec step

%{
%!!!!!!!!!!!!!!!!Symbolic capabilities taken out!!!!!!!!!!!!
%Defining the 12 states, the inertias, the mass and the gravity in symbolic
syms x y z phi_r theta_r psi_r u v w p q r u_T u_phi u_theta u_psi Ixx ...
    Iyy Izz Ixy Iyz Izx m g real

%Defining the 12 states steady state values in symbolic, to be evaluated
syms x_ss y_ss z_ss phi_ss theta_ss psi_ss u_ss v_ss w_ss p_ss q_ss ...
    r_ss real
%}

%Linear interpolation of the reference at each ode45 time step, t
xref = interp1(t_vec, x_ref(1,:), tdum);
yref = interp1(t_vec, x_ref(2,:), tdum);
zref = interp1(t_vec, x_ref(3,:), tdum);
phiref = interp1(t_vec, x_ref(4,:), tdum);
thetaref = interp1(t_vec, x_ref(5,:), tdum);
psiref = interp1(t_vec, x_ref(6,:), tdum);
uref = interp1(t_vec, x_ref(7,:), tdum);
vref = interp1(t_vec, x_ref(8,:), tdum);
wref = interp1(t_vec, x_ref(9,:), tdum);
pref = interp1(t_vec, x_ref(10,:), tdum);
qref = interp1(t_vec, x_ref(11,:), tdum);
rref = interp1(t_vec, x_ref(12,:), tdum);

ref_vec = [xref; yref; zref; phiref; thetaref; psiref; uref; vref; wref; ...
    pref; qref; rref];

%Compute the LQR input to the plant
ucntrl = - Klqr * (xdum - ref_vec);

%{
%!!!!!!!!!!!!!!!!Symbolic capabilities taken out!!!!!!!!!!!!
%Substitute the numeric values into the symbolic functions
dx = subs(fxu, [x, y, z, phi_r, theta_r, psi_r, u, v, w, p, q, r, u_T, ...
    u_phi, u_theta, u_psi, m, g, Ixx, Iyy, Izz, Ixy, Iyz, Izx ], ...
    [xdum(1), xdum(2), xdum(3), xdum(4), xdum(5), xdum(6), xdum(7), ...
    xdum(8), xdum(9), xdum(10), xdum(11), xdum(12), ulqr(1), ulqr(2), ulqr(3), ...
    ulqr(4), val(5), val(6), val(7), val(8), val(9), val(10), val(11), val(12)]);

dx = double(dx);
%}

%Using symbolic functions are inefficient, so converting them to MATLAB
%function handle and computing the states
x = xdum(1); y = xdum(2); z = xdum(3); phi_r = xdum(4); theta_r = xdum(5);
psi_r= xdum(6); u = xdum(7); v = xdum(8); w= xdum(9); p = xdum(10);
 q = xdum(11); r = xdum(12); u_T = ucntrl(1); u_phi = ucntrl(2); 
u_theta = ucntrl(3); u_psi = ucntrl(4); m = val(5); g = val(6); 
Ixx = val(7); Iyy = val(8); Izz = val(9); Ixy = val(10); Iyz = val(11); 
Izx = val(12);

%Use a linear model: x_dot = A*x + B*u
dx = A*(xdum-ref_vec) + B*ucntrl;
