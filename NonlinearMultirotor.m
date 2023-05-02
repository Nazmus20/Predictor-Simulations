function [dx] = NonlinearMultirotor(tdum, xdum, fxu, Klqr, t_vec, ...
    x_ref, val)

% NonlinearMultirotor.m
%
% Nonlinear multirotor EOM output at some specified values. Initially
% starts from steady-state. If an LQR controller is used then the input is
% u=uSS - Klqr*(x - xref) which replaces the inputs u in f(x,u)
% val: substituting the symbolic steady-state values with actual numbers
% the first 4 elements will be the steady-state values of the inputs and
% the last 8 elements will be the mass and inertia properties
% val = [u_T_ss; u_phi_ss; u_theta_ss; u_psi_ss; m; g; Ixx; Iyy;
% Izz; Ixy; Iyz; Izx]

%%%INPUTS%%%
% dxdt: The 12 nonlinear symbolic EOMs, obtained from MultirotorEOM()
% uSS: Numerical values of the input at the steady-state
% Klqr: The LQR gain matrix obtained from the linearized EOMs, applied to
% control a nonlinear system
% x_ref: The reference vector
% x: The symbolic state vector whose values are every time step needs to be
% found
% t_vec = Time vector at which points the values are needed to be found

%%%OUTPUTS%%%
% dx: State derivatives of the 12-states
% 
%Defining the 12 states, the inertias, the mass and the gravity in symbolic
syms x y z phi_r theta_r psi_r u v w p q r u_T u_phi u_theta u_psi Ixx ...
    Iyy Izz Ixy Iyz Izx m g real

%Defining the 12 states steady state values in symbolic, to be evaluated
syms x_ss y_ss z_ss phi_ss theta_ss psi_ss u_ss v_ss w_ss p_ss q_ss ...
    r_ss real

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
ulqr = val(1:4) - Klqr * (xdum - ref_vec);

%Substitute the numeric values into the symbolic functions
dx = subs(fxu, [x, y, z, phi_r, theta_r, psi_r, u, v, w, p, q, r, u_T, ...
    u_phi, u_theta, u_psi, m, g, Ixx, Iyy, Izz, Ixy, Iyz, Izx ], ...
    [xdum(1), xdum(2), xdum(3), xdum(4), xdum(5), xdum(6), xdum(7), ...
    xdum(8), xdum(9), xdum(10), xdum(11), xdum(12), ulqr(1), ulqr(2), ulqr(3), ...
    ulqr(4), val(5), val(6), val(7), val(8), val(9), val(10), val(11), val(12)]);

dx = double(dx);

