function F = HoverEOM(fxu, Param)

% HoverEOM.m
%
% Static force and moment balance for equilibrium thrust force, and roll,
% pitch, and yaw torques at the steady states

%The 12 multirotor aircraft state eqations are solved for steady flight

%Assuming ambient wind gives x_dot = f(x) = 0 from Eqns. (3.10) for
%hovering flight. We take only the equations where u_T, u_phi, u_theta and
%u_psi show up as those are the unknowns. The rest of the equations give 
%trivial 0=0 solutions, these trivial solutions cannot be used in MATLAB
%numerical solver as the solver cannot make 0=0 due to numerical issues.
%So, only use the equations that contain non-trivial solutions.

Inertia = Param.I; m = Param.m; g = Param.g;

%State vectors
%Position vector of FB w.r.t. FI, [m]
s_vec = [Param.states(1); Param.states(2); Param.states(3)]; 
%Rotation of FB w.r.t. FI, [rad]
Theta_vec = [Param.states(4); Param.states(5); Param.states(6)];
%Velocity in FB, [m/s]
v_vec = [Param.states(7); Param.states(8); Param.states(9)]; 
%FB angular rates, rad/s
omega_vec = [Param.states(10); Param.states(11); ...
    Param.states(12)];

%Input forces and moments
u_T = Param.inputs(1); u_phi = Param.inputs(2); u_theta = Param.inputs(3);
u_psi = Param.inputs(4);

%Extracting individual states
x = s_vec(1); y = s_vec(2); z = s_vec(3); phi_r = Theta_vec(1);
theta_r = Theta_vec(2); psi_r = Theta_vec(3); u = v_vec(1); v = v_vec(2);
w = v_vec(3); p = omega_vec(1); q = omega_vec(2); r = omega_vec(3);

% From Eqns. (3.10) 9th to 12th terms 
F(1) = solve(fxu(9)==0, u_T);   
F(2) = solve(fxu(10)==0, u_phi);
F(3) = solve(fxu(11)==0, u_theta);
F(4) = solve(fxu(12)==0, u_psi);

%This gives us four equations and four unknowns which after solving
%provides the equilibrium values. However, for hover this is trivial as we
%can easily find the answer to be u_Teq = m*g; u_phieq = 0; u_thetaeq = 0;
%and u_psieq = 0; For this code, this step is done for verification only.
%But for more complex cases this step will be reqired explicitly to solve
%for equilibrium/steady-state EOMs.