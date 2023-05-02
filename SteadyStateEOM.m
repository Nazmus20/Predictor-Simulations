function F = SteadyStateEOM(fxu)

% SteadyStateEOM.m
%
% Static force and moment balance for equilibrium thrust force, and roll,
% pitch, and yaw torques at the steady states

%The 12 multirotor aircraft state eqations are solved for steady flight



%Defining the 12 states, the inertias, the mass and the gravity in symbolic
syms x y z phi_r theta_r psi_r u v w p q r u_T u_phi u_theta u_psi Ixx ...
    Iyy Izz Ixy Iyz Izx m g real

%Defining the 12 states steady state values in symbolic, to be evaluated
syms x_ss y_ss z_ss phi_ss theta_ss psi_ss u_ss v_ss w_ss p_ss q_ss ...
    r_ss u_T_ss u_phi_ss u_theta_ss u_psi_ss real

%Assuming ambient wind gives x_dot = f(x) = 0 from Eqns. (3.10) for
%hovering flight. We take only the equations where u_T, u_phi, u_theta and
%u_psi show up as those are the unknowns. The rest equations give the
%trivial 0=0 solution

% From Eqns. (3.10) 9th to 12th terms 
F(1) = solve(fxu(9)==0, u_T);   
F(2) = solve(fxu(10)==0, u_phi);
F(3) = solve(fxu(11)==0, u_theta);
F(4) = solve(fxu(12)==0, u_psi);

F = subs(F, [x y z phi_r theta_r psi_r u v w p q r u_T u_phi u_theta ...
    u_psi], [x_ss y_ss z_ss phi_ss theta_ss psi_ss u_ss v_ss w_ss p_ss ...
    q_ss r_ss u_T_ss u_phi_ss u_theta_ss u_psi_ss]);

%This gives us four equations and four unknowns which after solving
%provides the equilibrium values. However, for hover this is trivial as we
%can easily find the answer to be u_Teq = m*g; u_phieq = 0; u_thetaeq = 0;
%and u_psieq = 0; For this code, this step is done for verification only.
%But for more complex cases this step will be reqired explicitly to solve
%for equilibrium/steady-state EOMs.