function [ref_states, ref_state_derivs, ref_inputs] = ...
    DesiredTrajectoryGenerator(fxu, Sim, Values, Param, des_vals)

% DesiredTrajectoryGenerator.m
%
% Desired trajectory generator for the LQR/PID or other controller to
% follow. Solve the nonlinear EOM to get the relationship and set some
% desired thrust to obtain the desired values of the other inputs and
% states. In total there are 12-states: [x_D; y_D; z_D; phi_D; theta_D; 
% psi_D; u_D; v_D; w_D; p_D; q_D; r_D], and 4-inputs: [u_T_D; u_phi_D; 
% u_theta_D; u_psi_D]

%%%INPUTS%%%
% fxu: The 12 linear symbolic EOMs, obtained from MultirotorEOM()
% uSS: Numerical values of the input at the steady-state
% Klqr: The LQR gain matrix obtained from the linearized EOMs, applied to
% control a linear/nonlinear system
% t_vec: Time vector at which points the values are needed to be found

%%%OUTPUTS%%%
% dx: State derivatives of the 12-states at each t_vec step
% ucntrl: The control input of the controller to the UAV at each t_vec step

syms  xD yD zD phiD thetaD psiD uD vD wD pD qD rD u_TD u_phiD u_thetaD ...
    u_psiD x_dotD y_dotD z_dotD phi_dotD theta_dotD psi_dotD u_dotD ...
    v_dotD w_dotD p_dotD q_dotD r_dotD real


desired_state_derivs = [x_dotD y_dotD z_dotD phi_dotD theta_dotD ...
    psi_dotD u_dotD v_dotD w_dotD p_dotD q_dotD r_dotD]';
desired_states = [xD yD zD phiD thetaD psiD uD vD wD pD qD rD]';
desired_inputs = [u_TD u_phiD u_thetaD u_psiD]';

desired_symbolic_vec = [desired_states; desired_inputs; ...
    desired_state_derivs];

%Remove the NaN values from the desired known values input
idx = []; not_idx = [];
for i = 1:length(des_vals)
    if isnan(des_vals(i)) ~= 1
        idx = [idx; i];
    else
        not_idx = [not_idx; i];
    end
end

Roots = desired_symbolic_vec(not_idx); %The symbolic roots to solve for
desired_symbolic_vec = desired_symbolic_vec(idx);
des_vals_sub = des_vals(idx); %subsystem of the desired vals

%Extract the mass and inertia properties
m = Values.m; g = Values.g; rho = Values.rho;
Ixx = Values.I(1,1); Iyy = Values.I(2,2); 
Izz = Values.I(3,3); Ixy = Values.I(2,1);
Iyz = Values.I(2,3); Izx = Values.I(3,1); 
%}
    
%Desired values to solve for
dxdt = subs(fxu, [Param.m; Param.g; Param.I(1,1); Param.I(1,2); ...
    Param.I(1,3); Param.I(2,1); Param.I(2,2); Param.I(2,3); ...
    Param.I(3,1); Param.I(3,2); Param.I(3,3); Param.states; ...
    Param.inputs; Param.rho; Param.aero], [Values.m; Values.g; ...
    Values.I(1,1); Values.I(1,2); Values.I(1,3); Values.I(2,1); ...
    Values.I(2,2); Values.I(2,3); Values.I(3,1); Values.I(3,2); ...
    Values.I(3,3); desired_states; desired_inputs; Values.rho; Values.aero]);

%Create a subsystem which shows the non-zero equations needed to be solved
dxdt_sub = subs(dxdt, desired_symbolic_vec, des_vals_sub);
idx2 = [];
for i = 1:length(dxdt_sub)
    if dxdt_sub(i) ~= 0
        idx2 = [idx2; i];
    end
end

%Create the 12 system of equations to solve for numerically
F(1) = dxdt(1); 
F(2) = dxdt(2);
F(3) = dxdt(3);
F(4) = dxdt(4); 
F(5) = dxdt(5); 
F(6) = dxdt(6);
F(7) = dxdt(7);  
F(8) = dxdt(8);
F(9) = dxdt(9);
F(10) = dxdt(10);
F(11) = dxdt(11);
F(12) = dxdt(12);

EQN = vpa(subs(F, desired_symbolic_vec, des_vals_sub));

%Here the Equations need to be solved manually depending on the conditions.
%Use the output of the EQN above to determine which equations are suitable
%For example, the following has been solved for steady translation along 
%the x-y plane.

%First, take the equations that matter, some equations are in 0=0 form
%Here, var(1) = uD; var(2) = vD; var(3) = wD; var(4) = u_TD;
var = [u_TD; u_phiD; u_thetaD; u_psiD; phiD; uD; vD; wD];
FUNC = [EQN(1:3), EQN(8:12) ]; %The other Eqns are 0=0 so they are ignored
ht = matlabFunction(FUNC, 'Vars', {var});
%Rewrite them in 'fsolve()' required format

var_val = fsolve(ht, [1; 1; 1; 1; 1; 1; 1; 1]);

x_init = 0; y_init = 0;
%Create the reference trajectories
for i=1:length(Sim.t_vec)
    phi(i) = 1*pi/180;
    theta(i) = des_vals(5);
    psi(i) = des_vals(6);
    u(i) = 0;
    v(i) = 1.1236;
    w(i) = -.0197;
    p(i) = 0;
    q(i) = 0;
    r(i) = 0;
    u_T(i) = -14.721; u_phi(i) = 0; u_theta(i) = 0; u_psi(i) = 0;
    xdot(i) = 0; ydot(i) = 1.1238; zdot(i) = 0; phi_dot(i) = 0; 
    theta_dot(i) = 0; psi_dot(i) = 0; u_dot(i) = 0; v_dot(i) = 0;
    w_dot(i) = 0; p_dot(i) = 0; q_dot(i) = 0; r_dot(i) = 0;
    x(i) = x_init+xdot(i)*Sim.t_vec(i);
    y(i) = y_init+ydot(i)*Sim.t_vec(i);
    z(i) = des_vals(3);
end
ref_states = [x;y;z;phi;theta;psi;u;v;w;p;q;r];
ref_state_derivs = [xdot; ydot; zdot; phi_dot; theta_dot; psi_dot; u_dot; v_dot; w_dot; p_dot; q_dot; r_dot];
ref_inputs = [u_T; u_phi; u_theta; u_psi];



