function [x_D, u_D] = DesiredTrajectoryGenerator(fxu, t_vec, x_vec_ss, ...
    u_vec_ss, mass_inertia_vec)

% DesiredTrajectoryGenerator.m
%
% Desired trajectory generator for the LQR/PID or other controller to
% follow. Solve the nonlinear EOM to get the relationship and set some
% desired thrust to obtain the desired values of the other inputs and
% states. In total there are 12-states: [x_D; y_D; z_D; phi_D; theta_D; 
% psi_D; u_D; v_D; w_D; p_D; q_D; r_D], and 4-inputs: [u_T_D; u_phi_D; 
% u_theta_D; u_psi_D]

%%%INPUTS%%%
% dxdt: The 12 linear symbolic EOMs, obtained from LinearMultirotor()
% uSS: Numerical values of the input at the steady-state
% Klqr: The LQR gain matrix obtained from the linearized EOMs, applied to
% control a nonlinear system
% t_vec: Time vector at which points the values are needed to be found

%%%OUTPUTS%%%
% dx: State derivatives of the 12-states at each t_vec step
% ucntrl: The control input of the controller to the UAV at each t_vec step
syms  xD yD zD phiD thetaD psiD uD vD wD pD qD rD u_TD u_phiD u_thetaD ...
    u_psiD x_dotD y_dotD z_dotD phi_dotD theta_dotD psi_dotD u_dotD ...
    v_dotD w_dotD p_dotD q_dotD r_dotD u_TD u_phiD u_thetaD u_psiD real

%Extract the mass and inertia properties
m = mass_inertia_vec(1); g = mass_inertia_vec(2); 
Ixx = mass_inertia_vec(3); Iyy = mass_inertia_vec(4); 
Izz = mass_inertia_vec(5); Ixy = mass_inertia_vec(6);
Iyz = mass_inertia_vec(7); Izx = mass_inertia_vec(8);

%From Chen's paper on invariant EKF, adding drag force
m = 1.5; Ixx = .0348; Iyy = .0459; Izz = .0977; Dx = .3265; Dy = .3265; Dz = 2*.3265;
rho = 1.225;

%Desired values to solve for
dxdt = feval(fxu, Ixx, Ixy, Iyy, Iyz, Izx, Izz, g, m, pD, phiD, psiD, ...
    qD, rD, thetaD, uD, u_TD, u_phiD, u_psiD, u_thetaD, vD, wD);

F(1) = z_dotD - dxdt(3); F(2) = phi_dotD - dxdt(4); 
F(3) = theta_dotD - dxdt(5); F(4) = psi_dotD - dxdt(6); 
F(5) = u_dotD - dxdt(7) - .5/m*rho*Dx*sqrt(uD^2 + vD^2 + wD^2)*uD;  
F(6) = v_dotD - dxdt(8) - .5/m*rho*Dy*sqrt(uD^2 + vD^2 + wD^2)*vD;
F(7) = w_dotD - dxdt(9) - .5/m*rho*Dz*sqrt(uD^2 + vD^2 + wD^2)*wD;
F(8) = p_dotD - dxdt(10); F(9) = q_dotD - dxdt(11); 
F(10) = r_dotD - dxdt(12);

%Convert symbolic to function handle with some known constraints
F = subs(F, [zD phiD thetaD psiD uD pD qD rD z_dotD phi_dotD, ...
    theta_dotD psi_dotD u_dotD v_dotD w_dotD p_dotD q_dotD r_dotD ...
    u_phiD u_thetaD u_psiD], ...
    [x_vec_ss(3) x_vec_ss(4)-15*pi/180 0 0 0 0 0 0 0 0 0 0 ...
    0 0 0 0 0 0 0 0 0]);

%The function handle is of the form f@(pD,qD,rD,uD,u_TD,u_phiD,u_psiD, ...
%u_thetaD,vD,wD,x_dotD,y_dotD)
Roots = [pD,qD,rD,uD,u_TD,u_phiD,u_psiD, u_thetaD,vD,wD];
Fnew = matlabFunction(vpa(F), 'Vars', {Roots});

%Fnew = subs(Fnew, pD,qD,rD,uD,u_TD,u_phiD,u_psiD, u_thetaD,vD,wD)

%Initial guess
pD0 = 0; qD0 = 0; rD0 = 0; uD0 = 0; u_TD0 = u_vec_ss(1); 
u_phiD0 = u_vec_ss(2); u_psiD0 = u_vec_ss(4); u_thetaD0 = u_vec_ss(3);
vD0 = 0; wD0 = 0; x_dotD0 = 0; y_dotD0 = 0;;
Roots0 = [pD0,qD0,rD0,uD0,u_TD0,u_phiD0,u_psiD0, u_thetaD0, vD0, wD0, ...
    x_dotD0, y_dotD0];

Roots1 = zeros(size(Roots));
Ans = fsolve(Fnew, Roots0)

Ans1 = fsolve(Fnew, Roots1)

Fn(1) = F(1);
Fn(2) = F(6);
Fn(3) = F(7);
Vec = [vD; wD; u_TD];
Fnew = matlabFunction(vpa(Fn), 'Vars', {Vec});
Fsol= fsolve(Fnew, [1;1;1])

y_dot = Fsol(1) * cos(15*pi/180) - Fsol(2)*sin(15*pi/180)
%Input change from steady-state
delta_u = u_vec_ss - [Ans(5); Ans(6); Ans(8); Ans(7)]
delta_u1 = u_vec_ss - [Ans1(5); Ans1(6); Ans1(8); Ans1(7)]
x_D = 0; u_D = 0; 