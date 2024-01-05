function [exdot_vec] = LinearMultirotorTrajectory(t, ex, A, B, ...
    Klqr, Sim)

% LinearMultirotorTrajectory.m
%
% Linear multirotor trajectory at some specified values.
% Initially starts from steady-state/hover. If an LQR controller is used 
% then the input is ue=uD-Klqr*(x-xD) which replaces the inputs 
% u in Ax+Bu The state xe=x-xD is used in place of x in Ax+Bu

%%%INPUTS%%%
% t: ode45() evaluation time steps
% x_vec: ode45() evaluated solutions 
% A: Linearized state matrix
% B: Linearized input matrix
% Klqr: The LQR gain matrix obtained from the linearized EOMs, applied to
% control a nonlinear system
% Sim: Simulation parameters included in a structure
% Sim.Ts: Sampling time, s
% Sim.t_vec: Time vector, s
% Sim.out_del: Outgoing delay between the operator's command and the
% command reaching the system, s
% Sim.in_del: Incoming delay between the system's output and the output 
% reaching the operator, s
% Sim.out_delDT: Discrete Time (DT) converions of Sim.out_del
% Sim.in_delDT: Discrete Time (DT) converions of Sim.in_del
% Sim.initial_state: Initial states of the system, usually the steady state
% Sim.initial_input: Initial inputs of the system, usually the steady input

%%%OUTPUTS%%%
% dx: State derivatives of the 12-states at each t_vec step
% ucntrl: The control input of the controller to the UAV at each t_vec step

if (t<=Sim.in_del+1)      
    exdot_vec=zeros(12,1);
else 

% %Linear interpolation of the desired states, derivatives, and inputs at
% %each ode45 time step, t
% xref = interp1(Sim.t_vec, Sim.del_des_state(1,:), t);
% yref = interp1(Sim.t_vec, Sim.del_des_state(2,:), t);
% zref = interp1(Sim.t_vec, Sim.del_des_state(3,:), t);
% phiref = interp1(Sim.t_vec, Sim.del_des_state(4,:), t);
% thetaref = interp1(Sim.t_vec, Sim.del_des_state(5,:), t);
% psiref = interp1(Sim.t_vec, Sim.del_des_state(6,:), t);
% uref = interp1(Sim.t_vec, Sim.del_des_state(7,:), t);
% vref = interp1(Sim.t_vec, Sim.del_des_state(8,:), t);
% wref = interp1(Sim.t_vec, Sim.del_des_state(9,:), t);
% pref = interp1(Sim.t_vec, Sim.del_des_state(10,:), t);
% qref = interp1(Sim.t_vec, Sim.del_des_state(11,:), t);
% rref = interp1(Sim.t_vec, Sim.del_des_state(12,:), t);
% delta_des_state = [xref; yref; zref; phiref; thetaref; psiref; uref; vref; ...
%     wref; pref; qref; rref] - Sim.del_des_state;
% 
% u_Tref = interp1(Sim.t_vec, Sim.del_des_input(1,:), t);
% u_phiref = interp1(Sim.t_vec, Sim.del_des_input(2,:), t);
% u_thetaref = interp1(Sim.t_vec, Sim.del_des_input(3,:), t);
% u_psiref = interp1(Sim.t_vec, Sim.del_des_input(4,:), t);
% delta_des_input = [u_Tref; u_phiref; u_thetaref; u_psiref]-Sim.del_des_input;
% end
% 
% %Compute the LQR input to the plant (From Eqn B.9)
% delta_ucntrl = delta_des_input - Klqr*(ex-delta_des_state);
% %delta_ucntrl = delta_des_input - Klqr*(delta_x-delta_des_state);
exdot_vec = (A-B*Klqr)*ex;
end
