function [delta_des_state, delta_des_input, delta_des_state_deriv, Time] = DesiredTrajectoryGenerator(A, B)

syms xD yD zD phiD thetaD psiD uD vD wD pD qD rD u_TD u_phiD u_thetaD u_psiD real

t_vec = 0:.0001:20; %High precision time vector

delta_XD = [xD yD zD phiD thetaD psiD uD vD wD pD qD rD]';
delta_UD = [u_TD u_phiD u_thetaD u_psiD]';

%Desired state derivatives for translation
delta_x_dotD = 0; delta_y_dotD = 2.13015; delta_z_dotD = 0; %m/s
delta_phi_dotD = 0; delta_theta_dotD = 0; delta_psi_dotD = 0; %rad/s
delta_u_dotD = 0; delta_v_dotD = 0; delta_w_dotD = 0; %m/s^2
delta_p_dotD = 0; delta_q_dotD = 0; delta_r_dotD = 0; %rad/s^2


delta_X_dotD = vpa(A*delta_XD + B*delta_UD);

EQ(1) = delta_X_dotD(1)-delta_x_dotD == 0;
EQ(2) = delta_X_dotD(2)-delta_y_dotD == 0;
EQ(3) = delta_X_dotD(3)-delta_z_dotD == 0;
EQ(4) = delta_X_dotD(4)-delta_phi_dotD == 0;
EQ(5) = delta_X_dotD(5)-delta_theta_dotD == 0;
EQ(6) = delta_X_dotD(6)-delta_psi_dotD == 0;
EQ(7) = delta_X_dotD(7)-delta_u_dotD == 0;
EQ(8) = delta_X_dotD(8)-delta_v_dotD == 0;
EQ(9) = delta_X_dotD(9)-delta_w_dotD == 0;
EQ(10) = delta_X_dotD(10)-delta_p_dotD == 0;
EQ(11) = delta_X_dotD(11)-delta_q_dotD == 0;
EQ(12) = delta_X_dotD(12)-delta_r_dotD == 0;

S = solve(EQ, [phiD thetaD psiD uD vD wD pD qD rD u_TD u_phiD u_thetaD u_psiD]);

S.phiD=3.52877*pi/180;
S.thetaD=0;
S.psiD=0;
S.uD=0;
S.vD=2.12612;
S.wD=-0.131111;
S.qD=0;
S.pD=0;
S.rD=0;
S.u_TD=-14.7988;
S.u_phiD=0;
S.u_psiD=0;
S.u_thetaD=0;

%initial values
XD0 = double([0; 0; -5; S.phiD; S.thetaD; S.psiD; S.uD; S.vD; S.wD; S.pD; S.qD; S.rD]);

delta_xD = XD0(1) + delta_x_dotD*t_vec; 
delta_yD = XD0(2) + delta_y_dotD*t_vec;
delta_zD = XD0(3) + delta_z_dotD*t_vec; 
delta_phiD = XD0(4) + delta_phi_dotD*t_vec;
delta_thetaD = XD0(5) + delta_theta_dotD*t_vec; 
delta_psiD = XD0(6) + delta_psi_dotD*t_vec;
delta_uD = XD0(7) + delta_u_dotD*t_vec; 
delta_vD = XD0(8) + delta_v_dotD*t_vec;
delta_wD = XD0(9) + delta_w_dotD*t_vec; 
delta_pD = XD0(10) + delta_p_dotD*t_vec;
delta_qD = XD0(11) + delta_q_dotD*t_vec; 
delta_rD = XD0(12) + delta_r_dotD*t_vec;

delta_XD = [delta_xD; delta_yD; delta_zD; delta_phiD; delta_thetaD;
    delta_psiD; delta_uD; delta_vD; delta_wD; delta_pD; delta_qD; 
    delta_rD];

delta_UD = double([S.u_TD; S.u_phiD; S.u_thetaD; S.u_psiD])*ones(size(t_vec));

delta_X_dotD = [delta_x_dotD; delta_y_dotD; delta_z_dotD ;
    delta_phi_dotD; delta_theta_dotD; delta_psi_dotD;
    delta_u_dotD; delta_v_dotD; delta_w_dotD;
    delta_p_dotD; delta_q_dotD; delta_r_dotD] * ones(size(t_vec));

delta_des_state=delta_XD;
delta_des_input=delta_UD;
delta_des_state_deriv=delta_X_dotD; 
Time=t_vec;


