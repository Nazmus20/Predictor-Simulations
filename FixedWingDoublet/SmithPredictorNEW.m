function [Time_pred, Y_pred] = ...
    SmithPredictorNEW(sysDT, Time, Measurement, Input, VEq, thetaEq, deEq, ICdel, ICund, d1, d2, e1, e2, e3, Ts)

%%%INPUTS%%%
% sysCT: The continuous time (CT) system of the UAV
% Klqr: The LQR controller gain
% Sim.t_vec: Time_vector of the simulation [s]
% x_vec0: Initial states for the simulation to start. The UAV is assumed to
% start from its steady-states
% ref_vec: The actual reference vector without any delay
% d1: The value of the delay in the outgoing signal from the
% groundstation to the UAV, seconds
% d2: The value of the delay in the incoming signal from the UAV to
% the groundstation, seconds
% uSS: Steady state input values. As the simulation starts from
% steady-state these are also the initial values of the input thrusts and
% torques
% isLinear: Boolean. Should the linear or nonlinear model be used to
% generate the actual output

%%%OUTPUTS%%%
% Y_pred: The output of the Smith's Predictor

%%% EQUATIONS OF MOTION %%%
%Steady-state values
%Obtained by solving Vss^2 = uss^2+vss^2+wss^2 and alpha_ss = arctan(wss/uss)
uss = VEq*cos(thetaEq); wss = VEq*sin(thetaEq); vss = 0;
%Others are 0
phiss = 0; thetass = thetaEq; psiss = 0; pss = 0; qss = 0; rss = 0;

%%% EOMS IN DISCRETE TIME %%%
%Initial states
Xss = [phiss; thetass; psiss; uss;vss;wss; pss;qss;rss];
Sund(:,1) = ICund(1:3); deltaXund(:,1) = ICund(4:12) - Xss;
Sdel(:,1) = ICdel(1:3,1); deltaXdel(:,1) = ICdel(4:12) - Xss;

deltaU = Input - [0;deEq;0];

kmax=length(Time);

%Initialize state and output
xk=zeros(9,1);
xkp1=zeros(9,1);
yhat=zeros(9,1);
Y_pred=zeros(12,kmax-d2-d1-1);
Time_pred=zeros(1,kmax-d2-d1-1);

xk=deltaXdel(:,1);
Sk=Sdel;


for k=1:kmax
   if k-d2-d1-1>0
        uk=deltaU(:,k-d2-d1-1);
        %One step propagation (delayed)
        %Kinematic
        phi = xk(1) + Xss(1); theta = xk(2) + Xss(2); psi = xk(3) + Xss(3); 
        u = xk(4) + Xss(4); v = xk(5) + Xss(5); w = xk(6) + Xss(6);
        RIB = expm(psi*hat(e3))*expm(theta*hat(e2))*expm(phi*hat(e1));
        Skp1=Sk + RIB*[u;v;w;]*Ts;
        
        %Dynamics
        xkp1=sysDT.A*xk + sysDT.B*uk;
        
        %Delayed output
        yhatkmd2=eye(12)*[Skp1; xkp1+Xss];
        
        %Update for next step
        xk=xkp1;
        Sk=Skp1;
        
        % Finding undelayed states
        xprev=xkp1;
        Sprev=Skp1;
        for i=1:d2+d1
           %Kinematic propagation
           phiprev = xprev(1) + Xss(1); thetaprev = xprev(2) + Xss(2); psiprev = xprev(3) + Xss(3); 
           uprev = xprev(4) + Xss(4); vprev = xprev(5) + Xss(5); wprev = xprev(6) + Xss(6);
           RIBprev = expm(psiprev*hat(e3))*expm(thetaprev*hat(e2))*expm(phiprev*hat(e1));
           Spred=Sprev + RIBprev*[uprev;vprev;wprev]*Ts;
           
           %Dyanmic propagation
           ukpi=deltaU(:,k-d2-d1-1+i); 
           xpred=sysDT.A*xprev+sysDT.B*ukpi;
           
           %Update for next step
           xprev=xpred;
           Sprev=Spred;
        end
       
        %Undelayed output
        yhatkpd1=eye(12)*[Spred ; xpred + Xss];
        
        %Delayed measurement
        ykmd2=Measurement(:,k);
        
        l=k-d2-d1-1; %"actual index" 
        Y_pred(:,l)=ykmd2 + (yhatkpd1 - yhatkmd2);
        Time_pred(l)=Time(k);
   end
   
    
end
















end