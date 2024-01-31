function [Time_pred, Y_pred]=EKP(Time, Measurement, Input, f, F, h, H, W, V, IC, d1, d2, Ts)
%
% Measurement is the vector of measurements
% Input is the input history
% f describes the dynamics, F is the Jacobian of the dynamics w.r.t. the state
% h is the output equation, H is the Jacobian of the output w.r.t the state
% W is the covariance of the process noise
% V is the covariance of the measurement noise
% IC,P0 are the initial state and covariance
% nRK is the number of RK steps 
% Ts is the discrete time interval
% kmax is the number of discrete time steps
% sz is the dimension of the state vector 
% sz_out is the dimension of the output 
% d1 is the input delay steps
% d2 is the output delay steps

global XdEq YdEq

%Initialize data arrays
nRK=20;
kmax=length(Time);
sz=12;
sz_out=12;
P0=10*eye(sz);
x = zeros(sz,kmax);
xm = x;
xp = x;
y = zeros(sz_out,kmax);
yhat = y;
Pm = zeros(sz,sz,kmax);
Pp = Pm;
x0=IC;

%This is the index for both delays -1
i=d1+d2-1;

    for k=1:kmax-1
        if k-d2-d1-1>0
        Pp(:,:,k)=P0;
        xp(:,k)=x0;
        %4th-order Runge-Kutta method begins here 
        x=xp(:,k);
        P=Pp(:,:,k);
        uk=Input(:,k-d2-d1-1);
        [x,Fk]=EKF_prediction(x,uk',f,F,Ts,nRK);
        %Propagated state estimate
        xm(:,k+1)=x;    
        %Propagated covariance estimate
        Pm(:,:,k+1)=Fk*P*Fk' + W;
        ukp1=Input(:,k-d2-d1);
        %Actual Measurement
        y(:,k+1)= Measurement(:,k+1);

        %Measurement model
        yhat(:,k+1)=feval(h,xm(:,k+1), ukp1');

        %Innovation
        nu=y(:,k+1)-yhat(:,k+1);
        nu_all(:,k+1)=y(:,k+1)-yhat(:,k+1);


        %Gain matrix
        Hmat=feval(H, xm(:,k+1));
        K(:,:,k)=Pm(:,:,k+1)*Hmat'*inv(Hmat*Pm(:,:,k+1)*Hmat' + V);

        %Measurement updated state and covariance estimates
        xp(:,k+1)=xm(:,k+1)+K(:,:,k)*(nu);
        Pp(:,:,k+1)=(eye(sz)-K(:,:,k)*Hmat)*Pm(:,:,k+1);

        xprev=xp(:,k+1);
        
        %This is where we will do prediction
            for j=1:i
                
                [xhat_pred,~]=EKF_prediction(xprev,Input(:,k-d1-d2+j),f,F,Ts,nRK);
                
                xprev=xhat_pred;
            end
            
        P0=Pp(:,:,k+1);
        x0=xp(:,k+1);
        el=k-d1-d2-1; %"actual index"
        Y_pred(:,el)=feval(h, xprev, Input(:, k-d1-d2+i))-[XdEq*d1*Ts; YdEq*d1*Ts; zeros(10,1)];
        Time_pred(el)=Time(k);
        
        end
        
    end
num = 5;
figure
plot(Time, y(num,:), 'k-', Time, yhat(num,:), 'r--')
title('Plot')
legend('Measurement', 'Estimate')

end