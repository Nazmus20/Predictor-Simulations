function [xp,Pp,nu_all]=ExtendedKalmanFilter(z,f,u,h,F,H,Q,R,v,x0,P0,nRK,dt,kmax,sz,sz_out, d1, d2)

% z is the vector of measurements
% f describes the dynamics, F is the Jacobian of the dynamics w.r.t. the state
% h is the output equation, H is the Jacobian of the output w.r.t the state
% R is the covariance of the measurement noise
% v is the vector of measurement noise
% x0,P0 are the initial state and covariance
% nRK is the number of RK steps 
% dt is the discrete time interval
% kmax is the number of discrete time steps
% sz is the dimension of the state vector 
% sz_out is the dimension of the output 
% d1 is the input delay steps
% d2 is the output delay steps


%Initialize data arrays
x = zeros(sz,kmax);
xm = x;
xp = x;
y = zeros(sz_out,kmax);
yhat = y;
Pm = zeros(sz,sz,kmax);
Pp = Pm;

Pp(:,:,1)=P0;
xp(:,1)=x0;
nu_all=zeros(sz_out,kmax);

%This is the index for both delays -1
i=d1+d2-1;

for k=1:kmax-1
    %4th-order Runge-Kutta method begins here 
    x=xp(:,k);
    P=Pp(:,:,k);
    uk=u(k,:);
    [x,Fk]=EKF_prediction(x,uk',f,F,dt,nRK);
    %Propagated state estimate
    xm(:,k+1)=x;    
    %Propagated covariance estimate
    Pm(:,:,k+1)=Fk*P*Fk' + Q;
    
    % %Observability
    %     A=feval(F,xp(:,k));
    %     C=feval(H, xp(:,k));
    %     Ob=obsv(A,C);
    %     if rank(Ob)<sz
    %         fprintf('Pair (A,C) is not observable.')
    %         fprintf('%d',A)
    %         break
    %     end 
        
    ukp1=u(k+1,:);
    %Actual Measurement
    y(:,k+1)= (z(k+1,:))'+ v;

    %Measurement model
    yhat(:,k+1)=feval(h,xm(:,k+1), ukp1');
    
    %Innovation
    nu=y(:,k+1)-yhat(:,k+1);
    nu_all(:,k+1)=y(:,k+1)-yhat(:,k+1);
    

    %Gain matrix
    Hmat=feval(H, xm(:,k+1));
    K(:,:,k)=Pm(:,:,k+1)*Hmat'*inv(Hmat*Pm(:,:,k+1)*Hmat' + R);
    
    %Measurement updated state and covariance estimates
    xp(:,k+1)=xm(:,k+1)+K(:,:,k)*(nu);
    Pp(:,:,k+1)=(eye(sz)-K(:,:,k)*Hmat)*Pm(:,:,k+1);
    
    xprev=xp(:,k+1);
    %This is where we will do prediction
    for j=0:i
        xhat_pred=feval(f,xprev,u(k-d1-d2+j,:));
        P_pred=eye(sz);
        
        xprev=xhat_pred;

    end


end


end