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
nRK=10;
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


        %Gain matrix
        Hmat=feval(H, xm(:,k+1));
        K(:,:,k)=Pm(:,:,k+1)*Hmat'*inv(Hmat*Pm(:,:,k+1)*Hmat' + V);

        %Measurement updated state and covariance estimates
        xp(:,k+1)=xm(:,k+1)+K(:,:,k)*(nu);
        Pp(:,:,k+1)=(eye(sz)-K(:,:,k)*Hmat)*Pm(:,:,k+1);
        
        sum=zeros(1,12);
        for kk=1:d1
           xdot_eq(:,kk)=feval(f,xp(:,k-d1+kk), Input(:,k-d1-d2+kk)); 
           if kk==1 || kk==d1
               sum=sum+xdot_eq(:,kk);
           else
               sum=sum+2*xdot_eq(:,kk);
           end
        end
        xprev=xp(:,k+1)- (d1/(2* d1))*sum;
        
        %This is where we will do prediction
            for j=1:i
                
                [xhat_pred,~]=EKF_prediction(xprev,Input(:,k-d1-d2+j),f,F,Ts,nRK);
                
                xprev=xhat_pred;
            end
            
        P0=Pp(:,:,k+1);
        x0=xp(:,k+1);
        el=k-d1-d2-1; %"actual index"
        Y_pred(:,el)=feval(h, xprev, Input(:, k-d1-d2+i));        
        Time_pred(el)=Time(k);
        end 
    end

figure
subplot(1,2,1)
plot(Time, yhat(1,:), 'k-', Time, xp(1,:), 'g--', 'LineWidth', 1.5)
xlabel('Time, $t$ (s)', 'Interpreter', 'latex')
ylabel('$X$ (m)', 'Interpreter', 'latex')
%xlim([4 60])
%legend('Actual state', 'Estimated state')
hold on
x1 = Time(71); 
y1 = yhat(1,71) 
plot(x1,y1,'bo', 'MarkerFaceColor', 'b')
labels = {'$t=7$ s, $X=109.41$ m'};
text(x1,y1,labels,'VerticalAlignment','top','HorizontalAlignment','left', 'FontSize',14, 'Interpreter','latex', 'Color','b')
hold on
x2 = Time(91); 
y2 = yhat(1,91)
plot(x2,y2,'ro', 'MarkerFaceColor', 'r')
labels = {'$t=9$ s, $X=139.33$ m'};
text(x2,y2,labels,'VerticalAlignment','top','HorizontalAlignment','left', 'FontSize',14, 'Interpreter','latex', 'Color','r')
set(gca,'FontSize',14)

subplot(1,2,2)
plot(Time, yhat(5,:)*180/pi, 'k-', Time, xp(5,:)*180/pi, 'g--', 'LineWidth', 1.5)
xlabel('Time, $t$ (s)', 'Interpreter', 'latex')
ylabel('$\theta$ (deg)', 'Interpreter', 'latex')
%xlim([4 20])
hold on
x1 = Time(71); 
y1 = yhat(5,71)*180/pi 
plot(x1,y1,'bo', 'MarkerFaceColor', 'b')
labels = {'$t=7$ s, $\theta=0.91$ deg'};
text(x1,y1,labels,'VerticalAlignment','bottom','HorizontalAlignment','center', 'FontSize',14, 'Interpreter','latex', 'Color', 'b')
hold on
x2 = Time(91); 
y2 = yhat(5,91)*180/pi
plot(x2,y2,'ro', 'MarkerFaceColor', 'r')
labels = {'$t=9$ s, $\theta=-78.98$ deg'};
text(x2, y2,labels,'VerticalAlignment','top','HorizontalAlignment','center', 'FontSize',14, 'Interpreter','latex', 'Color', 'r')
set(gca,'FontSize',14)