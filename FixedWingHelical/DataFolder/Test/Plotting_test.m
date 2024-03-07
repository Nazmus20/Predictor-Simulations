clear all

for i=1:10
    data(i)=load(['data_',num2str(i),'.mat']);
    info(i,1)=data(i).data.Ts;
    info(i,2)=data(i).data.d1+data(i).data.d2;
    info(i,3)=data(i).data.noise_factor; 
    info(i,4)=data(i).data.W(1,1); %replace later with amplitude
    
    nstart=(data(i).data.d1+data(i).data.d2)/data(i).data.Ts + 1;
    y=data(i).data.undelayed_measurement(:,nstart+1:end-1);
    t=data(i).data.total_time(nstart+1:end-1);
    ySP=data(i).data.SP_output(:,1:end-1);
    yKP=data(i).data.KP_output(:,1:end-1);
    yEKP=data(i).data.EKP_output;
    N=length(t);
    for j=1:12
        RMSE_SP(j,i)=sqrt(mean((y(j,:)-ySP(j,:)).^2));
        RMSE_KP(j,i)=sqrt(mean((y(j,:)-yKP(j,:)).^2));
        RMSE_EKP(j,i)=sqrt(mean((y(j,:)-yEKP(j,:)).^2));
    end
    nn=1;
    for n=5:3:5+3*11
        info(i,n:n+2)=[RMSE_SP(nn,i),RMSE_KP(nn,i),RMSE_EKP(nn,i)];
        nn=nn+1;
    end
end

Table1=array2table(info,'VariableNames',{'Ts','d1+d2','NF','Amp (deg)', 'xSP','xKP','xEKP','ySP','yKP','yEKP','zSP','zKP','zEKP',...
    'phiSP','phiKP','phiEKP','thetaSP','thetaKP','thetaEKP','psiSP','psiKP','psiEKP','uSP','uKP','uEKP','vSP','vKP','vEKP',...
    'wSP','wKP','wEKP','pSP','pKP','pEKP','qSP','qKP','qEKP','rSP','rKP','rEKP'});



