clear all
restoredefaultpath

for CaseNo=1:36

filename=['Case',num2str(CaseNo)]; %Change the folder name you want to work in

curr_path = pwd;
addpath([curr_path,'\',filename])

for i=1:10
    data(i)=load(['data_',num2str(i),'.mat']);
    info(1,1)=data(i).data.Ts;
    info(1,2)=data(i).data.d1+data(i).data.d2;
    info(1,3)=data(i).data.noise_factor; 
    info(1,4)=data(i).data.doublet_amplitude; %replace later with amplitude
    
    nstart=(data(i).data.d1+data(i).data.d2)/data(i).data.Ts + 1;
end
    y=[data(1).data.undelayed_measurement(:,nstart+3:end-1),...
       data(2).data.undelayed_measurement(:,nstart+3:end-1),...
       data(3).data.undelayed_measurement(:,nstart+3:end-1),...
       data(4).data.undelayed_measurement(:,nstart+3:end-1),...
       data(5).data.undelayed_measurement(:,nstart+3:end-1),...
       data(6).data.undelayed_measurement(:,nstart+3:end-1),...
       data(7).data.undelayed_measurement(:,nstart+3:end-1),...
       data(8).data.undelayed_measurement(:,nstart+3:end-1),...
       data(9).data.undelayed_measurement(:,nstart+3:end-1),...
       data(10).data.undelayed_measurement(:,nstart+3:end-1)];
       
    t=data(1).data.total_time(nstart+1:end-1);
    ySP=[data(1).data.SP_output(:,3:end-1),...
         data(2).data.SP_output(:,3:end-1),...
         data(3).data.SP_output(:,3:end-1),...
         data(4).data.SP_output(:,3:end-1),...
         data(5).data.SP_output(:,3:end-1),...
         data(6).data.SP_output(:,3:end-1),...
         data(7).data.SP_output(:,3:end-1),...
         data(8).data.SP_output(:,3:end-1),...
         data(9).data.SP_output(:,3:end-1),...
         data(10).data.SP_output(:,3:end-1)];
    yKP=[data(1).data.KP_output(:,3:end-1),...
         data(2).data.KP_output(:,3:end-1),...
         data(3).data.KP_output(:,3:end-1),...
         data(4).data.KP_output(:,3:end-1),...
         data(5).data.KP_output(:,3:end-1),...
         data(6).data.KP_output(:,3:end-1),...
         data(7).data.KP_output(:,3:end-1),...
         data(8).data.KP_output(:,3:end-1),...
         data(9).data.KP_output(:,3:end-1),...
         data(10).data.KP_output(:,3:end-1)];
    yEKP=[data(1).data.EKP_output(:,3:end),...
          data(2).data.EKP_output(:,3:end),...
          data(3).data.EKP_output(:,3:end),...
          data(4).data.EKP_output(:,3:end),...
          data(5).data.EKP_output(:,3:end),...
          data(6).data.EKP_output(:,3:end),...
          data(7).data.EKP_output(:,3:end),...
          data(8).data.EKP_output(:,3:end),...
          data(9).data.EKP_output(:,3:end),...
          data(10).data.EKP_output(:,3:end)];
    N=length(t);
    for j=1:12
        if j==4 || j==5 || j==6 || j==10 || j==11 || j==12
            RMSE_SP(j)=180/pi*sqrt(mean((y(j,:)-ySP(j,:)).^2));
            RMSE_KP(j)=180/pi*sqrt(mean((y(j,:)-yKP(j,:)).^2));
            RMSE_EKP(j)=180/pi*sqrt(mean((y(j,:)-yEKP(j,:)).^2));
        else 
            RMSE_SP(j)=sqrt(mean((y(j,:)-ySP(j,:)).^2));
            RMSE_KP(j)=sqrt(mean((y(j,:)-yKP(j,:)).^2));
            RMSE_EKP(j)=sqrt(mean((y(j,:)-yEKP(j,:)).^2));            
        end
    end
    nn=1;
    for n=5:3:5+3*11
        info(1,n:n+2)=[RMSE_SP(nn),RMSE_KP(nn),RMSE_EKP(nn)];
        nn=nn+1;
    end


Table=array2table(info,'VariableNames',{'Ts','TotalDelay','NF','Amp', 'xSP','xKP','xEKP','ySP','yKP','yEKP','zSP','zKP','zEKP',...
    'phiSP','phiKP','phiEKP','thetaSP','thetaKP','thetaEKP','psiSP','psiKP','psiEKP','uSP','uKP','uEKP','vSP','vKP','vEKP',...
    'wSP','wKP','wEKP','pSP','pKP','pEKP','qSP','qKP','qEKP','rSP','rKP','rEKP'});

save([filename,'.mat'], 'Table')


end