function [data2deg,data9deg,data16deg]=getdata(Ts,NF,T)
for CaseNo=1:36
   if T.Ts(CaseNo)==Ts && T.NF(CaseNo)==NF
        % 2 deg
        if T.Amp(CaseNo)==2 && T.TotalDelay(CaseNo)==2
        data2deg(:,1:3)= [T.xSP(CaseNo) T.xKP(CaseNo) T.xEKP(CaseNo);
                          T.zSP(CaseNo) T.zKP(CaseNo) T.zEKP(CaseNo);
                          T.thetaSP(CaseNo) T.thetaKP(CaseNo) T.thetaEKP(CaseNo);
                          T.uSP(CaseNo) T.uKP(CaseNo) T.uEKP(CaseNo);
                          T.wSP(CaseNo) T.wKP(CaseNo) T.wEKP(CaseNo);
                          T.qSP(CaseNo) T.qKP(CaseNo) T.qEKP(CaseNo)];
        end
        if T.Amp(CaseNo)==2 && T.TotalDelay(CaseNo)==4
        data2deg(:,4:6)= [T.xSP(CaseNo) T.xKP(CaseNo) T.xEKP(CaseNo);
                          T.zSP(CaseNo) T.zKP(CaseNo) T.zEKP(CaseNo);
                          T.thetaSP(CaseNo) T.thetaKP(CaseNo) T.thetaEKP(CaseNo);
                          T.uSP(CaseNo) T.uKP(CaseNo) T.uEKP(CaseNo);
                          T.wSP(CaseNo) T.wKP(CaseNo) T.wEKP(CaseNo);
                          T.qSP(CaseNo) T.qKP(CaseNo) T.qEKP(CaseNo)];
        end
        % 9 deg
        if T.Amp(CaseNo)==9 && T.TotalDelay(CaseNo)==2
        data9deg(:,1:3)= [T.xSP(CaseNo) T.xKP(CaseNo) T.xEKP(CaseNo);
                          T.zSP(CaseNo) T.zKP(CaseNo) T.zEKP(CaseNo);
                          T.thetaSP(CaseNo) T.thetaKP(CaseNo) T.thetaEKP(CaseNo);
                          T.uSP(CaseNo) T.uKP(CaseNo) T.uEKP(CaseNo);
                          T.wSP(CaseNo) T.wKP(CaseNo) T.wEKP(CaseNo);
                          T.qSP(CaseNo) T.qKP(CaseNo) T.qEKP(CaseNo)];
        end
        if T.Amp(CaseNo)==9 && T.TotalDelay(CaseNo)==4
        data9deg(:,4:6)= [T.xSP(CaseNo) T.xKP(CaseNo) T.xEKP(CaseNo);
                          T.zSP(CaseNo) T.zKP(CaseNo) T.zEKP(CaseNo);
                          T.thetaSP(CaseNo) T.thetaKP(CaseNo) T.thetaEKP(CaseNo);
                          T.uSP(CaseNo) T.uKP(CaseNo) T.uEKP(CaseNo);
                          T.wSP(CaseNo) T.wKP(CaseNo) T.wEKP(CaseNo);
                          T.qSP(CaseNo) T.qKP(CaseNo) T.qEKP(CaseNo)];
        end
        % 16 deg
        if T.Amp(CaseNo)==16 && T.TotalDelay(CaseNo)==2
        data16deg(:,1:3)= [T.xSP(CaseNo) T.xKP(CaseNo) T.xEKP(CaseNo);
                          T.zSP(CaseNo) T.zKP(CaseNo) T.zEKP(CaseNo);
                          T.thetaSP(CaseNo) T.thetaKP(CaseNo) T.thetaEKP(CaseNo);
                          T.uSP(CaseNo) T.uKP(CaseNo) T.uEKP(CaseNo);
                          T.wSP(CaseNo) T.wKP(CaseNo) T.wEKP(CaseNo);
                          T.qSP(CaseNo) T.qKP(CaseNo) T.qEKP(CaseNo)];
        end
        if T.Amp(CaseNo)==16 && T.TotalDelay(CaseNo)==4
        data16deg(:,4:6)= [T.xSP(CaseNo) T.xKP(CaseNo) T.xEKP(CaseNo);
                          T.zSP(CaseNo) T.zKP(CaseNo) T.zEKP(CaseNo);
                          T.thetaSP(CaseNo) T.thetaKP(CaseNo) T.thetaEKP(CaseNo);
                          T.uSP(CaseNo) T.uKP(CaseNo) T.uEKP(CaseNo);
                          T.wSP(CaseNo) T.wKP(CaseNo) T.wEKP(CaseNo);
                          T.qSP(CaseNo) T.qKP(CaseNo) T.qEKP(CaseNo)];
        end
   end
    
end
end