clc
clear all

T1=load('Case1.mat');

T=T1.Table;

for CaseNo=2:36
   LoadTable=load(['Case',num2str(CaseNo),'.mat']);
    T=[T;
      LoadTable.Table];  
end

save('FinalResultsRMSE.mat', 'T')