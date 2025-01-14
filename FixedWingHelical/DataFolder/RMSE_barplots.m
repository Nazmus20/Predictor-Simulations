clc
close all
clear all

restoredefaultpath
addpath('C:\Users\zakiaahmed\Documents\GitHub\Predictor-Simulations\FixedWingDoublet\DataFolder')
load('FinalResultsRMSE.mat')

%% Data for Ts=0.01;
Ts=0.01;
NF=10;

if NF==1
    caseNF=['i'];
    View= [-13.5000   29.3212]; %For NF=1;

else 
    caseNF=['ii'];
    View= [11.2331   34.6208]; %For NF=10;
end

[data2deg,data9deg,data16deg]=getdata(Ts,NF,T);


%%
set(groot,'defaultAxesTickLabelInterpreter','latex');  
set(groot,'defaulttextinterpreter','latex');
set(groot,'defaultLegendInterpreter','latex');
x1=categorical({'$X$(m)','$Z$(m)'});
x1=reordercats(x1,{'$X$(m)','$Z$(m)'});
x2=categorical({'$u$(m/s)', '$w$(m/s)'});
x2=reordercats(x2,{'$u$(m/s)', '$w$(m/s)'});
x3=categorical({'$\theta$(deg)','$q$(deg/s)'});
x3=reordercats(x3,{'$\theta$(deg)','$q$(deg/s)'});

fs=14;
Position=[681   528   613   451];
digits(2)

% Delay = 2s
XZdata=[data16deg(1,1:3), data16deg(2,1:3); data9deg(1,1:3), data9deg(2,1:3); data2deg(1,1:3), data2deg(2,1:3)];
figure
nPlot = get(gcf,'Number');
filename{nPlot}=strcat('case_abc_',caseNF,'_1_C_XZ');
XZbar=bar3(XZdata);
for k=1:length(XZbar)
    XZbar(k).FaceAlpha = '0.9';
    XZbar(k).EdgeAlpha = '0.8';
    if k==1 || k==4
        XZbar(k).FaceColor = ['#E899C5'];
    elseif k==2 || k==5
        XZbar(k).FaceColor = ['#FB892F'];
    elseif k==3 || k==6
        XZbar(k).FaceColor = ['#FFBC25'];
    end
end
zlabel({'$T_s=0.01$s','RMSE'})
set(gca,'ytick',[1:3],'yticklabel',{'$16^\circ$';'$9^\circ$';'$2^\circ$'});
set(gca, 'xtick',[1:6],'xticklabel',{'$\begin{array}{c}X\rm{(m)} \\ \rm{SP}\end{array}$',...
                                     '$\begin{array}{c}X\rm{(m)} \\ \rm{KP}\end{array}$',...
                                     '$\begin{array}{c}X\rm{(m)} \\ \rm{EKP}\end{array}$',...
                                     '$\begin{array}{c}Z\rm{(m)} \\ \rm{SP}\end{array}$',...
                                     '$\begin{array}{c}Z\rm{(m)} \\ \rm{KP}\end{array}$',...
                                     '$\begin{array}{c}Z\rm{(m)} \\ \rm{EKP}\end{array}$'});
set(gca, 'View', View); set(gca, 'FontSize', fs)

zscale=0.75;
text(1-0.25,1, XZdata(1,1)+zscale,sprintf('%0.1f',XZdata(1,1)),'HorizontalAlignment','left', 'FontSize', fs)
text(1-0.25,2, XZdata(2,1)+zscale,sprintf('%0.1f',XZdata(2,1)),'HorizontalAlignment','left', 'FontSize', fs)
text(1-0.25,3, XZdata(3,1)+zscale,sprintf('%0.1f',XZdata(3,1)),'HorizontalAlignment','left', 'FontSize', fs)

text(2-0.25,1, XZdata(1,2)+zscale,sprintf('%0.1f',XZdata(1,2)),'HorizontalAlignment','left', 'FontSize', fs)
text(2-0.25,2, XZdata(2,2)+zscale,sprintf('%0.1f',XZdata(2,2)),'HorizontalAlignment','left', 'FontSize', fs)
text(2-0.25,3, XZdata(3,2)+zscale,sprintf('%0.1f',XZdata(3,2)),'HorizontalAlignment','left', 'FontSize', fs)

text(3-0.25,1, XZdata(1,3)+zscale,sprintf('%0.1f',XZdata(1,3)),'HorizontalAlignment','left', 'FontSize', fs)
text(3-0.25,2, XZdata(2,3)+zscale,sprintf('%0.1f',XZdata(2,3)),'HorizontalAlignment','left', 'FontSize', fs)
text(3-0.25,3, XZdata(3,3)+zscale,sprintf('%0.1f',XZdata(3,3)),'HorizontalAlignment','left', 'FontSize', fs)

text(4-0.25,1, XZdata(1,4)+zscale,sprintf('%0.1f',XZdata(1,4)),'HorizontalAlignment','left', 'FontSize', fs)
text(4-0.25,2, XZdata(2,4)+zscale,sprintf('%0.1f',XZdata(2,4)),'HorizontalAlignment','left', 'FontSize', fs)
text(4-0.25,3, XZdata(3,4)+zscale,sprintf('%0.1f',XZdata(3,4)),'HorizontalAlignment','left', 'FontSize', fs)

text(5-0.25,1, XZdata(1,5)+zscale,sprintf('%0.1f',XZdata(1,5)),'HorizontalAlignment','left', 'FontSize', fs)
text(5-0.25,2, XZdata(2,5)+zscale,sprintf('%0.1f',XZdata(2,5)),'HorizontalAlignment','left', 'FontSize', fs)
text(5-0.25,3, XZdata(3,5)+zscale,sprintf('%0.1f',XZdata(3,5)),'HorizontalAlignment','left', 'FontSize', fs)

text(6-0.25,1, XZdata(1,6)+zscale,sprintf('%0.1f',XZdata(1,6)),'HorizontalAlignment','left', 'FontSize', fs)
text(6-0.25,2, XZdata(2,6)+zscale,sprintf('%0.1f',XZdata(2,6)),'HorizontalAlignment','left', 'FontSize', fs)
text(6-0.25,3, XZdata(3,6)+zscale,sprintf('%0.1f',XZdata(3,6)),'HorizontalAlignment','left', 'FontSize', fs)
set(gcf, 'Position', Position)            

% u and w plots RMSE                                  
uwdata=[data16deg(4,1:3), data16deg(5,1:3); data9deg(4,1:3), data9deg(5,1:3); data2deg(4,1:3), data2deg(5,1:3)];
figure
nPlot = get(gcf,'Number');
filename{nPlot}=strcat('case_abc_',caseNF,'_1_C_uw');
uwbar=bar3(uwdata);
for k=1:length(uwbar)
    uwbar(k).FaceAlpha = '0.9';
    uwbar(k).EdgeAlpha = '0.8';
    if k==1 || k==4
        uwbar(k).FaceColor = ['#E899C5'];
    elseif k==2 || k==5
        uwbar(k).FaceColor = ['#FB892F'];
    elseif k==3 || k==6
        uwbar(k).FaceColor = ['#FFBC25'];
    end
end

set(gca,'ytick',[1:3],'yticklabel',{'$16^\circ$';'$9^\circ$';'$2^\circ$'});
set(gca, 'xtick',[1:6],'xticklabel',{'$\begin{array}{c}u\rm{(m/s)} \\ \rm{SP}\end{array}$',...
                                     '$\begin{array}{c}u\rm{(m/s)} \\ \rm{KP}\end{array}$',...
                                     '$\begin{array}{c}u\rm{(m/s)} \\ \rm{EKP}\end{array}$',...
                                     '$\begin{array}{c}w\rm{(m/s)} \\ \rm{SP}\end{array}$',...
                                     '$\begin{array}{c}w\rm{(m/s)} \\ \rm{KP}\end{array}$',...
                                     '$\begin{array}{c}w\rm{(m/s)} \\ \rm{EKP}\end{array}$'});
set(gca, 'View', View); set(gca, 'FontSize', fs);
zscale=0.025;
text(1-0.25,1, uwdata(1,1)+zscale,sprintf('%0.1f',uwdata(1,1)),'HorizontalAlignment','left', 'FontSize', fs)
text(1-0.25,2, uwdata(2,1)+zscale,sprintf('%0.1f',uwdata(2,1)),'HorizontalAlignment','left', 'FontSize', fs)
text(1-0.25,3, uwdata(3,1)+zscale,sprintf('%0.1f',uwdata(3,1)),'HorizontalAlignment','left', 'FontSize', fs)

text(2-0.25,1, uwdata(1,2)+zscale,sprintf('%0.1f',uwdata(1,2)),'HorizontalAlignment','left', 'FontSize', fs)
text(2-0.25,2, uwdata(2,2)+zscale,sprintf('%0.1f',uwdata(2,2)),'HorizontalAlignment','left', 'FontSize', fs)
text(2-0.25,3, uwdata(3,2)+zscale,sprintf('%0.1f',uwdata(3,2)),'HorizontalAlignment','left', 'FontSize', fs)

text(3-0.25,1, uwdata(1,3)+zscale,sprintf('%0.1f',uwdata(1,3)),'HorizontalAlignment','left', 'FontSize', fs)
text(3-0.25,2, uwdata(2,3)+zscale,sprintf('%0.1f',uwdata(2,3)),'HorizontalAlignment','left', 'FontSize', fs)
text(3-0.25,3, uwdata(3,3)+zscale,sprintf('%0.1f',uwdata(3,3)),'HorizontalAlignment','left', 'FontSize', fs)

text(4-0.25,1, uwdata(1,4)+zscale,sprintf('%0.1f',uwdata(1,4)),'HorizontalAlignment','left', 'FontSize', fs)
text(4-0.25,2, uwdata(2,4)+zscale,sprintf('%0.1f',uwdata(2,4)),'HorizontalAlignment','left', 'FontSize', fs)
text(4-0.25,3, uwdata(3,4)+zscale,sprintf('%0.1f',uwdata(3,4)),'HorizontalAlignment','left', 'FontSize', fs)

text(5-0.25,1, uwdata(1,5)+zscale,sprintf('%0.1f',uwdata(1,5)),'HorizontalAlignment','left', 'FontSize', fs)
text(5-0.25,2, uwdata(2,5)+zscale,sprintf('%0.1f',uwdata(2,5)),'HorizontalAlignment','left', 'FontSize', fs)
text(5-0.25,3, uwdata(3,5)+zscale,sprintf('%0.1f',uwdata(3,5)),'HorizontalAlignment','left', 'FontSize', fs)

text(6-0.25,1, uwdata(1,6)+zscale,sprintf('%0.1f',uwdata(1,6)),'HorizontalAlignment','left', 'FontSize', fs)
text(6-0.25,2, uwdata(2,6)+zscale,sprintf('%0.1f',uwdata(2,6)),'HorizontalAlignment','left', 'FontSize', fs)
text(6-0.25,3, uwdata(3,6)+zscale,sprintf('%0.1f',uwdata(3,6)),'HorizontalAlignment','left', 'FontSize', fs)
set(gcf, 'Position', Position)                               


% theta and q RMSE 
thetaqdata=[data16deg(3,1:3), data16deg(6,1:3); data9deg(3,1:3), data9deg(6,1:3); data2deg(3,1:3), data2deg(6,1:3)];
figure
nPlot = get(gcf,'Number');
filename{nPlot}=strcat('case_abc_',caseNF,'_1_C_thetaq');
thetaqbar=bar3(thetaqdata);
for k=1:length(thetaqbar)
    thetaqbar(k).FaceAlpha = '0.9';
    thetaqbar(k).EdgeAlpha = '0.8';
    if k==1 || k==4
        thetaqbar(k).FaceColor = ['#E899C5'];
    elseif k==2 || k==5
        thetaqbar(k).FaceColor = ['#FB892F'];
    elseif k==3 || k==6
        thetaqbar(k).FaceColor = ['#FFBC25'];
    end
end
set(gca,'ytick',[1:3],'yticklabel',{'$16^\circ$';'$9^\circ$';'$2^\circ$'});
set(gca, 'xtick',[1:6],'xticklabel',{'$\begin{array}{c}\theta\rm{(deg)} \\ \rm{SP}\end{array}$',...
                                     '$\begin{array}{c}\theta\rm{(deg)} \\ \rm{KP}\end{array}$',...
                                     '$\begin{array}{c}\theta\rm{(deg)} \\ \rm{EKP}\end{array}$',...
                                     '$\begin{array}{c}q\\ \rm{(deg/s)} \\ \rm{SP}\end{array}$',...
                                     '$\begin{array}{c}q\\ \rm{(deg/s)} \\ \rm{KP}\end{array}$',...
                                     '$\begin{array}{c}q\\ \rm{(deg/s)} \\ \rm{EKP}\end{array}$'});
set(gca, 'View', View); set(gca, 'FontSize', fs);
zscale=0.25;
text(1-0.25,1, thetaqdata(1,1)+zscale,sprintf('%0.1f',thetaqdata(1,1)),'HorizontalAlignment','left', 'FontSize', fs)
text(1-0.25,2, thetaqdata(2,1)+zscale,sprintf('%0.1f',thetaqdata(2,1)),'HorizontalAlignment','left', 'FontSize', fs)
text(1-0.25,3, thetaqdata(3,1)+zscale,sprintf('%0.1f',thetaqdata(3,1)),'HorizontalAlignment','left', 'FontSize', fs)

text(2-0.25,1, thetaqdata(1,2)+zscale,sprintf('%0.1f',thetaqdata(1,2)),'HorizontalAlignment','left', 'FontSize', fs)
text(2-0.25,2, thetaqdata(2,2)+zscale,sprintf('%0.1f',thetaqdata(2,2)),'HorizontalAlignment','left', 'FontSize', fs)
text(2-0.25,3, thetaqdata(3,2)+zscale,sprintf('%0.1f',thetaqdata(3,2)),'HorizontalAlignment','left', 'FontSize', fs)

text(3-0.25,1, thetaqdata(1,3)+zscale,sprintf('%0.1f',thetaqdata(1,3)),'HorizontalAlignment','left', 'FontSize', fs)
text(3-0.25,2, thetaqdata(2,3)+zscale,sprintf('%0.1f',thetaqdata(2,3)),'HorizontalAlignment','left', 'FontSize', fs)
text(3-0.25,3, thetaqdata(3,3)+zscale,sprintf('%0.1f',thetaqdata(3,3)),'HorizontalAlignment','left', 'FontSize', fs)

text(4-0.25,1, thetaqdata(1,4)+zscale,sprintf('%0.1f',thetaqdata(1,4)),'HorizontalAlignment','left', 'FontSize', fs)
text(4-0.25,2, thetaqdata(2,4)+zscale,sprintf('%0.1f',thetaqdata(2,4)),'HorizontalAlignment','left', 'FontSize', fs)
text(4-0.25,3, thetaqdata(3,4)+zscale,sprintf('%0.1f',thetaqdata(3,4)),'HorizontalAlignment','left', 'FontSize', fs)

text(5-0.25,1, thetaqdata(1,5)+zscale,sprintf('%0.1f',thetaqdata(1,5)),'HorizontalAlignment','left', 'FontSize', fs)
text(5-0.25,2, thetaqdata(2,5)+zscale,sprintf('%0.1f',thetaqdata(2,5)),'HorizontalAlignment','left', 'FontSize', fs)
text(5-0.25,3, thetaqdata(3,5)+zscale,sprintf('%0.1f',thetaqdata(3,5)),'HorizontalAlignment','left', 'FontSize', fs)

text(6-0.25,1, thetaqdata(1,6)+zscale,sprintf('%0.1f',thetaqdata(1,6)),'HorizontalAlignment','left', 'FontSize', fs)
text(6-0.25,2, thetaqdata(2,6)+zscale,sprintf('%0.1f',thetaqdata(2,6)),'HorizontalAlignment','left', 'FontSize', fs)
text(6-0.25,3, thetaqdata(3,6)+zscale,sprintf('%0.1f',thetaqdata(3,6)),'HorizontalAlignment','left', 'FontSize', fs)

set(gcf, 'Position', Position)
%% delay = 4s
XZdata=[data16deg(1,4:6), data16deg(2,4:6); data9deg(1,4:6), data9deg(2,4:6); data2deg(1,4:6), data2deg(2,4:6)];
figure
nPlot = get(gcf,'Number');
filename{nPlot}=strcat('case_abc_',caseNF,'_2_C_XZ');
XZbar=bar3(XZdata);
for k=1:length(XZbar)
    XZbar(k).FaceAlpha = '0.9';
    XZbar(k).EdgeAlpha = '0.8';
    if k==1 || k==4
        XZbar(k).FaceColor = ['#E899C5'];
    elseif k==2 || k==5
        XZbar(k).FaceColor = ['#FB892F'];
    elseif k==3 || k==6
        XZbar(k).FaceColor = ['#FFBC25'];
    end
end
zlabel({'$T_s=0.01$s','RMSE'})
set(gca,'ytick',[1:3],'yticklabel',{'$16^\circ$';'$9^\circ$';'$2^\circ$'});
set(gca, 'xtick',[1:6],'xticklabel',{'$\begin{array}{c}X\rm{(m)} \\ \rm{SP}\end{array}$',...
                                     '$\begin{array}{c}X\rm{(m)} \\ \rm{KP}\end{array}$',...
                                     '$\begin{array}{c}X\rm{(m)} \\ \rm{EKP}\end{array}$',...
                                     '$\begin{array}{c}Z\rm{(m)} \\ \rm{SP}\end{array}$',...
                                     '$\begin{array}{c}Z\rm{(m)} \\ \rm{KP}\end{array}$',...
                                     '$\begin{array}{c}Z\rm{(m)} \\ \rm{EKP}\end{array}$'});
set(gca, 'View', View); set(gca, 'FontSize', fs);
zscale=0.75;
text(1-0.25,1, XZdata(1,1)+zscale,sprintf('%0.1f',XZdata(1,1)),'HorizontalAlignment','left', 'FontSize', fs)
text(1-0.25,2, XZdata(2,1)+zscale,sprintf('%0.1f',XZdata(2,1)),'HorizontalAlignment','left', 'FontSize', fs)
text(1-0.25,3, XZdata(3,1)+zscale,sprintf('%0.1f',XZdata(3,1)),'HorizontalAlignment','left', 'FontSize', fs)

text(2-0.25,1, XZdata(1,2)+zscale,sprintf('%0.1f',XZdata(1,2)),'HorizontalAlignment','left', 'FontSize', fs)
text(2-0.25,2, XZdata(2,2)+zscale,sprintf('%0.1f',XZdata(2,2)),'HorizontalAlignment','left', 'FontSize', fs)
text(2-0.25,3, XZdata(3,2)+zscale,sprintf('%0.1f',XZdata(3,2)),'HorizontalAlignment','left', 'FontSize', fs)

text(3-0.25,1, XZdata(1,3)+zscale,sprintf('%0.1f',XZdata(1,3)),'HorizontalAlignment','left', 'FontSize', fs)
text(3-0.25,2, XZdata(2,3)+zscale,sprintf('%0.1f',XZdata(2,3)),'HorizontalAlignment','left', 'FontSize', fs)
text(3-0.25,3, XZdata(3,3)+zscale,sprintf('%0.1f',XZdata(3,3)),'HorizontalAlignment','left', 'FontSize', fs)

text(4-0.25,1, XZdata(1,4)+zscale,sprintf('%0.1f',XZdata(1,4)),'HorizontalAlignment','left', 'FontSize', fs)
text(4-0.25,2, XZdata(2,4)+zscale,sprintf('%0.1f',XZdata(2,4)),'HorizontalAlignment','left', 'FontSize', fs)
text(4-0.25,3, XZdata(3,4)+zscale,sprintf('%0.1f',XZdata(3,4)),'HorizontalAlignment','left', 'FontSize', fs)

text(5-0.25,1, XZdata(1,5)+zscale,sprintf('%0.1f',XZdata(1,5)),'HorizontalAlignment','left', 'FontSize', fs)
text(5-0.25,2, XZdata(2,5)+zscale,sprintf('%0.1f',XZdata(2,5)),'HorizontalAlignment','left', 'FontSize', fs)
text(5-0.25,3, XZdata(3,5)+zscale,sprintf('%0.1f',XZdata(3,5)),'HorizontalAlignment','left', 'FontSize', fs)

text(6-0.25,1, XZdata(1,6)+zscale,sprintf('%0.1f',XZdata(1,6)),'HorizontalAlignment','left', 'FontSize', fs)
text(6-0.25,2, XZdata(2,6)+zscale,sprintf('%0.1f',XZdata(2,6)),'HorizontalAlignment','left', 'FontSize', fs)
text(6-0.25,3, XZdata(3,6)+zscale,sprintf('%0.1f',XZdata(3,6)),'HorizontalAlignment','left', 'FontSize', fs)
   set(gcf, 'Position', Position)
   
% u and w plots RMSE                                  
uwdata=[data16deg(4,4:6), data16deg(5,4:6); data9deg(4,4:6), data9deg(5,4:6); data2deg(4,4:6), data2deg(5,4:6)];
figure
nPlot = get(gcf,'Number');
filename{nPlot}=strcat('case_abc_',caseNF,'_2_C_uw');
uwbar=bar3(uwdata);
for k=1:length(uwbar)
    uwbar(k).FaceAlpha = '0.9';
    uwbar(k).EdgeAlpha = '0.8';
    if k==1 || k==4
        uwbar(k).FaceColor = ['#E899C5'];
    elseif k==2 || k==5
        uwbar(k).FaceColor = ['#FB892F'];
    elseif k==3 || k==6
        uwbar(k).FaceColor = ['#FFBC25'];
    end
end

set(gca,'ytick',[1:3],'yticklabel',{'$16^\circ$';'$9^\circ$';'$2^\circ$'});
set(gca, 'xtick',[1:6],'xticklabel',{'$\begin{array}{c}u\rm{(m/s)} \\ \rm{SP}\end{array}$',...
                                     '$\begin{array}{c}u\rm{(m/s)} \\ \rm{KP}\end{array}$',...
                                     '$\begin{array}{c}u\rm{(m/s)} \\ \rm{EKP}\end{array}$',...
                                     '$\begin{array}{c}w\rm{(m/s)} \\ \rm{SP}\end{array}$',...
                                     '$\begin{array}{c}w\rm{(m/s)} \\ \rm{KP}\end{array}$',...
                                     '$\begin{array}{c}w\rm{(m/s)} \\ \rm{EKP}\end{array}$'});
set(gca, 'View', View); set(gca, 'FontSize', fs);
zscale=0.025;
text(1-0.25,1, uwdata(1,1)+zscale,sprintf('%0.1f',uwdata(1,1)),'HorizontalAlignment','left', 'FontSize', fs)
text(1-0.25,2, uwdata(2,1)+zscale,sprintf('%0.1f',uwdata(2,1)),'HorizontalAlignment','left', 'FontSize', fs)
text(1-0.25,3, uwdata(3,1)+zscale,sprintf('%0.1f',uwdata(3,1)),'HorizontalAlignment','left', 'FontSize', fs)

text(2-0.25,1, uwdata(1,2)+zscale,sprintf('%0.1f',uwdata(1,2)),'HorizontalAlignment','left', 'FontSize', fs)
text(2-0.25,2, uwdata(2,2)+zscale,sprintf('%0.1f',uwdata(2,2)),'HorizontalAlignment','left', 'FontSize', fs)
text(2-0.25,3, uwdata(3,2)+zscale,sprintf('%0.1f',uwdata(3,2)),'HorizontalAlignment','left', 'FontSize', fs)

text(3-0.25,1, uwdata(1,3)+zscale,sprintf('%0.1f',uwdata(1,3)),'HorizontalAlignment','left', 'FontSize', fs)
text(3-0.25,2, uwdata(2,3)+zscale,sprintf('%0.1f',uwdata(2,3)),'HorizontalAlignment','left', 'FontSize', fs)
text(3-0.25,3, uwdata(3,3)+zscale,sprintf('%0.1f',uwdata(3,3)),'HorizontalAlignment','left', 'FontSize', fs)

text(4-0.25,1, uwdata(1,4)+zscale,sprintf('%0.1f',uwdata(1,4)),'HorizontalAlignment','left', 'FontSize', fs)
text(4-0.25,2, uwdata(2,4)+zscale,sprintf('%0.1f',uwdata(2,4)),'HorizontalAlignment','left', 'FontSize', fs)
text(4-0.25,3, uwdata(3,4)+zscale,sprintf('%0.1f',uwdata(3,4)),'HorizontalAlignment','left', 'FontSize', fs)

text(5-0.25,1, uwdata(1,5)+zscale,sprintf('%0.1f',uwdata(1,5)),'HorizontalAlignment','left', 'FontSize', fs)
text(5-0.25,2, uwdata(2,5)+zscale,sprintf('%0.1f',uwdata(2,5)),'HorizontalAlignment','left', 'FontSize', fs)
text(5-0.25,3, uwdata(3,5)+zscale,sprintf('%0.1f',uwdata(3,5)),'HorizontalAlignment','left', 'FontSize', fs)

text(6-0.25,1, uwdata(1,6)+zscale,sprintf('%0.1f',uwdata(1,6)),'HorizontalAlignment','left', 'FontSize', fs)
text(6-0.25,2, uwdata(2,6)+zscale,sprintf('%0.1f',uwdata(2,6)),'HorizontalAlignment','left', 'FontSize', fs)
text(6-0.25,3, uwdata(3,6)+zscale,sprintf('%0.1f',uwdata(3,6)),'HorizontalAlignment','left', 'FontSize', fs)
set(gcf, 'Position', Position)                               


% theta and q RMSE 
thetaqdata=[data16deg(3,4:6), data16deg(6,4:6); data9deg(3,4:6), data9deg(6,4:6); data2deg(3,4:6), data2deg(6,4:6)];
figure
nPlot = get(gcf,'Number');
filename{nPlot}=strcat('case_abc_',caseNF,'_2_C_thetaq');
thetaqbar=bar3(thetaqdata);
for k=1:length(thetaqbar)
    thetaqbar(k).FaceAlpha = '0.9';
    thetaqbar(k).EdgeAlpha = '0.8';
    if k==1 || k==4
        thetaqbar(k).FaceColor = ['#E899C5'];
    elseif k==2 || k==5
        thetaqbar(k).FaceColor = ['#FB892F'];
    elseif k==3 || k==6
        thetaqbar(k).FaceColor = ['#FFBC25'];
    end
end
set(gca,'ytick',[1:3],'yticklabel',{'$16^\circ$';'$9^\circ$';'$2^\circ$'});
set(gca, 'xtick',[1:6],'xticklabel',{'$\begin{array}{c}\theta\rm{(deg)} \\ \rm{SP}\end{array}$',...
                                     '$\begin{array}{c}\theta\rm{(deg)} \\ \rm{KP}\end{array}$',...
                                     '$\begin{array}{c}\theta\rm{(deg)} \\ \rm{EKP}\end{array}$',...
                                     '$\begin{array}{c}q\\ \rm{(deg/s)} \\ \rm{SP}\end{array}$',...
                                     '$\begin{array}{c}q\\ \rm{(deg/s)} \\ \rm{KP}\end{array}$',...
                                     '$\begin{array}{c}q\\ \rm{(deg/s)} \\ \rm{EKP}\end{array}$'});
set(gca, 'View', View); set(gca, 'FontSize', fs);
zscale=0.25;
text(1-0.25,1, thetaqdata(1,1)+zscale,sprintf('%0.1f',thetaqdata(1,1)),'HorizontalAlignment','left', 'FontSize', fs)
text(1-0.25,2, thetaqdata(2,1)+zscale,sprintf('%0.1f',thetaqdata(2,1)),'HorizontalAlignment','left', 'FontSize', fs)
text(1-0.25,3, thetaqdata(3,1)+zscale,sprintf('%0.1f',thetaqdata(3,1)),'HorizontalAlignment','left', 'FontSize', fs)

text(2-0.25,1, thetaqdata(1,2)+zscale,sprintf('%0.1f',thetaqdata(1,2)),'HorizontalAlignment','left', 'FontSize', fs)
text(2-0.25,2, thetaqdata(2,2)+zscale,sprintf('%0.1f',thetaqdata(2,2)),'HorizontalAlignment','left', 'FontSize', fs)
text(2-0.25,3, thetaqdata(3,2)+zscale,sprintf('%0.1f',thetaqdata(3,2)),'HorizontalAlignment','left', 'FontSize', fs)

text(3-0.25,1, thetaqdata(1,3)+zscale,sprintf('%0.1f',thetaqdata(1,3)),'HorizontalAlignment','left', 'FontSize', fs)
text(3-0.25,2, thetaqdata(2,3)+zscale,sprintf('%0.1f',thetaqdata(2,3)),'HorizontalAlignment','left', 'FontSize', fs)
text(3-0.25,3, thetaqdata(3,3)+zscale,sprintf('%0.1f',thetaqdata(3,3)),'HorizontalAlignment','left', 'FontSize', fs)

text(4-0.25,1, thetaqdata(1,4)+zscale,sprintf('%0.1f',thetaqdata(1,4)),'HorizontalAlignment','left', 'FontSize', fs)
text(4-0.25,2, thetaqdata(2,4)+zscale,sprintf('%0.1f',thetaqdata(2,4)),'HorizontalAlignment','left', 'FontSize', fs)
text(4-0.25,3, thetaqdata(3,4)+zscale,sprintf('%0.1f',thetaqdata(3,4)),'HorizontalAlignment','left', 'FontSize', fs)

text(5-0.25,1, thetaqdata(1,5)+zscale,sprintf('%0.1f',thetaqdata(1,5)),'HorizontalAlignment','left', 'FontSize', fs)
text(5-0.25,2, thetaqdata(2,5)+zscale,sprintf('%0.1f',thetaqdata(2,5)),'HorizontalAlignment','left', 'FontSize', fs)
text(5-0.25,3, thetaqdata(3,5)+zscale,sprintf('%0.1f',thetaqdata(3,5)),'HorizontalAlignment','left', 'FontSize', fs)

text(6-0.25,1, thetaqdata(1,6)+zscale,sprintf('%0.1f',thetaqdata(1,6)),'HorizontalAlignment','left', 'FontSize', fs)
text(6-0.25,2, thetaqdata(2,6)+zscale,sprintf('%0.1f',thetaqdata(2,6)),'HorizontalAlignment','left', 'FontSize', fs)
text(6-0.25,3, thetaqdata(3,6)+zscale,sprintf('%0.1f',thetaqdata(3,6)),'HorizontalAlignment','left', 'FontSize', fs)
set(gcf, 'Position', Position)

%% Data for Ts=0.05;
Ts=0.05;

[data2deg,data9deg,data16deg]=getdata(Ts,NF,T);


%%
set(groot,'defaultAxesTickLabelInterpreter','latex');  
set(groot,'defaulttextinterpreter','latex');
set(groot,'defaultLegendInterpreter','latex');
x1=categorical({'$X$(m)','$Z$(m)'});
x1=reordercats(x1,{'$X$(m)','$Z$(m)'});
x2=categorical({'$u$(m/s)', '$w$(m/s)'});
x2=reordercats(x2,{'$u$(m/s)', '$w$(m/s)'});
x3=categorical({'$\theta$(deg)','$q$(deg/s)'});
x3=reordercats(x3,{'$\theta$(deg)','$q$(deg/s)'});

digits(2)
%Delay = 2s
XZdata=[data16deg(1,1:3), data16deg(2,1:3); data9deg(1,1:3), data9deg(2,1:3); data2deg(1,1:3), data2deg(2,1:3)];
figure
nPlot = get(gcf,'Number');
filename{nPlot}=strcat('case_abc_',caseNF,'_1_B_XZ');
XZbar=bar3(XZdata);
for k=1:length(XZbar)
    XZbar(k).FaceAlpha = '0.9';
    XZbar(k).EdgeAlpha = '0.8';
    if k==1 || k==4
        XZbar(k).FaceColor = ['#E899C5'];
    elseif k==2 || k==5
        XZbar(k).FaceColor = ['#FB892F'];
    elseif k==3 || k==6
        XZbar(k).FaceColor = ['#FFBC25'];
    end
end
zlabel({'$T_s=0.05$s','RMSE'})
set(gca,'ytick',[1:3],'yticklabel',{'$16^\circ$';'$9^\circ$';'$2^\circ$'});
set(gca, 'xtick',[1:6],'xticklabel',{'$\begin{array}{c}X\rm{(m)} \\ \rm{SP}\end{array}$',...
                                     '$\begin{array}{c}X\rm{(m)} \\ \rm{KP}\end{array}$',...
                                     '$\begin{array}{c}X\rm{(m)} \\ \rm{EKP}\end{array}$',...
                                     '$\begin{array}{c}Z\rm{(m)} \\ \rm{SP}\end{array}$',...
                                     '$\begin{array}{c}Z\rm{(m)} \\ \rm{KP}\end{array}$',...
                                     '$\begin{array}{c}Z\rm{(m)} \\ \rm{EKP}\end{array}$'});
set(gca, 'View', View); set(gca, 'FontSize', fs);
zscale=0.75;
text(1-0.25,1, XZdata(1,1)+zscale,sprintf('%0.1f',XZdata(1,1)),'HorizontalAlignment','left', 'FontSize', fs)
text(1-0.25,2, XZdata(2,1)+zscale,sprintf('%0.1f',XZdata(2,1)),'HorizontalAlignment','left', 'FontSize', fs)
text(1-0.25,3, XZdata(3,1)+zscale,sprintf('%0.1f',XZdata(3,1)),'HorizontalAlignment','left', 'FontSize', fs)

text(2-0.25,1, XZdata(1,2)+zscale,sprintf('%0.1f',XZdata(1,2)),'HorizontalAlignment','left', 'FontSize', fs)
text(2-0.25,2, XZdata(2,2)+zscale,sprintf('%0.1f',XZdata(2,2)),'HorizontalAlignment','left', 'FontSize', fs)
text(2-0.25,3, XZdata(3,2)+zscale,sprintf('%0.1f',XZdata(3,2)),'HorizontalAlignment','left', 'FontSize', fs)

text(3-0.25,1, XZdata(1,3)+zscale,sprintf('%0.1f',XZdata(1,3)),'HorizontalAlignment','left', 'FontSize', fs)
text(3-0.25,2, XZdata(2,3)+zscale,sprintf('%0.1f',XZdata(2,3)),'HorizontalAlignment','left', 'FontSize', fs)
text(3-0.25,3, XZdata(3,3)+zscale,sprintf('%0.1f',XZdata(3,3)),'HorizontalAlignment','left', 'FontSize', fs)

text(4-0.25,1, XZdata(1,4)+zscale,sprintf('%0.1f',XZdata(1,4)),'HorizontalAlignment','left', 'FontSize', fs)
text(4-0.25,2, XZdata(2,4)+zscale,sprintf('%0.1f',XZdata(2,4)),'HorizontalAlignment','left', 'FontSize', fs)
text(4-0.25,3, XZdata(3,4)+zscale,sprintf('%0.1f',XZdata(3,4)),'HorizontalAlignment','left', 'FontSize', fs)

text(5-0.25,1, XZdata(1,5)+zscale,sprintf('%0.1f',XZdata(1,5)),'HorizontalAlignment','left', 'FontSize', fs)
text(5-0.25,2, XZdata(2,5)+zscale,sprintf('%0.1f',XZdata(2,5)),'HorizontalAlignment','left', 'FontSize', fs)
text(5-0.25,3, XZdata(3,5)+zscale,sprintf('%0.1f',XZdata(3,5)),'HorizontalAlignment','left', 'FontSize', fs)

text(6-0.25,1, XZdata(1,6)+zscale,sprintf('%0.1f',XZdata(1,6)),'HorizontalAlignment','left', 'FontSize', fs)
text(6-0.25,2, XZdata(2,6)+zscale,sprintf('%0.1f',XZdata(2,6)),'HorizontalAlignment','left', 'FontSize', fs)
text(6-0.25,3, XZdata(3,6)+zscale,sprintf('%0.1f',XZdata(3,6)),'HorizontalAlignment','left', 'FontSize', fs)
set(gcf, 'Position', Position)

% u and w plots RMSE                                  
uwdata=[data16deg(4,1:3), data16deg(5,1:3); data9deg(4,1:3), data9deg(5,1:3); data2deg(4,1:3), data2deg(5,1:3)];
figure
nPlot = get(gcf,'Number');
filename{nPlot}=strcat('case_abc_',caseNF,'_1_B_uw');
uwbar=bar3(uwdata);
for k=1:length(uwbar)
    uwbar(k).FaceAlpha = '0.9';
    uwbar(k).EdgeAlpha = '0.8';
    if k==1 || k==4
        uwbar(k).FaceColor = ['#E899C5'];
    elseif k==2 || k==5
        uwbar(k).FaceColor = ['#FB892F'];
    elseif k==3 || k==6
        uwbar(k).FaceColor = ['#FFBC25'];
    end
end

set(gca,'ytick',[1:3],'yticklabel',{'$16^\circ$';'$9^\circ$';'$2^\circ$'});
set(gca, 'xtick',[1:6],'xticklabel',{'$\begin{array}{c}u\rm{(m/s)} \\ \rm{SP}\end{array}$',...
                                     '$\begin{array}{c}u\rm{(m/s)} \\ \rm{KP}\end{array}$',...
                                     '$\begin{array}{c}u\rm{(m/s)} \\ \rm{EKP}\end{array}$',...
                                     '$\begin{array}{c}w\rm{(m/s)} \\ \rm{SP}\end{array}$',...
                                     '$\begin{array}{c}w\rm{(m/s)} \\ \rm{KP}\end{array}$',...
                                     '$\begin{array}{c}w\rm{(m/s)} \\ \rm{EKP}\end{array}$'});
set(gca, 'View', View); set(gca, 'FontSize', fs);
zscale=0.025;
text(1-0.25,1, uwdata(1,1)+zscale,sprintf('%0.1f',uwdata(1,1)),'HorizontalAlignment','left', 'FontSize', fs)
text(1-0.25,2, uwdata(2,1)+zscale,sprintf('%0.1f',uwdata(2,1)),'HorizontalAlignment','left', 'FontSize', fs)
text(1-0.25,3, uwdata(3,1)+zscale,sprintf('%0.1f',uwdata(3,1)),'HorizontalAlignment','left', 'FontSize', fs)

text(2-0.25,1, uwdata(1,2)+zscale,sprintf('%0.1f',uwdata(1,2)),'HorizontalAlignment','left', 'FontSize', fs)
text(2-0.25,2, uwdata(2,2)+zscale,sprintf('%0.1f',uwdata(2,2)),'HorizontalAlignment','left', 'FontSize', fs)
text(2-0.25,3, uwdata(3,2)+zscale,sprintf('%0.1f',uwdata(3,2)),'HorizontalAlignment','left', 'FontSize', fs)

text(3-0.25,1, uwdata(1,3)+zscale,sprintf('%0.1f',uwdata(1,3)),'HorizontalAlignment','left', 'FontSize', fs)
text(3-0.25,2, uwdata(2,3)+zscale,sprintf('%0.1f',uwdata(2,3)),'HorizontalAlignment','left', 'FontSize', fs)
text(3-0.25,3, uwdata(3,3)+zscale,sprintf('%0.1f',uwdata(3,3)),'HorizontalAlignment','left', 'FontSize', fs)

text(4-0.25,1, uwdata(1,4)+zscale,sprintf('%0.1f',uwdata(1,4)),'HorizontalAlignment','left', 'FontSize', fs)
text(4-0.25,2, uwdata(2,4)+zscale,sprintf('%0.1f',uwdata(2,4)),'HorizontalAlignment','left', 'FontSize', fs)
text(4-0.25,3, uwdata(3,4)+zscale,sprintf('%0.1f',uwdata(3,4)),'HorizontalAlignment','left', 'FontSize', fs)

text(5-0.25,1, uwdata(1,5)+zscale,sprintf('%0.1f',uwdata(1,5)),'HorizontalAlignment','left', 'FontSize', fs)
text(5-0.25,2, uwdata(2,5)+zscale,sprintf('%0.1f',uwdata(2,5)),'HorizontalAlignment','left', 'FontSize', fs)
text(5-0.25,3, uwdata(3,5)+zscale,sprintf('%0.1f',uwdata(3,5)),'HorizontalAlignment','left', 'FontSize', fs)

text(6-0.25,1, uwdata(1,6)+zscale,sprintf('%0.1f',uwdata(1,6)),'HorizontalAlignment','left', 'FontSize', fs)
text(6-0.25,2, uwdata(2,6)+zscale,sprintf('%0.1f',uwdata(2,6)),'HorizontalAlignment','left', 'FontSize', fs)
text(6-0.25,3, uwdata(3,6)+zscale,sprintf('%0.1f',uwdata(3,6)),'HorizontalAlignment','left', 'FontSize', fs)
set(gcf, 'Position', Position)                               


% theta and q RMSE 
thetaqdata=[data16deg(3,1:3), data16deg(6,1:3); data9deg(3,1:3), data9deg(6,1:3); data2deg(3,1:3), data2deg(6,1:3)];
figure
nPlot = get(gcf,'Number');
filename{nPlot}=strcat('case_abc_',caseNF,'_1_B_thetaq');
thetaqbar=bar3(thetaqdata);
for k=1:length(thetaqbar)
    thetaqbar(k).FaceAlpha = '0.9';
    thetaqbar(k).EdgeAlpha = '0.8';
    if k==1 || k==4
        thetaqbar(k).FaceColor = ['#E899C5'];
    elseif k==2 || k==5
        thetaqbar(k).FaceColor = ['#FB892F'];
    elseif k==3 || k==6
        thetaqbar(k).FaceColor = ['#FFBC25'];
    end
end
set(gca,'ytick',[1:3],'yticklabel',{'$16^\circ$';'$9^\circ$';'$2^\circ$'});
set(gca, 'xtick',[1:6],'xticklabel',{'$\begin{array}{c}\theta\rm{(deg)} \\ \rm{SP}\end{array}$',...
                                     '$\begin{array}{c}\theta\rm{(deg)} \\ \rm{KP}\end{array}$',...
                                     '$\begin{array}{c}\theta\rm{(deg)} \\ \rm{EKP}\end{array}$',...
                                     '$\begin{array}{c}q\\ \rm{(deg/s)} \\ \rm{SP}\end{array}$',...
                                     '$\begin{array}{c}q\\ \rm{(deg/s)} \\ \rm{KP}\end{array}$',...
                                     '$\begin{array}{c}q\\ \rm{(deg/s)} \\ \rm{EKP}\end{array}$'});
set(gca, 'View', View); set(gca, 'FontSize', fs);
zscale=0.25;
text(1-0.25,1, thetaqdata(1,1)+zscale,sprintf('%0.1f',thetaqdata(1,1)),'HorizontalAlignment','left', 'FontSize', fs)
text(1-0.25,2, thetaqdata(2,1)+zscale,sprintf('%0.1f',thetaqdata(2,1)),'HorizontalAlignment','left', 'FontSize', fs)
text(1-0.25,3, thetaqdata(3,1)+zscale,sprintf('%0.1f',thetaqdata(3,1)),'HorizontalAlignment','left', 'FontSize', fs)

text(2-0.25,1, thetaqdata(1,2)+zscale,sprintf('%0.1f',thetaqdata(1,2)),'HorizontalAlignment','left', 'FontSize', fs)
text(2-0.25,2, thetaqdata(2,2)+zscale,sprintf('%0.1f',thetaqdata(2,2)),'HorizontalAlignment','left', 'FontSize', fs)
text(2-0.25,3, thetaqdata(3,2)+zscale,sprintf('%0.1f',thetaqdata(3,2)),'HorizontalAlignment','left', 'FontSize', fs)

text(3-0.25,1, thetaqdata(1,3)+zscale,sprintf('%0.1f',thetaqdata(1,3)),'HorizontalAlignment','left', 'FontSize', fs)
text(3-0.25,2, thetaqdata(2,3)+zscale,sprintf('%0.1f',thetaqdata(2,3)),'HorizontalAlignment','left', 'FontSize', fs)
text(3-0.25,3, thetaqdata(3,3)+zscale,sprintf('%0.1f',thetaqdata(3,3)),'HorizontalAlignment','left', 'FontSize', fs)

text(4-0.25,1, thetaqdata(1,4)+zscale,sprintf('%0.1f',thetaqdata(1,4)),'HorizontalAlignment','left', 'FontSize', fs)
text(4-0.25,2, thetaqdata(2,4)+zscale,sprintf('%0.1f',thetaqdata(2,4)),'HorizontalAlignment','left', 'FontSize', fs)
text(4-0.25,3, thetaqdata(3,4)+zscale,sprintf('%0.1f',thetaqdata(3,4)),'HorizontalAlignment','left', 'FontSize', fs)

text(5-0.25,1, thetaqdata(1,5)+zscale,sprintf('%0.1f',thetaqdata(1,5)),'HorizontalAlignment','left', 'FontSize', fs)
text(5-0.25,2, thetaqdata(2,5)+zscale,sprintf('%0.1f',thetaqdata(2,5)),'HorizontalAlignment','left', 'FontSize', fs)
text(5-0.25,3, thetaqdata(3,5)+zscale,sprintf('%0.1f',thetaqdata(3,5)),'HorizontalAlignment','left', 'FontSize', fs)

text(6-0.25,1, thetaqdata(1,6)+zscale,sprintf('%0.1f',thetaqdata(1,6)),'HorizontalAlignment','left', 'FontSize', fs)
text(6-0.25,2, thetaqdata(2,6)+zscale,sprintf('%0.1f',thetaqdata(2,6)),'HorizontalAlignment','left', 'FontSize', fs)
text(6-0.25,3, thetaqdata(3,6)+zscale,sprintf('%0.1f',thetaqdata(3,6)),'HorizontalAlignment','left', 'FontSize', fs)
set(gcf, 'Position', Position)

%% delay = 4s
XZdata=[data16deg(1,4:6), data16deg(2,4:6); data9deg(1,4:6), data9deg(2,4:6); data2deg(1,4:6), data2deg(2,4:6)];
figure
nPlot = get(gcf,'Number');
filename{nPlot}=strcat('case_abc_',caseNF,'_2_B_XZ');
XZbar=bar3(XZdata);
for k=1:length(XZbar)
    XZbar(k).FaceAlpha = '0.9';
    XZbar(k).EdgeAlpha = '0.8';
    if k==1 || k==4
        XZbar(k).FaceColor = ['#E899C5'];
    elseif k==2 || k==5
        XZbar(k).FaceColor = ['#FB892F'];
    elseif k==3 || k==6
        XZbar(k).FaceColor = ['#FFBC25'];
    end
end
zlabel({'$T_s=0.05$s','RMSE'})
set(gca,'ytick',[1:3],'yticklabel',{'$16^\circ$';'$9^\circ$';'$2^\circ$'});
set(gca, 'xtick',[1:6],'xticklabel',{'$\begin{array}{c}X\rm{(m)} \\ \rm{SP}\end{array}$',...
                                     '$\begin{array}{c}X\rm{(m)} \\ \rm{KP}\end{array}$',...
                                     '$\begin{array}{c}X\rm{(m)} \\ \rm{EKP}\end{array}$',...
                                     '$\begin{array}{c}Z\rm{(m)} \\ \rm{SP}\end{array}$',...
                                     '$\begin{array}{c}Z\rm{(m)} \\ \rm{KP}\end{array}$',...
                                     '$\begin{array}{c}Z\rm{(m)} \\ \rm{EKP}\end{array}$'});
set(gca, 'View', View); set(gca, 'FontSize', fs);
zscale=0.75;
text(1-0.25,1, XZdata(1,1)+zscale,sprintf('%0.1f',XZdata(1,1)),'HorizontalAlignment','left', 'FontSize', fs)
text(1-0.25,2, XZdata(2,1)+zscale,sprintf('%0.1f',XZdata(2,1)),'HorizontalAlignment','left', 'FontSize', fs)
text(1-0.25,3, XZdata(3,1)+zscale,sprintf('%0.1f',XZdata(3,1)),'HorizontalAlignment','left', 'FontSize', fs)

text(2-0.25,1, XZdata(1,2)+zscale,sprintf('%0.1f',XZdata(1,2)),'HorizontalAlignment','left', 'FontSize', fs)
text(2-0.25,2, XZdata(2,2)+zscale,sprintf('%0.1f',XZdata(2,2)),'HorizontalAlignment','left', 'FontSize', fs)
text(2-0.25,3, XZdata(3,2)+zscale,sprintf('%0.1f',XZdata(3,2)),'HorizontalAlignment','left', 'FontSize', fs)

text(3-0.25,1, XZdata(1,3)+zscale,sprintf('%0.1f',XZdata(1,3)),'HorizontalAlignment','left', 'FontSize', fs)
text(3-0.25,2, XZdata(2,3)+zscale,sprintf('%0.1f',XZdata(2,3)),'HorizontalAlignment','left', 'FontSize', fs)
text(3-0.25,3, XZdata(3,3)+zscale,sprintf('%0.1f',XZdata(3,3)),'HorizontalAlignment','left', 'FontSize', fs)

text(4-0.25,1, XZdata(1,4)+zscale,sprintf('%0.1f',XZdata(1,4)),'HorizontalAlignment','left', 'FontSize', fs)
text(4-0.25,2, XZdata(2,4)+zscale,sprintf('%0.1f',XZdata(2,4)),'HorizontalAlignment','left', 'FontSize', fs)
text(4-0.25,3, XZdata(3,4)+zscale,sprintf('%0.1f',XZdata(3,4)),'HorizontalAlignment','left', 'FontSize', fs)

text(5-0.25,1, XZdata(1,5)+zscale,sprintf('%0.1f',XZdata(1,5)),'HorizontalAlignment','left', 'FontSize', fs)
text(5-0.25,2, XZdata(2,5)+zscale,sprintf('%0.1f',XZdata(2,5)),'HorizontalAlignment','left', 'FontSize', fs)
text(5-0.25,3, XZdata(3,5)+zscale,sprintf('%0.1f',XZdata(3,5)),'HorizontalAlignment','left', 'FontSize', fs)

text(6-0.25,1, XZdata(1,6)+zscale,sprintf('%0.1f',XZdata(1,6)),'HorizontalAlignment','left', 'FontSize', fs)
text(6-0.25,2, XZdata(2,6)+zscale,sprintf('%0.1f',XZdata(2,6)),'HorizontalAlignment','left', 'FontSize', fs)
text(6-0.25,3, XZdata(3,6)+zscale,sprintf('%0.1f',XZdata(3,6)),'HorizontalAlignment','left', 'FontSize', fs)
set(gcf, 'Position', Position)

% u and w plots RMSE                                  
uwdata=[data16deg(4,4:6), data16deg(5,4:6); data9deg(4,4:6), data9deg(5,4:6); data2deg(4,4:6), data2deg(5,4:6)];
figure
nPlot = get(gcf,'Number');
filename{nPlot}=strcat('case_abc_',caseNF,'_2_B_uw');
uwbar=bar3(uwdata);
for k=1:length(uwbar)
    uwbar(k).FaceAlpha = '0.9';
    uwbar(k).EdgeAlpha = '0.8';
    if k==1 || k==4
        uwbar(k).FaceColor = ['#E899C5'];
    elseif k==2 || k==5
        uwbar(k).FaceColor = ['#FB892F'];
    elseif k==3 || k==6
        uwbar(k).FaceColor = ['#FFBC25'];
    end
end

set(gca,'ytick',[1:3],'yticklabel',{'$16^\circ$';'$9^\circ$';'$2^\circ$'});
set(gca, 'xtick',[1:6],'xticklabel',{'$\begin{array}{c}u\rm{(m/s)} \\ \rm{SP}\end{array}$',...
                                     '$\begin{array}{c}u\rm{(m/s)} \\ \rm{KP}\end{array}$',...
                                     '$\begin{array}{c}u\rm{(m/s)} \\ \rm{EKP}\end{array}$',...
                                     '$\begin{array}{c}w\rm{(m/s)} \\ \rm{SP}\end{array}$',...
                                     '$\begin{array}{c}w\rm{(m/s)} \\ \rm{KP}\end{array}$',...
                                     '$\begin{array}{c}w\rm{(m/s)} \\ \rm{EKP}\end{array}$'});
set(gca, 'View', View); set(gca, 'FontSize', fs);
zscale=0.025;
text(1-0.25,1, uwdata(1,1)+zscale,sprintf('%0.1f',uwdata(1,1)),'HorizontalAlignment','left', 'FontSize', fs)
text(1-0.25,2, uwdata(2,1)+zscale,sprintf('%0.1f',uwdata(2,1)),'HorizontalAlignment','left', 'FontSize', fs)
text(1-0.25,3, uwdata(3,1)+zscale,sprintf('%0.1f',uwdata(3,1)),'HorizontalAlignment','left', 'FontSize', fs)

text(2-0.25,1, uwdata(1,2)+zscale,sprintf('%0.1f',uwdata(1,2)),'HorizontalAlignment','left', 'FontSize', fs)
text(2-0.25,2, uwdata(2,2)+zscale,sprintf('%0.1f',uwdata(2,2)),'HorizontalAlignment','left', 'FontSize', fs)
text(2-0.25,3, uwdata(3,2)+zscale,sprintf('%0.1f',uwdata(3,2)),'HorizontalAlignment','left', 'FontSize', fs)

text(3-0.25,1, uwdata(1,3)+zscale,sprintf('%0.1f',uwdata(1,3)),'HorizontalAlignment','left', 'FontSize', fs)
text(3-0.25,2, uwdata(2,3)+zscale,sprintf('%0.1f',uwdata(2,3)),'HorizontalAlignment','left', 'FontSize', fs)
text(3-0.25,3, uwdata(3,3)+zscale,sprintf('%0.1f',uwdata(3,3)),'HorizontalAlignment','left', 'FontSize', fs)

text(4-0.25,1, uwdata(1,4)+zscale,sprintf('%0.1f',uwdata(1,4)),'HorizontalAlignment','left', 'FontSize', fs)
text(4-0.25,2, uwdata(2,4)+zscale,sprintf('%0.1f',uwdata(2,4)),'HorizontalAlignment','left', 'FontSize', fs)
text(4-0.25,3, uwdata(3,4)+zscale,sprintf('%0.1f',uwdata(3,4)),'HorizontalAlignment','left', 'FontSize', fs)

text(5-0.25,1, uwdata(1,5)+zscale,sprintf('%0.1f',uwdata(1,5)),'HorizontalAlignment','left', 'FontSize', fs)
text(5-0.25,2, uwdata(2,5)+zscale,sprintf('%0.1f',uwdata(2,5)),'HorizontalAlignment','left', 'FontSize', fs)
text(5-0.25,3, uwdata(3,5)+zscale,sprintf('%0.1f',uwdata(3,5)),'HorizontalAlignment','left', 'FontSize', fs)

text(6-0.25,1, uwdata(1,6)+zscale,sprintf('%0.1f',uwdata(1,6)),'HorizontalAlignment','left', 'FontSize', fs)
text(6-0.25,2, uwdata(2,6)+zscale,sprintf('%0.1f',uwdata(2,6)),'HorizontalAlignment','left', 'FontSize', fs)
text(6-0.25,3, uwdata(3,6)+zscale,sprintf('%0.1f',uwdata(3,6)),'HorizontalAlignment','left', 'FontSize', fs)
set(gcf, 'Position', Position)                               


% theta and q RMSE 
thetaqdata=[data16deg(3,4:6), data16deg(6,4:6); data9deg(3,4:6), data9deg(6,4:6); data2deg(3,4:6), data2deg(6,4:6)];
figure
nPlot = get(gcf,'Number');
filename{nPlot}=strcat('case_abc_',caseNF,'_2_B_thetaq');
thetaqbar=bar3(thetaqdata);
for k=1:length(thetaqbar)
    thetaqbar(k).FaceAlpha = '0.9';
    thetaqbar(k).EdgeAlpha = '0.8';
    if k==1 || k==4
        thetaqbar(k).FaceColor = ['#E899C5'];
    elseif k==2 || k==5
        thetaqbar(k).FaceColor = ['#FB892F'];
    elseif k==3 || k==6
        thetaqbar(k).FaceColor = ['#FFBC25'];
    end
end
set(gca,'ytick',[1:3],'yticklabel',{'$16^\circ$';'$9^\circ$';'$2^\circ$'});
set(gca, 'xtick',[1:6],'xticklabel',{'$\begin{array}{c}\theta\rm{(deg)} \\ \rm{SP}\end{array}$',...
                                     '$\begin{array}{c}\theta\rm{(deg)} \\ \rm{KP}\end{array}$',...
                                     '$\begin{array}{c}\theta\rm{(deg)} \\ \rm{EKP}\end{array}$',...
                                     '$\begin{array}{c}q\\ \rm{(deg/s)} \\ \rm{SP}\end{array}$',...
                                     '$\begin{array}{c}q\\ \rm{(deg/s)} \\ \rm{KP}\end{array}$',...
                                     '$\begin{array}{c}q\\ \rm{(deg/s)} \\ \rm{EKP}\end{array}$'});
set(gca, 'View', View); set(gca, 'FontSize', fs);
zscale=0.25;
text(1-0.25,1, thetaqdata(1,1)+zscale,sprintf('%0.1f',thetaqdata(1,1)),'HorizontalAlignment','left', 'FontSize', fs)
text(1-0.25,2, thetaqdata(2,1)+zscale,sprintf('%0.1f',thetaqdata(2,1)),'HorizontalAlignment','left', 'FontSize', fs)
text(1-0.25,3, thetaqdata(3,1)+zscale,sprintf('%0.1f',thetaqdata(3,1)),'HorizontalAlignment','left', 'FontSize', fs)

text(2-0.25,1, thetaqdata(1,2)+zscale,sprintf('%0.1f',thetaqdata(1,2)),'HorizontalAlignment','left', 'FontSize', fs)
text(2-0.25,2, thetaqdata(2,2)+zscale,sprintf('%0.1f',thetaqdata(2,2)),'HorizontalAlignment','left', 'FontSize', fs)
text(2-0.25,3, thetaqdata(3,2)+zscale,sprintf('%0.1f',thetaqdata(3,2)),'HorizontalAlignment','left', 'FontSize', fs)

text(3-0.25,1, thetaqdata(1,3)+zscale,sprintf('%0.1f',thetaqdata(1,3)),'HorizontalAlignment','left', 'FontSize', fs)
text(3-0.25,2, thetaqdata(2,3)+zscale,sprintf('%0.1f',thetaqdata(2,3)),'HorizontalAlignment','left', 'FontSize', fs)
text(3-0.25,3, thetaqdata(3,3)+zscale,sprintf('%0.1f',thetaqdata(3,3)),'HorizontalAlignment','left', 'FontSize', fs)

text(4-0.25,1, thetaqdata(1,4)+zscale,sprintf('%0.1f',thetaqdata(1,4)),'HorizontalAlignment','left', 'FontSize', fs)
text(4-0.25,2, thetaqdata(2,4)+zscale,sprintf('%0.1f',thetaqdata(2,4)),'HorizontalAlignment','left', 'FontSize', fs)
text(4-0.25,3, thetaqdata(3,4)+zscale,sprintf('%0.1f',thetaqdata(3,4)),'HorizontalAlignment','left', 'FontSize', fs)

text(5-0.25,1, thetaqdata(1,5)+zscale,sprintf('%0.1f',thetaqdata(1,5)),'HorizontalAlignment','left', 'FontSize', fs)
text(5-0.25,2, thetaqdata(2,5)+zscale,sprintf('%0.1f',thetaqdata(2,5)),'HorizontalAlignment','left', 'FontSize', fs)
text(5-0.25,3, thetaqdata(3,5)+zscale,sprintf('%0.1f',thetaqdata(3,5)),'HorizontalAlignment','left', 'FontSize', fs)

text(6-0.25,1, thetaqdata(1,6)+zscale,sprintf('%0.1f',thetaqdata(1,6)),'HorizontalAlignment','left', 'FontSize', fs)
text(6-0.25,2, thetaqdata(2,6)+zscale,sprintf('%0.1f',thetaqdata(2,6)),'HorizontalAlignment','left', 'FontSize', fs)
text(6-0.25,3, thetaqdata(3,6)+zscale,sprintf('%0.1f',thetaqdata(3,6)),'HorizontalAlignment','left', 'FontSize', fs)
set(gcf, 'Position', Position)

%% Data for Ts=0.1;
Ts=0.1;

[data2deg,data9deg,data16deg]=getdata(Ts,NF,T);


%%
set(groot,'defaultAxesTickLabelInterpreter','latex');  
set(groot,'defaulttextinterpreter','latex');
set(groot,'defaultLegendInterpreter','latex');
x1=categorical({'$X$(m)','$Z$(m)'});
x1=reordercats(x1,{'$X$(m)','$Z$(m)'});
x2=categorical({'$u$(m/s)', '$w$(m/s)'});
x2=reordercats(x2,{'$u$(m/s)', '$w$(m/s)'});
x3=categorical({'$\theta$(deg)','$q$(deg/s)'});
x3=reordercats(x3,{'$\theta$(deg)','$q$(deg/s)'});

digits(2)
XZdata=[data16deg(1,1:3), data16deg(2,1:3); data9deg(1,1:3), data9deg(2,1:3); data2deg(1,1:3), data2deg(2,1:3)];

figure
nPlot = get(gcf,'Number');
filename{nPlot}=strcat('case_abc_',caseNF,'_1_A_XZ');
XZbar=bar3(XZdata);
for k=1:length(XZbar)
    XZbar(k).FaceAlpha = '0.9';
    XZbar(k).EdgeAlpha = '0.8';
    if k==1 || k==4
        XZbar(k).FaceColor = ['#E899C5'];
    elseif k==2 || k==5
        XZbar(k).FaceColor = ['#FB892F'];
    elseif k==3 || k==6
        XZbar(k).FaceColor = ['#FFBC25'];
    end
end
zlabel({'$T_s=0.1$s','RMSE'})
set(gca,'ytick',[1:3],'yticklabel',{'$16^\circ$';'$9^\circ$';'$2^\circ$'});
set(gca, 'xtick',[1:6],'xticklabel',{'$\begin{array}{c}X\rm{(m)} \\ \rm{SP}\end{array}$',...
                                     '$\begin{array}{c}X\rm{(m)} \\ \rm{KP}\end{array}$',...
                                     '$\begin{array}{c}X\rm{(m)} \\ \rm{EKP}\end{array}$',...
                                     '$\begin{array}{c}Z\rm{(m)} \\ \rm{SP}\end{array}$',...
                                     '$\begin{array}{c}Z\rm{(m)} \\ \rm{KP}\end{array}$',...
                                     '$\begin{array}{c}Z\rm{(m)} \\ \rm{EKP}\end{array}$'});
set(gca, 'View', View); set(gca, 'FontSize', fs);
zscale=0.75;
text(1-0.25,1, XZdata(1,1)+zscale,sprintf('%0.1f',XZdata(1,1)),'HorizontalAlignment','left', 'FontSize', fs)
text(1-0.25,2, XZdata(2,1)+zscale,sprintf('%0.1f',XZdata(2,1)),'HorizontalAlignment','left', 'FontSize', fs)
text(1-0.25,3, XZdata(3,1)+zscale,sprintf('%0.1f',XZdata(3,1)),'HorizontalAlignment','left', 'FontSize', fs)

text(2-0.25,1, XZdata(1,2)+zscale,sprintf('%0.1f',XZdata(1,2)),'HorizontalAlignment','left', 'FontSize', fs)
text(2-0.25,2, XZdata(2,2)+zscale,sprintf('%0.1f',XZdata(2,2)),'HorizontalAlignment','left', 'FontSize', fs)
text(2-0.25,3, XZdata(3,2)+zscale,sprintf('%0.1f',XZdata(3,2)),'HorizontalAlignment','left', 'FontSize', fs)

text(3-0.25,1, XZdata(1,3)+zscale,sprintf('%0.1f',XZdata(1,3)),'HorizontalAlignment','left', 'FontSize', fs)
text(3-0.25,2, XZdata(2,3)+zscale,sprintf('%0.1f',XZdata(2,3)),'HorizontalAlignment','left', 'FontSize', fs)
text(3-0.25,3, XZdata(3,3)+zscale,sprintf('%0.1f',XZdata(3,3)),'HorizontalAlignment','left', 'FontSize', fs)

text(4-0.25,1, XZdata(1,4)+zscale,sprintf('%0.1f',XZdata(1,4)),'HorizontalAlignment','left', 'FontSize', fs)
text(4-0.25,2, XZdata(2,4)+zscale,sprintf('%0.1f',XZdata(2,4)),'HorizontalAlignment','left', 'FontSize', fs)
text(4-0.25,3, XZdata(3,4)+zscale,sprintf('%0.1f',XZdata(3,4)),'HorizontalAlignment','left', 'FontSize', fs)

text(5-0.25,1, XZdata(1,5)+zscale,sprintf('%0.1f',XZdata(1,5)),'HorizontalAlignment','left', 'FontSize', fs)
text(5-0.25,2, XZdata(2,5)+zscale,sprintf('%0.1f',XZdata(2,5)),'HorizontalAlignment','left', 'FontSize', fs)
text(5-0.25,3, XZdata(3,5)+zscale,sprintf('%0.1f',XZdata(3,5)),'HorizontalAlignment','left', 'FontSize', fs)

text(6-0.25,1, XZdata(1,6)+zscale,sprintf('%0.1f',XZdata(1,6)),'HorizontalAlignment','left', 'FontSize', fs)
text(6-0.25,2, XZdata(2,6)+zscale,sprintf('%0.1f',XZdata(2,6)),'HorizontalAlignment','left', 'FontSize', fs)
text(6-0.25,3, XZdata(3,6)+zscale,sprintf('%0.1f',XZdata(3,6)),'HorizontalAlignment','left', 'FontSize', fs)
set(gcf, 'Position', Position)

% u and w plots RMSE                                  
uwdata=[data16deg(4,1:3), data16deg(5,1:3); data9deg(4,1:3), data9deg(5,1:3); data2deg(4,1:3), data2deg(5,1:3)];
figure
nPlot = get(gcf,'Number');
filename{nPlot}=strcat('case_abc_',caseNF,'_1_A_uw');
uwbar=bar3(uwdata);
for k=1:length(uwbar)
    uwbar(k).FaceAlpha = '0.9';
    uwbar(k).EdgeAlpha = '0.8';
    if k==1 || k==4
        uwbar(k).FaceColor = ['#E899C5'];
    elseif k==2 || k==5
        uwbar(k).FaceColor = ['#FB892F'];
    elseif k==3 || k==6
        uwbar(k).FaceColor = ['#FFBC25'];
    end
end

set(gca,'ytick',[1:3],'yticklabel',{'$16^\circ$';'$9^\circ$';'$2^\circ$'});
set(gca, 'xtick',[1:6],'xticklabel',{'$\begin{array}{c}u\rm{(m/s)} \\ \rm{SP}\end{array}$',...
                                     '$\begin{array}{c}u\rm{(m/s)} \\ \rm{KP}\end{array}$',...
                                     '$\begin{array}{c}u\rm{(m/s)} \\ \rm{EKP}\end{array}$',...
                                     '$\begin{array}{c}w\rm{(m/s)} \\ \rm{SP}\end{array}$',...
                                     '$\begin{array}{c}w\rm{(m/s)} \\ \rm{KP}\end{array}$',...
                                     '$\begin{array}{c}w\rm{(m/s)} \\ \rm{EKP}\end{array}$'});
set(gca, 'View', View); set(gca, 'FontSize', fs);
zscale=0.025;
text(1-0.25,1, uwdata(1,1)+zscale,sprintf('%0.1f',uwdata(1,1)),'HorizontalAlignment','left', 'FontSize', fs)
text(1-0.25,2, uwdata(2,1)+zscale,sprintf('%0.1f',uwdata(2,1)),'HorizontalAlignment','left', 'FontSize', fs)
text(1-0.25,3, uwdata(3,1)+zscale,sprintf('%0.1f',uwdata(3,1)),'HorizontalAlignment','left', 'FontSize', fs)

text(2-0.25,1, uwdata(1,2)+zscale,sprintf('%0.1f',uwdata(1,2)),'HorizontalAlignment','left', 'FontSize', fs)
text(2-0.25,2, uwdata(2,2)+zscale,sprintf('%0.1f',uwdata(2,2)),'HorizontalAlignment','left', 'FontSize', fs)
text(2-0.25,3, uwdata(3,2)+zscale,sprintf('%0.1f',uwdata(3,2)),'HorizontalAlignment','left', 'FontSize', fs)

text(3-0.25,1, uwdata(1,3)+zscale,sprintf('%0.1f',uwdata(1,3)),'HorizontalAlignment','left', 'FontSize', fs)
text(3-0.25,2, uwdata(2,3)+zscale,sprintf('%0.1f',uwdata(2,3)),'HorizontalAlignment','left', 'FontSize', fs)
text(3-0.25,3, uwdata(3,3)+zscale,sprintf('%0.1f',uwdata(3,3)),'HorizontalAlignment','left', 'FontSize', fs)

text(4-0.25,1, uwdata(1,4)+zscale,sprintf('%0.1f',uwdata(1,4)),'HorizontalAlignment','left', 'FontSize', fs)
text(4-0.25,2, uwdata(2,4)+zscale,sprintf('%0.1f',uwdata(2,4)),'HorizontalAlignment','left', 'FontSize', fs)
text(4-0.25,3, uwdata(3,4)+zscale,sprintf('%0.1f',uwdata(3,4)),'HorizontalAlignment','left', 'FontSize', fs)

text(5-0.25,1, uwdata(1,5)+zscale,sprintf('%0.1f',uwdata(1,5)),'HorizontalAlignment','left', 'FontSize', fs)
text(5-0.25,2, uwdata(2,5)+zscale,sprintf('%0.1f',uwdata(2,5)),'HorizontalAlignment','left', 'FontSize', fs)
text(5-0.25,3, uwdata(3,5)+zscale,sprintf('%0.1f',uwdata(3,5)),'HorizontalAlignment','left', 'FontSize', fs)

text(6-0.25,1, uwdata(1,6)+zscale,sprintf('%0.1f',uwdata(1,6)),'HorizontalAlignment','left', 'FontSize', fs)
text(6-0.25,2, uwdata(2,6)+zscale,sprintf('%0.1f',uwdata(2,6)),'HorizontalAlignment','left', 'FontSize', fs)
text(6-0.25,3, uwdata(3,6)+zscale,sprintf('%0.1f',uwdata(3,6)),'HorizontalAlignment','left', 'FontSize', fs)
set(gcf, 'Position', Position)                               


% theta and q RMSE 
thetaqdata=[data16deg(3,1:3), data16deg(6,1:3); data9deg(3,1:3), data9deg(6,1:3); data2deg(3,1:3), data2deg(6,1:3)];
figure
nPlot = get(gcf,'Number');
filename{nPlot}=strcat('case_abc_',caseNF,'_1_A_thetaq');
thetaqbar=bar3(thetaqdata);
for k=1:length(thetaqbar)
    thetaqbar(k).FaceAlpha = '0.9';
    thetaqbar(k).EdgeAlpha = '0.8';
    if k==1 || k==4
        thetaqbar(k).FaceColor = ['#E899C5'];
    elseif k==2 || k==5
        thetaqbar(k).FaceColor = ['#FB892F'];
    elseif k==3 || k==6
        thetaqbar(k).FaceColor = ['#FFBC25'];
    end
end
set(gca,'ytick',[1:3],'yticklabel',{'$16^\circ$';'$9^\circ$';'$2^\circ$'});
set(gca, 'xtick',[1:6],'xticklabel',{'$\begin{array}{c}\theta\rm{(deg)} \\ \rm{SP}\end{array}$',...
                                     '$\begin{array}{c}\theta\rm{(deg)} \\ \rm{KP}\end{array}$',...
                                     '$\begin{array}{c}\theta\rm{(deg)} \\ \rm{EKP}\end{array}$',...
                                     '$\begin{array}{c}q\\ \rm{(deg/s)} \\ \rm{SP}\end{array}$',...
                                     '$\begin{array}{c}q\\ \rm{(deg/s)} \\ \rm{KP}\end{array}$',...
                                     '$\begin{array}{c}q\\ \rm{(deg/s)} \\ \rm{EKP}\end{array}$'});
set(gca, 'View', View); set(gca, 'FontSize', fs);
zscale=0.25;
text(1-0.25,1, thetaqdata(1,1)+zscale,sprintf('%0.1f',thetaqdata(1,1)),'HorizontalAlignment','left', 'FontSize', fs)
text(1-0.25,2, thetaqdata(2,1)+zscale,sprintf('%0.1f',thetaqdata(2,1)),'HorizontalAlignment','left', 'FontSize', fs)
text(1-0.25,3, thetaqdata(3,1)+zscale,sprintf('%0.1f',thetaqdata(3,1)),'HorizontalAlignment','left', 'FontSize', fs)

text(2-0.25,1, thetaqdata(1,2)+zscale,sprintf('%0.1f',thetaqdata(1,2)),'HorizontalAlignment','left', 'FontSize', fs)
text(2-0.25,2, thetaqdata(2,2)+zscale,sprintf('%0.1f',thetaqdata(2,2)),'HorizontalAlignment','left', 'FontSize', fs)
text(2-0.25,3, thetaqdata(3,2)+zscale,sprintf('%0.1f',thetaqdata(3,2)),'HorizontalAlignment','left', 'FontSize', fs)

text(3-0.25,1, thetaqdata(1,3)+zscale,sprintf('%0.1f',thetaqdata(1,3)),'HorizontalAlignment','left', 'FontSize', fs)
text(3-0.25,2, thetaqdata(2,3)+zscale,sprintf('%0.1f',thetaqdata(2,3)),'HorizontalAlignment','left', 'FontSize', fs)
text(3-0.25,3, thetaqdata(3,3)+zscale,sprintf('%0.1f',thetaqdata(3,3)),'HorizontalAlignment','left', 'FontSize', fs)

text(4-0.25,1, thetaqdata(1,4)+zscale,sprintf('%0.1f',thetaqdata(1,4)),'HorizontalAlignment','left', 'FontSize', fs)
text(4-0.25,2, thetaqdata(2,4)+zscale,sprintf('%0.1f',thetaqdata(2,4)),'HorizontalAlignment','left', 'FontSize', fs)
text(4-0.25,3, thetaqdata(3,4)+zscale,sprintf('%0.1f',thetaqdata(3,4)),'HorizontalAlignment','left', 'FontSize', fs)

text(5-0.25,1, thetaqdata(1,5)+zscale,sprintf('%0.1f',thetaqdata(1,5)),'HorizontalAlignment','left', 'FontSize', fs)
text(5-0.25,2, thetaqdata(2,5)+zscale,sprintf('%0.1f',thetaqdata(2,5)),'HorizontalAlignment','left', 'FontSize', fs)
text(5-0.25,3, thetaqdata(3,5)+zscale,sprintf('%0.1f',thetaqdata(3,5)),'HorizontalAlignment','left', 'FontSize', fs)

text(6-0.25,1, thetaqdata(1,6)+zscale,sprintf('%0.1f',thetaqdata(1,6)),'HorizontalAlignment','left', 'FontSize', fs)
text(6-0.25,2, thetaqdata(2,6)+zscale,sprintf('%0.1f',thetaqdata(2,6)),'HorizontalAlignment','left', 'FontSize', fs)
text(6-0.25,3, thetaqdata(3,6)+zscale,sprintf('%0.1f',thetaqdata(3,6)),'HorizontalAlignment','left', 'FontSize', fs)
set(gcf, 'Position', Position)                                

%% delay = 4s
XZdata=[data16deg(1,4:6), data16deg(2,4:6); data9deg(1,4:6), data9deg(2,4:6); data2deg(1,4:6), data2deg(2,4:6)];
figure
nPlot = get(gcf,'Number');
filename{nPlot}=strcat('case_abc_',caseNF,'_2_A_XZ');
XZbar=bar3(XZdata);
for k=1:length(XZbar)
    XZbar(k).FaceAlpha = '0.9';
    XZbar(k).EdgeAlpha = '0.8';
    if k==1 || k==4
        XZbar(k).FaceColor = ['#E899C5'];
    elseif k==2 || k==5
        XZbar(k).FaceColor = ['#FB892F'];
    elseif k==3 || k==6
        XZbar(k).FaceColor = ['#FFBC25'];
    end
end
zlabel({'$T_s=0.1$s','RMSE'})
set(gca,'ytick',[1:3],'yticklabel',{'$16^\circ$';'$9^\circ$';'$2^\circ$'});
set(gca, 'xtick',[1:6],'xticklabel',{'$\begin{array}{c}X\rm{(m)} \\ \rm{SP}\end{array}$',...
                                     '$\begin{array}{c}X\rm{(m)} \\ \rm{KP}\end{array}$',...
                                     '$\begin{array}{c}X\rm{(m)} \\ \rm{EKP}\end{array}$',...
                                     '$\begin{array}{c}Z\rm{(m)} \\ \rm{SP}\end{array}$',...
                                     '$\begin{array}{c}Z\rm{(m)} \\ \rm{KP}\end{array}$',...
                                     '$\begin{array}{c}Z\rm{(m)} \\ \rm{EKP}\end{array}$'});
set(gca, 'View', View); set(gca, 'FontSize', fs);
zscale=0.75;
text(1-0.25,1, XZdata(1,1)+zscale,sprintf('%0.1f',XZdata(1,1)),'HorizontalAlignment','left', 'FontSize', fs)
text(1-0.25,2, XZdata(2,1)+zscale,sprintf('%0.1f',XZdata(2,1)),'HorizontalAlignment','left', 'FontSize', fs)
text(1-0.25,3, XZdata(3,1)+zscale,sprintf('%0.1f',XZdata(3,1)),'HorizontalAlignment','left', 'FontSize', fs)

text(2-0.25,1, XZdata(1,2)+zscale,sprintf('%0.1f',XZdata(1,2)),'HorizontalAlignment','left', 'FontSize', fs)
text(2-0.25,2, XZdata(2,2)+zscale,sprintf('%0.1f',XZdata(2,2)),'HorizontalAlignment','left', 'FontSize', fs)
text(2-0.25,3, XZdata(3,2)+zscale,sprintf('%0.1f',XZdata(3,2)),'HorizontalAlignment','left', 'FontSize', fs)

text(3-0.25,1, XZdata(1,3)+zscale,sprintf('%0.1f',XZdata(1,3)),'HorizontalAlignment','left', 'FontSize', fs)
text(3-0.25,2, XZdata(2,3)+zscale,sprintf('%0.1f',XZdata(2,3)),'HorizontalAlignment','left', 'FontSize', fs)
text(3-0.25,3, XZdata(3,3)+zscale,sprintf('%0.1f',XZdata(3,3)),'HorizontalAlignment','left', 'FontSize', fs)

text(4-0.25,1, XZdata(1,4)+zscale,sprintf('%0.1f',XZdata(1,4)),'HorizontalAlignment','left', 'FontSize', fs)
text(4-0.25,2, XZdata(2,4)+zscale,sprintf('%0.1f',XZdata(2,4)),'HorizontalAlignment','left', 'FontSize', fs)
text(4-0.25,3, XZdata(3,4)+zscale,sprintf('%0.1f',XZdata(3,4)),'HorizontalAlignment','left', 'FontSize', fs)

text(5-0.25,1, XZdata(1,5)+zscale,sprintf('%0.1f',XZdata(1,5)),'HorizontalAlignment','left', 'FontSize', fs)
text(5-0.25,2, XZdata(2,5)+zscale,sprintf('%0.1f',XZdata(2,5)),'HorizontalAlignment','left', 'FontSize', fs)
text(5-0.25,3, XZdata(3,5)+zscale,sprintf('%0.1f',XZdata(3,5)),'HorizontalAlignment','left', 'FontSize', fs)

text(6-0.25,1, XZdata(1,6)+zscale,sprintf('%0.1f',XZdata(1,6)),'HorizontalAlignment','left', 'FontSize', fs)
text(6-0.25,2, XZdata(2,6)+zscale,sprintf('%0.1f',XZdata(2,6)),'HorizontalAlignment','left', 'FontSize', fs)
text(6-0.25,3, XZdata(3,6)+zscale,sprintf('%0.1f',XZdata(3,6)),'HorizontalAlignment','left', 'FontSize', fs)
set(gcf, 'Position', Position)

% u and w plots RMSE                                  
uwdata=[data16deg(4,4:6), data16deg(5,4:6); data9deg(4,4:6), data9deg(5,4:6); data2deg(4,4:6), data2deg(5,4:6)];
figure
nPlot = get(gcf,'Number');
filename{nPlot}=strcat('case_abc_',caseNF,'_2_A_uw');
uwbar=bar3(uwdata);
for k=1:length(uwbar)
    uwbar(k).FaceAlpha = '0.9';
    uwbar(k).EdgeAlpha = '0.8';
    if k==1 || k==4
        uwbar(k).FaceColor = ['#E899C5'];
    elseif k==2 || k==5
        uwbar(k).FaceColor = ['#FB892F'];
    elseif k==3 || k==6
        uwbar(k).FaceColor = ['#FFBC25'];
    end
end

set(gca,'ytick',[1:3],'yticklabel',{'$16^\circ$';'$9^\circ$';'$2^\circ$'});
set(gca, 'xtick',[1:6],'xticklabel',{'$\begin{array}{c}u\rm{(m/s)} \\ \rm{SP}\end{array}$',...
                                     '$\begin{array}{c}u\rm{(m/s)} \\ \rm{KP}\end{array}$',...
                                     '$\begin{array}{c}u\rm{(m/s)} \\ \rm{EKP}\end{array}$',...
                                     '$\begin{array}{c}w\rm{(m/s)} \\ \rm{SP}\end{array}$',...
                                     '$\begin{array}{c}w\rm{(m/s)} \\ \rm{KP}\end{array}$',...
                                     '$\begin{array}{c}w\rm{(m/s)} \\ \rm{EKP}\end{array}$'});
set(gca, 'View', View); set(gca, 'FontSize', fs);
zscale=0.02;
text(1-0.25,1, uwdata(1,1)+zscale,sprintf('%0.1f',uwdata(1,1)),'HorizontalAlignment','left', 'FontSize', fs)
text(1-0.25,2, uwdata(2,1)+zscale,sprintf('%0.1f',uwdata(2,1)),'HorizontalAlignment','left', 'FontSize', fs)
text(1-0.25,3, uwdata(3,1)+zscale,sprintf('%0.1f',uwdata(3,1)),'HorizontalAlignment','left', 'FontSize', fs)

text(2-0.25,1, uwdata(1,2)+zscale,sprintf('%0.1f',uwdata(1,2)),'HorizontalAlignment','left', 'FontSize', fs)
text(2-0.25,2, uwdata(2,2)+zscale,sprintf('%0.1f',uwdata(2,2)),'HorizontalAlignment','left', 'FontSize', fs)
text(2-0.25,3, uwdata(3,2)+zscale,sprintf('%0.1f',uwdata(3,2)),'HorizontalAlignment','left', 'FontSize', fs)

text(3-0.25,1, uwdata(1,3)+zscale,sprintf('%0.1f',uwdata(1,3)),'HorizontalAlignment','left', 'FontSize', fs)
text(3-0.25,2, uwdata(2,3)+zscale,sprintf('%0.1f',uwdata(2,3)),'HorizontalAlignment','left', 'FontSize', fs)
text(3-0.25,3, uwdata(3,3)+zscale,sprintf('%0.1f',uwdata(3,3)),'HorizontalAlignment','left', 'FontSize', fs)

text(4-0.25,1, uwdata(1,4)+zscale,sprintf('%0.1f',uwdata(1,4)),'HorizontalAlignment','left', 'FontSize', fs)
text(4-0.25,2, uwdata(2,4)+zscale,sprintf('%0.1f',uwdata(2,4)),'HorizontalAlignment','left', 'FontSize', fs)
text(4-0.25,3, uwdata(3,4)+zscale,sprintf('%0.1f',uwdata(3,4)),'HorizontalAlignment','left', 'FontSize', fs)

text(5-0.25,1, uwdata(1,5)+zscale,sprintf('%0.1f',uwdata(1,5)),'HorizontalAlignment','left', 'FontSize', fs)
text(5-0.25,2, uwdata(2,5)+zscale,sprintf('%0.1f',uwdata(2,5)),'HorizontalAlignment','left', 'FontSize', fs)
text(5-0.25,3, uwdata(3,5)+zscale,sprintf('%0.1f',uwdata(3,5)),'HorizontalAlignment','left', 'FontSize', fs)

text(6-0.25,1, uwdata(1,6)+zscale,sprintf('%0.1f',uwdata(1,6)),'HorizontalAlignment','left', 'FontSize', fs)
text(6-0.25,2, uwdata(2,6)+zscale,sprintf('%0.1f',uwdata(2,6)),'HorizontalAlignment','left', 'FontSize', fs)
text(6-0.25,3, uwdata(3,6)+zscale,sprintf('%0.1f',uwdata(3,6)),'HorizontalAlignment','left', 'FontSize', fs)
set(gcf, 'Position', Position)                               


% theta and q RMSE 
thetaqdata=[data16deg(3,4:6), data16deg(6,4:6); data9deg(3,4:6), data9deg(6,4:6); data2deg(3,4:6), data2deg(6,4:6)];
figure
nPlot = get(gcf,'Number');
filename{nPlot}=strcat('case_abc_',caseNF,'_2_A_thetaq');
thetaqbar=bar3(thetaqdata);
for k=1:length(thetaqbar)
    thetaqbar(k).FaceAlpha = '0.9';
    thetaqbar(k).EdgeAlpha = '0.8';
    if k==1 || k==4
        thetaqbar(k).FaceColor = ['#E899C5'];
    elseif k==2 || k==5
        thetaqbar(k).FaceColor = ['#FB892F'];
    elseif k==3 || k==6
        thetaqbar(k).FaceColor = ['#FFBC25'];
    end
end
set(gca,'ytick',[1:3],'yticklabel',{'$16^\circ$';'$9^\circ$';'$2^\circ$'});
set(gca, 'xtick',[1:6],'xticklabel',{'$\begin{array}{c}\theta\rm{(deg)} \\ \rm{SP}\end{array}$',...
                                     '$\begin{array}{c}\theta\rm{(deg)} \\ \rm{KP}\end{array}$',...
                                     '$\begin{array}{c}\theta\rm{(deg)} \\ \rm{EKP}\end{array}$',...
                                     '$\begin{array}{c}q\\ \rm{(deg/s)} \\ \rm{SP}\end{array}$',...
                                     '$\begin{array}{c}q\\ \rm{(deg/s)} \\ \rm{KP}\end{array}$',...
                                     '$\begin{array}{c}q\\ \rm{(deg/s)} \\ \rm{EKP}\end{array}$'});
set(gca, 'View', View); set(gca, 'FontSize', fs);
zscale=0.2;
text(1-0.25,1, thetaqdata(1,1)+zscale,sprintf('%0.1f',thetaqdata(1,1)),'HorizontalAlignment','left', 'FontSize', fs)
text(1-0.25,2, thetaqdata(2,1)+zscale,sprintf('%0.1f',thetaqdata(2,1)),'HorizontalAlignment','left', 'FontSize', fs)
text(1-0.25,3, thetaqdata(3,1)+zscale,sprintf('%0.1f',thetaqdata(3,1)),'HorizontalAlignment','left', 'FontSize', fs)

text(2-0.25,1, thetaqdata(1,2)+zscale,sprintf('%0.1f',thetaqdata(1,2)),'HorizontalAlignment','left', 'FontSize', fs)
text(2-0.25,2, thetaqdata(2,2)+zscale,sprintf('%0.1f',thetaqdata(2,2)),'HorizontalAlignment','left', 'FontSize', fs)
text(2-0.25,3, thetaqdata(3,2)+zscale,sprintf('%0.1f',thetaqdata(3,2)),'HorizontalAlignment','left', 'FontSize', fs)

text(3-0.25,1, thetaqdata(1,3)+zscale,sprintf('%0.1f',thetaqdata(1,3)),'HorizontalAlignment','left', 'FontSize', fs)
text(3-0.25,2, thetaqdata(2,3)+zscale,sprintf('%0.1f',thetaqdata(2,3)),'HorizontalAlignment','left', 'FontSize', fs)
text(3-0.25,3, thetaqdata(3,3)+zscale,sprintf('%0.1f',thetaqdata(3,3)),'HorizontalAlignment','left', 'FontSize', fs)

text(4-0.25,1, thetaqdata(1,4)+zscale,sprintf('%0.1f',thetaqdata(1,4)),'HorizontalAlignment','left', 'FontSize', fs)
text(4-0.25,2, thetaqdata(2,4)+zscale,sprintf('%0.1f',thetaqdata(2,4)),'HorizontalAlignment','left', 'FontSize', fs)
text(4-0.25,3, thetaqdata(3,4)+zscale,sprintf('%0.1f',thetaqdata(3,4)),'HorizontalAlignment','left', 'FontSize', fs)

text(5-0.25,1, thetaqdata(1,5)+zscale,sprintf('%0.1f',thetaqdata(1,5)),'HorizontalAlignment','left', 'FontSize', fs)
text(5-0.25,2, thetaqdata(2,5)+zscale,sprintf('%0.1f',thetaqdata(2,5)),'HorizontalAlignment','left', 'FontSize', fs)
text(5-0.25,3, thetaqdata(3,5)+zscale,sprintf('%0.1f',thetaqdata(3,5)),'HorizontalAlignment','left', 'FontSize', fs)

text(6-0.25,1, thetaqdata(1,6)+zscale,sprintf('%0.1f',thetaqdata(1,6)),'HorizontalAlignment','left', 'FontSize', fs)
text(6-0.25,2, thetaqdata(2,6)+zscale,sprintf('%0.1f',thetaqdata(2,6)),'HorizontalAlignment','left', 'FontSize', fs)
text(6-0.25,3, thetaqdata(3,6)+zscale,sprintf('%0.1f',thetaqdata(3,6)),'HorizontalAlignment','left', 'FontSize', fs)
set(gcf, 'Position', Position)

%% saving plots
cd 'C:\Users\zakiaahmed\Documents\GitHub\Predictor-Simulations\FixedWingDoublet\Figures and data for presentation\MatlabSimPlots\NEWRMSE'


for i=1:18
    file=filename{i};
    figure(i)
    savefig([file,'.fig'])
    eps=[file,'.eps'];
    exportgraphics(figure(i), eps, 'Resolution', 600)
    svg1=[file,'.svg'];
    saveas(figure(i), svg1)
    png=[file,'.png'];
    exportgraphics(figure(i), png, 'Resolution', 600)
end

% file1=['RMSE_NF',num2str(NF),'_del4'];
% figure(2)
% set(gcf, 'Position', get(0, 'Screensize'));
% savefig([file1,'.fig'])
% eps1=[file1,'.eps'];
% exportgraphics(figure(2), eps1, 'Resolution', 1000)
% png1=[file1,'.png'];
%exportgraphics(figure(2), png1, 'Resolution', 500)