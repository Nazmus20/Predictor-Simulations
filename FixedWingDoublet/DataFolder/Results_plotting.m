clc;
clear all;
close all;

set(groot,'defaultAxesTickLabelInterpreter','latex');  
set(groot,'defaulttextinterpreter','latex');
set(groot,'defaultLegendInterpreter','latex');

CaseNo=3;

filename=['Case',num2str(CaseNo)]; %Change the folder name you want to work in

curr_path = pwd;
addpath([curr_path,'\',filename])

load(['data_',num2str(1),'.mat']);

XUnd=data.undelayed_measurement(1,:);
ZUnd=-data.undelayed_measurement(3,:);
ThetaUnd=data.undelayed_measurement(5,:)*180/pi;
uUnd=data.undelayed_measurement(7,:);
wUnd=data.undelayed_measurement(9,:);
qUnd=data.undelayed_measurement(11,:)*180/pi;
tUnd=data.total_time;

%SP data
XSP=data.SP_output(1,:);
ZSP=-data.SP_output(3,:);
ThetaSP=data.SP_output(5,:)*180/pi;
uSP=data.SP_output(7,:);
wSP=data.SP_output(9,:);
qSP=data.SP_output(11,:)*180/pi;
tSP=data.SP_time;

%KP data 
XKP=data.KP_output(1,:);
ZKP=-data.KP_output(3,:);
ThetaKP=data.KP_output(5,:)*180/pi;
uKP=data.KP_output(7,:);
wKP=data.KP_output(9,:);
qKP=data.KP_output(11,:)*180/pi;
tKP=data.KP_time;

%EKP data
XEKP=data.EKP_output(1,:);
ZEKP=-data.EKP_output(3,:);
ThetaEKP=data.EKP_output(5,:)*180/pi;
uEKP=data.EKP_output(7,:);
wEKP=data.EKP_output(9,:);
qEKP=data.EKP_output(11,:)*180/pi;
tEKP=data.EKP_time;

figure
set(gcf, 'Position', [-1685 522 1094 810])
fs=12;
subplot(3,2,1)
plot(tUnd, XUnd, 'k', 'LineWidth', 1.5)
hold on
plot(tSP,XSP, 'r--', tKP, XKP, 'b--', tEKP, XEKP, 'g--', 'LineWidth', 1.25)
set(gca,'FontSize', fs)
xlabel('Time (s)')
ylabel('$X$(m)')
legend('Undelayed Measurement', 'SP', 'KP', 'EKP', 'Location', 'best')

subplot(3,2,2)
plot(tUnd, ZUnd, 'k', 'LineWidth', 1.5)
hold on
plot(tSP,ZSP, 'r--', tKP, ZKP, 'b--', tEKP, ZEKP, 'g--', 'LineWidth', 1.25)
set(gca,'FontSize', fs)
xlabel('Time (s)')
ylabel('$Z$(m)')

subplot(3,2,3)
plot(tUnd, ThetaUnd, 'k', 'LineWidth', 1.5)
hold on
plot(tSP,ThetaSP, 'r--', tKP, ThetaKP, 'b--', tEKP, ThetaEKP, 'g--', 'LineWidth', 1.25)
set(gca,'FontSize', fs)
xlabel('Time (s)')
ylabel('$\theta$(deg)')

subplot(3,2,4)
plot(tUnd, uUnd, 'k', 'LineWidth', 1.5)
hold
plot(tSP, uSP, 'r--', tKP, uKP, 'b--', tEKP, uEKP, 'g--', 'LineWidth', 1.25)
set(gca,'FontSize', fs)
xlabel('Time (s)')
ylabel('$u$(m/s)')

subplot(3,2,5)
plot(tUnd, wUnd, 'k', 'LineWidth', 1.5)
hold on
plot(tSP, wSP, 'r--', tKP, wKP, 'b--', tEKP, wEKP, 'g--', 'LineWidth', 1.25)
set(gca,'FontSize', fs)
xlabel('Time (s)')
ylabel('$w$(m/s)')

subplot(3,2,6)
plot(tUnd, qUnd, 'k', 'LineWidth', 1.5)
hold on
plot(tSP, qSP, 'r--', tKP, qKP, 'b--', tEKP, qEKP, 'g--', 'LineWidth', 1.25)
set(gca,'FontSize', fs)
xlabel('Time (s)')
ylabel('$q$(deg/s)')

%Plotting
cd 'C:\Users\zakiaahmed\Documents\GitHub\Predictor-Simulations\FixedWingDoublet\Figures and data for presentation\MatlabSimPlots'

file1=['StatePlot_Tsp1_NF1_del2'];

savefig([file1,'.fig'])
eps1=[file1,'.eps'];
exportgraphics(figure(1), eps1, 'Resolution', 1000)
png1=[file1,'.png'];
exportgraphics(figure(1), png1, 'Resolution', 500)