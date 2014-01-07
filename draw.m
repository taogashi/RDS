% data organized in
% AX.txt AY.txt AZ.txt FX.txt FY.txt FZ.txt
% sampled at 100Hz
clear;
% close all;
%% preproccess
%load data
% gx = load('AX.txt');
% gy = load('AY.txt');
% gz = load('AZ.txt');
% magx = load('FX.txt');
% magy = load('FY.txt');
% magz = load('FZ.txt');
% 
% mhs = load('MHS.txt');
% 
% gx = gx(82501:102500);
% gy = gy(82501:102500);
% gz = gz(82501:102500);
% magx = magx(82501:102500);
% magy = magy(82501:102500);
% magz = magz(82501:102500);
% 
% mhs = mhs(82501:102500);
% 
% save;
load matlab.mat;

figure;
subplot(2,1,1);
plot(gx);
hold on;
plot(gy,'r');
plot(gz,'g');
grid on;
legend('ax','ay','az');
subplot(2,1,2);
plot(magx);
hold on;
plot(magy,'r');
plot(magz,'g');
grid on;
legend('mx','my','mz');

%% draw ref
figure;
data = textread('INCMHS&INC.txt');
plot(data(82:102,1:4));

