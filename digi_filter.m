clear;
close all;
%digital filter implemention
%1. load data--------------------------------------
raw_data = xlsread('inc5az70.xlsx');

%2. do data calibration------------------------------
%mag
raw_data(:,1) = raw_data(:,1)-35;
raw_data(:,2) = raw_data(:,2)-43;
raw_data(:,3) = raw_data(:,3)-98;
raw_data(:,4) = raw_data(:,4)-80;
raw_data(:,5) = raw_data(:,5)-18;
raw_data(:,6) = (raw_data(:,6)-123)*1.062;
raw_data(:,7) = -raw_data(:,7);
raw_data(:,8) = raw_data(:,8);
%acc
raw_data(:,9) = raw_data(:,9)*1.105-78;
raw_data(:,10) = raw_data(:,10)-60;
raw_data(:,11) = raw_data(:,11)*1.03-60;
raw_data(:,12) = raw_data(:,12)*1.03-60;
raw_data(:,13) = -(raw_data(:,13)-60);  %z axis inversed

gx = (raw_data(:,9)-raw_data(:,10))/2;
gy = (raw_data(:,11)-raw_data(:,12))/2;
gz = raw_data(:,13);

magx_ratio = [1/3 1/3 1/3];
magy_ratio = [1/3 1/3 1/3];
magz_ratio = [0 1];

magx = magx_ratio*raw_data(:,1:3)';
magy = magy_ratio*raw_data(:,4:6)';
magz = magz_ratio*raw_data(:,7:8)';

%generate low pass filter---------------------------------------
[b,a] = cheby1(10,0.5,0.8);
% uncomment this section to see the visualized result of low pass filter
% figure;
% plot(gx);
% hold on;
% gx=lowPassProccess(b,a,gx,100);
% plot(gx,'r');

% %calculate INC-----------------------------------------------
% Gabs = sqrt(gx.^2+gy.^2+gz.^2);
% inc = asin(sqrt(gx.^2+gy.^2)./Gabs);
% figure;
% plot(inc*57.3);

% %apply low pass filter to raw acc data----------------------------------
% gx=lowPassProccess(b,a,gx,100);
% gy=lowPassProccess(b,a,gy,100);
% gz=lowPassProccess(b,a,gz,100);
% %apply low pass filter to raw mag data----------------------------------
% magx=lowPassProccess(b,a,magx,100);
% magy=lowPassProccess(b,a,magy,100);
% magz=lowPassProccess(b,a,magz,100);

%re-generate low pass filter--------------------------------------
[b,a] = cheby1(4,0.5,0.01);

%calculate INC-----------------------------------------------
Gabs = sqrt(gx.^2+gy.^2+gz.^2);
inc = asin(sqrt(gx.^2+gy.^2)./Gabs);
% uncomment this section to see the visualized result of low pass filter
figure;
plot(inc*57.3);
hold on;
inc=lowPassProccess(b,a,inc,100);
plot(inc*57.3,'r');

% hold on;
% plot(inc*57.3,'r');

MHS = atan2(magy,magx);
temp = atan2(gy,gx);
incmhs = temp-MHS';
incmhs(incmhs<-pi) = incmhs(incmhs<-pi) + 2*pi;
incmhs(incmhs>pi) = incmhs(incmhs>pi) - 2*pi;
for i=1:length(incmhs)-1
    if abs(incmhs(i+1)-incmhs(i))>pi
        incmhs(i+1)=incmhs(i);
    end
end
% uncomment this section to see the visualized result of low pass filter
figure;
plot(incmhs*57.3);
hold on;
incmhs=lowPassProccess(b,a,incmhs,100);
plot(incmhs*57.3,'r');

