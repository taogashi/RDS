%original algorithm

clear;
% close all;
%% preproccess
%load data
load matlab.mat;

MHS = atan2(magy,magx);
dMHS = 100*(MHS(2:end)-MHS(1:end-1));
index = find(abs(dMHS)>6*pi);
dMHS(index) = dMHS(index-1);

Gabs = sqrt(gx.^2+gy.^2+gz.^2);
inc = asin(sqrt(gx.^2+gy.^2)./Gabs);
figure;
plot(inc*57.3);
ylabel('angle (degree)');
grid on

temp = atan2(gy,gx);
% temp(temp<0) = temp(temp<0) + 2*pi;
incmhs = temp-MHS;
incmhs(incmhs<-pi) = incmhs(incmhs<-pi) + 2*pi;
myINCMHS=incmhs;
for i=2:length(myINCMHS)
    myINCMHS(i) = 0.99*myINCMHS(i-1)+0.01*myINCMHS(i);
end
figure;
plot(incmhs*57.3);
%%
% figure(2);
% center = zeros(2,50);
% for i=1:50
% %     hold off;
% %     plot(magx(4000+1+(i-1)*200:4000+i*200),magy(4000+1+(i-1)*200:4000+i*200));
%     [center(1,i) center(2,i) R a] = circfit(magx(4000+1+(i-1)*200:4000+i*200),magy(4000+1+(i-1)*200:4000+i*200));
%     hold on;
%     plot(center(1,i),center(2,i),'ro');
%     grid on;
%     axis equal;
%     pause(1);
% end