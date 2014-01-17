%attitude determination in rotating drilling system

clear;
close all;
%% preproccess
%load data
load matlab.mat;
% matlab.mat 必须包含三个磁通门数据magx magy
% magz，均为单列，20000行以上的数据，必须包含三个加速度计的数据gx,gy,gz，均为单列，20000行以上的数据

%直接套公式，计算磁工具面角MHS
MHS = atan2(magy,magx);
%对MHS进行差分，得到角速率，作为钻铤旋转角速率的观测
dMHS = 100*(MHS(2:end)-MHS(1:end-1));
%差分运算会引入噪声，需要把野值剔除，这里我们认为超过6*pi rad/s的角速率为野值
index = find(abs(dMHS)>6*pi);
dMHS(index) = dMHS(index-1);
%调整符号，因为直接差分出来的角速率符号是反的
dMHS = -dMHS;

%% set parameters
sampleNum=19000;    %用于设定仿真总步数，必须小于数据的行数
dt=0.01;            %采样周期，与实际情况一致，这里的采样率为100Hz

%filter parameters
P=diag([0.007 0.007 0.007 0.007 0.1])^2;    %估计误差协方差阵
Q=diag([0.002 0.002 0.002 0.002 0.06])^2;   %过程噪声协方差阵
R=diag([0.9 0.9 0.8 0.003 0.003 0.003 0.25])^2; %观测噪声协方差阵

%% alignment
alignNum = 1:1000;      %用前面处于静止状态的1000组数据来进行初始化
x=zeros(5,1);           %状态变量，5维，包括姿态四元数和旋转角速率

av_acc=[gx(alignNum),gy(alignNum),gz(alignNum)];
av_acc = mean(av_acc);  %前面1000组数据的均值
gnorm = norm(av_acc);   %重力加速度的模，用于将加速度计的单位换算为g
av_acc = av_acc/gnorm;
roll = atan2(-av_acc(2),-av_acc(3));    %初始横滚角
pitch = asin(av_acc(1));                %初始俯仰角

av_mag=[magx(alignNum) magy(alignNum) magz(alignNum)];
av_mag = mean(av_mag);  %前面1000组磁通门数据的均值
mnorm = norm(av_mag);   %地磁矢量的模
av_mag = av_mag/mnorm;

%已知滚转角、俯仰角，机体坐标系内的重力矢量、地磁矢量，导航坐标系内的重力矢量、地磁矢量，求偏航角
sinfi = sin(roll);
cosfi = cos(roll);
sinth = sin(pitch);
costh = cos(pitch);
gb1=av_mag(1);
gb2=av_mag(2);
gb3=av_mag(3);
[u,v]=solve('u^2+v^2=1','costh*u*gb1+(cosfi*v+sinfi*sinth*u)*gb2+(-sinfi*v+cosfi*sinth*u)*gb3=0');
nume=subs([u,v]);
yaw_possible = [asin(nume(1,1)),asin(nume(2,1))];
if nume(1,1)>0
    if nume(1,2) < 0
        yaw_possible(1) = pi-yaw_possible(1);
    end
else
    if nume(1,2) < 0
        yaw_possible(1) = -pi-yaw_possible(1);
    end
end

if yaw_possible(1)<0
    yaw_possible(1) = yaw_possible(1)+2*pi;
end

if nume(2,1)>0
    if nume(2,2) < 0
        yaw_possible(2) = pi-yaw_possible(2);
    end
else
    if nume(2,2) < 0
        yaw_possible(2) = -pi-yaw_possible(2);
    end
end

if yaw_possible(2)<0
    yaw_possible(2) = yaw_possible(2)+2*pi;
end

if av_mag(1) > 0 && av_mag(2)>0
    yaw = yaw_possible(yaw_possible > 1.5*pi & yaw_possible < 2*pi);
elseif av_mag(1) > 0 && av_mag(2)<0
        yaw = yaw_possible(yaw_possible > 0 & yaw_possible < 0.5*pi);
elseif av_mag(1) < 0 && av_mag(2)>0
        yaw = yaw_possible(yaw_possible > pi & yaw_possible < 1.5*pi);
elseif av_mag(1) < 0 && av_mag(2)<0
    yaw = yaw_possible(yaw_possible > 0.5*pi & yaw_possible < pi);
end

%根据求得的欧拉角，计算初始四元数
x(1:4) = myAngle2Quat([roll pitch yaw]);
%% estimate geomagn
m0=myAngle2Cbn([roll pitch yaw])*av_mag'; 

%% kalman processing
%record interesting data
%用于记录数据的变量
xRec = zeros(5,sampleNum);
wzRec = zeros(1,sampleNum);
accRec = zeros(3,sampleNum);
myAccRec = zeros(3,sampleNum);
RRec = zeros(1,sampleNum);
PRec = zeros(5,sampleNum);

%非线性系统方程
afunc = @INS_update;
%非线性观测方程
hfunc = @GetEstiOb;
%计算系统雅克比矩阵的函数
geta = @GetA;
%计算观测方程雅克比矩阵的函数
geth = @GetH;

%程序使用了matlab ekf_ukf算法包，采用的是UKF方法
for i=1:sampleNum
    %predict
%     [x, P] = ekf_predict1(x, P, geta, Q, afunc, eye(length(x)), dt);%[M,P] = ekf_predict1(M,P,A,Q,a,W,param)
    %预测更新
    [x, P, D] = ukf_predict1(x, P, afunc, Q, dt);%[M,P,D] = ukf_predict1(M,P,f,Q,f_param,alpha,beta,kappa,mat)

%    x = INS_update(x,dt);
%    A = GetA(x,dt);
%    P = A*P*A'+Q;
% 
%    hx = GetEstiOb(x,m0);
   
   %extract measure from raw_data---------------------------------
   %获取观测量
   z = zeros(7,1);
   z(1) = gx(i);    %g component along x axis
   z(2) = gy(i);    %g component along y axis
   z(3) = gz(i);    %g component along z axis
   
   z(4) = magx(i);  %geomagn along x axis
   z(5) = magy(i);  %y
   z(6) = magz(i);  %z
   
   %单位换算
   z(1:3) = z(1:3)/gnorm;    %normalize
   z(4:6) = z(4:6)/mnorm;    %normalize   
   %rotation rate
   z(7) = dMHS(i);
   wzRec(i) = z(7);

    %update
    if abs(z(1))<2 && abs(z(2))<2 %若加速度计噪声超过设定阈值，则本次不融合
%        [x, P, K, Mu, S, LH] = ekf_update1(x, P, z, geth, R, hfunc, eye(length(z)), m0);     %[M,P,K,MU,S,LH] = ekf_update1(M,P,y,H,R,h,V,param)
     [x, P, K, Mu, S, LH] = ukf_update1(x, P, z, hfunc, R, m0);     %
    end

%    H = GetH(x,m0);
%    K = P*H'*(H*P*H'+R)^-1;  
%    resiErr = z-hx;
%    x = x+K*resiErr;
%    P = (eye(5)-K*H)*P;
   
   %normalize quaternion
   x(1:4) = x(1:4)/norm(x(1:4));    %四元数归一化
   xRec(:,i) = x;
   PRec(:,i) = diag(P,0);
  
   Cbn = myQuat2Cbn(x(1:4));
   
   %根据估计的姿态重新估计钻铤坐标系内的加速度分量
   acc_guess = Cbn'*[0;0;-1];
   accRec(:,i) = z(1:3);
   myAccRec(:,i)=acc_guess;
end
%%
figure;
plot([accRec;myAccRec]');
legend('x','y','z','mx','my','mz');

%%
figure;
subplot(2,1,1);
hold off;
plot(wzRec);
hold on;
plot(xRec(5,:),'r');
grid on;
ylabel('rotation speed(rad/s)');
title('rotation speed');
legend('wz','esti wz');
angle=zeros(3,sampleNum);
inc = zeros(1,sampleNum);
incmhs = zeros(1,sampleNum);
mb = zeros(3,sampleNum);
for i=1:sampleNum
    [angle(3,i) angle(2,i) angle(1,i)]=quat2angle(xRec(1:4,i)');
    Cbn = myQuat2Cbn(xRec(1:4,i));
    zn = [0 0 1]';
    rn = Cbn*[0 0 1]';
    inc(i) = acos(dot(zn,rn));
    
   mb(:,i)=Cbn'*m0;
   incmhs(i) = acos(dot(mb(1:2,i),myAccRec(1:2,i))/(norm(mb(1:2,i))*norm(myAccRec(1:2,i))));
   
   crs = cross(mb(:,i),myAccRec(:,i));
   if crs(3)<0
       incmhs(i) = 2*pi-incmhs(i);
   end
   
end
subplot(2,1,2);
hold off;
plot(angle(1,:)*57.3);
hold on;
plot(angle(2,:)*57.3,'r');
% plot(refAngle(1,:)*57.3,'m');
% plot(refAngle(2,:)*57.3,'c');
plot(inc*57.3,'g');
ylabel('angle (degree)');
title('angle');
% legend('roll','pitch','refroll','refpitch');
grid on;
%%
figure;
subplot(2,1,1);
hold off;
plot(xRec(1,:));
hold on;
plot(xRec(2,:),'r');
plot(xRec(3,:),'g');
plot(xRec(4,:),'y');
grid on;
title('quaternion');
subplot(2,1,2);
plot(inc*57.3);
% [b,a]=cheby1(3,0.6,0.001);
% inc = lowPassProccess(b,a,inc,100);
% hold on;
% plot(inc*57.3,'r');
title('INC');
grid on;

%%
figure;
plot(incmhs*57.3);
% incmhs = lowPassProccess(b,a,incmhs,100);
% hold on;
% plot(360-incmhs*57.3,'r');
grid on;

%%
figure;
figure;
refData = textread('INCMHS&INC.txt');
refData = refData(82:102,1:4);
for i=1:floor(sampleNum/1000)
    inc_av(i) = mean(inc((i-1)*1000+1:i*1000));
    incmhs_av(i) = mean(incmhs((i-1)*1000+1:i*1000));
end
subplot(2,1,1);
plot(refData(:,4),'g');
hold on;
plot(refData(:,2),'b');
plot(inc_av*57.3,'r');
grid on;
legend('ref','PLM','UKF');
ylabel('INC (degree)');

subplot(2,1,2);
plot(refData(:,3),'g');
hold on;
plot(refData(:,1),'b');
plot(incmhs_av*57.3,'r');
legend('ref','PLM','UKF');
ylabel('INCMHS (degree)');
grid on;