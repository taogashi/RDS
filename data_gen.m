%data generator
clc;
clear;
close all;

m0=[0.4;0;0.6];
m0 = m0/norm(m0);
g0=[0; 0; -1];

init_angle = [0,-0.1,0.1];
quat = myAngle2Quat(init_angle);
quat = reshape(quat,4,1);

totalNum = 20000;
stableNum = 2000;
wz=2*pi*(sin(0:0.01:0.01*totalNum)+1);

refAngle=zeros(3,totalNum);
mbRec = zeros(totalNum,3);
abRec = zeros(totalNum,3);

refINC = zeros(totalNum,1);
refINCMHS = zeros(totalNum,1);

Cbn = myQuat2Cbn(quat);
for i=1:stableNum
    [refAngle(3,i) refAngle(2,i) refAngle(1,i)] = quat2angle(quat');
    mbRec(i,:) = (Cbn'*m0)';
    abRec(i,:) = (Cbn'*g0)';
    zn = [0 0 1]';
    rn = Cbn*[0 0 1]';
    refINC(i) = acos(dot(zn,rn));
    refINCMHS(i) = acos(dot(mbRec(i,1:2),abRec(i,1:2))/(norm(mbRec(i,1:2))*norm(abRec(i,1:2))));
    
end

for i=stableNum+1:totalNum
    quat = QuatUpdate(quat,[0,0,3.0],0.01);
    [refAngle(3,i) refAngle(2,i) refAngle(1,i)] = quat2angle(quat');
    Cbn = myQuat2Cbn(quat);
    mbRec(i,:) = (Cbn'*m0)';
    abRec(i,:) = (Cbn'*g0)';
    zn = [0 0 1]';
    rn = Cbn*[0 0 1]';
    refINC(i) = acos(dot(zn,rn));
    refINCMHS(i) = acos(dot(mbRec(i,1:2),abRec(i,1:2))/(norm(mbRec(i,1:2))*norm(abRec(i,1:2))));
%     incmhsRec(i)
end;

figure(1);
subplot(2,1,1);
plot(abRec);
mbRec(1:stableNum,:) = mbRec(1:stableNum,:) + random('norm',0,0.0008,stableNum,3);
abRec(1:stableNum,:) = abRec(1:stableNum,:) + random('norm',0,0.04,stableNum,3);

mbRec(stableNum+1:end,:) = mbRec(stableNum+1:end,:) + random('norm',0,0.0008,totalNum-stableNum,3);
abRec(stableNum+1:end,:) = abRec(stableNum+1:end,:) + random('norm',0,0.5,totalNum-stableNum,3);

magx = mbRec(:,1);
magy = mbRec(:,2);
magz = mbRec(:,3);

gx = abRec(:,1);
gy = abRec(:,2);
gz = abRec(:,3);

save;

figure;
plot(refAngle'*57.3);
%%
figure;
plot(mbRec);

figure(1);
subplot(2,1,2);
plot(abRec);