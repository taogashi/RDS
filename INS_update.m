function y=INS_update(x,dt)
% x: 5*1 state vector
% dt: time interval

y=zeros(size(x));
% sigma = 0.5*x(5)*dt;
% A=[1 0 0 -sigma
%     0 1 sigma 0
%     0 -sigma 1 0
%     sigma 0 0 1];
% y(1:4) = A*x(1:4);
y(1:4) = QuatUpdate(x(1:4),[0,0,x(5)],dt);
y(5) = x(5);

end
