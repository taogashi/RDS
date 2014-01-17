function y=INS_update(x,dt)
% x: 5*1 state vector
% dt: time interval

y=zeros(size(x));
y(1:4) = QuatUpdate(x(1:4),[0,0,x(5)],dt);
y(5) = x(5);

end
