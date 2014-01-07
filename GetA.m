function y=GetA(x,dt)
% x:state vector n x 1
% dt:time interval

sigma = 0.5*x(5)*dt;
y=eye(length(x));
y(1,4)=-sigma;  y(1,5)=-0.5*x(4)*dt;
y(2,3)=sigma;   y(2,5)=0.5*x(3)*dt;
y(3,2)=-sigma;  y(3,5)=-0.5*x(2)*dt;
y(4,1)=sigma;   y(4,5)=0.5*x(1)*dt;

end