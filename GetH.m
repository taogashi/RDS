function y=GetH(x,m0)
% x: 5x1 state vector
% m0: normalized geomagnetism
% r: radias of the driller

y = zeros(6,5);
y(1:3,:)=2*[
    x(3)  -x(4) x(1)  -x(2) 0
    -x(2) -x(1) -x(4) -x(3) 0
    -x(1)  x(2) x(3)  -x(4) 0];
y(4:6,:)=2*[
    m0(1)*x(1)-m0(3)*x(3)   m0(1)*x(2)+m0(3)*x(4)   -m0(1)*x(3)-m0(3)*x(1)    -m0(1)*x(4)+m0(3)*x(2)    0
    -m0(1)*x(4)+m0(3)*x(2)  m0(1)*x(3)+m0(3)*x(1)   m0(1)*x(2)+m0(3)*x(4)    -m0(1)*x(1)+m0(3)*x(3)     0
    m0(1)*x(3)+m0(3)*x(1)  m0(1)*x(4)-m0(3)*x(2)   m0(1)*x(1)-m0(3)*x(3)    m0(1)*x(2)+m0(3)*x(4)     0];
y(7,:)=[0   0   0   0   1];

end