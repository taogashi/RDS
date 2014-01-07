function y=myCbn2Angle(Cbn)
y=zeros(3,1);
% y(1) = atan2(Cbn(3,3),Cbn(3,2));
% y(2) = asin(-Cbn(3,1));
% y(3) = atan2(Cbn(1,1),Cbn(2,1));

y(1) = atan(Cbn(3,2)/Cbn(3,3));
y(2) = asin(-Cbn(3,1));
y(3) = atan(Cbn(2,1)/Cbn(1,1));