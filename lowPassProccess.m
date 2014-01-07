function y=lowPassProccess(b,a,x,initNum)
x0=mean(x(1:initNum));
x=x-x0;
y=filter(b,a,x);
y=y+x0;
end