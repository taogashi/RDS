%filter analyze
[b,a] = cheby1(2,0.1,0.001);
freqz(b,a,512,100)