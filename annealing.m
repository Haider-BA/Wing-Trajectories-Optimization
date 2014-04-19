LB = [15 40 10 600];
UB = [30 200 20 8000];

x0=[22 116.6124 11.7201 1372.6];

option = saoptimset('Display','iter','PlotFcns',{@saplotbestf,@saplotbestx,@saplotf});

[x,fval,exitflag,output] = simulannealbnd(@main,x0,LB,UB,options);
