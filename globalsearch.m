
LB = [15 40 10 10];
UB = [30 200 30 80];
nvars = 4;



% options = gaoptimset('PlotFcns',{@gaplotbestf,@gaplotstopping});
% patternsearchOptions = psoptimset('Display','iter');
% options = gaoptimset(options,'HybridFcn',{@patternsearch, patternsearchOptions});
% [x,fval,exitflag,output] = ga(@main,nvars,[],[],[],[],LB,UB,[],options);
% 
% save hybrid_pattern.mat


x0=[26.40644558	155.8019051	19.73891555	71.69333469;
    27.2528	164.6853 18.6853 29.756;
    22 116.6124 11.7201 13.726;
    18 68 13 16.308];



%% fmincon, 'sqp' is not effective.

options = gaoptimset('InitialPopulation',x0);
fminuncOptions = optimoptions(@fminunc,'Algorithm','quasi-newton','Display','iter');
options = gaoptimset(options,'HybridFcn',{@fminunc, fminuncOptions});
[x,fval,exitflag,output] = ga(@main,nvars,[],[],[],[],LB,UB,[],options);

save hybrid_fminunc.mat