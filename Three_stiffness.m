
LB = [10 10 10];
UB = [100 30 80];
nvars = 3;



% options = gaoptimset('PlotFcns',{@gaplotbestf,@gaplotstopping});
% patternsearchOptions = psoptimset('Display','iter');
% options = gaoptimset(options,'HybridFcn',{@patternsearch, patternsearchOptions});
% [x,fval,exitflag,output] = ga(@main,nvars,[],[],[],[],LB,UB,[],options);
% 
% save hybrid_pattern.mat


x0=[45 13 16.308];



%% fmincon, 'sqp' is not effective.

options = gaoptimset('InitialPopulation',x0,'PlotFcns',{@gaplotbestf,@gaplotstopping});
fminuncOptions = optimoptions(@fminunc,'Algorithm','quasi-newton','Display','iter');
options = gaoptimset(options,'HybridFcn',{@fminunc, fminuncOptions});
[x,fval,exitflag,output] = ga(@main,nvars,[],[],[],[],LB,UB,[],options);

save hybrid_fminunc.mat