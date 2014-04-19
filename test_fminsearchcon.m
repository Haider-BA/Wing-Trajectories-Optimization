

LB = [0.2 0.7 0.6];
UB = [1.8 2.5 7.5];

nvars = 3;
% % This is useless.
% % Try setting the initial range.
% % x0 = [2.867 1.698248 2.15651];
% 
% x_initial = [2.767 1.598248 2.05651;
%              2.967 1.798248 2.25651];
% 
% % 'InitialPopulation',x0,
% ,'PopInitRange',x_initial
ga_options = gaoptimset('PlotFcns',@gaplotbestf);

[x_ga1,fval_ga1,exitflag_ga1,output_ga1] = ga(@main,nvars,[],[],[],[],LB,UB,[],ga_options);



% x_results = [1.8005 0.7830 2.1084];
% [x_fmin1,fval_fmin1,exitflag_fmin1,output_fmin1] = fminsearchcon(@fun,x_results,LB,UB,[],[],@seminfcon);


% [x_ga2,fval_ga2,exitflag_ga2,output_ga2] = ga(@main,nvars,[],[],[],[],LB,UB,[],ga_options);
% 
% [x_fmin2,fval_fmin2,exitflag_fmin2,output_fmin2] = fminsearchcon(@fun,x_ga2,LB,UB,[],[],@seminfcon);



save test_f.mat