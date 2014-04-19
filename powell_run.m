% Testing powell, by using test10 (problem found on Edgar & Himmelblau, 1988)
%
%
% [xo,Ot,nS]=powell('test10',[-1.2, 1],0,[],[],[],[],[],300)
% 
% xo = 
%   1.0e-018 *
% 
%     0.1599
%    -0.0652
% 
% 
% Ot =  1.2742e-037
% 
% 
% nS = 87


[X, FxVal, Iters] = powell(2, [-0.9 -0.5], 1e-7, [1e-5 1e-5 1e-5 1e-5], 10000, 'fx1')
% [X,F,Iters] = steepdescent(4, [20 104 13 1.50], 1e-7, [1e-5 1e-5 1e-5 1e-5], 10000, 'fx1')