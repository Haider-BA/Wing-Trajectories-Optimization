function [fitresult, gof] = createFit_rotation(t, x3)
%CREATEFIT1(T,X3)
%  Create a fit.
%
%  Data for 'untitled fit 1' fit:
%      X Input : t
%      Y Output: x3
%  Output:
%      fitresult : a fit object representing the fit.
%      gof : structure with goodness-of fit info.
%
%  See also FIT, CFIT, SFIT.

%  Auto-generated by MATLAB on 02-Mar-2013 00:41:58


%% Fit: 'untitled fit 1'.
[xData, yData] = prepareCurveData( t, x3 );

% Set up fittype and options.
% ft = fittype( 'a0 + a1*cos(w*x+b1) + a2*cos(2*w*x+b2) + a3*cos(3*w*x+b3)', 'independent', 'x', 'dependent', 'y' );
ft = fittype( 'fourier3' );
opts = fitoptions( ft );
% opts.Algorithm = 'Levenberg-Marquardt';
opts.Display = 'Off';
opts.Lower = [-Inf -Inf -Inf -Inf -Inf -Inf -Inf -Inf];
opts.StartPoint = [0 0 0 0 0 0 0 2*pi];
opts.Upper = [Inf Inf Inf Inf Inf Inf Inf Inf];

% Fit model to data.
[fitresult, gof] = fit( xData, yData, ft, opts );

% % Plot fit with data.
% figure( 'Name', 'rotation' );
% h = plot( fitresult, xData, yData );
% legend( h, 'x3 vs. t', 'rotation', 'Location', 'NorthEast' );
% % Label axes
% xlabel( 't' );
% ylabel( 'x3' );
% grid on


