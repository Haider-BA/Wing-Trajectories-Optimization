function [fitresult, gof] = createFit_deviation(t, x2)
%CREATEFIT1(T,X2)
%  Create a fit.
%
%  Data for 'untitled fit 1' fit:
%      X Input : t
%      Y Output: x2
%  Output:
%      fitresult : a fit object representing the fit.
%      gof : structure with goodness-of fit info.
%
%  See also FIT, CFIT, SFIT.

%  Auto-generated by MATLAB on 04-Mar-2013 01:19:03


%% Fit: 'untitled fit 1'.
[xData, yData] = prepareCurveData( t, x2 );

% Set up fittype and options.
ft = fittype( 'fourier6' );
opts = fitoptions( ft );
opts.Display = 'Off';
opts.Lower = [-Inf -Inf -Inf -Inf -Inf -Inf -Inf -Inf -Inf -Inf -Inf -Inf -Inf -Inf];
opts.StartPoint = [0 0 0 0 0 0 0 0 0 0 0 0 0 2*pi];
opts.Upper = [Inf Inf Inf Inf Inf Inf Inf Inf Inf Inf Inf Inf Inf Inf];

% Fit model to data.
[fitresult, gof] = fit( xData, yData, ft, opts );

% % Plot fit with data.
% figure( 'Name', 'deviation' );
% h = plot( fitresult, xData, yData );
% legend( h, 'x2 vs. t', 'deviation', 'Location', 'NorthEast' );
% % Label axes
% xlabel( 't' );
% ylabel( 'x2' );
% grid on

