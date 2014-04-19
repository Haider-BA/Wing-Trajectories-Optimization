function [X,F,Iters] = modnewton_opt(N, X, gradToler, XToler, MaxIter, myFx)
% Function MODNEWTON_OPT performs multivariate optimization using the
% modified Newton's method.
%
% Input
%
% N - number of variables
% X - array of initial guesses
% gradToler - tolerance for the norm of the slopes
% XToler - array of tolerance values for the variables' refinement
% MaxIter - maximum number of iterations
% myFx - name of the optimized function
%
% Output
%
% X - array of optimized variables
% F - function value at optimum
% Iters - number of iterations
%

bGoOn = true;
Iters = 0;

while bGoOn

  Iters = Iters + 1;
  if Iters > MaxIter
    break;
  end

  g = FirstDerivatives(X, N, myFx);
  fnorm = norm(g);
  if fnorm < gradToler
    break;
  end
  J = SecondDerivatives(X, N, myFx);
  DeltaX = g / J;

  lambda = 0;
  lambda = linsearch(X, N, lambda, DeltaX, myFx);
  X = X + lambda * DeltaX;

  bStop = true;
  for i=1:N
    if abs(lambda * DeltaX(i)) > XToler(i)
      bStop = false;
    end
  end

  bGoOn = ~bStop;

end

F = feval(myFx, X, N);

% end

function y = myFxEx(N, X, DeltaX, lambda, myFx)

  X = X + lambda * DeltaX;
  y = feval(myFx, X, N);

% end

function FirstDerivX = FirstDerivatives(X, N, myFx)

for iVar=1:N
  xt = X(iVar);
  h = 0.01 * (1 + abs(xt));
  X(iVar) = xt + h;
  fp = feval(myFx, X, N);
  X(iVar) = xt - h;
  fm = feval(myFx, X, N);
  X(iVar) = xt;
  FirstDerivX(iVar) = (fp - fm) / 2 / h;
end

% end

function SecondDerivX = SecondDerivatives(X, N, myFx)

for i=1:N
  for j=1:N
    % calculate second derivative?
    if i == j
      f0 = feval(myFx, X, N);
      xt = X(i);
      hx = 0.01 * (1 + abs(xt));
      X(i) = xt + hx;
      fp = feval(myFx, X, N);
      X(i) = xt - hx;
      fm = feval(myFx, X, N);
      X(i) = xt;
      y = (fp - 2 * f0 + fm) / hx ^ 2;
    else
      xt = X(i);
      yt = X(j);
      hx = 0.01 * (1 + abs(xt));
      hy = 0.01 * (1 + abs(yt));
      % calculate fpp;
      X(i) = xt + hx;
      X(j) = yt + hy;
      fpp = feval(myFx, X, N);
      % calculate fmm;
      X(i) = xt - hx;
      X(j) = yt - hy;
      fmm = feval(myFx, X, N);
      % calculate fpm;
      X(i) = xt + hx;
      X(j) = yt - hy;
      fpm = feval(myFx, X, N);
      % calculate fmp
      X(i) = xt - hx;
      X(j) = yt + hy;
      fmp = feval(myFx, X, N);
      X(i) = xt;
      X(j) = yt;
      y = (fpp - fmp - fpm + fmm) / (4 * hx * hy);
    end
    SecondDerivX(i,j) = y;
  end
end

%end

function lambda = linsearch(X, N, lambda, D, myFx)

  MaxIt = 100;
  Toler = 0.000001;

  iter = 0;
  bGoOn = true;
  while bGoOn
    iter = iter + 1;
    if iter > MaxIt
      lambda = 0;
      break
    end

    h = 0.01 * (1 + abs(lambda));
    f0 = myFxEx(N, X, D, lambda, myFx);
    fp = myFxEx(N, X, D, lambda+h, myFx);
    fm = myFxEx(N, X, D, lambda-h, myFx);
    deriv1 = (fp - fm) / 2 / h;
    deriv2 = (fp - 2 * f0 + fm) / h ^ 2;
    diff = deriv1 / deriv2;
    lambda = lambda - diff;
    if abs(diff) < Toler
      bGoOn = false;
    end
  end

% end
