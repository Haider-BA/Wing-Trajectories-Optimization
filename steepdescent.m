function [X,F,Iters] = steepdescent(N, X, Eps_Deriv, Eps_Fx, MaxIter, myFx)
% Function STEEPDESCENT performs multivariate optimization using the
% Steepest Descent method.
%
% Input
%
% N - number of variables
% X - array of initial guesses
% Eps_Deriv - tolerance for slopes
% Eps_Fx - tolerance for difference in successive function values
% MaxIter - maximum number of iterations
% myFx - name of the optimized function
%
% Output
%
% X - array of optimized variables
% F - function value at optimum
% Iters - number of iterations
%

% calculate initial function value
LastF = feval(myFx, X, N);

Iters = 0;
bGoOn = true;

while bGoOn
  Iters = Iters + 1;
  if Iters > MaxIter
    break;
  end
  [dfnorm,Derivs] = getgradients(X, N, myFx);
  if dfnorm < Eps_Deriv
    F = feval(myFx, X, N);
    disp(['dfnorm = ' num2str(dfnorm)])
    break;
  end
  Derivs = -1 * Derivs / dfnorm;
  lambda = 0.1;
  lambda = linsearch(X, N, lambda, Derivs, myFx);
  X = X + lambda * Derivs;
  F = feval(myFx, X, N);
  if abs(F - LastF) <= Eps_Fx
    disp(['dfnorm = ' num2str(F - LastF)])
    break;
  else
    LastF = F;
  end
end


% end

function y = myFxEx(N, X, Derivs, lambda, myFx)

  X = X + lambda * Derivs;
  y = feval(myFx, X, N);

% end

function [fnorm,Deriv] = getgradients(X, N, myFx)

  for i=1:N
    xx = X(i);
    h = 0.01 * (1 + abs(xx));
    X(i) = xx + h;
    Fp = feval(myFx, X, N);
    X(i) = xx - h;
    Fm = feval(myFx, X, N);
    X(i) = xx;
    Deriv(i) = (Fp - Fm) / 2 / h;
  end
  fnorm = norm(Deriv);
% end

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
