function [X, FxVal, Iters] = powell(N, X, Eps_Fx, Eps_Step, MaxIter, myFx)
%
% Function POWELL_OPT performs Powerl search optimization
%
% Input
%
% N - number of variables
% X - row array of initial guesses
% Eps_Fx - tolerance for function value
% Eps_Step - tolerance for step values
% MaxIter - maximum number of iterations
% myFx - string name of the target functions
%
% Output
%
% X - row array of optimum coordinates
% FxVal - value of the optimized function
% Iters - number of iterations
%

Iters = 0;
f1 = feval(myFx, X, N);
X1 = X;
S = eye(N+1,N);

bGoOn = true;

while bGoOn
  S(N+1,:) = 0; % reset row N+1

  for i= 1:N
    alpha = 0.1;
    alpha = linsearch(X, N, alpha, S, i, myFx);
    X = X + alpha * S(i,:);
    S(N+1,:) = S(N+1,:) + alpha * S(i,:);
%    for k=1:N
%      X(k) = X(k) + alpha * S(i,k);
%      S(N+1,k) = S(N+1,k) + alpha * S(i,k);
%    end
  end

  alpha = 0.1;
  alpha = linsearch(X, N, alpha, S, N+1, myFx);
  X = X + alpha * S(N+1,:);
  X2 = X;

  f2 = feval(myFx, X2, N);

  if abs(f2 - f1) < Eps_Fx
    break;
  end

  if norm(X2 - X1) < Eps_Step
    break
  end

  Iters = Iters + 1;

  if Iters >= MaxIter
    break
  end

  X1 = X2;
  for k=1:N
    for m=1:N
      S(k, m) = S(k+1,m);
    end
  end

end

FxVal = feval(myFx, X, N);

function y = myFxEx(N, X, S, ii, alpha, myFx)

  X = X + alpha * S(ii,:);
  y = feval(myFx, X, N);

% end

function alpha = linsearch(X, N, alpha, S, ii, myFx)

  MaxIt = 100;
  Toler = 0.0001;

  iter = 0;
  bGoOn = true;
  while bGoOn
    iter = iter + 1;
    if iter > MaxIt
      alpha = 0;
      break
    end

    h = 0.01 * (1 + abs(alpha));
    f0 = myFxEx(N, X, S, ii, alpha, myFx);
    fp = myFxEx(N, X, S, ii, alpha+h, myFx);
    fm = myFxEx(N, X, S, ii, alpha-h, myFx);
    deriv1 = (fp - fm) / 2 / h;
    deriv2 = (fp - 2 * f0 + fm) / h ^ 2;
    if deriv2 == 0
        break
    end
    diff = deriv1 / deriv2;
    alpha = alpha - diff;
    if abs(diff) < Toler
      bGoOn = false;
    end
  end

% end
