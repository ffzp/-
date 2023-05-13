function [est_var] = RLS(est_var, e, x)
    lambda = 0.99; % 每次更新带来的影响程度
    P = (1/lambda)*(est_var);
    K = P*x/(lambda+P*x^2);
    est_var = lambda*est_var - K*x'*P;
    est_var = est_var + e^2;
end