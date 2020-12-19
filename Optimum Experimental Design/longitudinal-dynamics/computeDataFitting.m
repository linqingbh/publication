function [theta_est,X_est] = computeDataFitting(w,w0,lbw,ubw,J,g,lbg,ubg)
  global np nx
  import casadi.*
  w  = vertcat(w{:});
  g  = vertcat(g{:});

  % Create an NLP solver
  prob   = struct('f', J, 'x', w, 'g', g);
  % option IPOPT
  opts                             = struct;
  opts.ipopt.linear_solver         = 'ma97';
  opts.ipopt.max_iter              = 50;
  opts.ipopt.hessian_approximation = 'exact';  % 3 iteration

  % Solve the NLP
  solver  = nlpsol('solver', 'ipopt', prob,opts);
  w_sol   = solver('x0', w0, 'lbx', lbw, 'ubx', ubw,'lbg', lbg, 'ubg', ubg);

  theta_est = full(w_sol.x(1:np));  % retrieve parameters
  
  
  X_sol  = full(w_sol.x(np+1:end)); % retrieve states
  X_est = [];
  for i=1:nx
    X_est = [X_est, X_sol(i:nx:end)];
  end
  
end