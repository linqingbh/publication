function [ode,states,controls,theta] = symbolic_LongDyn_AP2
  import casadi.*
  % x(1) = vt;   % airspeed        [m/s]
  % x(2) = alpha % angle of attack [rad] 
  % x(3) = theta % pitch angle     [rad]
  % x(4) = q     % pitch rate      [rad/s]

  % u(1) = de    % elevator deflection [rad]
  %% Model generation via casADi/OPTistack ==================================
  x1 = MX.sym('x1'); % vt
  x2 = MX.sym('x2'); % alpha
  x3 = MX.sym('x3'); % theta
  x4 = MX.sym('x4'); % q
  u  = MX.sym('u');  % de

  states   = [x1;x2;x3;x4];
  controls = [u];

  % define uknow parametes as design variable
  a11 = MX.sym('a11');   
  a12 = MX.sym('a12');    
  a14 = MX.sym('a14');
  a21 = MX.sym('a21');
  a22 = MX.sym('a22');
  a24 = MX.sym('a24');
  a41 = MX.sym('a41');
  a42 = MX.sym('a42');
  a44 = MX.sym('a44');
  b1  = MX.sym('b1'); 
  b2  = MX.sym('b2');
  b4  = MX.sym('b4');

  % store uknow parameters
  theta = [a11;a12;a14;a21;a22;a24;a41;a42;a44;b1;b2;b4];   

  % xdot = f(x,u,p) <==> rhs = f(x,u,p)
  g0 = -9.8066;
  rhs = [a11*x1 + a12*x2 + g0*x3 + a14*x4 + b1*u;
         a21*x1 + a22*x2         + a24*x4 + b2*u;
                                   1.0*x4;
         a41*x1 + a42*x2         + a44*x4 + b4*u];

  % Form an ode function
  ode = Function('ode',{states,controls,theta},{rhs});
  %ode.printDimensions();

  %% numerical evaluation ===================================================
  % x_test     = zeros(nx,1);
  % u_test     = zeros(nu,1);
  % theta_test = p_truth;
  % f_out      = full(ode(x_test,u_test,theta_test));   
  % disp(['rhs(xtest,utest,ptest) = [',num2str(f_out'),']'])

end