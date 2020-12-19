function [theta,states,controls,ode] = LongitudinalDynamic(PhysicalProperties)

  Sref = PhysicalProperties.Value.wingArea;
  bref = PhysicalProperties.Value.wingSpan;
  cref = PhysicalProperties.Value.chord;

  Jx   = PhysicalProperties.Value.inertias.variableParameter.value(1);
  Jy   = PhysicalProperties.Value.inertias.variableParameter.value(2);
  Jz   = PhysicalProperties.Value.inertias.variableParameter.value(3);
  Jxz  = PhysicalProperties.Value.inertias.variableParameter.value(4);
  
  import casadi.*
  p              = MX.sym('p');  
  q              = MX.sym('q');      
  r              = MX.sym('r');

  deltaE         = MX.sym('deltaE');
  alpha          = MX.sym('alpha');
  V              = MX.sym('V');
  qbar           = MX.sym('qbar');

  states         = q;
  controls       = [p;r;deltaE;alpha;V;qbar];

  Cm_0           = MX.sym('Cm_0');
  Cm_alpha       = MX.sym('Cm_alpha');
  Cm_deltaE      = MX.sym('Cm_deltaE');
  Cm_q           = MX.sym('Cm_q');

  theta         =  [Cm_0;Cm_alpha;Cm_q;Cm_deltaE];

  qhat = (cref*q)/(2*V);
  Cm   = Cm_0 + Cm_alpha*alpha + Cm_q*qhat + Cm_deltaE*deltaE;
  M    = qbar*Sref*cref*Cm;
  rhs  = solve(Jy,(M - Jxz*p^2 + Jxz*r^2 - Jx*p*r + Jz*p*r));
  % Form an ode function
  ode = Function('ode',{states,controls,theta},{rhs});   
end