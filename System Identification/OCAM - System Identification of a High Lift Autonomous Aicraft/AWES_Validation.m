function AWES_Validation
% ======================================================================= %
% Title:  Validation Mathematical Model Aircraft Equation of Motion       %
%         Longitudinal dynamics                                           %
% Author: Giovanni Licitra                                                %
% Date:   20-11-2017                                                      %
% ======================================================================= %

load('Data_Validation.mat')

ts   = 0.01;                                            % sample time [s]
N    = length(Udata);                                   % number of samples 
t    = linspace(0,ts*N,N)';                             % define range
ut   = t;                                               % time varing parameter
u    = zeros(length(t),3);u(:,2) = Udata;               % load inputs 
x0   = [Ydata(1);0;Ydata(2);0;Ydata(3);0;0;Ydata(4);0]; % initial states: steady wing level  
p    = loadParameters;                                  % load Aircraft Physical Parameters

[time,Xsim] = ode45(@(t,x) ode(t,x,u,ut,p),t,x0);
plotForwardSimulation(time,Xsim,Udata,Ydata)

end

function dx              = ode(t,x,u,ut,p)                                           
  % Aircraft equation of motion suitable for system identification purpose
  u = interp1(ut,u,t);    % Interpolate the data set (ut,u) at time t
  
  % Differential states  
  Vt    = x(1); % Airspeed        [m/s]
  beta  = x(2); % Side slip angle [rad]
  alpha = x(3); % Angle of attack [rad]
  phi   = x(4); % Roll angle      [rad]
  theta = x(5); % Pitch angle     [rad]
  psi   = x(6); % Yaw angle       [rad]
  P     = x(7); % Roll rate       [rad/s]
  Q     = x(8); % Pitch rate      [rad/s]
  R     = x(9); % Yaw rate        [rad/s]
  
  % Control inputs
  dA    = u(1); % Aileron deflection  [rad]
  dE    = u(2); % Elevator deflection [rad]
  dR    = u(3); % Rudder deflection   [rad]
  
  PHI    = [phi;theta;psi];        % Euler angles
  wBody  = [P;Q;R];                % Angular body rates
  Rsb    = Cbody2stability(alpha); % DCM: Body2Stability frame (Rb2s)    
  wBodyS = Rsb*wBody;              % Angular Body rates in Stability frame
  Ps     = wBodyS(1);
  Qs     = wBodyS(2);
  Rs     = wBodyS(3);

  % get aeroDynamics and engineDynamics: Body-fixed frame
  [FaeroB  , MaeroB] = aeroDynamicsIdentification(Vt,alpha,beta,dA,dE,dR,wBody,p);
 
  %Compute Force Equation [Wind Axes]
  Rwb    = Cbody2wind(alpha,beta);
  FaeroW = Rwb*FaeroB;   % Aerodynamic forces in wind axes FaeroW = [-L;-C;-D]
   
  D = -FaeroW(1); % Drag Force
  C = -FaeroW(2); % Cross-Wind Force 
  L = -FaeroW(3); % Lift Force
  
  % gravity Contribution [Wind Axes]
  gD = p.environment.graity; 
  g1 = gD*(-cos(alpha)*cos(beta)*sin(theta) + sin(beta)*sin(phi)*sin(theta) + sin(alpha)*cos(beta)*cos(phi)*cos(theta));   
  g2 = gD*( cos(alpha)*sin(beta)*sin(theta) + cos(beta)*sin(phi)*cos(theta) - sin(alpha)*sin(beta)*cos(phi)*cos(theta));
  g3 = gD*( sin(alpha)*sin(theta) + cos(alpha)*cos(phi)*cos(theta));
  
  % Make System
  mass      = p.physicalParameters.mass; 
  inertia   = p.physicalParameters.inertia;
  Vt_dot    = (1/mass)*(-D + mass*g1);
  beta_dot  = (1/(mass*Vt))*(-C + mass*g2 - mass*Vt*Rs); 
  alpha_dot = (1/(mass*Vt*cos(beta)))*(-L + mass*g3 + mass*Vt*(Qs*cos(beta) - Ps*sin(beta)));  
  PHI_dot   = Heuler(PHI)*wBody;                              % Kinematics  
  wBody_dot = inv(inertia)*MaeroB+cross(wBody,inertia*wBody); % angular acceleration [Body Frame]
 
  dx        = [Vt_dot;beta_dot;alpha_dot;PHI_dot;wBody_dot];  
end
function p               = loadParameters                                            
p = struct;

%% Physical Parameters ====================================================
p.physicalParameters.wingArea   =  3;    % [m^2]
p.physicalParameters.wingSpan   =  5.50; % [m]
p.physicalParameters.chord      =  0.55; % [m]
p.physicalParameters.mass       = 36.80; % [Kg]
p.physicalParameters.inertia    = [25.00, 0   ,-0.47 ;...
                                    0   ,32.00, 0    ;...
                                   -0.47, 0   ,56.00]; % [Kg*m^2]

%% Dimensionless Aerodynamic Derivatives ==================================
% Longitudinal Derivatives (coming from flight test) ----------------------

% Force Coefficient: Xbody axis
p.derivatives.dCX.CX0   =  -0.0190;    
p.derivatives.dCX.CXa   =  -0.4700;  
p.derivatives.dCX.CXq   =  -0.6029;  
p.derivatives.dCX.CXdE  =  -0.0106; 

% Force Coefficient: Zbody axis
p.derivatives.dCZ.CZ0   =  -0.5953; 
p.derivatives.dCZ.CZa   =  -5.3432;
p.derivatives.dCZ.CZq   =  -7.5000; 
p.derivatives.dCZ.CZdE  =   0.1616; 

% Moment Coefficient: Pitch
p.derivatives.dCm.Cm0   =   0.0645;  
p.derivatives.dCm.Cma   =  -0.8148; 
p.derivatives.dCm.Cmq   = -25.4218; 
p.derivatives.dCm.CmdE  =  -1.0535; 

% Lateral Derivatives (coming from lifting line method) -------------------

% Force Coefficient: Ybody axis
p.derivatives.dCY.CYb   =  -0.1850; 
p.derivatives.dCY.CYp   =  -0.1020;
p.derivatives.dCY.CYr   =   0.1700;
p.derivatives.dCY.CYdA  =  -0.0510;
p.derivatives.dCY.CYdR  =   0.1030;

% Moment Coefficient: Roll
p.derivatives.dCl.Clb   =  -0.0630;
p.derivatives.dCl.Clp   =  -0.5600;
p.derivatives.dCl.Clr   =   0.1500;
p.derivatives.dCl.CldA  =  -0.2480;
p.derivatives.dCl.CldR  =   0.0040;

% Moment Coefficient: Yaw
p.derivatives.dCn.Cnb   =   0.0590;
p.derivatives.dCn.Cnp   =  -0.0400;
p.derivatives.dCn.Cnr   =  -0.0560;
p.derivatives.dCn.CndA  =   0.0210;
p.derivatives.dCn.CndR  =  -0.0400;

%% Environmental Parameter ================================================
p.environment.airDensity =  1.225; % [Kg/m^3]
p.environment.graity     =  9.81;  % [m/s^2] 

end
function [FaeroB,MaeroB] = aeroDynamicsIdentification(Vt,alpha,beta,dA,dE,dR,wBody,p)
% Compute aerodynamic forces and moments [body-fixed axis]
% Input -------------------------------------------------------------------
% Vt          - airspeed [m/s]
% alpha       - angle of attack [rad]
% beta        - angle of sideslip [rad]
% dA,dE,dR    - aileron, elevator, rudder deflection [rad]
% wBody       - angular body rate [rad/s] 
% p           - structure which contains physical parameters
% Output ------------------------------------------------------------------
% Faero       - BodyAxis Frame [FX;FY;FZ]
% Maero       - BodyAxis Frame [L ;M ;N ]

% define dynamics pressure qbar
airspeed = Vt;
rho      = p.environment.airDensity;
qbar     = 0.5*rho*airspeed^2; 

% get Aarcraft geometry
wingSpan = p.physicalParameters.wingSpan;
chord    = p.physicalParameters.chord;
wingArea = p.physicalParameters.wingArea;

% Normalized Body Rates
phat     = 0.5 * wingSpan / airspeed * wBody(1);
qhat     = 0.5 * chord    / airspeed * wBody(2);
rhat     = 0.5 * wingSpan / airspeed * wBody(3);

% Force Coefficient: Xbody axis
CX0   = p.derivatives.dCX.CX0;   
CXa   = p.derivatives.dCX.CXa;    
CXq   = p.derivatives.dCX.CXq;  
CXdE  = p.derivatives.dCX.CXdE; 

CX    = CX0 + CXa * alpha + CXq * qhat + CXdE * dE;

% Force Coefficient: Ybody axis
CYb   = p.derivatives.dCY.CYb;  
CYp   = p.derivatives.dCY.CYp;  
CYr   = p.derivatives.dCY.CYr;  
CYdA  = p.derivatives.dCY.CYdA; 
CYdR  = p.derivatives.dCY.CYdR; 

CY    = CYb * beta + CYp * phat + CYr * rhat + CYdA * dA + CYdR * dR;

% Force Coefficient: Zbody axis
CZ0   = p.derivatives.dCZ.CZ0; 
CZa   = p.derivatives.dCZ.CZa; 
CZq   = p.derivatives.dCZ.CZq; 
CZdE  = p.derivatives.dCZ.CZdE; 

CZ    = CZ0 + CZa * alpha + CZq * qhat + CZdE * dE;

% Moment Coefficient: Roll
Clb   = p.derivatives.dCl.Clb; 
Clp   = p.derivatives.dCl.Clp; 
Clr   = p.derivatives.dCl.Clr; 
CldA  = p.derivatives.dCl.CldA; 
CldR  = p.derivatives.dCl.CldR; 

Cl    = Clb * beta + Clp * phat  + Clr * rhat + CldA * dA + CldR * dR;

% Moment Coefficient: Pitch
Cm0   = p.derivatives.dCm.Cm0; 
Cma   = p.derivatives.dCm.Cma; 
Cmq   = p.derivatives.dCm.Cmq; 
CmdE  = p.derivatives.dCm.CmdE; 

Cm    = Cm0  + Cma * alpha + Cmq * qhat + CmdE * dE;

% Moment Coefficient: Yaw
Cnb   = p.derivatives.dCn.Cnb; 
Cnp   = p.derivatives.dCn.Cnp; 
Cnr   = p.derivatives.dCn.Cnr;  
CndA  = p.derivatives.dCn.CndA;  
CndR  = p.derivatives.dCn.CndR;  

Cn    = Cnb * beta + Cnp * phat  + Cnr * rhat + CndA * dA + CndR * dR ;

% Assemble Faero
FX_BODY  = CX * qbar * wingArea;
FY_BODY  = CY * qbar * wingArea;
FZ_BODY  = CZ * qbar * wingArea;
FaeroB    = [FX_BODY; FY_BODY; FZ_BODY];

% Assemble Maero
rollingMoment  = qbar * wingArea * wingSpan * Cl;
pitchingMoment = qbar * wingArea * chord    * Cm;
yawingMoment   = qbar * wingArea * wingSpan * Cn;
MaeroB          = [rollingMoment; pitchingMoment; yawingMoment];

end
function Rsb             = Cbody2stability(alpha)                                    
% Direction Cosine Matrix from Body -> Stability(alpha)
% Reference Stevens: Aircraft Control and Simulation pag.77

% DCM stability
Rsb = [ cos(alpha), 0 ,  sin(alpha);
           0      , 1 ,     0      ;
       -sin(alpha), 0 ,  cos(alpha)];
end
function Rwb             = Cbody2wind(alpha,beta)                                    
% Direction Cosine Matrix from Body -> Stability(alpha) |-> Wind (beta)
% Reference Stevens: Aircraft Control and Simulation pag.77

% DCM stability
Rsb = [ cos(alpha), 0 ,  sin(alpha);
           0      , 1 ,     0      ;
       -sin(alpha), 0 ,  cos(alpha)];
        
Rws = [ cos(beta),sin(beta), 0;
       -sin(beta),cos(beta), 0;
            0    ,   0     , 1];
          
Rwb = Rws*Rsb;
end
function H               = Heuler(PHI)                                               
% Rates of Euler Angles Matrix
% Reference Stevens: Aircraft Control and Simulation pag.20
phi   = PHI(1);
theta = PHI(2);
psi   = PHI(3);

H = [1 , sin(phi)*tan(theta) , cos(phi)*tan(theta)   ;
     0 ,       cos(phi)      ,      -sin(phi)        ;
     0 ,(sin(phi)/cos(theta)), (cos(phi)/cos(theta))];
end
function                   plotForwardSimulation(time,Xsim,Udata,Ydata)              
  Vt_sim     = Xsim(:,1);
  % beta_sim   = Xsim(:,2);  
  alpha_sim  = Xsim(:,3);
  % phi_sim    = Xsim(:,4);
  theta_sim  = Xsim(:,5);
  % psi_sim    = Xsim(:,6);
  % p_sim      = Xsim(:,7);
  q_sim      = Xsim(:,8);
  % r_sim      = Xsim(:,9);
  
  LineWidth = 2;
  figure;
  %----------------------------------------------------------------- Airspeed
  subplot(5,1,1);hold on;grid on;ylabel('V_{T} [m/s]');
  title('Model Validation');
  plot(time,Ydata(1,:),'LineWidth',LineWidth);
  plot(time,Vt_sim    ,'LineWidth',LineWidth);
  xlim([0,time(end)]);
  ylim([15,35]);
  legend('Validation Data','Estimated model');
  %---------------------------------------------------------- Angle of Attack
  subplot(5,1,2);hold on;grid on;ylabel('\alpha [deg]');  
  plot(time,rad2deg(Ydata(2,:)),'LineWidth',LineWidth);
  plot(time,rad2deg(alpha_sim),'LineWidth',LineWidth);
  xlim([0,time(end)]);
  ylim([-6,5]);
  %------------------------------------------------------------- Pitch Angles
  subplot(5,1,3);hold on;grid on;ylabel('\theta [deg]');
  plot(time,rad2deg(Ydata(3,:)),'LineWidth',LineWidth);
  plot(time,rad2deg(theta_sim),'LineWidth',LineWidth);
  xlim([0,time(end)]);
  %--------------------------------------------------------------- Pitch Rate
  subplot(5,1,4);hold on;grid on;ylabel('q [deg/s]');
  plot(time,rad2deg(Ydata(4,:)),'LineWidth',LineWidth);
  plot(time,rad2deg(q_sim),'LineWidth',LineWidth);
  xlim([0,time(end)]);
  ylim([-15,26]);
  %----------------------------------------------------------------- Elevator
  subplot(5,1,5);hold on;grid on;ylabel('\delta_{e} [deg]');
  plot(time,rad2deg(Udata),'LineWidth',LineWidth);
  xlim([0,time(end)]);ylim([-1,11]);
  xlabel('time [s]');
end