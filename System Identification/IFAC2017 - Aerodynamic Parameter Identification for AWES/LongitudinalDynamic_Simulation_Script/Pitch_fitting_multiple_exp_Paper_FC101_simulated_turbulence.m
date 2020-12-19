clc;clear all;close all;
load('systemID_Data_FC101_simulated.mat')
load('PhysicalProperties.mat')
rng;

Case = struct;

%option = 'FCC'; % fitting from FCC data
option = 'IVP';  % fitting from IVP data

%% Experiment1 ============================================================
IndexIn         = 21640;
IndexOut        = 23350;

% Get flight test 
indexFlightTest = 3;
sampleTime      = systemID_Data(1).ServoDemands.time(2) - systemID_Data(1).ServoDemands.time(1);
i               = indexFlightTest;    

Exp1            = struct; 
Exp1.N          = length(systemID_Data(i).ServoDemands.packetData.elevator(IndexIn:IndexOut));    
Exp1.time       = [0:sampleTime:(Exp1.N-1)*sampleTime]'; 

% get Control Surfaces ---------------------------------------------- [rad]
Exp1.deltaE_in  = systemID_Data(i).ProcessedSensorData.packetData.servoDeflections.stbdElevator(IndexIn:IndexOut);
% get aero States --------------------------------------------------------- 
Exp1.alpha_in   = systemID_Data(i).processed.alphaFiltered(IndexIn:IndexOut);    
Exp1.V_in       = systemID_Data(i).processed.TAS_Filtered(IndexIn:IndexOut);    
% get body angural rate and accel -----------------------------------------
Exp1.q_in       = systemID_Data(i).processed.angularRate(IndexIn:IndexOut,2);    % [wx,wy,wz] = [p,q,r] [rad/s]
Exp1.qdot_in   = systemID_Data(i).processed.bodyAccel(IndexIn:IndexOut,2);    % [wx,wy,wz] = [p,q,r] [rad/s]

Exp1.qdot_in   = Exp1.qdot_in + deg2rad(0.1).*randn(Exp1.N,1);

figure;hold on;grid on;
plot(Exp1.q_in);plot(Exp1.qdot_in)

%% Experiment2 ============================================================
IndexIn         = 33030;
IndexOut        = 35230;

% Get flight test 
indexFlightTest = 3;
sampleTime      = systemID_Data(1).ServoDemands.time(2) - systemID_Data(1).ServoDemands.time(1);
i               = indexFlightTest;    

Exp2            = struct; 
Exp2.N          = length(systemID_Data(i).ServoDemands.packetData.elevator(IndexIn:IndexOut));    
Exp2.time       = [0:sampleTime:(Exp2.N-1)*sampleTime]'; 

% get Control Surfaces ---------------------------------------------- [rad]
Exp2.deltaE_in  = systemID_Data(i).ProcessedSensorData.packetData.servoDeflections.stbdElevator(IndexIn:IndexOut);
% get aero States ---------------------------------------------------------
Exp2.alpha_in  = systemID_Data(i).processed.alphaFiltered(IndexIn:IndexOut);    
Exp2.V_in      = systemID_Data(i).processed.TAS_Filtered(IndexIn:IndexOut);

plot(Exp2.V_in);hold on;grid on;

% get body angural rate and accel -----------------------------------------
Exp2.q_in      = systemID_Data(i).processed.angularRate(IndexIn:IndexOut,2);    % [wx,wy,wz] = [p,q,r] [rad/s]
Exp2.qdot_in   = systemID_Data(i).processed.bodyAccel(IndexIn:IndexOut,2);    % [wx,wy,wz] = [p,q,r] [rad/s]

Exp2.qdot_in   = Exp2.qdot_in +deg2rad(0.1).*randn(Exp2.N,1); 

figure;hold on;grid on;
plot(Exp2.q_in);plot(Exp2.qdot_in)

%% SETTINGS ===============================================================
ts                 = sampleTime;
fs                 = 1/ts;                     % Sampling frequency [hz]
N_steps_per_sample = 4;   
dt                 = 1/fs/N_steps_per_sample;  % integration step for ode 

% Get true derivatives =================================================
% theta taken from fcc updated
%   Cm_0           = -0.0300
%   Cm_alpha       = -0.0520
%   Cm_deltaA_Port = -0.0394
%   Cm_deltaA_Stbd = -0.0394
%   Cm_deltaE      = -1.0415
%   Cm_q           = -11.2947

theta_truth  = [-0.0300;-0.0520;-1.0415;-11.2947];
np           = length(theta_truth);
scale        = ones(np,1);
theta_guess  = theta_truth + 0.1*theta_truth;
nx           = 1;           % q = pitch rate

% Get geometry AP2 and Inertia =========================================
Sref = PhysicalProperties.Value.wingArea;
bref = PhysicalProperties.Value.wingSpan;
cref = PhysicalProperties.Value.chord;

Jx   = PhysicalProperties.Value.inertias.variableParameter.value(1);
Jy   = PhysicalProperties.Value.inertias.variableParameter.value(2);
Jz   = PhysicalProperties.Value.inertias.variableParameter.value(3);
Jxz  = PhysicalProperties.Value.inertias.variableParameter.value(4);

% Model generation via casADi ==========================================
import casadi.* 
q             = MX.sym('q');  % state

deltaE        = MX.sym('deltaE');
alpha         = MX.sym('alpha');
V             = MX.sym('V');

states        = [q];
controls      = [deltaE;alpha;V];

Cm_0          = MX.sym('Cm_0');
Cm_alpha      = MX.sym('Cm_alpha');
Cm_deltaE     = MX.sym('Cm_deltaE');
Cm_q          = MX.sym('Cm_q');

theta         =  [Cm_0;Cm_alpha;Cm_deltaE;Cm_q];

qhat = (cref*q) / (2*V);
Cm   = Cm_0 + Cm_alpha*alpha + Cm_q*qhat + Cm_deltaE*deltaE;
M    = 0.5*1.23*V^2*Sref*cref*Cm;
rhs  = solve(Jy,M);
% Form an ode function
ode  = Function('ode',{states,controls,theta},{rhs});   

% build integrator: RK4 ==============================================
k1 = ode(states          ,controls,theta);
k2 = ode(states+dt/2.0*k1,controls,theta);
k3 = ode(states+dt/2.0*k2,controls,theta);
k4 = ode(states+dt*k3    ,controls,theta);
xf = states + dt/6.0*(k1+2*k2+2*k3+k4);
% Create a function that simulates one step propagation in a sample
one_step = Function('one_step',{states, controls, theta},{xf});

X = states;
for i=1:N_steps_per_sample
X = one_step(X, controls, theta);
end

% Create a function that simulates all step propagation on a sample
one_sample = Function('one_sample',{states, controls, theta}, {X});
% speedup trick: expand into scalar operations
one_sample = one_sample.expand();

% Forward Simulation ===================================================== 
% real IMU: noise expressed in standard deviation
rho       = 1.23;
n_qbar    = 8;    % [Pa]

sigma_V       = sqrt(2*n_qbar/rho);
sigma_a       = deg2rad(0.5);   
sigma_q       = deg2rad(0.1);
sigma_qdot    = deg2rad(0.2);
%sigma_theta  = deg2rad(0.1); 

quantError_de = deg2rad(0.25); % quatization error

% Experiement 1 -----------------------------------------------------------
Exp1.Udata        = [Exp1.deltaE_in,Exp1.alpha_in,Exp1.V_in]'; 
Exp1.x0           = DM([Exp1.q_in(1)]); 

% choose number of simulation step
Exp1.all_samples  = one_sample.mapaccum('all_samples', Exp1.N);
Exp1.X_truth      = Exp1.all_samples(Exp1.x0, Exp1.Udata, repmat(theta_truth,1,Exp1.N));
Exp1.yn           = sigma_q*randn(Exp1.N,1);  % noise gyro pitch rate

if strcmp(option,'FCC')
  Exp1.y_data = Exp1.q_in;
else
  Exp1.y_data = full(Exp1.X_truth)' + Exp1.yn;
end

% check IVP simulated vs FCC
figure;
plot(Exp1.y_data);hold on;plot(Exp1.q_in)

% Experiement 2 -----------------------------------------------------------
Exp2.Udata        = [Exp2.deltaE_in,Exp2.alpha_in,Exp2.V_in]'; 
Exp2.x0           = DM([Exp2.q_in(1)]); 
% choose number of simulation step
Exp2.all_samples  = one_sample.mapaccum('all_samples', Exp2.N);
Exp2.X_truth      = Exp2.all_samples(Exp1.x0, Exp2.Udata, repmat(theta_truth,1,Exp2.N));
Exp2.yn           = sigma_q*randn(Exp2.N,1);  % noise gyro pitch rate

if strcmp(option,'FCC')
  Exp2.y_data = Exp2.q_in;
else
  Exp2.y_data = full(Exp2.X_truth)' + Exp2.yn;
end

% check IVP simulated vs FCC
figure;
plot(Exp2.y_data);hold on;plot(Exp2.q_in)

%% add noise to measurements input measurement ============================
Exp1.UdataNoisy   = [round(Exp1.deltaE_in / quantError_de)*quantError_de,...
                     Exp1.alpha_in  + sigma_a.*randn(Exp1.N,1),...
                     Exp1.V_in      + sigma_V.*randn(Exp1.N,1)]'; 
                 
Exp2.UdataNoisy   = [round(Exp2.deltaE_in / quantError_de)*quantError_de,...
                     Exp2.alpha_in  + sigma_a.*randn(Exp2.N,1),...
                     Exp2.V_in      + sigma_V.*randn(Exp2.N,1)]'; 

%% Set Identification Algorithm =========================================== 
disp('Multiple shooting: Multiple Experiment')
X1 = MX.sym('X1',nx, Exp1.N); % continuity for exp1
X2 = MX.sym('X2',nx, Exp2.N); % continuity for exp2

theta_scale = theta.*scale;   % params contains my design variable 

Xn1 = one_sample.map({X1, Exp1.UdataNoisy, theta_scale});Xn1 = Xn1{1};
Xn2 = one_sample.map({X2, Exp2.UdataNoisy, theta_scale});Xn2 = Xn2{1};

% gap-closing constraints
gaps1 = Xn1(:,1:end-1) - X1(:,2:end);
gaps2 = Xn2(:,1:end-1) - X2(:,2:end);

% error = y_data - x_symbolic  ============================================ 
e_exp1 = vec(Exp1.y_data'-X1);
e_exp2 = vec(Exp2.y_data'-X2);

%% multiple shooting casadi only ============================ 2 experiments
w      = {theta;vec(X1);vec(X2)};
w0     = [theta_guess; vec(Exp1.y_data') ; vec(Exp2.y_data')];
lbw    = -inf*ones(size(w0));
ubw    =  inf*ones(size(w0));
J      = 0.5*(e_exp1'*e_exp1)+0.5*(e_exp2'*e_exp2);
g      = {vec(gaps1);vec(gaps2)};
lbg    = zeros(nx*(Exp1.N-1)+nx*(Exp2.N-1),1);
ubg    = zeros(nx*(Exp1.N-1)+nx*(Exp2.N-1),1);

w  = vertcat(w{:});
g  = vertcat(g{:});

% use Gauss-Newton Hessian
Jw      = jacobian([e_exp1;e_exp2],w);
H       = triu(Jw'*Jw);
sigma   = MX.sym('sigma');
hessLag = Function('nlp_hess_l',struct('x',w,'lam_f',sigma, 'hess_gamma_x_x',sigma*H),...
                     char('x','p','lam_f','lam_g'), char('hess_gamma_x_x'));

% Create an NLP solver
prob   = struct('f', J, 'x', w, 'g', g);
% option IPOPT
opts                             = struct;
opts.ipopt.linear_solver         = 'ma27';
opts.ipopt.max_iter              = 50;
%opts.ipopt.hessian_approximation = 'exact';  % 3 iteration
opts.hess_lag                    = hessLag; % 5 iteration

% Solve the NLP
solver  = nlpsol('solver', 'ipopt', prob,opts);
w_sol   = solver('x0', w0, 'lbx', lbw, 'ubx', ubw,'lbg', lbg, 'ubg', ubg);

Case.theta_noTurb = scale.*full(w_sol.x(1:np)); % retrieve parameters

X_est        = full(w_sol.x(np+1:end));    % retrieve states
Exp1.X_est   = X_est(1:Exp1.N);
Exp2.X_est   = X_est(Exp1.N+1:end);

%% Compute estimated Covariance Matrix
% assemble covariance matrix for multiexperiment
% [Cm_0;Cm_alpha;Cm_deltaE;Cm_q] order regressors

% Phi1 = [ones(Exp1.N,1),Exp1.alpha_in,Exp1.deltaE_in, (cref.*Exp1.q_in)./(2.*Exp1.V_in)];
% Phi2 = [ones(Exp2.N,1),Exp2.alpha_in,Exp2.deltaE_in, (cref.*Exp2.q_in)./(2.*Exp2.V_in)];
% Phi  = [Phi1;Phi2];
% %sigma_est2 = (full(w_sol.f)*8)/(Exp1.N+Exp2.N-np);
% Case.cov_noTurb = sigma_q^2.*inv(Phi'*Phi);

%% Retrieve covariance matrix using casadi
Jac_Xn1Fun = Function('Xn1Fun',{X1,theta_scale},{jacobian(Xn1,theta_scale)});
Jac_Xn1Fun.printDimensions();
Jac_Xn2Fun = Function('Xn2Fun',{X2,theta_scale},{jacobian(Xn2,theta_scale)});
Jac_Xn2Fun.printDimensions();

JacExp1 = full(Jac_Xn1Fun(Exp1.q_in,Case.theta_noTurb));
JacExp2 = full(Jac_Xn2Fun(Exp2.q_in,Case.theta_noTurb));

Case.cov_noTurb = sigma_q^2.*inv(JacExp1'*JacExp1 + JacExp2'*JacExp2);

% Print some information ==================================================
deviationPercentage = abs((theta_truth-Case.theta_noTurb)./theta_truth)*100;
DispResult          = [theta_truth,Case.theta_noTurb,deviationPercentage];
disp('     Truth   Estimate    Dev%')
disp(DispResult)

%%
load('systemID_Data_FC101_simulated_wind10_turb5.mat')
rng(1);

%% Experiment1 ============================================================
IndexIn         = 37420;
IndexOut        = 39127;%39550;

% Get flight test 
indexFlightTest = 3;
sampleTime      = systemID_Data(1).ServoDemands.time(2) - systemID_Data(1).ServoDemands.time(1);
i               = indexFlightTest;    

Exp1            = struct; 
Exp1.N          = length(systemID_Data(i).ServoDemands.packetData.elevator(IndexIn:IndexOut));    
Exp1.time       = [0:sampleTime:(Exp1.N-1)*sampleTime]'; 

% get Control Surfaces ---------------------------------------------- [rad]
Exp1.deltaE_in  = systemID_Data(i).ProcessedSensorData.packetData.servoDeflections.stbdElevator(IndexIn:IndexOut);
% get aero States --------------------------------------------------------- 
Exp1.alpha_in   = systemID_Data(i).processed.alphaFiltered(IndexIn:IndexOut);    
Exp1.V_in       = systemID_Data(i).processed.TAS_Filtered(IndexIn:IndexOut);    
% get body angural rate and accel -----------------------------------------
Exp1.q_in       = systemID_Data(i).processed.angularRate(IndexIn:IndexOut,2);    % [wx,wy,wz] = [p,q,r] [rad/s]

%% Experiment2 ============================================================
IndexIn         = 60800;
IndexOut        = 62930;

% Get flight test 
indexFlightTest = 3;
sampleTime      = systemID_Data(1).ServoDemands.time(2) - systemID_Data(1).ServoDemands.time(1);
i               = indexFlightTest;    

Exp2            = struct; 
Exp2.N          = length(systemID_Data(i).ServoDemands.packetData.elevator(IndexIn:IndexOut));    
Exp2.time       = [0:sampleTime:(Exp2.N-1)*sampleTime]'; 

% get Control Surfaces ---------------------------------------------- [rad]
Exp2.deltaE_in  = systemID_Data(i).ProcessedSensorData.packetData.servoDeflections.stbdElevator(IndexIn:IndexOut);

% get aero States ---------------------------------------------------------
Exp2.alpha_in  = systemID_Data(i).processed.alphaFiltered(IndexIn:IndexOut);    
Exp2.V_in      = systemID_Data(i).processed.TAS_Filtered(IndexIn:IndexOut);    

plot(Exp2.V_in)
% get body angural rate and accel -----------------------------------------
Exp2.q_in      = systemID_Data(i).processed.angularRate(IndexIn:IndexOut,2);    % [wx,wy,wz] = [p,q,r] [rad/s]

%% Experiement 1 -----------------------------------------------------------
Exp1.Udata        = [Exp1.deltaE_in,Exp1.alpha_in,Exp1.V_in]'; 
Exp1.x0           = DM([Exp1.q_in(1)]); 

% choose number of simulation step
Exp1.all_samples  = one_sample.mapaccum('all_samples', Exp1.N);
Exp1.X_truth      = Exp1.all_samples(Exp1.x0, Exp1.Udata, repmat(theta_truth,1,Exp1.N));
Exp1.yn           = sigma_q*randn(Exp1.N,1);  % noise gyro pitch rate

if strcmp(option,'FCC')
  Exp1.y_data = Exp1.q_in;
else
  Exp1.y_data = full(Exp1.X_truth)' + Exp1.yn;
end

% check IVP simulated vs FCC
figure;
plot(Exp1.y_data);hold on;plot(Exp1.q_in)

% Experiement 2 -----------------------------------------------------------
Exp2.Udata        = [Exp2.deltaE_in,Exp2.alpha_in,Exp2.V_in]'; 
Exp2.x0           = DM([Exp2.q_in(1)]); 
% choose number of simulation step
Exp2.all_samples  = one_sample.mapaccum('all_samples', Exp2.N);
Exp2.X_truth      = Exp2.all_samples(Exp1.x0, Exp2.Udata, repmat(theta_truth,1,Exp2.N));
Exp2.yn           = sigma_q*randn(Exp2.N,1);  % noise gyro pitch rate

if strcmp(option,'FCC')
  Exp2.y_data = Exp2.q_in;
else
  Exp2.y_data = full(Exp2.X_truth)' + Exp2.yn;
end

%check IVP simulated vs FCC
figure;
plot(Exp2.y_data);hold on;plot(Exp2.q_in)

%% add noise to measurements input measurement ============================
Exp1.UdataNoisy   = [round(Exp1.deltaE_in / quantError_de)*quantError_de,...
                     Exp1.alpha_in  + sigma_a.*randn(Exp1.N,1),...
                     Exp1.V_in      + sigma_V.*randn(Exp1.N,1)]'; 
                 
Exp2.UdataNoisy   = [round(Exp2.deltaE_in / quantError_de)*quantError_de,...
                     Exp2.alpha_in  + sigma_a.*randn(Exp2.N,1),...
                     Exp2.V_in      + sigma_V.*randn(Exp2.N,1)]'; 

%% Set Identification Algorithm =========================================== 
disp('Multiple shooting: Multiple Experiment')
X1 = MX.sym('X1',nx, Exp1.N); % continuity for exp1
X2 = MX.sym('X2',nx, Exp2.N); % continuity for exp2

theta_scale = theta.*scale;   % params contains my design variable 

Xn1 = one_sample.map({X1, Exp1.UdataNoisy, theta_scale});Xn1 = Xn1{1};
Xn2 = one_sample.map({X2, Exp2.UdataNoisy, theta_scale});Xn2 = Xn2{1};

% gap-closing constraints
gaps1 = Xn1(:,1:end-1) - X1(:,2:end);
gaps2 = Xn2(:,1:end-1) - X2(:,2:end);

% error = y_data - x_symbolic  ============================================ 
e_exp1 = vec(Exp1.y_data'-X1);
e_exp2 = vec(Exp2.y_data'-X2);

%% multiple shooting casadi only ============================ 2 experiments
w      = {theta;vec(X1);vec(X2)};
w0     = [theta_guess; vec(Exp1.y_data') ; vec(Exp2.y_data')];
lbw    = -inf*ones(size(w0));
ubw    =  inf*ones(size(w0));
J      = 0.5*(e_exp1'*e_exp1)+0.5*(e_exp2'*e_exp2);
g      = {vec(gaps1);vec(gaps2)};
lbg    = zeros(nx*(Exp1.N-1)+nx*(Exp2.N-1),1);
ubg    = zeros(nx*(Exp1.N-1)+nx*(Exp2.N-1),1);

w  = vertcat(w{:});
g  = vertcat(g{:});

% use Gauss-Newton Hessian
Jw      = jacobian([e_exp1;e_exp2],w);
H       = triu(Jw'*Jw);
sigma   = MX.sym('sigma');
hessLag = Function('nlp_hess_l',struct('x',w,'lam_f',sigma, 'hess_gamma_x_x',sigma*H),...
                     char('x','p','lam_f','lam_g'), char('hess_gamma_x_x'));

% Create an NLP solver
prob   = struct('f', J, 'x', w, 'g', g);
% option IPOPT
opts                             = struct;
opts.ipopt.linear_solver         = 'ma27';
opts.ipopt.max_iter              = 50;
%opts.ipopt.hessian_approximation = 'exact';  % 3 iteration
opts.hess_lag                    = hessLag; % 5 iteration

% Solve the NLP
solver  = nlpsol('solver', 'ipopt', prob,opts);
w_sol   = solver('x0', w0, 'lbx', lbw, 'ubx', ubw,'lbg', lbg, 'ubg', ubg);

Case.theta_Turb = scale.*full(w_sol.x(1:np)); % retrieve parameters
X_est        = full(w_sol.x(np+1:end));    % retrieve states
Exp1.X_est   = X_est(1:Exp1.N);
Exp2.X_est   = X_est(Exp1.N+1:end);

%% Compute estimated Covariance Matrix
% assemble covariance matrix for multiexperiment
% [Cm_0;Cm_alpha;Cm_deltaE;Cm_q] order regressors

% Phi1 = [ones(Exp1.N,1),Exp1.alpha_in,Exp1.deltaE_in, (cref.*Exp1.q_in)./(2.*Exp1.V_in)];
% Phi2 = [ones(Exp2.N,1),Exp2.alpha_in,Exp2.deltaE_in, (cref.*Exp2.q_in)./(2.*Exp2.V_in)];
% Phi  = [Phi1;Phi2];
% %sigma_est2 = (full(w_sol.f)*8)/(Exp1.N+Exp2.N-np); % estimated variance 
% Case.cov_Turb = sigma_q^2.*inv(Phi'*Phi);

%% Retrieve covariance matrix using casadi
Jac_Xn1Fun = Function('Xn1Fun',{X1,theta_scale},{jacobian(Xn1,theta_scale)});
Jac_Xn1Fun.printDimensions();
Jac_Xn2Fun = Function('Xn2Fun',{X2,theta_scale},{jacobian(Xn2,theta_scale)});
Jac_Xn2Fun.printDimensions();

JacExp1 = full(Jac_Xn1Fun(Exp1.q_in,Case.theta_Turb));
JacExp2 = full(Jac_Xn2Fun(Exp2.q_in,Case.theta_Turb));

Case.cov_Turb = sigma_q^2.*inv(JacExp1'*JacExp1 + JacExp2'*JacExp2);

plotConfidenceEllipsoids(theta_truth,Case)

% Print some information ==================================================
deviationPercentage = abs((theta_truth-Case.theta_Turb)./theta_truth)*100;
DispResult          = [theta_truth,Case.theta_Turb,deviationPercentage];
disp('     Truth   Estimate    Dev%')
disp(DispResult)



