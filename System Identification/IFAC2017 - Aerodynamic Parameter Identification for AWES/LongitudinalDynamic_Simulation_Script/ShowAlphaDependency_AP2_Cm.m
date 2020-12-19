clc;clear all;clc;
load('systemID_Data.mat');
load('LongitudinalAeroAP2.mat');
load('PhysicalProperties');

IndexIn  = 14500;
IndexOut = 60000;
%% Get flight test ====================================================
indexFlightTest = 6;
sampleTime      = systemID_Data(1).ServoDemands.time(2) - systemID_Data(1).ServoDemands.time(1);
i               = indexFlightTest;    
N               = length(systemID_Data(i).ServoDemands.packetData.elevator(IndexIn:IndexOut));    
time            = [0:sampleTime:(N-1)*sampleTime]';             % time array  for plot

%% get Control Surfaces ----------------------------------------- [rad]
deltaA_Port_in = systemID_Data(i).ProcessedSensorData.packetData.servoDeflections.portAileron(IndexIn:IndexOut);
deltaA_Stbd_in = systemID_Data(i).ProcessedSensorData.packetData.servoDeflections.stbdAileron(IndexIn:IndexOut); 
deltaE_in      = systemID_Data(i).ProcessedSensorData.packetData.servoDeflections.stbdElevator(IndexIn:IndexOut);

%% get aero States -----------------------------------------------------
alpha_in       = systemID_Data(i).processed.alphaFiltered(IndexIn:IndexOut);    % angle of attack  [Filtered]  [rad]    
V_in           = systemID_Data(i).processed.TAS_Filtered(IndexIn:IndexOut);     % [m/s]   
qbar_in        = systemID_Data(i).processed.qbarFiltered(IndexIn:IndexOut);

%% get body angural rate and accel-------------------------------------
p_in          = systemID_Data(i).processed.angularRate(IndexIn:IndexOut,1);    % [wx,wy,wz] = [p,q,r] [rad/s]
q_in          = systemID_Data(i).processed.angularRate(IndexIn:IndexOut,2);    % [wx,wy,wz] = [p,q,r] [rad/s]
r_in          = systemID_Data(i).processed.angularRate(IndexIn:IndexOut,3);    % [wx,wy,wz] = [p,q,r] [rad/s]

%% Plot ===============================================================    
figure;
subplot(4,1,1);hold on;grid on;ylabel('Pitch Rate [rad/s]');      
plot  (time, q_in ,'Color','b','LineWidth',1.5 );
legend('\omega_{Y}=q measured');
subplot(4,1,2);grid on;hold on;ylabel('Elevator [deg]');
stairs(time, rad2deg(deltaE_in),'Color','b','LineWidth',1.5);
legend('elevator');             
subplot(4,1,3);grid on;hold on;ylabel('[deg]');
plot(time, rad2deg(alpha_in),'Color','b','LineWidth',1.5);
legend('Angle of Attack');
subplot(4,1,4);grid on;hold on;ylabel('[m/s]');
plot(time, V_in,'Color','b','LineWidth',1.5);
legend('True Airspeed');
xlabel('time [s]');

%% SETTINGS ===============================================================
ts                 = sampleTime;
fs                 = 1/ts;                     % Sampling frequency [hz]
N_steps_per_sample = 1;   
dt                 = 1/fs/N_steps_per_sample;  % integration step for ode 

%% Get true derivatives =================================================
alphaInTime            =  alpha_in;    
Cm_alpha_inTime        =  ComputeDerivativeInTime(Cm_alpha       ,alphaTable_Cm_alpha,alphaInTime);
Cm_deltaA_Port_inTime  =  ComputeDerivativeInTime(Cm_deltaA_Port ,alphaTable         ,alphaInTime);
Cm_deltaA_Stbd_inTime  =  ComputeDerivativeInTime(Cm_deltaA_Stbd ,alphaTable         ,alphaInTime);
Cm_deltaE_inTime       =  ComputeDerivativeInTime(Cm_deltaE      ,alphaTable         ,alphaInTime);      
Cm_q_inTime            =  ComputeDerivativeInTime(Cm_q           ,alphaTable         ,alphaInTime);

mean_Cm_0              =  Cm_0;

mean_Cm_alpha          =  mean(Cm_alpha_inTime);
mean_Cm_dA_Port        =  mean(Cm_deltaA_Port_inTime);
mean_Cm_dA_Stbd        =  mean(Cm_deltaA_Stbd_inTime);
mean_Cm_dE             =  mean(Cm_deltaE_inTime);
mean_Cm_q              =  mean(Cm_q_inTime);

std_Cm_alpha           =  std(Cm_alpha_inTime);
std_Cm_dA_Port         =  std(Cm_deltaA_Port_inTime);
std_Cm_dA_Stbd         =  std(Cm_deltaA_Stbd_inTime);
std_Cm_dE              =  std(Cm_deltaE_inTime);
std_Cm_q               =  std(Cm_q_inTime);

confidenceIterval_Cm_alpha   = [mean_Cm_alpha   - std_Cm_alpha   , mean_Cm_alpha   + std_Cm_alpha  ];
confidenceIterval_Cm_dA_Port = [mean_Cm_dA_Port - std_Cm_dA_Port , mean_Cm_dA_Port + std_Cm_dA_Port];
confidenceIterval_Cm_dA_Stbd = [mean_Cm_dA_Stbd - std_Cm_dA_Stbd , mean_Cm_dA_Stbd + std_Cm_dA_Stbd];
confidenceIterval_Cm_dE      = [mean_Cm_dE      - std_Cm_dE      , mean_Cm_dE      + std_Cm_dE     ];
confidenceIterval_Cm_q       = [mean_Cm_q       - std_Cm_q       , mean_Cm_q       + std_Cm_q      ];

Cm_true = [mean_Cm_0;
           mean_Cm_alpha;
           mean_Cm_dA_Port;
           mean_Cm_dA_Stbd;
           mean_Cm_dE;
           mean_Cm_q];

std_Cm = [ std_Cm_alpha;
           std_Cm_dA_Port;
           std_Cm_dA_Stbd;
           std_Cm_dE;
           std_Cm_q];

figure('units','normalized','outerposition',[0 0 1 1]);  
subplot(3,1,1);hold on;grid on;
plot(time,  Cm_0 + Cm_alpha_inTime,                     'LineWidth',2,'Color','b');   
plot(time, (Cm_0 + mean_Cm_alpha)*ones(length(time),1), 'LineWidth',1,'Color','r','LineStyle','--');
legend('Cm_{0}+Cm_{alpha}(t)','mean value');

subplot(3,1,2);hold on;grid on;
plot(time, Cm_q_inTime,                                 'LineWidth',2,'Color','b');   
plot(time, mean_Cm_q*ones(length(time),1),              'LineWidth',1,'Color','r','LineStyle','--');
legend('Cm_{q}(t)','mean value');

subplot(3,1,3);hold on;grid on;
plot(time, Cm_deltaE_inTime,                            'LineWidth',2,'Color','b' );   
plot(time, mean_Cm_dE*ones(length(time),1),             'LineWidth',1,'Color','r','LineStyle','--');
legend('Cm_{deltaE}(t)','mean value');

%percentage_deviation = std_Cm ./ Cm_true(2:end) * 100
theta_true  = Cm_true;

%% Get geometry AP2 and Inertia =========================================
Sref = PhysicalProperties.Value.wingArea;
bref = PhysicalProperties.Value.wingSpan;
cref = PhysicalProperties.Value.chord;

Jx   = PhysicalProperties.Value.inertias.variableParameter.value(1);
Jy   = PhysicalProperties.Value.inertias.variableParameter.value(2);
Jz   = PhysicalProperties.Value.inertias.variableParameter.value(3);
Jxz  = PhysicalProperties.Value.inertias.variableParameter.value(4);

%% Model generation via casADi ==========================================
import casadi.*
p             = MX.sym('p');  
q             = MX.sym('q');  % state
r             = MX.sym('r');

deltaA_Port   = MX.sym('deltaA_Port');
deltaA_Stbd   = MX.sym('deltaA_Stbd');
deltaE        = MX.sym('deltaE');

alpha         = MX.sym('alpha');
V             = MX.sym('V');
qbar          = MX.sym('qbar');

states        = [q];
controls      = [p;r;deltaA_Port;deltaA_Stbd;deltaE;alpha;V;qbar];

Cm_0           = MX.sym('Cm_0');
Cm_alpha       = MX.sym('Cm_alpha');
Cm_deltaA_Port = MX.sym('Cm_deltaA_Port');
Cm_deltaA_Stbd = MX.sym('Cm_deltaA_Stbd');
Cm_deltaE      = MX.sym('Cm_deltaE');
Cm_q           = MX.sym('Cm_q');

theta         =  [Cm_0;Cm_alpha;Cm_deltaA_Port;Cm_deltaA_Stbd;Cm_deltaE;Cm_q];

qhat = (cref*q) / (2*V);
Cm   = Cm_0 + Cm_alpha*alpha + Cm_q*qhat + Cm_deltaE*deltaE + Cm_deltaA_Port*deltaA_Port + Cm_deltaA_Stbd*deltaA_Stbd;
M    = qbar*Sref*cref*Cm;
rhs  = solve(Jy,(M - Jxz*p^2 + Jxz*r^2 - Jx*p*r + Jz*p*r));
% Form an ode function
ode  = Function('ode',{states,controls,theta},{rhs});   

%% build integrator: RK4 ==============================================
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

% choose number of simulation step
all_samples = one_sample.mapaccum('all_samples', N);
%% Forward Simulation ===================================================
% select input, initial condition and y_data for fitting
u_data = [p_in,r_in,deltaA_Port_in,deltaA_Stbd_in,deltaE_in,alpha_in,V_in,qbar_in]';                   
x0     = DM([q_in(1)]);                          
X_sim  = all_samples(x0, u_data, repmat(theta_true,1,N)); 
X_sim  = full(X_sim)';

residual = X_sim - q_in;
mean_res = mean(residual)
var_res  = var(residual)

%% ======================================================================
figure;
subplot(2,1,1);
plot(time,X_sim,'Color','r','LineWidth',2);hold on;grid on;
plot(time,q_in ,'Color','b','LineWidth',2);
xlabel('time');ylabel('[rad/sec]');
legend('q simalted with constant true derivatives','q from FCC');

subplot(2,1,2);hold on;grid on;
plot(time,residual ,'Color','g','LineWidth',2);ylabel('[rad/sec]');
legend('residual');

