function [one_sample,N_samples,Jac_one_step] = Runge_Kutta4(ode,states,controls,theta)
import casadi.*
global N fs

Mrk4 = 4;             % step RK4   
dt = 1/fs/Mrk4;       % integration step for ode

k1 = ode(states          ,controls,theta);
k2 = ode(states+dt/2.0*k1,controls,theta);
k3 = ode(states+dt/2.0*k2,controls,theta);
k4 = ode(states+dt*k3    ,controls,theta);
xf = states + dt/6.0*(k1+2*k2+2*k3+k4);

% Define Jacobian one step 
Jac_one_step = Function('Jac_one_step',{states, controls, theta},{jacobian(xf,theta)});

% Create a function that simulates one step propagation in a sample
one_step = Function('one_step',{states, controls, theta},{xf});

X = states;
for i=1:Mrk4
    X = one_step(X, controls, theta);
end

% Create a function that simulates all step propagation on a sample
one_sample = Function('one_sample',{states, controls, theta}, {X});
% speedup trick: expand into scalar operations
one_sample = one_sample.expand();

%% Compute Forward Simulation =============================================
% choose number of simulation step
N_samples = one_sample.mapaccum('all_samples', N);

end