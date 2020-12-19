# -*- coding: utf-8 -*-
"""
Created on Thu Jul 07 16:21:12 2016

@author: Gianni
"""

import casadi as ca
import pylab as pl
import casiopeia as cp
import scipy.io

# Data initialization
dataInit = scipy.io.loadmat('DOE_initialization.mat')
x0       = dataInit["x0"]
u_guess  = dataInit["u_DOE_init"]
time     = dataInit["t_DOE"]
p_guess  = dataInit["p_guess"]  # p_guess
Sigma    = dataInit["Sigma_ny"]

a11_s = p_guess[0]  
a12_s = p_guess[1]   
a14_s = p_guess[2]
a21_s = p_guess[3]
a22_s = p_guess[4]  
a24_s = p_guess[5]
a41_s = p_guess[6]  
a42_s = p_guess[7] 
a44_s = p_guess[8]

b1_s =  p_guess[9]  
b2_s =  p_guess[10]  
b4_s =  p_guess[11]

sigma_V     = Sigma[0]
sigma_a     = Sigma[1]
sigma_theta = Sigma[2]
sigma_q     = Sigma[3]

wv = pl.zeros((4, time.shape[0]))
wv[0,:] = (1.0 / sigma_V**2)
wv[1,:] = (1.0 / sigma_a**2)
wv[2,:] = (1.0 / sigma_theta**2)
wv[3,:] = (1.0 / sigma_q**2)

## plot controls --------------------------------------------------------------
pl.figure()

pl.plot(time[:-1],(u_guess))
pl.legend(loc = "lower right")
pl.title("Initial control")
pl.xlabel("Time (s)")
pl.show()

nx = 5
nu = 1
np = 12

x  = ca.MX.sym("x", nx) # [v,alpha,theta,q,de]
u  = ca.MX.sym("u", nu) # [dde]
p  = ca.MX.sym("p", np) 

vt    = x[0]  # airspeed        [m/s]
alpha = x[1]  # angle of attack [rad] 
theta = x[2]  # pitch angle     [rad]
q     = x[3]  # pitch rate      [rad/s]
de    = x[4]  # the input enter as state [rad]

dde   = u[0]  # derivatives of elevator [rad] 

a11 = p[0]  
a12 = p[1]   
a14 = p[2]
a21 = p[3]
a22 = p[4]  
a24 = p[5]
a41 = p[6]  
a42 = p[7] 
a44 = p[8]

b1 =  p[9]  
b2 =  p[10]  
b4 =  p[11]

g0 = -9.8066;

f = ca.vertcat([a11_s*a11*vt + a12_s*a12*alpha + g0*theta + a14_s*a14*q + b1_s*b1*de,
                a21_s*a21*vt + a22_s*a22*alpha +   0      + a24_s*a24*q + b2_s*b2*de,
                        0    +         0       +   0      +         1*q +       0*de,
                a41_s*a41*vt + a42_s*a42*alpha +   0      + a44_s*a44*q + b4_s*b4*de,
                        0    +          0      +   0      +          0  +       0*de + 1*dde])
                  
PHI = x[0:4]
 

p_guess = [1,1,1,1,1,1,1,1,1,1,1,1]

system = cp.system.System(x = x, u = u, p = p, f = f, phi = PHI)

## set the bound 

# state
vt_max    =  3;            # deviation on airspeed
alpha_max =  0.0698;   # deviation in angle of attack 4 deg
theta_min = -0.4712; # pitchAngleMinimum            [rad]
theta_max =  0.6283; # pitchAngleMaximum            [rad]
q_max     =  0.6283;         # pitchRateMaximum             [rad/s]
de_bound  = pl.deg2rad(5);   # elevator deflection          [rad]

# input
dde_bound = 3.25;          # 3.25 FCC [rad/s]

#      [  vt   ,   alpha   , theta    ,   q   ,    de    ]
xmin = [-vt_max, -alpha_max, theta_min, -q_max, -de_bound]
xmax = [ vt_max,  alpha_max, theta_max,  q_max,  de_bound]  
# derivatives of elevator deflection bound bound    3.25 FCC [rad/s]   
umin = [-dde_bound] 
umax = [+dde_bound]

doe  = cp.doe.DoE(system = system, time_points = time, uinit = u_guess, pdata = p_guess, x0 = x0, \
                    umin = umin, umax = umax, xmin = xmin, xmax = xmax, wv = wv)   

doe.print_initial_experimental_properties()

 Optimize!!!
doe.run_experimental_design(solver_options = {"linear_solver": "ma86"})  
u_opt = pl.array(doe.optimized_controls)
scipy.io.savemat('DOE_optimization.mat', dict(u_opt=u_opt,time=time))
doe.print_optimized_experimental_properties()
