function u_3211 = generate_3211(A_3211,Dt_3211)
  global N fs
  A_3211    = deg2rad(A_3211); % amplitude 3-2-1-1

  N_3211  = 7*Dt_3211*fs; % number of measurements
  t_3211  = linspace(0,7*Dt_3211,N_3211)';
  u1      = A_3211.*ones(length(t_3211),1);
  u2      = zeros(length(t_3211),1);
  u2(round((3/7)*N_3211):end) = -2*A_3211; 
  u3      = zeros(length(t_3211),1);
  u3(round((5/7)*N_3211):end) = 2*A_3211;
  u4      = zeros(length(t_3211),1);
  u4(round((6/7)*N_3211):end) = -2*A_3211; 
  u5      = zeros(length(t_3211),1);
  u5(round((7/7)*N_3211):end) = A_3211;

  u_3211  = u1 + u2 + u3 + u4 + u5;
  u_3211  = [u_3211;zeros(N-N_3211,1)];

end