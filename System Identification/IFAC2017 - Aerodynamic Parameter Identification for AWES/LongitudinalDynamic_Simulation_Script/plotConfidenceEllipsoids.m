function plotConfidenceEllipsoids(theta_truth,Case)
% ELLIPSES PLOTTING
Cm_0     = theta_truth(1);
Cm_a     = theta_truth(2);
Cm_dde   = theta_truth(3);
Cm_q     = theta_truth(4);

% Case no turbulence
% theta_est = Cm_0 Cm_alpha Cm_q Cm_deltaE 
Cm_0_noTurb   = Case.theta_noTurb(1);
Cm_a_noTurb   = Case.theta_noTurb(2);
Cm_dde_noTurb = Case.theta_noTurb(3);
Cm_q_noTurb   = Case.theta_noTurb(4);

% Case turbulence
% theta_est = Cm_0 Cm_alpha Cm_q Cm_deltaE 
Cm_0_Turb   = Case.theta_Turb(1);
Cm_a_Turb   = Case.theta_Turb(2);
Cm_dde_Turb = Case.theta_Turb(3);
Cm_q_Turb   = Case.theta_Turb(4);

% [ones(Exp1.N,1),Exp1.alpha,(cref.*q)/(2*V),dde] order regressors
scale = 1;
% The fist thing is to compute eigenvalues and eigenvector of covariances
[V_c_noTurb,Diag_c_noTurb] = eig(Case.cov_noTurb);
% V_c_noTurb = V_c_noTurb;
Diag_c_noTurb = scale.*Diag_c_noTurb;

[V_c_Turb,Diag_c_Turb] = eig(Case.cov_Turb);
% V_c_Turb = V_c_Turb;
Diag_c_Turb = scale.*Diag_c_Turb;

% generate coordinates of 50 points unit circle
xy = [cos(linspace(0,2*pi,50));sin(linspace(0,2*pi,50))];

% generate points of the confidence ellipse, note that the eigenvector V_c
% indicate the directions of the axes of the ellipsoid, and the eigenvalues
% the size of these vector. Since it is asked the 1-sigma ellipsoid, and
% the eigenvalues are the variances of the LLS estimator components as random
% variables, the square root of the eigenvalues as axes length draw the 1
% sigma ellipsoid.
xy_ellipse_Cm0Cma_noTurb   = [Cm_0_noTurb;Cm_a_noTurb  ]*ones(1,50) + V_c_noTurb([1 2],[1 2]) * sqrt(Diag_c_noTurb([1 2],[1 2]))*xy;
xy_ellipse_Cm0Cmdde_noTurb = [Cm_0_noTurb;Cm_dde_noTurb]*ones(1,50) + V_c_noTurb([1 3],[1 3]) * sqrt(Diag_c_noTurb([1 3],[1 3]))*xy;
xy_ellipse_Cm0Cmq_noTurb   = [Cm_0_noTurb;Cm_q_noTurb  ]*ones(1,50) + V_c_noTurb([1 4],[1 4]) * sqrt(Diag_c_noTurb([1 4],[1 4]))*xy;

xy_ellipse_Cm0Cma_Turb   = [Cm_0_Turb;Cm_a_Turb  ]*ones(1,50) + V_c_Turb([1 2],[1 2]) * sqrt(Diag_c_Turb([1 2],[1 2]))*xy;
xy_ellipse_Cm0Cmdde_Turb = [Cm_0_Turb;Cm_dde_Turb]*ones(1,50) + V_c_Turb([1 3],[1 3]) * sqrt(Diag_c_Turb([1 3],[1 3]))*xy;
xy_ellipse_Cm0Cmq_Turb   = [Cm_0_Turb;Cm_q_Turb  ]*ones(1,50) + V_c_Turb([1 4],[1 4]) * sqrt(Diag_c_Turb([1 4],[1 4]))*xy;


% plotting the ellipses (Cm_0,Cm_alpha),(Cm_0,Cm_q),(Cm_0,Cm_dde)
figure;
hold on;grid on;
plot (Cm_0    ,Cm_a    ,'Marker','o','MarkerSize',6,'MarkerFaceColor','g','Color','none')

plot (Cm_0_noTurb,Cm_a_noTurb,'Marker','o','MarkerSize',6,'MarkerFaceColor','b','Color','none')
plot (xy_ellipse_Cm0Cma_noTurb(1,:), xy_ellipse_Cm0Cma_noTurb(2,:), '-b', 'LineWidth',2);

plot (Cm_0_Turb,Cm_a_Turb,'Marker','o','MarkerSize',6,'MarkerFaceColor','r','Color','none')
plot (xy_ellipse_Cm0Cma_Turb(1,:), xy_ellipse_Cm0Cma_Turb(2,:), '-r', 'LineWidth',2);

title('1- \sigma confidence ellipsoid for (C_{m_{0}}^{*},C_{m_{\alpha}}^{*})')
legend('(C_{m_{0}}^{*},C_{m_{\alpha}})','noTurb','noTurb','Turb','Turb');

figure;
hold on;grid on;
plot (Cm_0,Cm_q,'Marker','o','MarkerSize',6,'MarkerFaceColor','g','Color','none')
plot (Cm_0_noTurb,Cm_q_noTurb,'Marker','o','MarkerSize',6,'MarkerFaceColor','b','Color','none')
plot (xy_ellipse_Cm0Cmq_noTurb(1,:), xy_ellipse_Cm0Cmq_noTurb(2,:), '-b', 'LineWidth',2);

plot (Cm_0_Turb,Cm_q_Turb,'Marker','o','MarkerSize',6,'MarkerFaceColor','r','Color','none')
plot (xy_ellipse_Cm0Cmq_Turb(1,:), xy_ellipse_Cm0Cmq_Turb(2,:), '-r', 'LineWidth',2);

title('1- \sigma confidence ellipsoid for (C_{m_{0}},C_{m_{q}})')
legend('(C_{m_{0}}^{*},C_{m_{q}}^{*})','noTurb','noTurb','Turb','Turb');

figure;
hold on;grid on;
plot (Cm_0    ,Cm_dde    ,'Marker','o','MarkerSize',6,'MarkerFaceColor','g','Color','none')
plot (Cm_0_noTurb,Cm_dde_noTurb,'Marker','o','MarkerSize',6,'MarkerFaceColor','b','Color','none')
plot (xy_ellipse_Cm0Cmdde_noTurb(1,:), xy_ellipse_Cm0Cmdde_noTurb(2,:), '-b', 'LineWidth',2);

plot (Cm_0_Turb,Cm_dde_Turb,'Marker','o','MarkerSize',6,'MarkerFaceColor','r','Color','none')
plot (xy_ellipse_Cm0Cmdde_Turb(1,:), xy_ellipse_Cm0Cmdde_Turb(2,:), '-r', 'LineWidth',2);

title('1- \sigma confidence ellipsoid for (C_{m_{0}},C_{m_{\delta e}})')
legend('(C_{m_{0}}^{*},C_{m_{\delta e}}^{*})','noTurb','noTurb','Turb','Turb');

%
xy_ellipse_CmaCmdde_noTurb = [Cm_a_noTurb;Cm_dde_noTurb]*ones(1,50) + V_c_noTurb([2 3],[2 3]) * sqrt(Diag_c_noTurb([2 3],[2 3]))*xy;
xy_ellipse_CmaCmq_noTurb   = [Cm_a_noTurb;Cm_q_noTurb  ]*ones(1,50) + V_c_noTurb([2 4],[2 4]) * sqrt(Diag_c_noTurb([2 4],[2 4]))*xy;
xy_ellipse_CmqCmdde_noTurb = [Cm_dde_noTurb;Cm_q_noTurb]*ones(1,50) + V_c_noTurb([3 4],[3 4]) * sqrt(Diag_c_noTurb([3 4],[3 4]))*xy;

xy_ellipse_CmaCmdde_Turb = [Cm_a_Turb;Cm_dde_Turb]*ones(1,50) + V_c_Turb([2 3],[2 3]) * sqrt(Diag_c_Turb([2 3],[2 3]))*xy;
xy_ellipse_CmaCmq_Turb   = [Cm_a_Turb;Cm_q_Turb  ]*ones(1,50) + V_c_Turb([2 4],[2 4]) * sqrt(Diag_c_Turb([2 4],[2 4]))*xy;
xy_ellipse_CmqCmdde_Turb = [Cm_dde_Turb;Cm_q_Turb]*ones(1,50) + V_c_Turb([3 4],[3 4]) * sqrt(Diag_c_Turb([3 4],[3 4]))*xy;

% plotting the ellipses
figure;hold on;grid on;
plot (Cm_a    ,Cm_q    ,'Marker','o','MarkerSize',6,'MarkerFaceColor','g','Color','none')

plot (Cm_a_noTurb,Cm_q_noTurb,'Marker','o','MarkerSize',6,'MarkerFaceColor','b','Color','none')
plot (xy_ellipse_CmaCmq_noTurb(1,:), xy_ellipse_CmaCmq_noTurb(2,:), '-b', 'LineWidth',2);

plot (Cm_a_Turb,Cm_q_Turb,'Marker','o','MarkerSize',6,'MarkerFaceColor','r','Color','none')
plot (xy_ellipse_CmaCmq_Turb(1,:), xy_ellipse_CmaCmq_Turb(2,:), '-r', 'LineWidth',2);

title('1- \sigma confidence ellipsoid for (C_{m_{\alpha}},C_{m_{q}})')
legend('(C_{m_{\alpha}}^{*},C_{m_{q}}^{*})','noTurb','noTurb','Turb','Turb');

figure;hold on;grid on;
plot (Cm_a    ,Cm_dde    ,'Marker','o','MarkerSize',6,'MarkerFaceColor','g','Color','none')

plot (Cm_a_noTurb,Cm_dde_noTurb,'Marker','o','MarkerSize',6,'MarkerFaceColor','b','Color','none')
plot (xy_ellipse_CmaCmdde_noTurb(1,:), xy_ellipse_CmaCmdde_noTurb(2,:), '-b', 'LineWidth',2);

plot (Cm_a_Turb,Cm_dde_Turb,'Marker','o','MarkerSize',6,'MarkerFaceColor','r','Color','none')
plot (xy_ellipse_CmaCmdde_Turb(1,:), xy_ellipse_CmaCmdde_Turb(2,:), '-r', 'LineWidth',2);

title('1- \sigma confidence ellipsoid for (C_{m_{\alpha}},C_{m_{\delta e}})')
legend('(C_{m_{\alpha}}^{*},C_{m_{\delta e}}^{*})','noTurb','noTurb','Turb','Turb');

figure;hold on;grid on;
plot (Cm_q    ,Cm_dde    ,'Marker','o','MarkerSize',6,'MarkerFaceColor','g','Color','none')

plot (Cm_dde_noTurb,Cm_q_noTurb,'Marker','o','MarkerSize',6,'MarkerFaceColor','b','Color','none')
plot (xy_ellipse_CmqCmdde_noTurb(1,:), xy_ellipse_CmqCmdde_noTurb(2,:), '-b', 'LineWidth',2);

plot (Cm_dde_Turb,Cm_q_Turb,'Marker','o','MarkerSize',6,'MarkerFaceColor','r','Color','none')
plot (xy_ellipse_CmqCmdde_Turb(1,:), xy_ellipse_CmqCmdde_Turb(2,:), '-r', 'LineWidth',2);

title('1- \sigma confidence ellipsoid for (C_{m_{q}},C_{m_{\delta e}})')
legend('(C_{m_{q}}^{*},C_{m_{\delta e}}^{*})','noTurb','noTurb','Turb','Turb');

end