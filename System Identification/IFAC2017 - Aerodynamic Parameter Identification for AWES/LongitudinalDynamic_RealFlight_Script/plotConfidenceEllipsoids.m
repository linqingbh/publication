function plotConfidenceEllipsoids(theta_est,covariance_est)
% ELLIPSES PLOTTING
% theta_est = Cm_0 Cm_alpha Cm_q Cm_deltaE 
Cm_0     = theta_est(1);
Cm_a     = theta_est(2);
Cm_q     = theta_est(3);
Cm_dde   = theta_est(4);

% [ones(Exp1.N,1),Exp1.alpha,(cref.*q)/(2*V),dde] order regressors

% The fist thing is to compute eigenvalues and eigenvector of covariances
[V_c,Diag_c] = eig(covariance_est);

% generate coordinates of 50 points unit circle
xy = [cos(linspace(0,2*pi,50));sin(linspace(0,2*pi,50))];

% generate points of the confidence ellipse, note that the eigenvector V_c
% indicate the directions of the axes of the ellipsoid, and the eigenvalues
% the size of these vector. Since it is asked the 1-sigma ellipsoid, and
% the eigenvalues are the variances of the LLS estimator components as random
% variables, the square root of the eigenvalues as axes length draw the 1
% sigma ellipsoid.
xy_ellipse_Cm0Cma   = [Cm_0;Cm_a  ]*ones(1,50) + V_c([1 2],[1 2]) * sqrt(Diag_c([1 2],[1 2]))*xy;
xy_ellipse_Cm0Cmq   = [Cm_0;Cm_q  ]*ones(1,50) + V_c([1 3],[1 3]) * sqrt(Diag_c([1 3],[1 3]))*xy;
xy_ellipse_Cm0Cmdde = [Cm_0;Cm_dde]*ones(1,50) + V_c([1 4],[1 4]) * sqrt(Diag_c([1 4],[1 4]))*xy;

% plotting the ellipses (Cm_0,Cm_alpha),(Cm_0,Cm_q),(Cm_0,Cm_dde)
figure;
subplot(3,2,1);hold on;grid on;
plot (Cm_0,Cm_a,'Marker','o','MarkerSize',6,'MarkerFaceColor','b','Color','none')
plot (xy_ellipse_Cm0Cma(1,:), xy_ellipse_Cm0Cma(2,:), '-r', 'LineWidth',2);
title('1- \sigma confidence ellipsoid for (C_{m_{0}}^{*},C_{m_{\alpha}}^{*})')
legend('(C_{m_{0}}^{*},C_{m_{\alpha}})');

subplot(3,2,2);hold on;grid on;
plot (Cm_0,Cm_q,'Marker','o','MarkerSize',6,'MarkerFaceColor','b','Color','none')
plot (xy_ellipse_Cm0Cmq(1,:), xy_ellipse_Cm0Cmq(2,:), '-r', 'LineWidth',2);
title('1- \sigma confidence ellipsoid for (C_{m_{0}},C_{m_{q}})')
legend('(C_{m_{0}}^{*},C_{m_{q}}^{*})');

subplot(3,2,3);hold on;grid on;
plot (Cm_0,Cm_dde,'Marker','o','MarkerSize',6,'MarkerFaceColor','b','Color','none')
plot (xy_ellipse_Cm0Cmdde(1,:), xy_ellipse_Cm0Cmdde(2,:), '-r', 'LineWidth',2);
title('1- \sigma confidence ellipsoid for (C_{m_{0}},C_{m_{\delta e}})')
legend('(C_{m_{0}}^{*},C_{m_{\delta e}}^{*})');

%%
xy_ellipse_CmaCmq   = [Cm_a;Cm_q  ]*ones(1,50) + V_c([2 3],[2 3]) * sqrt(Diag_c([2 3],[2 3]))*xy;
xy_ellipse_CmaCmdde = [Cm_a;Cm_dde]*ones(1,50) + V_c([2 4],[2 4]) * sqrt(Diag_c([2 4],[2 4]))*xy;
xy_ellipse_CmqCmdde = [Cm_q;Cm_dde]*ones(1,50) + V_c([3 4],[3 4]) * sqrt(Diag_c([3 4],[3 4]))*xy;

% plotting the ellipses
subplot(3,2,4);hold on;grid on;
plot (Cm_a,Cm_q,'Marker','o','MarkerSize',6,'MarkerFaceColor','b','Color','none')
plot (xy_ellipse_CmaCmq(1,:), xy_ellipse_CmaCmq(2,:), '-r', 'LineWidth',2);
title('1- \sigma confidence ellipsoid for (C_{m_{\alpha}},C_{m_{q}})')
legend('(C_{m_{\alpha}}^{*},C_{m_{q}}^{*})');

subplot(3,2,5);hold on;grid on;
plot (Cm_a,Cm_dde,'Marker','o','MarkerSize',6,'MarkerFaceColor','b','Color','none')
plot (xy_ellipse_CmaCmdde(1,:), xy_ellipse_CmaCmdde(2,:), '-r', 'LineWidth',2);
title('1- \sigma confidence ellipsoid for (C_{m_{\alpha}},C_{m_{\delta e}})')
legend('(C_{m_{\alpha}}^{*},C_{m_{\delta e}}^{*})');

subplot(3,2,6);hold on;grid on;
plot (Cm_q,Cm_dde,'Marker','o','MarkerSize',6,'MarkerFaceColor','b','Color','none')
plot (xy_ellipse_CmqCmdde(1,:), xy_ellipse_CmqCmdde(2,:), '-r', 'LineWidth',2);
title('1- \sigma confidence ellipsoid for (C_{m_{q}},C_{m_{\delta e}})')
legend('(C_{m_{q}}^{*},C_{m_{\delta e}}^{*})');

end