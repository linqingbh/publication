function plot2sigmaCondidenceEllipsoid(p_true1,p_true2,p_init1,p_init2,p_opt1,p_opt2)

  % generate the coordinates of 50 points on a unit circle
  N_ellips = 100;
  xy = [cos(linspace(0,2*pi,N_ellips)); sin(linspace(0,2*pi,N_ellips))];

  p_init1 = makeColumn(p_init1);
  p_init2 = makeColumn(p_init2);
  p_opt1  = makeColumn(p_opt1);
  p_opt2  = makeColumn(p_opt2);

  p_init1_mean    = mean(p_init1);          % mean value of p_init1 for Initial Input
  p_init2_mean    = mean(p_init2);          % mean value of p_init2 for Initial Input
  Sigma_init      = cov([p_init1 p_init2]); % covariance matrix for Initial Input
  [V_init,D_init] = eig(Sigma_init);        % eigenvalues and eigenvector computation

  % generate the points of the confidence ellipse
  xy_ellipse_int = [p_init1_mean; p_init2_mean]*ones(1,N_ellips) + V_init*sqrt(D_init)*xy;

  %%
  p_opt1_mean   = mean(p_opt1);         % mean value of p_init1 for Optimal Input
  p_opt2_mean   = mean(p_opt2);         % mean value of p_init2 for Optimal Input
  Sigma_opt     = cov([p_opt1 p_opt2]); % covariance matrix for case 1
  [V_opt,D_opt] = eig(Sigma_opt);       % eigenvalues and eigenvector computation

  % generate the points of the confidence ellipse
  xy_ellipse_opt = [p_opt1_mean; p_opt2_mean]*ones(1,N_ellips) + V_opt*sqrt(D_opt)*xy;

%   figure;hold on;grid on;
  plot(p_init1,p_init2,'m+');
  plot(p_opt1 ,p_opt2,'g+');
  
%   plot(p_init1_mean,p_init2_mean,'Marker','o','MarkerEdgeColor','k','MarkerFaceColor','b','MarkerSize',5);
%   plot(p_opt1_mean ,p_opt2_mean,'Marker','o','MarkerEdgeColor','k','MarkerFaceColor','r','MarkerSize',5);
%     
  plot(p_true1,p_true2,'Marker','o','MarkerEdgeColor','k','MarkerFaceColor','y','MarkerSize',5);
  plot(xy_ellipse_int(1,:), xy_ellipse_int(2,:),'Color','b','LineWidth',1.5);
  plot(xy_ellipse_opt(1,:), xy_ellipse_opt(2,:),'Color','r','LineWidth',1.5);
  legend('(p1,p2) estimated via Conventional Manoeuvre',...
         '(p1,p2) estimated via Optimized Manoeuvre',...
         '(p1,p2) true',...
         '1-\sigma Confidence Ellipsoid: Conventional Manoeuvre',...
         '1-\sigma Confidence Ellipsoid: Optimized Manoeuvre');
end

function p = makeColumn(p)
  if isrow(p)
    p = p';
  end
end