function PlotDataFitting(y_data,X_truth,X_est)
  global N time
  global Vmax alpha_max theta_max theta_min q_max q_min
  global theta_maxTol theta_minTol q_maxTol q_minTol
  %% plot response ==========================================================
  figure;
  subplot(2,2,1);hold on;grid on;
  plot(time, full(y_data(:,1)),'gx');
  plot(time, full(X_truth(1,:)),'LineWidth',2,'Color','b');
  plot(time, full(X_est(:,1)),'LineWidth',2,'Color','r');
  plot(time, Vmax.*ones(N,1),'LineWidth',1,'Color','r','LineStyle','-.');
  plot(time,-Vmax.*ones(N,1),'LineWidth',1,'Color','r','LineStyle','-.');
  xlabel('time [s]');ylabel('[m/s]');
  legend('v noisy','v true','v estimated','constraints');
  ylim([-Vmax-0.5,Vmax+0.5])

  subplot(2,2,3);hold on;grid on;
  plot(time,rad2deg(full(y_data(:,2))),'gx');
  plot(time,rad2deg(full(X_truth(2,:))),'LineWidth',2,'Color','b');
  plot(time,rad2deg(full(X_est(:,2))),'LineWidth',2,'Color','r');
  plot(time,rad2deg( alpha_max).*ones(N,1),'LineWidth',1,'Color','r','LineStyle','-.');
  plot(time,rad2deg(-alpha_max).*ones(N,1),'LineWidth',1,'Color','r','LineStyle','-.');
  xlabel('time [s]');ylabel('[deg]');
  legend('\alpha noisy','\alpha true','\alpha estimated','constraints');
  ylim([rad2deg(-alpha_max-0.1), rad2deg(alpha_max+0.1)])

  subplot(2,2,2);hold on;grid on;
  plot(time, rad2deg(full(y_data(:,3))),'gx');
  plot(time, rad2deg(full(X_truth(3,:))),'LineWidth',2,'Color','b');
  plot(time, rad2deg(full(X_est(:,3))),'LineWidth',2,'Color','r');
  plot(time, rad2deg(theta_max).*ones(N,1)   ,'LineWidth',1,'Color','r','LineStyle','-.');
  plot(time, rad2deg(theta_min).*ones(N,1)   ,'LineWidth',1,'Color','r','LineStyle','-.');
  plot(time, rad2deg(theta_maxTol).*ones(N,1),'LineWidth',1,'Color','r','LineStyle',':');
  plot(time, rad2deg(theta_minTol).*ones(N,1),'LineWidth',1,'Color','r','LineStyle',':');
  xlabel('time [s]');ylabel('\theta [deg]');
  legend('\theta noisy','\theta true','\theta estimated','constraints','Tollerance on constraints');
  ylim([rad2deg(theta_min-0.1), rad2deg(theta_max+0.1)])

  subplot(2,2,4);hold on;grid on;
  plot(time, rad2deg(full(y_data(:,4))),'gx');
  plot(time, rad2deg(full(X_truth(4,:))),'LineWidth',2,'Color','b');
  plot(time, rad2deg(full(X_est(:,4))),'LineWidth',2,'Color','r');
  plot(time,rad2deg(q_max).*ones(N,1)   ,'LineWidth',1,'Color','r','LineStyle','-.');
  plot(time,rad2deg(q_min).*ones(N,1)   ,'LineWidth',1,'Color','r','LineStyle','-.');
  plot(time,rad2deg(q_maxTol).*ones(N,1),'LineWidth',1,'Color','r','LineStyle',':');
  plot(time,rad2deg(q_minTol).*ones(N,1),'LineWidth',1,'Color','r','LineStyle',':');
  xlabel('time [s]');ylabel('q [deg/s]');
  legend('q noisy','q true','q estimated','constraints','Tollerance on constraints');
  ylim([rad2deg(q_min-0.1), rad2deg(q_max+0.1)])
  
end