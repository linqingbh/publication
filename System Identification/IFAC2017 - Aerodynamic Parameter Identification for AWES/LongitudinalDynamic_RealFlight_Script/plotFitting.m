function plotFitting(Exp1,Exp2,Exp3,Exp4,X_est)
  figure;% plot pitch rate
  subplot(2,4,1);hold on;grid on;ylabel('q [deg/s]');
                 title('Experiment 1');
                 plot(Exp1.time,rad2deg(Exp1.q)    ,'LineWidth',2,'Color','b');
                 plot(Exp1.time,rad2deg(X_est(:,1)),'LineWidth',2,'Color','r');
  subplot(2,4,2);hold on;grid on;
                 title('Experiment 2');
                 plot(Exp2.time,rad2deg(Exp2.q)    ,'LineWidth',2,'Color','b');
                 plot(Exp2.time,rad2deg(X_est(:,2)),'LineWidth',2,'Color','r');
  subplot(2,4,3);hold on;grid on;
                 title('Experiment 3');
                 plot(Exp3.time,rad2deg(Exp3.q)    ,'LineWidth',2,'Color','b');
                 plot(Exp3.time,rad2deg(X_est(:,3)),'LineWidth',2,'Color','r');
  subplot(2,4,4);hold on;grid on;
                 title('Experiment 4');
                 plot(Exp4.time,rad2deg(Exp4.q)    ,'LineWidth',2,'Color','b');
                 plot(Exp4.time,rad2deg(X_est(:,4)),'LineWidth',2,'Color','r');
  % plot elevator deflection        
  subplot(2,4,5);hold on;grid on;ylabel('\delta_{e} [deg]');
                 plot(Exp1.time,rad2deg(Exp1.dde),'LineWidth',1.5,'Color','b');                
                 xlabel('time [s]');
  subplot(2,4,6);hold on;grid on;
                 plot(Exp2.time,rad2deg(Exp2.dde),'LineWidth',1.5,'Color','b');                
                 xlabel('time [s]');
  subplot(2,4,7);hold on;grid on;
                 plot(Exp3.time,rad2deg(Exp3.dde),'LineWidth',1.5,'Color','b');                
  subplot(2,4,8);hold on;grid on;
                 plot(Exp4.time,rad2deg(Exp4.dde),'LineWidth',1.5,'Color','b');                
                 xlabel('time [s]');
                 
  %% residual analysis
  y     = -8.5:0.1:8.5;
  residual1 = rad2deg(Exp1.q) - rad2deg(X_est(:,1));
  residual2 = rad2deg(Exp2.q) - rad2deg(X_est(:,2));
  residual3 = rad2deg(Exp3.q) - rad2deg(X_est(:,3));
  residual4 = rad2deg(Exp4.q) - rad2deg(X_est(:,4));
                 
  figure;
  subplot(2,2,1);histogram(residual1,20,'Normalization','pdf');
  hold on;grid on;title('Experiment 1');xlim([-8,8]);ylim([0,0.45]);
  mu    = mean(residual1)
  sigma = std(residual1)
  f     = exp(-(y-mu).^2./(2*sigma^2))./(sigma*sqrt(2*pi)); 
  plot(y,f,'LineWidth',1.5);
  
  xlabel('\epsilon [deg/s]');
  subplot(2,2,2);histogram(residual2,20,'Normalization','pdf');
  hold on;grid on;title('Experiment 2');xlim([-8.5,8.5]);ylim([0,0.45]);
  mu    = mean(residual2)
  sigma = std(residual2)
  f     = exp(-(y-mu).^2./(2*sigma^2))./(sigma*sqrt(2*pi)); 
  plot(y,f,'LineWidth',1.5);
  xlabel('\epsilon [deg/s]');
  
  subplot(2,2,3);histogram(residual3,20,'Normalization','pdf');
  hold on;grid on;title('Experiment 3');xlim([-8.5,8.5]);ylim([0,0.45]);
  mu    = mean(residual3)
  sigma = std(residual3)
  f     = exp(-(y-mu).^2./(2*sigma^2))./(sigma*sqrt(2*pi)); 
  plot(y,f,'LineWidth',1.5);
  xlabel('\epsilon [deg/s]');
  
  subplot(2,2,4);histogram(residual4,20,'Normalization','pdf');
  hold on;grid on;title('Experiment 4');xlim([-8.5,8.5]);ylim([0,0.45]);
  mu    = mean(residual4)
  sigma = std(residual4)
  f     = exp(-(y-mu).^2./(2*sigma^2))./(sigma*sqrt(2*pi)); 
  plot(y,f,'LineWidth',1.5);
  xlabel('\epsilon [deg/s]');
end