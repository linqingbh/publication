function plotSignalExcitacion(u_3211,de_3211)
  % u_  = input demand
  % de_ = input deflection
  
  global time de_max N
  figure;hold on;grid on;title('Optimized signal excitacion');
  plot(time , rad2deg(u_3211) ,'LineWidth',1,'Color','g','LineStyle','-.');
  plot(time , rad2deg(de_3211),'LineWidth',1.5,'Color','b','LineStyle','-' );
  plot(time , de_max.*ones(N,1),'r-.');
  plot(time ,-de_max.*ones(N,1),'r-.');
  xlabel('time [s]');ylabel('[deg]');legend('u_{e}','\delta_{e}');
  ylim([-de_max-0.5, de_max+0.5]);

end