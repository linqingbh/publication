%
%     This file is part of CasADi.
%
%     CasADi -- A symbolic framework for dynamic optimization.
%     Copyright (C) 2010-2014 Joel Andersson, Joris Gillis, Moritz Diehl,
%                             K.U. Leuven. All rights reserved.
%     Copyright (C) 2011-2014 Greg Horn
%
%     CasADi is free software; you can redistribute it and/or
%     modify it under the terms of the GNU Lesser General Public
%     License as published by the Free Software Foundation; either
%     version 3 of the License, or (at your option) any later version.
%
%     CasADi is distributed in the hope that it will be useful,
%     but WITHOUT ANY WARRANTY; without even the implied warranty of
%     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
%     Lesser General Public License for more details.
%
%     You should have received a copy of the GNU Lesser General Public
%     License along with CasADi; if not, write to the Free Software
%     Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
%

classdef MyCallback < casadi.Callback
  properties
    nx
    ng
    np
    iter
  end
  methods
    function self = MyCallback(name, nx, ng, np)
      self@casadi.Callback();
      self.nx = nx;
      self.ng = ng;
      self.np = np;
      self.iter = 0;
      
      opts = struct;
      opts.input_scheme = casadi.nlpsol_out();
      opts.output_scheme = char('ret');
      construct(self, name, opts);
    end
    function out = get_sparsity_in(self,i)
      n = casadi.nlpsol_out(i);
      if strcmp(n,'f')
        out = [1 1];
      elseif strcmp(n,'lam_x') || strcmp(n,'x')
        out = [self.nx 1];
      elseif strcmp(n,'lam_g') || strcmp(n,'g')
        out = [self.ng 1];
      elseif strcmp(n,'p')  || strcmp(n,'lam_p')
        out = [self.np 1];
      else
        out = [0 0];
      end
      out = casadi.Sparsity.dense(out(1),out(2));
    end
    function out = get_n_in(self)
      out = casadi.nlpsol_n_out();
    end
    function out = get_n_out(self)
      out = 1;
    end

    function out = eval(self, arg)
      global np Nm Nexp Exp1 Exp2 Exp3 Exp4
      
      x = full(arg{1});
      f = full(arg{2});
      X_est = x(np+1:end);     % retrieve states
      X_est = reshape(X_est,Nm,Nexp);
      
      clf;
      subplot(2,4,1);hold on;grid on;ylabel('q [deg/s]');
                     plot(Exp1.time,rad2deg(X_est(:,1)),'LineWidth',2,'Color','r');
                     plot(Exp1.time,rad2deg(Exp1.q)    ,'LineWidth',2,'Color','b');
                     leg = legend('Data fitting','Measurements');
                     set(leg,'FontSize',14);xlabel('time [s]');
      subplot(2,4,2);hold on;grid on;
                     tl = title(['Data Fitting: ']);set(tl,'FontSize',18);
                     plot(Exp2.time,rad2deg(X_est(:,2)),'LineWidth',2,'Color','r');
                     plot(Exp2.time,rad2deg(Exp2.q)    ,'LineWidth',2,'Color','b');
                     leg = legend('Data fitting','Measurements');
                     set(leg,'FontSize',14);xlabel('time [s]');
      subplot(2,4,3);hold on;grid on;
                     tl = title(['# Iteration = ',num2str(self.iter)]);set(tl,'FontSize',18);
                     plot(Exp3.time,rad2deg(X_est(:,3)),'LineWidth',2,'Color','r');
                     plot(Exp3.time,rad2deg(Exp3.q)    ,'LineWidth',2,'Color','b');
                     leg = legend('Data fitting','Measurements');
                     set(leg,'FontSize',14);xlabel('time [s]');
      subplot(2,4,4);hold on;grid on;
                     plot(Exp4.time,rad2deg(X_est(:,4)),'LineWidth',2,'Color','r');
                     plot(Exp4.time,rad2deg(Exp4.q)    ,'LineWidth',2,'Color','b');
                     leg = legend('Data fitting','Measurements');
                     set(leg,'FontSize',14);xlabel('time [s]');
                 
      %% residual analysis
      y     = -8:0.1:8;
      residual1 = rad2deg(Exp1.q) - rad2deg(X_est(:,1));
      residual2 = rad2deg(Exp2.q) - rad2deg(X_est(:,2));
      residual3 = rad2deg(Exp3.q) - rad2deg(X_est(:,3));
      residual4 = rad2deg(Exp4.q) - rad2deg(X_est(:,4));

      subplot(2,4,5);histogram(residual1,20,'Normalization','pdf');
      hold on;grid on;xlim([-8,8]);ylim([0,0.45]);
      mu    = mean(residual1);
      sigma = std(residual1);
      f     = exp(-(y-mu).^2./(2*sigma^2))./(sigma*sqrt(2*pi)); 
      plot(y,f,'LineWidth',1.5);
      leg = legend(['\mu = ', num2str(mu)],['\sigma = ', num2str(sigma)]);
      set(leg,'FontSize',14);xlabel('q [deg/s]');
      
      subplot(2,4,6);histogram(residual2,20,'Normalization','pdf');
      hold on;grid on;
      tl = title('Residual ');set(tl,'FontSize',18);
      xlim([-8,8]);ylim([0,0.45]);
      mu    = mean(residual2);
      sigma = std(residual2);
      f     = exp(-(y-mu).^2./(2*sigma^2))./(sigma*sqrt(2*pi)); 
      plot(y,f,'LineWidth',1.5);
      leg = legend(['\mu = ', num2str(mu)],['\sigma = ', num2str(sigma)]);
      set(leg,'FontSize',14);xlabel('q [deg/s]');

      subplot(2,4,7);histogram(residual3,20,'Normalization','pdf');
      hold on;grid on;
      tl = title(' Distribution');set(tl,'FontSize',18);
      xlim([-8,8]);ylim([0,0.45]);
      mu    = mean(residual3);
      sigma = std(residual3);
      f     = exp(-(y-mu).^2./(2*sigma^2))./(sigma*sqrt(2*pi)); 
      plot(y,f,'LineWidth',1.5);
      leg   = legend(['\mu = ', num2str(mu)],['\sigma = ', num2str(sigma)]);
      set(leg,'FontSize',14);xlabel('q [deg/s]');
      
      subplot(2,4,8);histogram(residual4,20,'Normalization','pdf');
      hold on;grid on;xlim([-8,8]);ylim([0,0.45]);
      mu    = mean(residual4);
      sigma = std(residual4);
      f     = exp(-(y-mu).^2./(2*sigma^2))./(sigma*sqrt(2*pi)); 
      plot(y,f,'LineWidth',1.5);
      leg   = legend(['\mu = ', num2str(mu)],['\sigma = ', num2str(sigma)]);
      set(leg,'FontSize',14);xlabel('q [deg/s]');

      self.iter = self.iter + 1;
    
      pause(0.75);
      
      %saveas(gcf,['DataFitting',num2str(self.iter),'Iter.eps'])
      
      out = {0};
    end
  end
end
