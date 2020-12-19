%clc;clear all;close all;
cd ..    
cd DOE
load('p_truth.mat')
cd ..

a11 = p_truth(1);   
a12 = p_truth(2);    
a14 = p_truth(3);
a21 = p_truth(4);  
a22 = p_truth(5);  
a24 = p_truth(6);
a41 = p_truth(7);  
a42 = p_truth(8); 
a44 = p_truth(9);

b1 = p_truth(10);   
b2 = p_truth(11);   
b4 = p_truth(12);  

for i = [100,20,5]
switch i
    case 100
        %% Deviation 100 % priori model parameter
        cd Results
          load('Estimation_results_100%_1000exp.mat')
        cd ..
        Var100 = struct;
        covariance_init   = cov(theta_vec_init');
        %correlation_init = corr(theta_vec_init');
        covariance_opt    = cov(theta_vec_opt');
        %correlation_opt  = corr(theta_vec_opt');
        Sigma_Increase    = diag(covariance_opt) - diag(covariance_init);
        Var100.Sigma_Improvment_percentage = (Sigma_Increase./diag(covariance_init))*100;
        
        % results Initial input
        Var100.a11_init = theta_vec_init( 1,:);   
        Var100.a12_init = theta_vec_init( 2,:);    
        Var100.a14_init = theta_vec_init( 3,:);
        Var100.a21_init = theta_vec_init( 4,:);  
        Var100.a22_init = theta_vec_init( 5,:);  
        Var100.a24_init = theta_vec_init( 6,:);
        Var100.a41_init = theta_vec_init( 7,:);  
        Var100.a42_init = theta_vec_init( 8,:); 
        Var100.a44_init = theta_vec_init( 9,:);

        Var100.b1_init  = theta_vec_init( 10,:);   
        Var100.b2_init  = theta_vec_init( 11,:);   
        Var100.b4_init  = theta_vec_init( 12,:);  

        % results Optimal input
        Var100.a11_opt = theta_vec_opt( 1,:);   
        Var100.a12_opt = theta_vec_opt( 2,:);    
        Var100.a14_opt = theta_vec_opt( 3,:);
        Var100.a21_opt = theta_vec_opt( 4,:);  
        Var100.a22_opt = theta_vec_opt( 5,:);  
        Var100.a24_opt = theta_vec_opt( 6,:);
        Var100.a41_opt = theta_vec_opt( 7,:);  
        Var100.a42_opt = theta_vec_opt( 8,:); 
        Var100.a44_opt = theta_vec_opt( 9,:);

        Var100.b1_opt = theta_vec_opt( 10,:);   
        Var100.b2_opt = theta_vec_opt( 11,:);   
        Var100.b4_opt = theta_vec_opt( 12,:);  

    case 20
        %% Deviation 20 % priori model parameter
        cd Results
          load('Estimation_results_20%_1000exp.mat')
        cd ..
        Var20 = struct;
        covariance_init   = cov(theta_vec_init');
        %correlation_init = corr(theta_vec_init');
        covariance_opt    = cov(theta_vec_opt');
        %correlation_opt  = corr(theta_vec_opt');
        Sigma_Increase    = diag(covariance_opt) - diag(covariance_init);
        Var20.Sigma_Improvment_percentage = (Sigma_Increase./diag(covariance_init))*100;
        
                % results Initial input
        Var20.a11_init = theta_vec_init( 1,:);   
        Var20.a12_init = theta_vec_init( 2,:);    
        Var20.a14_init = theta_vec_init( 3,:);
        Var20.a21_init = theta_vec_init( 4,:);  
        Var20.a22_init = theta_vec_init( 5,:);  
        Var20.a24_init = theta_vec_init( 6,:);
        Var20.a41_init = theta_vec_init( 7,:);  
        Var20.a42_init = theta_vec_init( 8,:); 
        Var20.a44_init = theta_vec_init( 9,:);

        Var20.b1_init  = theta_vec_init( 10,:);   
        Var20.b2_init  = theta_vec_init( 11,:);   
        Var20.b4_init  = theta_vec_init( 12,:);  

        % results Optimal input
        Var20.a11_opt = theta_vec_opt( 1,:);   
        Var20.a12_opt = theta_vec_opt( 2,:);    
        Var20.a14_opt = theta_vec_opt( 3,:);
        Var20.a21_opt = theta_vec_opt( 4,:);  
        Var20.a22_opt = theta_vec_opt( 5,:);  
        Var20.a24_opt = theta_vec_opt( 6,:);
        Var20.a41_opt = theta_vec_opt( 7,:);  
        Var20.a42_opt = theta_vec_opt( 8,:); 
        Var20.a44_opt = theta_vec_opt( 9,:);

        Var20.b1_opt = theta_vec_opt( 10,:);   
        Var20.b2_opt = theta_vec_opt( 11,:);   
        Var20.b4_opt = theta_vec_opt( 12,:);
        
    case 5
        %% Deviation 5 % priori model parameter
        cd Results
          load('Estimation_results_5%_1000exp.mat')
        cd ..
        Var5 = struct;
        covariance_init   = cov(theta_vec_init');
        %correlation_init = corr(theta_vec_init');
        covariance_opt    = cov(theta_vec_opt');
        %correlation_opt  = corr(theta_vec_opt');
        Sigma_Increase    = diag(covariance_opt) - diag(covariance_init);
        Var5.Sigma_Improvment_percentage = (Sigma_Increase./diag(covariance_init))*100;
        
                % results Initial input
        Var5.a11_init = theta_vec_init( 1,:);   
        Var5.a12_init = theta_vec_init( 2,:);    
        Var5.a14_init = theta_vec_init( 3,:);
        Var5.a21_init = theta_vec_init( 4,:);  
        Var5.a22_init = theta_vec_init( 5,:);  
        Var5.a24_init = theta_vec_init( 6,:);
        Var5.a41_init = theta_vec_init( 7,:);  
        Var5.a42_init = theta_vec_init( 8,:); 
        Var5.a44_init = theta_vec_init( 9,:);

        Var5.b1_init  = theta_vec_init( 10,:);   
        Var5.b2_init  = theta_vec_init( 11,:);   
        Var5.b4_init  = theta_vec_init( 12,:);  

        % results Optimal input
        Var5.a11_opt = theta_vec_opt( 1,:);   
        Var5.a12_opt = theta_vec_opt( 2,:);    
        Var5.a14_opt = theta_vec_opt( 3,:);
        Var5.a21_opt = theta_vec_opt( 4,:);  
        Var5.a22_opt = theta_vec_opt( 5,:);  
        Var5.a24_opt = theta_vec_opt( 6,:);
        Var5.a41_opt = theta_vec_opt( 7,:);  
        Var5.a42_opt = theta_vec_opt( 8,:); 
        Var5.a44_opt = theta_vec_opt( 9,:);

        Var5.b1_opt = theta_vec_opt( 10,:);   
        Var5.b2_opt = theta_vec_opt( 11,:);   
        Var5.b4_opt = theta_vec_opt( 12,:);
    end
end

disp([Var100.Sigma_Improvment_percentage,...
      Var5.Sigma_Improvment_percentage,...
      Var20.Sigma_Improvment_percentage]);


% %% Generate Condidence Ellipsoids
figure;
subplot(3,2,1);hold on;grid on;
plot2sigmaCondidenceEllipsoid(a14,b1 ,Var100.a14_init,Var100.b1_init,Var100.a14_opt,Var100.b1_opt)
subplot(3,2,3);hold on;grid on;
plot2sigmaCondidenceEllipsoid(a14,b1 ,Var5.a14_init,Var5.b1_init,Var5.a14_opt,Var5.b1_opt)
subplot(3,2,5);hold on;grid on;
plot2sigmaCondidenceEllipsoid(a14,b1 ,Var20.a14_init,Var20.b1_init,Var20.a14_opt,Var20.b1_opt)

subplot(3,2,2);hold on;grid on;
plot2sigmaCondidenceEllipsoid(a12,b2 ,Var100.a12_init,Var100.b2_init,Var100.a12_opt,Var100.b2_opt)
subplot(3,2,4);hold on;grid on;
plot2sigmaCondidenceEllipsoid(a12,b2 ,Var5.a12_init,Var5.b2_init,Var5.a12_opt,Var5.b2_opt)
subplot(3,2,6);hold on;grid on;
plot2sigmaCondidenceEllipsoid(a12,b2 ,Var20.a12_init,Var20.b2_init,Var20.a12_opt,Var20.b2_opt)

figure;hold on;grid on;
plot2sigmaCondidenceEllipsoid(a42,a44 ,Var100.a42_init,Var100.a44_init,Var100.a42_opt,Var100.a44_opt)
plot2sigmaCondidenceEllipsoid(a42,a44 ,Var5.a42_init  ,Var5.a44_init  ,Var5.a42_opt,Var5.a44_opt)
plot2sigmaCondidenceEllipsoid(a42,a44 ,Var20.a42_init ,Var20.a44_init ,Var20.a42_opt,Var20.a44_opt)
ylabel('M_{q}');xlabel('M_{\alpha}');

%%
cd CDC_results