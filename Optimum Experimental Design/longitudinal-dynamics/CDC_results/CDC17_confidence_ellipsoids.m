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

cd Results
   load('Estimation_results_100%_1000exp.mat')
%  load('Estimation_results_20%_1000exp.mat')
%  load('Estimation_results_5%_1000exp.mat')
cd ..

%% results Initial input
a11_init = theta_vec_init( 1,:);   
a12_init = theta_vec_init( 2,:);    
a14_init = theta_vec_init( 3,:);
a21_init = theta_vec_init( 4,:);  
a22_init = theta_vec_init( 5,:);  
a24_init = theta_vec_init( 6,:);
a41_init = theta_vec_init( 7,:);  
a42_init = theta_vec_init( 8,:); 
a44_init = theta_vec_init( 9,:);

b1_init  = theta_vec_init( 10,:);   
b2_init  = theta_vec_init( 11,:);   
b4_init  = theta_vec_init( 12,:);  

covariance_init  = cov(theta_vec_init');
correlation_init = corr(theta_vec_init');

%% results Optimal input
a11_opt = theta_vec_opt( 1,:);   
a12_opt = theta_vec_opt( 2,:);    
a14_opt = theta_vec_opt( 3,:);
a21_opt = theta_vec_opt( 4,:);  
a22_opt = theta_vec_opt( 5,:);  
a24_opt = theta_vec_opt( 6,:);
a41_opt = theta_vec_opt( 7,:);  
a42_opt = theta_vec_opt( 8,:); 
a44_opt = theta_vec_opt( 9,:);

b1_opt = theta_vec_opt( 10,:);   
b2_opt = theta_vec_opt( 11,:);   
b4_opt = theta_vec_opt( 12,:);  

covariance_opt  = cov(theta_vec_opt');
correlation_opt = corr(theta_vec_opt');

%% Assess where we have improvment
Sigma_Increase = diag(covariance_opt) - diag(covariance_init);
Sigma_Improvment_percentage = (Sigma_Increase./diag(covariance_init))*100

%% Generate Condidence Ellipsoids
% figure;hold on;grid on;plot2sigmaCondidenceEllipsoid(a11,b1 ,a11_init, b1_init,a11_opt, b1_opt)
% figure;hold on;grid on;plot2sigmaCondidenceEllipsoid(a12,b2 ,a12_init, b2_init,a12_opt, b2_opt)
% figure;hold on;grid on;plot2sigmaCondidenceEllipsoid(b1 ,b2 ,b1_init , b2_init,b1_opt , b2_opt)
figure;hold on;grid on;plot2sigmaCondidenceEllipsoid(a14,b1 ,a14_init, b1_init,a14_opt, b1_opt)
%%
a11_init = mean(theta_vec_init( 1,:));   
a12_init = mean(theta_vec_init( 2,:));    
a14_init = mean(theta_vec_init( 3,:));
a21_init = mean(theta_vec_init( 4,:));  
a22_init = mean(theta_vec_init( 5,:));  
a24_init = mean(theta_vec_init( 6,:));
a41_init = mean(theta_vec_init( 7,:));  
a42_init = mean(theta_vec_init( 8,:)); 
a44_init = mean(theta_vec_init( 9,:));

b1_init  = mean(theta_vec_init( 10,:));   
b2_init  = mean(theta_vec_init( 11,:));   
b4_init  = mean(theta_vec_init( 12,:));  

p_init = [a11_init;a12_init;a14_init;a21_init;a22_init;a24_init;a41_init;a42_init;a44_init;b1_init;b2_init;b4_init];

%%
a11_opt = mean(theta_vec_opt( 1,:));   
a12_opt = mean(theta_vec_opt( 2,:));    
a14_opt = mean(theta_vec_opt( 3,:));
a21_opt = mean(theta_vec_opt( 4,:));  
a22_opt = mean(theta_vec_opt( 5,:));  
a24_opt = mean(theta_vec_opt( 6,:));
a41_opt = mean(theta_vec_opt( 7,:));  
a42_opt = mean(theta_vec_opt( 8,:)); 
a44_opt = mean(theta_vec_opt( 9,:));

b1_opt = mean(theta_vec_opt( 10,:));   
b2_opt = mean(theta_vec_opt( 11,:));   
b4_opt = mean(theta_vec_opt( 12,:));  

p_opt = [a11_opt;a12_opt;a14_opt;a21_opt;a22_opt;a24_opt;a41_opt;a42_opt;a44_opt;b1_opt;b2_opt;b4_opt];

p_Increase = abs(p_opt) - abs(p_init);
p_Improvment_percentage = (p_Increase./abs(p_init))*100

%%
cd CDC_results
