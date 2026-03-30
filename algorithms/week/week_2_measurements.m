close all;
clear;
clc;

%% Load measurements
load('../../lidar_saved.mat')
load('../../gnss_saved.mat')

%% Task 2
% STD
std_lidar = std(lidar_saved);
std_gnss = std(gnss_saved);

%% Histogram
figure(1)
for i = 1:8
    subplot(2,4,i)
    histogram(lidar_saved(:,i))
    title(['LiDAR' num2str(i)])
    ylim([0 40])
end

figure(2)
subplot(1,2,1)
histogram(gnss_saved(:,1))
title('GNSS X')
ylim([0 40])

subplot(1,2,2)
histogram(gnss_saved(:,2))
title('GNSS Y')
ylim([0 40])

%% Task 3 
cov_lidar = cov(lidar_saved);
cov_gnss = cov(gnss_saved);

%% Task 4
mu = 0;
sigma_lidar = std_lidar(1);
sigma_gnss = std_gnss(1);

x = linspace(-3*sigma_gnss, 3*sigma_gnss, 500);

y_lidar = norm_pdf(x, mu, sigma_lidar);
y_gnss  = norm_pdf(x,  mu, sigma_gnss);

figure(3)
plot(x, y_lidar, 'LineWidth', 2)
hold on
plot(x, y_gnss, 'LineWidth', 2)

legend('LiDAR', 'GNSS')
title('Probability Density Functions (PDF) of LiDAR and GNSS')
xlabel('Measurement error')
ylabel('Probability density')
grid on

function [pdf] = norm_pdf(x, mu,sigma)
    pdf=(1./(sigma*sqrt(2*pi))) * exp(-0.5*((x-mu)/sigma).^2);
end
