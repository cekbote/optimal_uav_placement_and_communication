%% Creating a new environment

clc;
clear all;
close all;

%% Functions Created: generate_data
% <include>generate_data.m</include>

%% Functions Created for the Assignment: optimal_points
% <include>optimal_points.m</include>

%% Generating the data from each of the 2D Gaussian Distributions.

% Parameters that can be changed according to the experiments.
num_of_clusters = 40;  
start_range_mean = -10;
end_range_mean = 10;
start_range_var = 0;
end_range_var =  5;
data_points_per_cluster = 100;

% Calling the generate_data function.
data = generate_data(num_of_clusters, start_range_mean, end_range_mean, ...
    start_range_var, end_range_var, data_points_per_cluster);

%% Plotting each of the Gaussian Dsitributions.

figure('Name', 'Gaussian Clusters', 'units','normalized','outerposition', ...
    [0 0 1 1]);
for i=1:num_of_clusters
    plot(data((i-1)*100 + 1: (i)*100, 1), data((i-1)*100 + 1: (i)*100, 2), '.');
    hold on;
end

hold off;
title('Gaussian Distributions');
xlabel('X Distance');
ylabel('Y Distance');

%% Getting the K-Means Centroids

num_of_centroids = 40;
[idx, centroids] = kmeans(data, num_of_centroids);

%% Plotting the K-Means Centroids

figure('Name', 'K Means Centroids', 'units','normalized','outerposition', ...
    [0 0 1 1]);
for i=1:num_of_clusters
    plot(data((i-1)*100 + 1: (i)*100, 1), data((i-1)*100 + 1: (i)*100, 2), '.');
    hold on;
end

p_centroids = plot(centroids(:,1), centroids(:,2), 'kx', 'MarkerSize', ...
    15, 'LineWidth', 3, 'DisplayName','Centroids'); 
hold off;

legend([p_centroids], 'Centroids');
title('K Means Centroids');
xlabel('X Distance');
ylabel('Y Distance');

%% Getting the optimal UAV locations.

% Two are required as we can have two optimal locations. 
uav_1 = [];
uav_2 = [];

% Parameters that can be changed according to the experiments.
x_bs = 0; 
y_bs = 0;
P_bs = 50;
P_uav = 10;
bw_bs = 10;
bw_uav  = 5;
h_uav = 1;
capacity_thresh = 1;

for i=1:num_of_centroids
    points = optimal_points(x_bs, y_bs, centroids(i,1), centroids(i,2), ...
        P_bs, P_uav, bw_bs, bw_uav, h_uav, capacity_thresh);
    uav_1 = [uav_1; points(1, :)];
    uav_2 = [uav_2; points(2, :)];
end

%% Plotting the optimal UAV placement locations

figure('Name', 'Optimal UAV Placement', 'units','normalized','outerposition', ...
    [0 0 1 1]);
for i=1:num_of_clusters
    plot(data((i-1)*100 + 1: (i)*100, 1), data((i-1)*100 + 1: (i)*100, 2), '.');
    hold on;
end

p_centroid = plot(centroids(:,1), centroids(:,2), 'kx', 'MarkerSize', 10, ...
    'LineWidth', 3, 'DisplayName','Centroids'); 
hold on;
p_uav_1 = plot(uav_1(:,1), uav_1(:,2), 'o', 'MarkerSize', 12, 'LineWidth', 3);
hold on;
p_uav_2 = plot(uav_2(:,1), uav_2(:,2), '+', 'MarkerSize', 10, 'LineWidth', 3);
hold on;
p_center = plot(0, 0, 'r*', 'MarkerSize', 15, 'LineWidth', 3);
hold off;

legend([p_centroid, p_uav_1, p_uav_2, p_center], 'Centroids', 'UAV 1', ... 
    'UAV 2', 'Center');
title('Optimal UAV Placement');
xlabel('X Distance');
ylabel('Y Distance');






