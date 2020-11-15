%% Creating a new environment

clc;
clear all;
close all;

%% Functions Created: generate_data
% <include>generate_data.m</include>

%% Functions Created: optimal_points
% <include>optimal_points.m</include>

%% Generating the data from each of the 2D Gaussian Distributions.

% Parameters that can be changed according to the experiments.
num_of_clusters = 40;  
start_range_mean = -30;
end_range_mean = 30;
start_range_var = 0;
end_range_var =  5;
data_points_per_cluster = 100;

% Calling the generate_data function.
data = generate_data(num_of_clusters, start_range_mean, end_range_mean, ...
    start_range_var, end_range_var, data_points_per_cluster);

X = data(:, 1);
Y = data(: ,2);

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


%% Getting the K-Means Centroids and Clusters

num_of_centroids = 40;
[idx, centroids] = kmeans(data, num_of_centroids);

% Getting the Clusters Associated with each centroid.
k_means_clusters = cell(num_of_centroids, 1);
for i = 1:num_of_centroids
    k_means_clusters{i} = [X(idx==i),Y(idx==i)] ;
end

%% Generating Random Points for Placing the the Random UAVs

start_range_random = start_range_mean - sqrt(end_range_var);
end_range_random = end_range_mean + sqrt(end_range_var);
X_random = start_range_random + (end_range_random - start_range_random) * ... 
    rand(num_of_clusters, 1);
Y_random = start_range_random + (end_range_random - start_range_random) * ...
    rand(num_of_clusters, 1);
random_centroids = [X_random, Y_random];

figure('Name', 'Random Centroids', 'units','normalized','outerposition', ...
    [0 0 1 1]);

gscatter(X, Y, idx);
hold on;
p_centroids_random = plot(random_centroids(:,1), random_centroids(:,2), ...
    'kx', 'MarkerSize', 15, 'LineWidth', 3, 'DisplayName','Random Centroids'); 
hold off;

legend([p_centroids_random], 'Random Centroids');
title('Random Centroids');
xlabel('X Distance');
ylabel('Y Distance');

%% Plotting the K-Means Centroids

figure('Name', 'K Means Centroids', 'units','normalized','outerposition', ...
    [0 0 1 1]);

gscatter(X, Y, idx);
hold on;
p_centroids = plot(centroids(:,1), centroids(:,2), 'kx', 'MarkerSize', ...
    15, 'LineWidth', 3, 'DisplayName','Centroids'); 
hold off;

legend([p_centroids], 'Centroids');
title('K Means Centroids');
xlabel('X Distance');
ylabel('Y Distance');

%% Getting the optimal UAV power, height, coverage area, and the users served per Cluster

% The five columns are optimal power, optimal height, radius of the users
% served, number of users served
optimal_data = zeros(num_of_centroids, 5);
power_threshold = 10;
height_threshold = 0.5;
bw_uav  = 5;
alpha = 0.5;
chan_capacity_thresh = 1;

for i=1:num_of_centroids
    [optimal_data(i,1), optimal_data(i,2), optimal_data(i,3), ...
        optimal_data(i,4), optimal_data(i,5)] = ...
        optimize_pow_height_cluster(k_means_clusters{i}, centroids(i,:), ...
        power_threshold, height_threshold, alpha, chan_capacity_thresh, bw_uav);
end

%% Getting the optimal UAV locations for the UAV Relays

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
h_relay= 1;
h_bs = 0.1;
capacity_thresh = 1;

for i=1:num_of_centroids
    points = optimal_points(x_bs, y_bs, centroids(i,1), centroids(i,2), ...
        P_bs, P_uav, bw_bs, bw_uav, optimal_data(i,2), h_bs, h_relay, ...
        capacity_thresh);
    uav_1 = [uav_1; points(1, :)];
    uav_2 = [uav_2; points(2, :)];
end

%% Plotting the optimal UAV Placement Locations

figure('Name', 'Optimal UAV Placement', 'units','normalized','outerposition', ...
    [0 0 1 1]);

% Finds the radius of the coverage of the base station.
syms x
capacity_bs = bw_bs*log(1 + P_bs/(x^2 + (h_bs)^2));
eqn = capacity_bs == capacity_thresh;
r = solve(eqn, x);
r = abs(r(1,1));
th = 0:pi/50:2*pi;
x_circle = r * cos(th);
y_circle = r * sin(th);
plot(x_circle, y_circle);
hold on;

gscatter(X, Y, idx);
hold on;
p_centroid = plot(centroids(:,1), centroids(:,2), 'kx', 'MarkerSize', 10, ...
    'LineWidth', 3, 'DisplayName','Centroids'); 
hold on;
p_uav_1 = plot(uav_1(:,1), uav_1(:,2), 'o', 'MarkerSize', 12, 'LineWidth', 3);
hold on;
p_uav_2 = plot(uav_2(:,1), uav_2(:,2), '+', 'MarkerSize', 10, 'LineWidth', 3);
hold on;
p_center = plot(0, 0, 'r*', 'MarkerSize', 15, 'LineWidth', 3);
hold off;

legend([p_centroid, p_uav_1, p_uav_2, p_center], 'Centroids', ... 
    'UAV Intersection 1', 'UAV Intersection 2', 'Base Station');
title('Optimal UAV Placement');
xlabel('X Distance');
ylabel('Y Distance');


%% Comparing the Utility of K-Means vs Random Placement.

% Computing the Random Placement Capacity
total_random_channel_cap = 0;
for i=1:num_of_centroids
    dist = (data(:,1) - random_centroids(i, 1)).^2 + (data(:,2) - ... 
        random_centroids(i, 2)).^2;
    total_random_channel_cap  = total_random_channel_cap + sum(bw_uav * log(1 + ... 
        P_uav./(dist + optimal_data(i,2)^2)));
    
end

% Computing the Total Channel Capacity.
total_channel_cap_opt = 0;
for i=1:num_of_centroids
    dist = (data(:,1) - centroids(i, 1)).^2 + (data(:,2) - centroids(i, 2)).^2;
    total_channel_cap_opt  = total_channel_cap_opt + sum(bw_uav * log(1 + ... 
        optimal_data(i,1)./(dist + optimal_data(i,2)^2)));
    
end

fprintf('Random Placement Total Channel Capacity: %f \n', total_random_channel_cap);
fprintf('Optimal Placement Total Channel Capacity: %f', total_channel_cap_opt);























