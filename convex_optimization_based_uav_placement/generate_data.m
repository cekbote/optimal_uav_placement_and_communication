% Function that generates data from multiple two dimensional Gaussians.

function [data] = generate_data(num_of_clusters, start_range_mean, ... 
    end_range_mean, start_range_var, end_range_var, data_points_per_cluster)

    % num_of_clusters: Number of clusters that data should be generating for
    % start_range_mean: Starting of the range of the mean
    % end_range_mean: Ending of the range of the mean
    % start_range_var: Starting of the range of the var
    % end_range_var: Ending of the range of the var
    % data_points_per_cluster: Number of data points that are generated per
    % cluster

    data = [];

    for i=1:num_of_clusters
        
        % Generating the mean per cluster
        mu = start_range_mean + (end_range_mean - start_range_mean).* ...
            rand(2,1);

        % Generating the variance per cluster

        % Creates a PSD Matrix
        while (true)
          % Such a scaling is done (from -1 to 1 as the rand function 
          % generates only from 0 to 1. Such a scaling is important as if 
          % the matrix only produces values from 0 to 1, the sigma matrix
          % is always positively correlated. Hence the range from -1 to 1
          % ensures that we get both positive and negatively correalted 2D
          % Gaussian Distributions.
          A = -1 + 2.*rand(2, 2);
          if (rank(A) == 2);
              % Ensures that the matrix is full rank.
              break; 
          end    
        end
        sigma = A' * A; % Makes sure that sigma is PSD.
        sigma = start_range_var + (end_range_var - start_range_var).*sigma;

        % Generating the Data  
        data_per_cluster = mvnrnd(mu, sigma, data_points_per_cluster);
        data = [data; data_per_cluster];
    end