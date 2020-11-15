% Function that gives the optimal Power, Height, Coverage Radius, Users
% Served with the given thresholds. 

function [pow, height, rad, users_served, total_users] = optimize_pow_height_cluster(k_means_cluster, ... 
    centroid, p_thresh, h_thresh, alpha, channel_cap_thresh, bw_uav)
    
    % Input:
    % k_means_cluster: Contains the x and y cooordinates of all points in
    % the cluster.
    % centroid: Contains the x and y coordinate of the centroid of the
    % cluster.
    % p_thresh: Power threshold of the UAV.
    % h_thresh: Minimum power threshold of the UAV.
    % alpha: Tradeoff between height and power.
    % channel_cap_thresh: Minimum channel capacity required for proper
    % communication.
    % bw_uav: Bandwidth of the UAV communication.
    
    % Output:
    % pow: Power required by the UAV to communicate.
    % height: Minimum height required by the UAV to communicate.
    % rad: Radius of the cluster that can be served.
    % users_served: Number of users served.
    
    % Latex Equation:
    % \textrm{min }(\alpha P + (1 - \alpha) * H) \\
    % \textrm{subject to: } \\
    % 0 <= P <= P_{Threshold} \\
    % H >= H_{Threshold} \\
    % BW*log (1 + \frac{P}{D^2 + H^2})

    
    % Defining the objective function
    prob = optimproblem('ObjectiveSense','min');
    x = optimvar('x',2,1);
    prob.Objective = alpha*x(1) + (1-alpha)*x(2);
    
    % Defining the Constraints
    cons1 = x(1) <= p_thresh;
    cons2 = x(2) >= h_thresh; 
    cons3 = x(1) >= 0;
    
    % Applying the constraints to the problem, leaving constraint 4. That
    % will be added in the for loop.
    prob.Constraints.cons1 = cons1;
    prob.Constraints.cons2 = cons2;
    prob.Constraints.cons3 = cons3;
    
    
    % Creating the Data Array,
    dist = (k_means_cluster(:, 1) - centroid(1,1)) .^ 2 + ... 
        (k_means_cluster(:, 2) - centroid(1,2)) .^ 2;
    
    % Sorting the array
    [~, id] = sort(dist(:,1));
    dist = dist(id, :);
    
    % Creating a Loop to check which max
    K = size(k_means_cluster, 1);
    
    pow = 0;
    height = 0;
    rad = 0;
    users_served = 0;
    total_users = K;
    
    for i=K:-1:1
        cons4 = bw_uav * log(1+x(1)/(dist(i,1) + x(2)^2))>=channel_cap_thresh;
        prob.Constraints.cons4 = cons4;
        x0.x = [p_thresh, 2*h_thresh];
        sol = solve(prob, x0);
        % Check whether the constrainsts are satisfied.
        check = infeasibility(cons1, sol) || infeasibility(cons2, sol) || ...
            infeasibility(cons3, sol) || infeasibility(cons4, sol);
        if (check==0)
            pow = sol.x(1,1);
            height = sol.x(2, 1);
            rad = sqrt(dist(i, 1));
            users_served = i;
            break;
        end
    end 