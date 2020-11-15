% Function that returns the points that the UAV should either stay or
% travel to and fro

function [points] = optimal_points(x_bs, y_bs, x_c, y_c, P_bs, P_uav, ...
    bw_bs, bw_uav, h_uav, h_bs, h_relay, capacity_thresh)

    % x_bs: x coordinate of base station
    % y_bs: y coordinate of base station
    % x_c: x coordinate of centroid
    % y_c: y coordinate of centroid
    % P_bs: Power of the base station
    % P_uav: Power of the UAV
    % bw_bs: Bandwidth of the base station
    % bw_uav: Bandwidth of the UAV
    % h_uav: Height of the uav;
    % capacity_thresh: Threshold of the capacity required to transmit.

    % Converting from a 2D plane to a single line
    d = abs(sqrt((x_c - x_bs)^2 + (y_c -  y_bs)^2));
    theta = atan2((y_c - y_bs), (x_c - x_bs));

    % Defining the equations
    syms x
    capacity_bs = bw_bs*log(1 + P_bs/(x^2 + (h_relay-h_bs)^2));
    capacity_uav = bw_uav*log(1 + P_uav/((x-d)^2 + (h_uav-h_relay)^2));
    
    % Solving the equations. vpasolve is a numerical solver.
    intersection_bs_thresh = vpasolve(capacity_bs == capacity_thresh, x, ...
        [0, d]);
    intersection_uav_thresh = vpasolve(capacity_uav == capacity_thresh, x, ...
        [0, d]);
    intersection_both_capacities = vpasolve(capacity_uav == capacity_bs, x, ...
        [0, d]);
    val_intersection_both_capacities = subs(capacity_uav, intersection_both_capacities);
  
    if (val_intersection_both_capacities > capacity_thresh)
        % if the intersection of both the capacities is more than the
        % threshold.
        x1 = x_bs + cos(theta) * intersection_both_capacities;
        y1 = y_bs + sin(theta) * intersection_both_capacities;
        points = [x1, y1; x1, y1];
%     elseif (isempty(intersection_bs_thresh) || intersection_uav_thresh)
%         % If the intersection is less than the threshold and there is no
%         % intersection between the threshold and the two curves. Basically
%         % this means that the power is not enough for establishing the
%         % optimal channel capacity.
%         points = [x_bs, y_bs; x_bs, y_bs];
    else
        x1 = x_bs + cos(theta) * intersection_bs_thresh;
        y1 = y_bs + sin(theta) * intersection_bs_thresh;
        x2 = x_bs + cos(theta) * intersection_uav_thresh;
        y2 = y_bs + sin(theta) * intersection_uav_thresh;
        points = [x1, y1; x2, y2];
    end