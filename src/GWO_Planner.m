function [best_path, best_fitness, convergence_curve] = GWO_Planner(map_grid, start_pt, goal_pt, max_iter)
    pop_size = 30;           
    num_waypoints = 5;       
    dim = num_waypoints * 2; 
    
    x_max = size(map_grid, 1); y_max = size(map_grid, 2);
    lb = ones(1, dim);       
    ub = repmat([x_max, y_max], 1, num_waypoints); 

    X = repmat(lb, pop_size, 1) + rand(pop_size, dim) .* repmat((ub - lb), pop_size, 1);
    
    % 初始化 Alpha, Beta, Delta 狼
    Alpha_pos = zeros(1, dim); Alpha_score = inf;
    Beta_pos = zeros(1, dim);  Beta_score = inf;
    Delta_pos = zeros(1, dim); Delta_score = inf;
    convergence_curve = zeros(1, max_iter);

    for t = 1:max_iter
        for i = 1:pop_size
            current_fit = EvaluatePath(X(i, :), start_pt, goal_pt, map_grid);
            
            % 更新三大头狼的位置
            if current_fit < Alpha_score
                Delta_score = Beta_score; Delta_pos = Beta_pos;
                Beta_score = Alpha_score; Beta_pos = Alpha_pos;
                Alpha_score = current_fit; Alpha_pos = X(i, :);
            elseif current_fit > Alpha_score && current_fit < Beta_score
                Delta_score = Beta_score; Delta_pos = Beta_pos;
                Beta_score = current_fit; Beta_pos = X(i, :);
            elseif current_fit > Alpha_score && current_fit > Beta_score && current_fit < Delta_score
                Delta_score = current_fit; Delta_pos = X(i, :);
            end
        end
        
        a = 2 - t * (2 / max_iter); % a 从 2 线性递减到 0
        
        for i = 1:pop_size
            for j = 1:dim
                r1 = rand(); r2 = rand();
                A1 = 2 * a * r1 - a; C1 = 2 * r2;
                D_alpha = abs(C1 * Alpha_pos(j) - X(i, j));
                X1 = Alpha_pos(j) - A1 * D_alpha;
                
                r1 = rand(); r2 = rand();
                A2 = 2 * a * r1 - a; C2 = 2 * r2;
                D_beta = abs(C2 * Beta_pos(j) - X(i, j));
                X2 = Beta_pos(j) - A2 * D_beta;
                
                r1 = rand(); r2 = rand();
                A3 = 2 * a * r1 - a; C3 = 2 * r2;
                D_delta = abs(C3 * Delta_pos(j) - X(i, j));
                X3 = Delta_pos(j) - A3 * D_delta;
                
                X(i, j) = (X1 + X2 + X3) / 3;
            end
            % 越界限制
            X(i, :) = max(X(i, :), lb);
            X(i, :) = min(X(i, :), ub);
        end
        convergence_curve(t) = Alpha_score;
    end
    best_fitness = Alpha_score;
    best_path = [start_pt; reshape(Alpha_pos, 2, num_waypoints)'; goal_pt];
end

% 局部辅助函数 (与 IHO 中完全一致，保证评价标准公平)
function fitness = EvaluatePath(agent_pos, start_pt, goal_pt, map_grid)
    num_waypoints = length(agent_pos) / 2;
    waypoints = reshape(agent_pos, 2, num_waypoints)';
    full_path = [start_pt; waypoints; goal_pt];
    path_length = 0; collision_count = 0; penalty_factor = 10000; 
    
    for k = 1:(size(full_path, 1) - 1)
        pt1 = full_path(k, :); pt2 = full_path(k+1, :);
        dist = norm(pt1 - pt2);
        path_length = path_length + dist;
        num_samples = ceil(dist * 3); 
        if num_samples > 0
            for s = 0:num_samples
                sample_pt = pt1 + (pt2 - pt1) * (s / num_samples);
                grid_x = round(sample_pt(1)); grid_y = round(sample_pt(2));
                if grid_x >= 1 && grid_x <= size(map_grid,1) && grid_y >= 1 && grid_y <= size(map_grid,2)
                    if map_grid(grid_x, grid_y) == 1, collision_count = collision_count + 1; end
                else
                    collision_count = collision_count + 1; 
                end
            end
        end
    end
    fitness = path_length + path_length * penalty_factor * collision_count;
end