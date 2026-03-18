function [best_path, best_fitness, convergence_curve] = PSO_Planner(map_grid, start_pt, goal_pt, max_iter)
    pop_size = 30;           
    num_waypoints = 5;       
    dim = num_waypoints * 2; 
    
    x_max = size(map_grid, 1); y_max = size(map_grid, 2);
    lb = ones(1, dim);       
    ub = repmat([x_max, y_max], 1, num_waypoints); 

    % PSO 特定参数
    w = 0.7; c1 = 1.5; c2 = 1.5; 
    
    % 初始化位置和速度
    X = repmat(lb, pop_size, 1) + rand(pop_size, dim) .* repmat((ub - lb), pop_size, 1);
    V = zeros(pop_size, dim);
    
    % 个体最优与全局最优
    pbest_X = X;
    pbest_fitness = inf(pop_size, 1);
    gbest_fitness = inf;
    gbest_X = zeros(1, dim);
    convergence_curve = zeros(1, max_iter);

    for t = 1:max_iter
        for i = 1:pop_size
            current_fit = EvaluatePath(X(i, :), start_pt, goal_pt, map_grid);
            % 更新个体最优
            if current_fit < pbest_fitness(i)
                pbest_fitness(i) = current_fit;
                pbest_X(i, :) = X(i, :);
            end
            % 更新全局最优
            if current_fit < gbest_fitness
                gbest_fitness = current_fit;
                gbest_X = X(i, :);
            end
        end
        
        % 速度与位置更新
        for i = 1:pop_size
            r1 = rand(); r2 = rand();
            V(i, :) = w * V(i, :) + c1 * r1 * (pbest_X(i, :) - X(i, :)) + c2 * r2 * (gbest_X - X(i, :));
            X(i, :) = X(i, :) + V(i, :);
            % 越界限制
            X(i, :) = max(X(i, :), lb);
            X(i, :) = min(X(i, :), ub);
        end
        convergence_curve(t) = gbest_fitness;
    end
    best_fitness = gbest_fitness;
    best_path = [start_pt; reshape(gbest_X, 2, num_waypoints)'; goal_pt];
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