function [best_path, best_fitness, convergence_curve] = ARO_Planner(map_grid, start_pt, goal_pt, max_iter)
    % ARO (Artificial Rabbits Optimization, 2022) 路径规划
    pop_size = 30;           
    num_waypoints = 5;       
    dim = num_waypoints * 2; 
    
    x_max = size(map_grid, 1); y_max = size(map_grid, 2);
    lb = ones(1, dim);       
    ub = repmat([x_max, y_max], 1, num_waypoints); 

    X = repmat(lb, pop_size, 1) + rand(pop_size, dim) .* repmat((ub - lb), pop_size, 1);
    
    best_fitness = inf;
    best_position = zeros(1, dim);
    convergence_curve = zeros(1, max_iter);

    for t = 1:max_iter
        % 评估适应度
        for i = 1:pop_size
            current_fit = EvaluatePath(X(i, :), start_pt, goal_pt, map_grid);
            if current_fit < best_fitness
                best_fitness = current_fit;
                best_position = X(i, :);
            end
        end
        
        % ARO 核心参数 L
        L = (exp(1) - exp(((t - 1) / max_iter)^2)) * sin(2 * pi * rand());
        
        for i = 1:pop_size
            if L > 1
                % 绕道觅食 (Exploration)
                R = rand(1, dim);
                index = randi([1, pop_size]);
                X_new = X(i, :) + R .* (X(index, :) - X(i, :)) + round(0.5 * (0.05 + rand())) * randn(1, dim);
            else
                % 随机隐藏 (Exploitation)
                H = randn(1, dim);
                X_new = best_position + L * H .* (X(i, :) - best_position);
            end
            
            % 越界限制
            X_new = max(X_new, lb);
            X_new = min(X_new, ub);
            
            % 更新策略
            if EvaluatePath(X_new, start_pt, goal_pt, map_grid) < EvaluatePath(X(i, :), start_pt, goal_pt, map_grid)
                X(i, :) = X_new;
            end
        end
        convergence_curve(t) = best_fitness;
    end
    best_path = [start_pt; reshape(best_position, 2, num_waypoints)'; goal_pt];
end

% 局部辅助函数: 适应度计算
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