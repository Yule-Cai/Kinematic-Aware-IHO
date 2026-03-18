function [best_path, best_fitness, convergence_curve] = HO_Planner(map_grid, start_pt, goal_pt, max_iter)
    % 原始河马优化算法 (带启发式初始化基准增强)
    pop_size = 30;           
    num_waypoints = 5;       
    dim = num_waypoints * 2; 
    
    x_max = size(map_grid, 1); y_max = size(map_grid, 2);
    lb = ones(1, dim);       
    ub = repmat([x_max, y_max], 1, num_waypoints); 

    % 启发式直线基准初始化
    X = zeros(pop_size, dim);
    for i = 1:pop_size
        for w = 1:num_waypoints
            alpha = w / (num_waypoints + 1);
            base_pt = start_pt + alpha * (goal_pt - start_pt);
            noise = (rand(1, 2) - 0.5) * (max(x_max, y_max) * 0.4); 
            X(i, (w*2-1):w*2) = base_pt + noise;
        end
    end
    X = max(X, repmat(lb, pop_size, 1)); X = min(X, repmat(ub, pop_size, 1));
    
    fit_X = inf(pop_size, 1);
    best_fitness = inf;
    best_position = zeros(1, dim);
    convergence_curve = zeros(1, max_iter);

    for i = 1:pop_size
        fit_X(i) = EvaluatePath(X(i, :), start_pt, goal_pt, map_grid);
        if fit_X(i) < best_fitness
            best_fitness = fit_X(i);
            best_position = X(i, :);
        end
    end

    for t = 1:max_iter
        for i = 1:pop_size
            r1 = rand(); r2 = rand();
            if r1 < 0.5
                Dominant_Hippo = best_position;
                I = round(1 + rand());
                X_new_i = X(i, :) + rand(1, dim) .* (Dominant_Hippo - I .* X(i, :));
            else
                X_new_i = X(i, :) + rand(1, dim) .* (mean(X) - X(i, :));
            end
            
            if rand() < 0.3
                X_new_i = X_new_i + (rand(1, dim) - 0.5) .* (ub(1) - lb(1)) .* 0.1;
            end
            
            X_new_i = max(X_new_i, lb); X_new_i = min(X_new_i, ub);
            
            % 严密贪婪选择
            fit_new = EvaluatePath(X_new_i, start_pt, goal_pt, map_grid);
            if fit_new < fit_X(i)
                X(i, :) = X_new_i;
                fit_X(i) = fit_new;
                if fit_new < best_fitness
                    best_fitness = fit_new;
                    best_position = X_new_i;
                end
            end
        end
        convergence_curve(t) = best_fitness;
    end
    best_path = [start_pt; reshape(best_position, 2, num_waypoints)'; goal_pt];
end

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