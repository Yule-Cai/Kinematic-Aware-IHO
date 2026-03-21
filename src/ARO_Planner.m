function [best_path, best_fitness, convergence_curve] = ARO_Planner(map_grid, start_pt, goal_pt, max_iter)
    % ARO (Artificial Rabbits Optimization, 2022) 路径规划 
    % 【已升级：统一标准适应度函数，包含五次方动态惩罚与静态基线】
    
    pop_size = 50;           
    num_waypoints = 5;       
    dim = num_waypoints * 2; 
    
    x_max = size(map_grid, 1); y_max = size(map_grid, 2);
    lb = ones(1, dim);       
    ub = repmat([x_max, y_max], 1, num_waypoints); 

    X = repmat(lb, pop_size, 1) + rand(pop_size, dim) .* repmat((ub - lb), pop_size, 1);
    
    best_fitness = inf;
    best_position = zeros(1, dim);
    convergence_curve = zeros(1, max_iter);
    
    % 初始化缓存每个兔子的动态适应度
    fit_dyn_pop = zeros(pop_size, 1);

    % 初始化评估 (此时 t = 1)
    for i = 1:pop_size
        [fit_dyn_pop(i), current_fit_static] = EvaluatePath(X(i, :), start_pt, goal_pt, map_grid, 1, max_iter);
        if current_fit_static < best_fitness
            best_fitness = current_fit_static;
            best_position = X(i, :);
        end
    end

    for t = 1:max_iter
        % 每一代开始前，重新计算动态适应度 (因为 t 变了，核爆惩罚权重上升了！)
        for i = 1:pop_size
            [fit_dyn_pop(i), current_fit_static] = EvaluatePath(X(i, :), start_pt, goal_pt, map_grid, t, max_iter);
            if current_fit_static < best_fitness
                best_fitness = current_fit_static;
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
            
            % 更新策略 (使用动态适应度 fit_dyn 进行物竞天择的比较)
            [new_fit_dyn, new_fit_static] = EvaluatePath(X_new, start_pt, goal_pt, map_grid, t, max_iter);
            
            if new_fit_dyn < fit_dyn_pop(i)
                X(i, :) = X_new;
                fit_dyn_pop(i) = new_fit_dyn; % 更新当前兔子的适应度缓存
                
                % 如果新位置的静态表现也破了全局记录，则更新全局 Best
                if new_fit_static < best_fitness
                    best_fitness = new_fit_static;
                    best_position = X_new;
                end
            end
        end
        convergence_curve(t) = best_fitness;
    end
    best_path = [start_pt; reshape(best_position, 2, num_waypoints)'; goal_pt];
end

% =========================================================================
% 统一标准局部辅助函数: 适应度计算 (必须与 Ours_Planner 100% 相同)
% =========================================================================
function [fit_dyn, fit_static] = EvaluatePath(agent_pos, start_pt, goal_pt, map_grid, t, max_iter)
    num_waypoints = length(agent_pos) / 2;
    waypoints = reshape(agent_pos, 2, num_waypoints)';
    full_path = [start_pt; waypoints; goal_pt];
    
    path_length = 0; 
    collision_count = 0; 
    smoothness_cost = 0; 
    
    % 1. 计算距离与碰撞
    for k = 1:(size(full_path, 1) - 1)
        pt1 = full_path(k, :); 
        pt2 = full_path(k+1, :);
        dist = norm(pt1 - pt2);
        path_length = path_length + dist;
        
        num_samples = ceil(dist * 3); 
        if num_samples > 0
            for s = 0:num_samples
                sample_pt = pt1 + (pt2 - pt1) * (s / num_samples);
                grid_x = round(sample_pt(1)); 
                grid_y = round(sample_pt(2));
                if grid_x >= 1 && grid_x <= size(map_grid,1) && grid_y >= 1 && grid_y <= size(map_grid,2)
                    if map_grid(grid_x, grid_y) == 1
                        collision_count = collision_count + 1; 
                    end
                else
                    collision_count = collision_count + 1; 
                end
            end
        end
    end
    
    % 2. 计算平滑度 (必须带上，保证所有算法考卷一样)
    for k = 2:(size(full_path, 1) - 1)
        v1 = full_path(k, :) - full_path(k-1, :);
        v2 = full_path(k+1, :) - full_path(k, :);
        norm_v1 = norm(v1); norm_v2 = norm(v2);
        if norm_v1 > 1e-5 && norm_v2 > 1e-5
            cos_theta = dot(v1, v2) / (norm_v1 * norm_v2);
            cos_theta = max(min(cos_theta, 1), -1); 
            smoothness_cost = smoothness_cost + (1 - cos_theta);
        end
    end
    
    smooth_weight = 2.0; 
    
    % =================================================================
    % 1. 自适应动态惩罚 (五次方核爆指数) - 用于内部优胜劣汰
    % =================================================================
    dyn_penalty_weight = 500 + 99500 * (t / max_iter)^5;
    fit_dyn = path_length + smooth_weight * smoothness_cost + dyn_penalty_weight * collision_count;
    
    % =================================================================
    % 2. 视觉级静态惩罚 (1000) - 用于展示和记录 Best
    % =================================================================
    static_penalty_weight = 1000; 
    fit_static = path_length + static_penalty_weight * collision_count;
end