function [best_path, best_fitness, convergence_curve] = PSO_Planner(map_grid, start_pt, goal_pt, max_iter)
    % 经典粒子群优化算法 (PSO)
    % 【已升级：统一标准适应度函数，包含五次方动态惩罚与静态基线】
    
    pop_size = 50;           
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
    
    % 个体最优与全局最优 (使用动态适应度来引导粒子飞行)
    pbest_X = X;
    pbest_fitness_dyn = inf(pop_size, 1);
    gbest_fitness_dyn = inf;
    gbest_X = zeros(1, dim);
    
    % 用于科研图表的全局绝对最优 (静态分数)
    best_static_score = inf;
    best_static_pos = zeros(1, dim);
    
    convergence_curve = zeros(1, max_iter);
    
    for t = 1:max_iter
        for i = 1:pop_size
            [current_fit_dyn, current_fit_static] = EvaluatePath(X(i, :), start_pt, goal_pt, map_grid, t, max_iter);
            
            % 更新用于对比的静态真实成绩
            if current_fit_static < best_static_score
                best_static_score = current_fit_static;
                best_static_pos = X(i, :);
            end
            
            % 更新个体最优 (动态规则)
            if current_fit_dyn < pbest_fitness_dyn(i)
                pbest_fitness_dyn(i) = current_fit_dyn;
                pbest_X(i, :) = X(i, :);
            end
            
            % 更新全局最优 (动态规则)
            if current_fit_dyn < gbest_fitness_dyn
                gbest_fitness_dyn = current_fit_dyn;
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
        
        % 记录收敛曲线 (使用静态最优分数)
        convergence_curve(t) = best_static_score;
    end
    
    best_fitness = best_static_score;
    best_path = [start_pt; reshape(best_static_pos, 2, num_waypoints)'; goal_pt];
end

% =========================================================================
% 统一标准局部辅助函数: 适应度计算 (与 Ours_Planner 100% 相同)
% =========================================================================
function [fit_dyn, fit_static] = EvaluatePath(agent_pos, start_pt, goal_pt, map_grid, t, max_iter)
    num_waypoints = length(agent_pos) / 2;
    waypoints = reshape(agent_pos, 2, num_waypoints)';
    full_path = [start_pt; waypoints; goal_pt];
    
    path_length = 0; 
    collision_count = 0; 
    smoothness_cost = 0; 
    
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
    dyn_penalty_weight = 500 + 99500 * (t / max_iter)^5;
    fit_dyn = path_length + smooth_weight * smoothness_cost + dyn_penalty_weight * collision_count;
    
    static_penalty_weight = 1000; 
    fit_static = path_length + static_penalty_weight * collision_count;
end