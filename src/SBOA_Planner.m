function [best_path, best_fitness, convergence_curve] = SBOA_Planner(map_grid, start_pt, goal_pt, max_iter)
    % 秘书鸟优化算法 (Secretary Bird Optimization Algorithm, 2024)
    % 【已升级：统一标准适应度函数，包含五次方动态惩罚与静态基线】
    
    pop_size = 50;           
    num_waypoints = 5;       
    dim = num_waypoints * 2; 
    
    x_max = size(map_grid, 1); y_max = size(map_grid, 2);
    lb = ones(1, dim);       
    ub = repmat([x_max, y_max], 1, num_waypoints); 
    X = repmat(lb, pop_size, 1) + rand(pop_size, dim) .* repmat((ub - lb), pop_size, 1);
    
    % 记录内部优胜劣汰的动态适应度
    fit_X_dyn = inf(pop_size, 1);
    
    % 记录用于展示和对比的绝对全局最优 (静态分数)
    best_static_score = inf;
    best_static_pos = zeros(1, dim);
    
    convergence_curve = zeros(1, max_iter);
    
    for t = 1:max_iter
        best_dyn_score = inf;
        best_dyn_pos = zeros(1, dim); % 用于引导 SBOA 行为的动态最优猎物
        
        % 1. 评估并更新全局最优 (因为惩罚会随 t 增加，必须重新评估)
        for i = 1:pop_size
            [fit_X_dyn(i), current_fit_static] = EvaluatePath(X(i, :), start_pt, goal_pt, map_grid, t, max_iter);
            
            if current_fit_static < best_static_score
                best_static_score = current_fit_static;
                best_static_pos = X(i, :);
            end
            
            if fit_X_dyn(i) < best_dyn_score
                best_dyn_score = fit_X_dyn(i);
                best_dyn_pos = X(i, :);
            end
        end
        
        % 2. SBOA 行为模拟 (追击目标使用动态最优位置 best_dyn_pos)
        for i = 1:pop_size
            r1 = rand();
            % 阶段 1: 狩猎行为 (Exploration)
            if t < max_iter / 3
                % 搜寻猎物
                random_idx = randperm(pop_size, 2);
                X_new = X(i, :) + (X(random_idx(1), :) - X(random_idx(2), :)) .* rand();
            elseif t < 2 * max_iter / 3
                % 消耗猎物能量
                RB = randn();
                X_new = best_dyn_pos + exp((t/max_iter)^4) * (RB - 0.5) * (best_dyn_pos - X(i, :));
            else
                % 攻击猎物
                RL = 0.5 * randn(1, dim); % 简化 Levy 飞行
                X_new = best_dyn_pos + (1 - t/max_iter)^(2 * t/max_iter) .* X(i, :) .* RL;
            end
            
            % 阶段 2: 逃跑行为 (Exploitation)
            if rand() < 0.5
                % 利用环境隐藏
                X_new = X_new + best_dyn_pos + (2 * randn() - 1) .* (1 - t/max_iter) .* X(i, :);
            end
            
            % 越界限制
            X_new = max(X_new, lb);
            X_new = min(X_new, ub);
            
            % 更新个体 (使用动态分数竞争)
            [new_fit_dyn, new_fit_static] = EvaluatePath(X_new, start_pt, goal_pt, map_grid, t, max_iter);
            
            if new_fit_dyn < fit_X_dyn(i)
                X(i, :) = X_new;
                fit_X_dyn(i) = new_fit_dyn;
                
                if new_fit_static < best_static_score
                    best_static_score = new_fit_static;
                    best_static_pos = X_new;
                end
            end
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