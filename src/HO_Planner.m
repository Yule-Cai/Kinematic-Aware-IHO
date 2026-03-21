function [best_path, best_fitness, convergence_curve] = HO_Planner(map_grid, start_pt, goal_pt, max_iter)
    % 原始河马优化算法 (带启发式初始化基准增强)
    % 【已升级：统一标准适应度函数，包含五次方动态惩罚与静态基线】
    
    pop_size = 50;           
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
    X = max(X, repmat(lb, pop_size, 1)); 
    X = min(X, repmat(ub, pop_size, 1));
    
    % 记录内部优胜劣汰的动态适应度
    fit_X_dyn = inf(pop_size, 1);
    
    % 记录用于展示和对比的绝对全局最优 (静态分数)
    best_static_score = inf;
    best_static_pos = zeros(1, dim);
    
    convergence_curve = zeros(1, max_iter);
    
    for t = 1:max_iter
        best_dyn_score = inf;
        Dominant_Hippo = zeros(1, dim); % 用于引导种群的动态最优河马
        
        % 1. 每一代重新评估所有个体的适应度 (因为惩罚权重随 t 增加)
        for i = 1:pop_size
            [fit_X_dyn(i), current_fit_static] = EvaluatePath(X(i, :), start_pt, goal_pt, map_grid, t, max_iter);
            
            % 更新用于科研图表的全局绝对最优 (静态分数)
            if current_fit_static < best_static_score
                best_static_score = current_fit_static;
                best_static_pos = X(i, :);
            end
            
            % 更新用于内部引导的头领河马 (动态分数)
            if fit_X_dyn(i) < best_dyn_score
                best_dyn_score = fit_X_dyn(i);
                Dominant_Hippo = X(i, :);
            end
        end
        
        % 2. 河马位置更新
        for i = 1:pop_size
            r1 = rand(); r2 = rand();
            if r1 < 0.5
                I = round(1 + rand());
                X_new_i = X(i, :) + rand(1, dim) .* (Dominant_Hippo - I .* X(i, :));
            else
                X_new_i = X(i, :) + rand(1, dim) .* (mean(X) - X(i, :));
            end
            
            if rand() < 0.3
                X_new_i = X_new_i + (rand(1, dim) - 0.5) .* (ub(1) - lb(1)) .* 0.1;
            end
            
            X_new_i = max(X_new_i, lb); 
            X_new_i = min(X_new_i, ub);
            
            % 严密贪婪选择 (用动态分数比拼)
            [fit_new_dyn, fit_new_static] = EvaluatePath(X_new_i, start_pt, goal_pt, map_grid, t, max_iter);
            
            if fit_new_dyn < fit_X_dyn(i)
                X(i, :) = X_new_i;
                fit_X_dyn(i) = fit_new_dyn; % 更新当前河马的动态适应度
                
                % 顺便检查是否破了全局静态记录
                if fit_new_static < best_static_score
                    best_static_score = fit_new_static;
                    best_static_pos = X_new_i;
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
    
    % 2. 计算平滑度 (统一所有算法的评价标准)
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
    % 1. 自适应动态惩罚 (五次方核爆指数) - 用于物竞天择淘汰
    % =================================================================
    dyn_penalty_weight = 500 + 99500 * (t / max_iter)^5;
    fit_dyn = path_length + smooth_weight * smoothness_cost + dyn_penalty_weight * collision_count;
    
    % =================================================================
    % 2. 视觉级静态惩罚 (1000) - 用于记录科学对比的最终成绩
    % =================================================================
    static_penalty_weight = 1000; 
    fit_static = path_length + static_penalty_weight * collision_count;
end